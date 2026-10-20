#pragma once
#include "dynamic_filter.hpp"
#include <unordered_map>
#include <unordered_set>

namespace dynamic_lio {
struct VisibilityConfig {
  double voxel = 0.20, margin = 0.30, range = 25., history = 5.;
  double min_observation = 0.4, increment = 0.35, decrement = 0.15;
  double remove_threshold = 0.8, backend_threshold = 0.2;
  int confirmations = 3;
  size_t capacity = 300000;
};
// Scores are bounded evidence, not calibrated probabilities. No truth input.
class VisibilityFilter {
 public:
  explicit VisibilityFilter(VisibilityConfig config = {}) : config_(config) {
    if (!std::isfinite(config.voxel) || config.voxel <= 0 ||
        !std::isfinite(config.margin) || config.margin < config.voxel ||
        !std::isfinite(config.range) || config.range <= config.margin ||
        !std::isfinite(config.history) || config.history <= 0 ||
        !std::isfinite(config.min_observation) || config.min_observation <= 0 ||
        config.min_observation > config.history || config.confirmations < 2 || !config.capacity ||
        !std::isfinite(config.increment) || config.increment <= 0 || config.increment > 1 ||
        !std::isfinite(config.decrement) || config.decrement <= 0 || config.decrement > 1 ||
        !std::isfinite(config.remove_threshold) || config.remove_threshold <= 0 || config.remove_threshold > 1 ||
        !std::isfinite(config.backend_threshold) || config.backend_threshold < 0 ||
        config.backend_threshold >= config.remove_threshold)
      throw std::invalid_argument("Invalid visibility confidence configuration");
  }
  double score(const Eigen::Vector3d &p) const {
    auto it = cells_.find(key(p));
    return it == cells_.end() ? 0. : it->second.dynamic;
  }
  bool isDynamic(const Eigen::Vector3d &p) const { return score(p)>=config_.remove_threshold; }
  double weight(const Eigen::Vector3d &p) const { return std::max(0.1, 1.-score(p)); }
  bool backendStatic(const Eigen::Vector3d &p) const { return score(p) <= config_.backend_threshold; }
  void reset() { cells_.clear(); last_stamp_ = -1; }
  size_t cells() const { return cells_.size(); }
  template<class Point>
  std::vector<Box> process(pcl::PointCloud<Point> &retained, const pcl::PointCloud<Point> &raw,
                          const Eigen::Isometry3d &pose, double stamp) {
    if (!std::isfinite(stamp) || !pose.matrix().allFinite())
      throw std::invalid_argument("Invalid visibility pose or timestamp");
    if (last_stamp_ >= 0 && (stamp <= last_stamp_ || stamp-last_stamp_ > 1. ||
        (pose.translation()-last_pose_.translation()).norm() > 0.8 ||
        Eigen::AngleAxisd(last_pose_.linear().transpose()*pose.linear()).angle() > 0.45)) reset();
    last_stamp_ = stamp; last_pose_ = pose;
    // Prune by observation time; unknown/expired space must not become dynamic.
    for (auto it=cells_.begin(); it!=cells_.end();) {
      if (stamp-it->second.last > config_.history) it=cells_.erase(it); else ++it;
    }
    std::unordered_map<Key, Eigen::Vector3d, Hash> endpoints;
    for (const auto &p : raw) {
      Eigen::Vector3d q=pose*Eigen::Vector3d(p.x,p.y,p.z);
      if (q.allFinite() && (q-pose.translation()).norm() >= 0.5)
        endpoints.emplace(key(q), q);
    }
    // Classify BEFORE this scan contributes free-space evidence. One vote/voxel/scan.
    std::vector<std::pair<Key,double>> updates;
    for (const auto &entry : endpoints) {
      if ((entry.second-pose.translation()).norm() > config_.range) continue;
      auto found=cells_.find(entry.first);
      if (found==cells_.end()) continue;
      auto &cell=found->second;
      const bool trusted_free=cell.free_hits>=config_.confirmations &&
        cell.free_last-cell.free_first>=config_.min_observation && stamp-cell.free_last<=config_.history;
      double previous=cell.dynamic;
      if (trusted_free) {
        // A moving surface can cross a voxel between scans. Borrow only prior
        // nearby evidence, never a score created by another ray in this scan.
        for(int x=-1;x<=1;++x) for(int y=-1;y<=1;++y) for(int z=-1;z<=1;++z) {
          auto neighbor=cells_.find({entry.first.x+x,entry.first.y+y,entry.first.z+z});
          if(neighbor!=cells_.end() && stamp-neighbor->second.last<=0.5)
            previous=std::max(previous,neighbor->second.dynamic);
        }
      }
      updates.push_back({entry.first,trusted_free?std::min(1.,previous+config_.increment):std::max(0.,previous-config_.decrement)});
      // An occupied return consumes the old free-space evidence. Repeated
      // stationary returns must decay, rather than reuse one contradiction.
      cell.free_hits=0; cell.free_first=cell.free_last=-1;
    }
    for(const auto &update:updates) { auto &cell=cells_.at(update.first);cell.dynamic=update.second;cell.last=stamp; }
    std::vector<Box> erased;
    for (const auto &entry : endpoints) {
      if (score(entry.second) >= config_.remove_threshold) {
        const auto &k=entry.first;
        Eigen::Vector3d low(k.x*config_.voxel,k.y*config_.voxel,k.z*config_.voxel);
        erased.push_back({low,low+Eigen::Vector3d::Constant(config_.voxel)});
      }
    }
    retained.erase(std::remove_if(retained.begin(),retained.end(),[&](const Point &p) {
      return score(pose*Eigen::Vector3d(p.x,p.y,p.z))>=config_.remove_threshold;
    }),retained.end());
    retained.width=retained.size(); retained.height=1;
    // A return occludes everything behind it. Guard surfaces in neighboring rays.
    std::unordered_set<Key, Hash> protected_cells, free;
    const int guard=static_cast<int>(std::ceil(config_.margin/config_.voxel));
    for (const auto &entry : endpoints) for(int x=-guard;x<=guard;++x)
      for(int y=-guard;y<=guard;++y) for(int z=-guard;z<=guard;++z)
        protected_cells.insert({entry.first.x+x,entry.first.y+y,entry.first.z+z});
    for (const auto &entry : endpoints) {
      Eigen::Vector3d delta=entry.second-pose.translation();
      const double distance=delta.norm(), stop=std::min(config_.range,distance-config_.margin);
      const Eigen::Vector3d direction=delta/distance;
      for(double d=0.5;d<stop;d+=config_.voxel/2) {
        const Key k=key(pose.translation()+direction*d);
        if (!protected_cells.count(k)) free.insert(k);
      }
    }
    for (const auto &k : free) {
      auto it=cells_.find(k);
      if (it==cells_.end()) {
        if (cells_.size()>=config_.capacity) continue;
        it=cells_.emplace(k,Cell{}).first;
      }
      auto &cell=it->second;
      if (cell.free_hits==0 || stamp-cell.free_last>0.5) { cell.free_hits=0; cell.free_first=stamp; }
      cell.free_hits=std::min(config_.confirmations,cell.free_hits+1);
      cell.free_last=cell.last=stamp;
      // An observed free cell is not currently occupied by a moving return.
      cell.dynamic=std::max(0.,cell.dynamic-config_.decrement);
    }
    return erased;
  }
 private:
  struct Key { int x,y,z; bool operator==(const Key &b) const { return x==b.x&&y==b.y&&z==b.z; } };
  struct Hash { size_t operator()(const Key &k) const {
    size_t h=std::hash<int>()(k.x); h^=std::hash<int>()(k.y)+0x9e3779b9+(h<<6)+(h>>2);
    return h^(std::hash<int>()(k.z)+0x9e3779b9+(h<<6)+(h>>2));
  } };
  struct Cell { int free_hits=0; double free_first=-1,free_last=-1,last=-1,dynamic=0; };
  Key key(const Eigen::Vector3d &p) const { return {int(std::floor(p.x()/config_.voxel)),int(std::floor(p.y()/config_.voxel)),int(std::floor(p.z()/config_.voxel))}; }
  VisibilityConfig config_;
  std::unordered_map<Key,Cell,Hash> cells_;
  Eigen::Isometry3d last_pose_=Eigen::Isometry3d::Identity();
  double last_stamp_=-1;
};
}  // namespace dynamic_lio
