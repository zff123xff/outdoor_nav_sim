#pragma once

#include "dynamic_filter.hpp"
#include <unordered_map>
#include <unordered_set>

namespace dynamic_lio {
// Clear only occupied voxels traversed by actual lidar rays. Missing returns
// and space behind an occluding return are never evidence of free space.
class MapUpdater {
 public:
  MapUpdater(double voxel = 0.20, double margin = 0.30, int confirmations = 2,
             double range = 25.0, size_t capacity = 2000000)
      : voxel_(voxel), margin_(margin), confirmations_(confirmations), range_(range), capacity_(capacity) {
    if (!std::isfinite(voxel) || voxel <= 0 || !std::isfinite(margin) || margin < voxel ||
        !std::isfinite(range) || range <= margin || confirmations < 1 || capacity == 0)
      throw std::invalid_argument("Invalid observed map update configuration");
  }

  template<class Point>
  std::vector<Box> clearObservedFree(const pcl::PointCloud<Point> &raw,
                                  const Eigen::Isometry3d &pose, double stamp) {
    if (last_stamp_ >= 0 && (stamp <= last_stamp_ || stamp-last_stamp_ > 1.0 ||
        (pose.translation()-last_pose_.translation()).norm() > 0.8 ||
        Eigen::AngleAxisd(last_pose_.linear().transpose()*pose.linear()).angle() > 0.45)) {
      occupied_.clear(); evidence_.clear();
    }
    last_stamp_ = stamp; last_pose_ = pose;
    std::unordered_set<Key, Hash> endpoints;
    std::vector<Eigen::Vector3d> targets;
    for (const auto &point : raw) {
      const Eigen::Vector3d p = pose*Eigen::Vector3d(point.x, point.y, point.z);
      if (!p.allFinite()) continue;
      const Key k = key(p);
      if (endpoints.insert(k).second) targets.push_back(p);
    }
    std::unordered_set<Key, Hash> free;
    const auto origin = pose.translation();
    for (const auto &target : targets) {
      const Eigen::Vector3d delta = target-origin;
      const double distance = delta.norm();
      const double stop = std::min(range_, distance-margin_);
      if (distance < 0.5) continue;
      const Eigen::Vector3d direction = delta/distance;
      for (double d = 0.5; d < stop; d += voxel_/2) {
        const Key k = key(origin+direction*d);
        if (occupied_.count(k)) free.insert(k);
      }
    }
    std::vector<Box> removed;
    const int guard = static_cast<int>(std::ceil(margin_/voxel_));
    for (const auto &k : free) {
      bool surface = false;
      for (int x = -guard; x <= guard && !surface; ++x)
        for (int y = -guard; y <= guard && !surface; ++y)
          for (int z = -guard; z <= guard && !surface; ++z)
            surface = endpoints.count({k.x+x, k.y+y, k.z+z});
      if (surface) { evidence_.erase(k); continue; }
      auto &evidence = evidence_[k];
      if (stamp-evidence.last > 0.5) evidence.hits = 0;
      evidence.last = stamp;
      if (++evidence.hits < confirmations_) continue;
      const Eigen::Vector3d low(k.x*voxel_, k.y*voxel_, k.z*voxel_);
      removed.push_back({low, low+Eigen::Vector3d::Constant(voxel_)});
      occupied_.erase(k); evidence_.erase(k);
    }
    for (auto it = evidence_.begin(); it != evidence_.end();) {
      if (stamp-it->second.last > 0.5) it = evidence_.erase(it); else ++it;
    }
    return removed;
  }

  template<class Point>
  void observeStatic(const pcl::PointCloud<Point> &cloud, const Eigen::Isometry3d &pose) {
    for (const auto &point : cloud) {
      const Eigen::Vector3d p = pose*Eigen::Vector3d(point.x, point.y, point.z);
      if (!p.allFinite()) continue;
      const Key k = key(p);
      evidence_.erase(k);
      if (occupied_.size() < capacity_) occupied_.insert(k);
    }
  }
  size_t occupiedVoxels() const { return occupied_.size(); }
  bool atCapacity() const { return occupied_.size() >= capacity_; }

  template<class Point>
  void eraseCleared(pcl::PointCloud<Point> &cloud, const std::vector<Box> &boxes) const {
    // Voxel lookup keeps deletion linear in map size, rather than map*boxes.
    std::unordered_set<Key, Hash> cleared;
    for (const auto &box : boxes) cleared.insert(key((box.low+box.high)/2));
    cloud.erase(std::remove_if(cloud.begin(), cloud.end(), [&](const Point &point) {
      return cleared.count(key(Eigen::Vector3d(point.x, point.y, point.z))) != 0;
    }), cloud.end());
  }

 private:
  struct Key {
    int x, y, z;
    bool operator==(const Key &other) const { return x == other.x && y == other.y && z == other.z; }
  };
  struct Hash {
    size_t operator()(const Key &k) const {
      size_t seed = std::hash<int>()(k.x);
      seed ^= std::hash<int>()(k.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= std::hash<int>()(k.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      return seed;
    }
  };
  struct Evidence { int hits = 0; double last = 0; };
  Key key(const Eigen::Vector3d &p) const {
    return {static_cast<int>(std::floor(p.x()/voxel_)), static_cast<int>(std::floor(p.y()/voxel_)),
            static_cast<int>(std::floor(p.z()/voxel_))};
  }
  double voxel_, margin_;
  int confirmations_;
  double range_;
  size_t capacity_;
  double last_stamp_ = -1;
  Eigen::Isometry3d last_pose_ = Eigen::Isometry3d::Identity();
  std::unordered_set<Key, Hash> occupied_;
  std::unordered_map<Key, Evidence, Hash> evidence_;
};
}  // namespace dynamic_lio
