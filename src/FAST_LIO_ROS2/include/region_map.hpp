#pragma once
#include "dynamic_filter.hpp"
#include <unordered_map>
#include <unordered_set>

namespace dynamic_lio {
struct RegionConfig {
  double voxel=.5, surface_voxel=.1, range=25.;
  double dynamic_weight=.2, recovering_weight=.8;
  double dynamic_hold=2., static_time=5., observation_gap=.5, history=120., overlap=.8;
  int static_observations=20;
  size_t capacity=200000;
};
// Sidecar metadata for ikd-tree geometry: no truth, centroid motion or tree pointers.
// Dynamic/recovering history survives map point deletion and occlusion.
class RegionMap {
 public:
  enum class State { Unknown, Static, Dynamic, Recovering };
  struct Stats { size_t unknown=0, stable=0, dynamic=0, recovering=0; bool at_capacity=false; };
  explicit RegionMap(RegionConfig config={}) : config_(config) {
    for(double v:{config.voxel,config.surface_voxel,config.range,config.dynamic_hold,
                  config.static_time,config.observation_gap,config.history})
      if(!std::isfinite(v)||v<=0) throw std::invalid_argument("Invalid region configuration");
    for(double v:{config.dynamic_weight,config.recovering_weight,config.overlap})
      if(!std::isfinite(v)||v<=0||v>1) throw std::invalid_argument("Invalid region weight/overlap");
    if(config.surface_voxel>config.voxel || config.recovering_weight<config.dynamic_weight ||
       config.static_observations<2 || !config.capacity || config.history<config.static_time)
      throw std::invalid_argument("Invalid region limits");
  }
  State state(const Eigen::Vector3d &p) const {
    const auto it=cells_.find(key(p,config_.voxel));
    return it==cells_.end()?State::Unknown:it->second.state;
  }
  double weight(const Eigen::Vector3d &p) const {
    const auto it=cells_.find(key(p,config_.voxel));
    if(it==cells_.end()) return 1.;
    const auto &c=it->second;
    if(c.state==State::Dynamic) return config_.dynamic_weight;
    if(c.state==State::Recovering) {
      const double time=c.stable_since<0?0.:std::max(0.,last_stamp_-c.stable_since)/config_.static_time;
      const double progress=std::min(1.,std::min(time,double(c.hits)/config_.static_observations));
      return config_.dynamic_weight+(config_.recovering_weight-config_.dynamic_weight)*progress;
    }
    return 1.;  // Unknown geometry must retain a route for map expansion.
  }
  bool eligible(const Eigen::Vector3d &p) const {
    const auto s=state(p);return s==State::Unknown||s==State::Static;
  }
  Stats stats() const { return stats_; }
  void reset() { cells_.clear();last_stamp_=-1;stats_={}; }

  template<class Point>
  void process(const pcl::PointCloud<Point> &raw, const Eigen::Isometry3d &pose,
               const std::vector<Eigen::Vector3d> &dynamic_points,
               const std::vector<Box> &observed_free, double stamp) {
    if(!std::isfinite(stamp)||!pose.matrix().allFinite()) throw std::invalid_argument("Invalid region pose/time");
    if(stamp==last_stamp_) return;  // One observation per region per scan.
    if(last_stamp_>=0 && (stamp<last_stamp_ || stamp-last_stamp_>1. ||
       (pose.translation()-last_pose_.translation()).norm()>.8 ||
       Eigen::AngleAxisd(last_pose_.linear().transpose()*pose.linear()).angle()>.45)) reset();
    last_stamp_=stamp;last_pose_=pose;
    for(auto it=cells_.begin();it!=cells_.end();) {
      // Forget inactive ordinary regions, but never silently rehabilitate a
      // previously dynamic region just because it has not been visible.
      if((it->second.state==State::Unknown||it->second.state==State::Static) &&
          stamp-it->second.last_seen>config_.history) it=cells_.erase(it);else ++it;
    }
    std::unordered_set<Key,Hash> changed;
    // Free evidence counts only when it actually contradicts a stored surface
    // anchor, not when a ray crosses some other empty part of this region.
    for(const auto &box:observed_free) {
      if(!box.low.allFinite()||!box.high.allFinite()||
         (box.high.array()<box.low.array()).any()||(box.high-box.low).maxCoeff()>10.)
        throw std::invalid_argument("Invalid region free box");
      const auto lo=key(box.low,config_.voxel),hi=key(box.high,config_.voxel);
      for(int x=lo.x;x<=hi.x;++x) for(int y=lo.y;y<=hi.y;++y) for(int z=lo.z;z<=hi.z;++z) {
        const Key k{x,y,z};auto it=cells_.find(k);if(it==cells_.end()) continue;
        for(const auto &anchor:it->second.reference) {
          const Eigen::Vector3d center((anchor.x+.5)*config_.surface_voxel,
              (anchor.y+.5)*config_.surface_voxel,(anchor.z+.5)*config_.surface_voxel);
          if(box.contains(center)) { changed.insert(k);break; }
        }
      }
    }
    for(const auto &p:dynamic_points) if(p.allFinite() &&
        (p-pose.translation()).norm()<=config_.range) changed.insert(key(p,config_.voxel));
    for(const auto &k:changed) {
      auto *cell=get(k);if(!cell) continue;
      cell->state=State::Dynamic;cell->last_dynamic=cell->last_seen=stamp;
      cell->stable_since=-1;cell->hits=0;cell->reference.clear();
    }
    std::unordered_map<Key,Surface,Hash> observations;
    for(const auto &p:raw) {
      const Eigen::Vector3d q=pose*Eigen::Vector3d(p.x,p.y,p.z);
      const double distance=(q-pose.translation()).norm();
      if(q.allFinite()&&distance>=.5&&distance<=config_.range)
        observations[key(q,config_.voxel)].insert(key(q,config_.surface_voxel));
    }
    for(const auto &entry:observations) {
      auto *cell=get(entry.first);if(!cell) continue;
      auto &c=*cell;c.last_seen=stamp;
      if(changed.count(entry.first)) { c.last_occupied=stamp;continue; }
      if(c.state==State::Dynamic && stamp-c.last_dynamic<config_.dynamic_hold) {
        c.last_occupied=stamp;continue;
      }
      if(c.state==State::Dynamic) c.state=State::Recovering;
      const bool gap=c.last_occupied<0||stamp-c.last_occupied>config_.observation_gap;
      c.last_occupied=stamp;
      // Compare with a fixed reference from the start of the stability window.
      // Comparing only consecutive frames would let slow moving surfaces pass.
      if(gap||c.reference.empty()||overlap(entry.second,c.reference)<config_.overlap) {
        c.reference=entry.second;c.stable_since=stamp;c.hits=1;
        if(c.state==State::Static && c.last_dynamic>=0) c.state=State::Recovering;
      } else c.hits=std::min(config_.static_observations,c.hits+1);
      if(c.hits>=config_.static_observations && stamp-c.stable_since>=config_.static_time)
        c.state=State::Static;
    }
    stats_={};stats_.at_capacity=cells_.size()>=config_.capacity;
    for(const auto &entry:cells_) switch(entry.second.state) {
      case State::Unknown:++stats_.unknown;break;
      case State::Static:++stats_.stable;break;
      case State::Dynamic:++stats_.dynamic;break;
      case State::Recovering:++stats_.recovering;break;
    }
  }
 private:
  struct Key { int x,y,z;bool operator==(const Key &b) const {return x==b.x&&y==b.y&&z==b.z;} };
  struct Hash { size_t operator()(const Key &k) const {
    size_t h=std::hash<int>()(k.x);h^=std::hash<int>()(k.y)+0x9e3779b9+(h<<6)+(h>>2);
    return h^(std::hash<int>()(k.z)+0x9e3779b9+(h<<6)+(h>>2));
  } };
  using Surface=std::unordered_set<Key,Hash>;
  struct Cell {
    State state=State::Unknown;
    double last_seen=-1,last_occupied=-1,last_dynamic=-1,stable_since=-1;
    int hits=0;Surface reference;
  };
  static Key key(const Eigen::Vector3d &p,double voxel) {
    return {int(std::floor(p.x()/voxel)),int(std::floor(p.y()/voxel)),int(std::floor(p.z()/voxel))};
  }
  Cell *get(const Key &k) {
    auto it=cells_.find(k);
    if(it==cells_.end()) {
      if(cells_.size()>=config_.capacity) return nullptr;
      it=cells_.emplace(k,Cell{}).first;
    }
    return &it->second;
  }
  static double overlap(const Surface &current,const Surface &reference) {
    if(current.empty()||reference.empty()) return 0.;
    size_t matched=0;
    for(const auto &k:current) {
      bool found=false;
      for(int x=-1;x<=1&&!found;++x) for(int y=-1;y<=1&&!found;++y)
        for(int z=-1;z<=1&&!found;++z) found=reference.count({k.x+x,k.y+y,k.z+z});
      if(found) ++matched;
    }
    return double(matched)/current.size();
  }
  RegionConfig config_;Stats stats_;
  std::unordered_map<Key,Cell,Hash> cells_;
  double last_stamp_=-1;Eigen::Isometry3d last_pose_=Eigen::Isometry3d::Identity();
};
}  // namespace dynamic_lio
