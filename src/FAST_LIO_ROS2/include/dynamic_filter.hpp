#pragma once

// Geometry-only moving-object filtering after IMU deskew, before LIO matching.
#include <Eigen/Geometry>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace dynamic_lio {
struct Config {
  double voxel = 0.12, cluster_tolerance = 0.30, range = 25.0;
  int min_cluster_points = 12, max_cluster_points = 4000;
  double max_object_width = 3.0, max_object_height = 2.8;
  double ground_distance = 0.08, association_distance = 0.8;
  double min_speed = 0.20, min_displacement = 0.25, observation_time = 0.8;
  int confirmations = 3;
  double padding = 0.10, track_timeout = 1.0, history_seconds = 5.0, hold_seconds = 2.0;
  double max_pose_step = 0.8, max_pose_angle = 0.45;
};
struct Box {
  Eigen::Vector3d low, high;
  bool contains(const Eigen::Vector3d &point) const {
    return (point.array() >= low.array()).all() && (point.array() <= high.array()).all();
  }
};
struct Detection {
  int id;
  Box box;
  Eigen::Vector3d velocity;
};
struct Result {
  std::vector<Box> erase_boxes;
  std::vector<Detection> moving;
  std::vector<size_t> dynamic_indices;
  size_t clusters = 0, tracks = 0;
  std::string state = "warming_up";
};

class Filter {
 public:
  explicit Filter(Config config = Config()) : config_(config) {
    for (double value : {config.voxel, config.cluster_tolerance, config.range,
         config.max_object_width, config.max_object_height, config.ground_distance,
         config.association_distance, config.min_speed, config.min_displacement,
         config.observation_time, config.padding, config.track_timeout,
         config.history_seconds, config.hold_seconds, config.max_pose_step, config.max_pose_angle}) {
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("Invalid dynamic filter configuration");
    }
    if (config.min_cluster_points < 3 || config.max_cluster_points < config.min_cluster_points ||
        config.confirmations < 1 || config.history_seconds < config.observation_time)
      throw std::invalid_argument("Invalid dynamic tracking configuration");
  }

  template<class Point>
  Result process(pcl::PointCloud<Point> &cloud, const Eigen::Isometry3d &world_from_lidar, double stamp) {
    Result result;
    if (!std::isfinite(stamp) || !world_from_lidar.matrix().allFinite())
      throw std::invalid_argument("Invalid dynamic filter pose or stamp");
    if (last_stamp_ >= 0 && stamp <= last_stamp_) {
      reset(); result.state = "time_reset";
      return result;  // Do not classify motion across simulation epochs.
    }
    if (last_stamp_ >= 0 && (stamp-last_stamp_ > config_.track_timeout ||
        (world_from_lidar.translation()-last_pose_.translation()).norm() > config_.max_pose_step ||
        Eigen::AngleAxisd(last_pose_.linear().transpose()*world_from_lidar.linear()).angle() > config_.max_pose_angle)) {
      reset(); result.state = "pose_reset";
    }
    last_pose_ = world_from_lidar; last_stamp_ = stamp;
    std::vector<Eigen::Vector3d> world;
    world.reserve(cloud.size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor_candidates(new pcl::PointCloud<pcl::PointXYZ>);
    const auto origin = world_from_lidar.translation();
    for (const auto &point : cloud) {
      const Eigen::Vector3d p = world_from_lidar * Eigen::Vector3d(point.x, point.y, point.z);
      world.push_back(p);
      if (!p.allFinite() || (p-origin).norm() > config_.range) continue;
      const pcl::PointXYZ q(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
      points->push_back(q);
      if ((p.head<2>()-origin.head<2>()).norm() < 8.0 && p.z() < origin.z()-0.12 && p.z() > origin.z()-2.0)
        floor_candidates->push_back(q);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(config_.voxel, config_.voxel, config_.voxel);
    voxel.setInputCloud(points); voxel.filter(*sampled);
    pcl::ModelCoefficients ground;
    pcl::PointIndices inliers;
    if (floor_candidates->size() >= 30) {
      pcl::SACSegmentation<pcl::PointXYZ> segmentation;
      segmentation.setOptimizeCoefficients(true);
      segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
      segmentation.setMethodType(pcl::SAC_RANSAC);
      segmentation.setAxis(Eigen::Vector3f::UnitZ()); segmentation.setEpsAngle(0.26);
      segmentation.setDistanceThreshold(config_.ground_distance); segmentation.setMaxIterations(100);
      segmentation.setInputCloud(floor_candidates); segmentation.segment(inliers, ground);
    }
    if (inliers.indices.size() < 30 || ground.values.size() != 4) {
      tracks_.clear(); result.state = "no_ground_plane";
      return result;  // Transparent fail-open bootstrap; expose it in status.
    }
    Eigen::Vector3d normal(ground.values[0], ground.values[1], ground.values[2]);
    double offset = ground.values[3];
    if (normal.z() < 0) { normal = -normal; offset = -offset; }
    const double norm = normal.norm(); normal /= norm; offset /= norm;
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonground(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &point : *sampled) {
      const double height = normal.dot(Eigen::Vector3d(point.x, point.y, point.z)) + offset;
      if (height > config_.ground_distance && height < config_.max_object_height)
        nonground->push_back(point);
    }
    std::vector<Observation> observations;
    if (!nonground->empty()) {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(nonground);
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
      extraction.setClusterTolerance(config_.cluster_tolerance);
      extraction.setMinClusterSize(config_.min_cluster_points);
      extraction.setMaxClusterSize(config_.max_cluster_points);
      extraction.setSearchMethod(tree); extraction.setInputCloud(nonground);
      std::vector<pcl::PointIndices> clusters; extraction.extract(clusters);
      for (const auto &cluster : clusters) {
        Observation observation;
        observation.center.setZero();
        observation.box.low.setConstant(std::numeric_limits<double>::infinity());
        observation.box.high.setConstant(-std::numeric_limits<double>::infinity());
        for (int index : cluster.indices) {
          const auto &point = (*nonground)[index]; Eigen::Vector3d p(point.x, point.y, point.z);
          observation.center += p;
          observation.box.low = observation.box.low.cwiseMin(p);
          observation.box.high = observation.box.high.cwiseMax(p);
        }
        observation.center /= cluster.indices.size();
        const auto extent = observation.box.high-observation.box.low;
        if (extent.x() > config_.max_object_width || extent.y() > config_.max_object_width || extent.z() < 0.20) continue;
        observations.push_back(observation);
      }
    }
    result.clusters = observations.size();
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track &track) {
      return stamp-track.last_seen > config_.track_timeout;
    }), tracks_.end());
    // Global nearest-pair matching prevents observation order from reusing an ID.
    struct Pair { double distance; size_t track, observation; };
    std::vector<Pair> pairs;
    for (size_t i = 0; i < tracks_.size(); ++i) {
      const auto predicted = tracks_[i].center + tracks_[i].velocity * (stamp-tracks_[i].last_seen);
      for (size_t j = 0; j < observations.size(); ++j) {
        const double distance = (predicted-observations[j].center).head<2>().norm();
        if (distance < config_.association_distance) pairs.push_back({distance, i, j});
      }
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair &a, const Pair &b) { return a.distance < b.distance; });
    std::vector<int> matches(observations.size(), -1);
    std::vector<bool> used(tracks_.size(), false);
    for (const auto &pair : pairs) if (!used[pair.track] && matches[pair.observation] < 0) {
      used[pair.track] = true; matches[pair.observation] = pair.track;
    }
    for (size_t i = 0; i < observations.size(); ++i) {
      if (matches[i] < 0) {
        if (tracks_.size() >= 128) continue;
        Track track; track.id = next_id_++; tracks_.push_back(track);
        matches[i] = tracks_.size()-1;
      }
      Track &track = tracks_[matches[i]];
      const auto &observation = observations[i];
      track.center = observation.center; track.box = observation.box; track.last_seen = stamp;
      track.history.push_back({stamp, observation.center, observation.box});
      while (!track.history.empty() && stamp-track.history.front().stamp > config_.history_seconds)
        track.history.pop_front();
      const Snapshot *reference = nullptr;
      for (const auto &sample : track.history) {
        if (stamp-sample.stamp >= config_.observation_time) reference = &sample;
      }
      if (reference) {
        const double elapsed = stamp-reference->stamp;
        const double travel = (observation.center-reference->center).head<2>().norm();
        track.velocity = (observation.center-reference->center)/elapsed;
        if (travel >= config_.min_displacement && track.velocity.head<2>().norm() >= config_.min_speed)
          ++track.evidence;
        else track.evidence = 0;
        if (track.evidence >= config_.confirmations) track.dynamic_until = stamp+config_.hold_seconds;
      }
    }
    std::vector<Box> current_boxes;
    for (const auto &track : tracks_) if (track.dynamic_until >= stamp) {
      Box box = padded(track.box);
      // Brief occlusion: include the predicted box, not just the last observation.
      const Eigen::Vector3d shift = track.velocity*(stamp-track.last_seen);
      box.low += shift; box.high += shift;
      current_boxes.push_back(box);
      result.moving.push_back({track.id, box, track.velocity});
      for (const auto &sample : track.history) result.erase_boxes.push_back(padded(sample.box));
      result.erase_boxes.push_back(box);
    }
    pcl::PointCloud<Point> retained;
    retained.reserve(cloud.size()); retained.header = cloud.header;
    retained.sensor_origin_ = cloud.sensor_origin_; retained.sensor_orientation_ = cloud.sensor_orientation_;
    for (size_t i = 0; i < cloud.size(); ++i) {
      const bool moving = std::any_of(current_boxes.begin(), current_boxes.end(), [&](const Box &box) { return box.contains(world[i]); });
      if (moving) result.dynamic_indices.push_back(i);
      else retained.push_back(cloud[i]);  // Retain all fields, especially curvature/point time.
    }
    cloud.swap(retained);
    result.tracks = tracks_.size();
    if (result.state != "pose_reset") result.state = "tracking";
    return result;
  }

  template<class Point>
  static void erase(pcl::PointCloud<Point> &map, const std::vector<Box> &boxes) {
    map.erase(std::remove_if(map.begin(), map.end(), [&](const Point &point) {
      const Eigen::Vector3d p(point.x, point.y, point.z);
      return std::any_of(boxes.begin(), boxes.end(), [&](const Box &box) { return box.contains(p); });
    }), map.end());
    map.width = map.size(); map.height = 1;
  }

 private:
  struct Observation { Eigen::Vector3d center; Box box; };
  struct Snapshot { double stamp; Eigen::Vector3d center; Box box; };
  struct Track {
    int id = 0, evidence = 0;
    double last_seen = 0, dynamic_until = -1;
    Eigen::Vector3d center = Eigen::Vector3d::Zero(), velocity = Eigen::Vector3d::Zero();
    Box box;
    std::deque<Snapshot> history;
  };
  Box padded(Box box) const {
    box.low.array() -= config_.padding; box.high.array() += config_.padding; return box;
  }
  void reset() { tracks_.clear(); last_stamp_ = -1; }
  Config config_;
  std::vector<Track> tracks_;
  int next_id_ = 0;
  double last_stamp_ = -1;
  Eigen::Isometry3d last_pose_ = Eigen::Isometry3d::Identity();
};
}  // namespace dynamic_lio
