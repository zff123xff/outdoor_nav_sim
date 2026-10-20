#include "dynamic_filter.hpp"
#include "observed_map_update.hpp"
#include "visibility_filter.hpp"
#include <iostream>
#include <pcl/point_types.h>

#define REQUIRE(condition) do { if (!(condition)) { \
  std::cerr << "Failed: " #condition << " at " << __LINE__ << '\n'; return 1; } } while (0)

using Point = pcl::PointXYZINormal;
using Cloud = pcl::PointCloud<Point>;

Cloud scene(double stamp, const Eigen::Isometry3d &pose, bool moving) {
  Cloud cloud;
  auto point = [&](double x, double y, double z, float label) {
    Point p{};
    const Eigen::Vector3d q = pose.inverse()*Eigen::Vector3d(x, y, z);
    p.x = q.x(); p.y = q.y(); p.z = q.z(); p.intensity = label;
    p.curvature = cloud.size()*0.001f; cloud.push_back(p);
  };
  for (int x = -35; x <= 35; ++x) for (int y = -35; y <= 35; ++y)
    point(x*0.15, y*0.15, -0.4, 1);
  for (int object = 0; object < 2; ++object) {
    const double cx = object == 0 ? -2.0 : 1.0+(moving ? std::min(stamp, 3.0)*0.5 : 0.0);
    const double cy = object == 0 ? 0.0 : 2.0;
    for (int x = -3; x <= 3; ++x) for (int y = -3; y <= 3; ++y)
      for (int z = 0; z <= 12; ++z) point(cx+x*.08, cy+y*.08, -.22+z*.08, object == 0 ? 10 : 20);
  }
  return cloud;
}

int main() {
  dynamic_lio::Filter filter;
  Cloud initial = scene(0, Eigen::Isometry3d::Identity(), true);
  dynamic_lio::Result detected;
  for (int i = 0; i <= 30; ++i) {
    const double stamp = i*.1;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = Eigen::AngleAxisd(stamp*.15, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation() = Eigen::Vector3d(stamp*.4, -.2*stamp, .04*std::sin(stamp));
    Cloud cloud = scene(stamp, pose, true);
    const Cloud before = cloud;
    detected = filter.process(cloud, pose, stamp);
    REQUIRE(detected.state == "tracking");
    REQUIRE(std::count_if(cloud.begin(), cloud.end(), [](const Point &p) { return p.intensity == 10; }) == 637);
    for (const auto &p : cloud) {
      const auto index = static_cast<size_t>(std::round(p.curvature/0.001f));
      REQUIRE(index < before.size());
      REQUIRE(p.curvature == before[index].curvature && p.intensity == before[index].intensity);
    }
    if (i >= 15) {
      REQUIRE(detected.moving.size() == 1);
      REQUIRE(std::none_of(cloud.begin(), cloud.end(), [](const Point &p) { return p.intensity == 20; }));
    }
  }
  dynamic_lio::Filter::erase(initial, detected.erase_boxes);
  REQUIRE(std::none_of(initial.begin(), initial.end(), [](const Point &p) { return p.intensity == 20; }));
  REQUIRE(std::count_if(initial.begin(), initial.end(), [](const Point &p) { return p.intensity == 10; }) == 637);
  // A stopped object stays masked briefly, then becomes part of the static scene.
  for (int i = 31; i <= 65; ++i) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = Eigen::AngleAxisd(.45, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation() = Eigen::Vector3d(1.2, -.6, .04*std::sin(3.0));
    Cloud cloud = scene(i*.1, pose, true);
    auto result = filter.process(cloud, pose, i*.1);
    if (i == 35) REQUIRE(!result.moving.empty());
    if (i == 65) REQUIRE(result.moving.empty());
  }
  // Robot motion alone must not mark either stationary cluster as dynamic.
  dynamic_lio::Filter static_filter;
  for (int i = 0; i < 30; ++i) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = Eigen::AngleAxisd(i*.02, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation().x() = i*.03;
    Cloud cloud = scene(i*.1, pose, false);
    auto result = static_filter.process(cloud, pose, i*.1);
    REQUIRE(result.dynamic_indices.empty());
  }
  Cloud restarted = scene(0, Eigen::Isometry3d::Identity(), false);
  auto reset = static_filter.process(restarted, Eigen::Isometry3d::Identity(), 0);
  REQUIRE(reset.state == "time_reset" && reset.moving.empty());
  Eigen::Isometry3d jump = Eigen::Isometry3d::Identity(); jump.translation().x() = 5;
  Cloud a = scene(.1, Eigen::Isometry3d::Identity(), false);
  static_filter.process(a, Eigen::Isometry3d::Identity(), .1);
  Cloud b = scene(.2, jump, false);
  auto jump_result = static_filter.process(b, jump, .2);
  REQUIRE(jump_result.state == "pose_reset" && jump_result.moving.empty());
  Cloud no_ground;
  auto empty = static_filter.process(no_ground, jump, .3);
  REQUIRE(empty.state == "no_ground_plane");
  bool invalid = false;
  try { dynamic_lio::Config bad; bad.min_speed = 0; dynamic_lio::Filter rejected(bad); }
  catch (const std::invalid_argument &) { invalid = true; }
  REQUIRE(invalid);
  // A newly observed wall behind a removed object clears the old occupied voxel.
  dynamic_lio::MapUpdater updater;
  Cloud old;
  Point p{}; p.x = 2.05; p.y = .05; p.z = .05; old.push_back(p);
  updater.observeStatic(old, Eigen::Isometry3d::Identity());
  Cloud wall; p.x = 4.05; wall.push_back(p);
  REQUIRE(updater.clearObservedFree(wall, Eigen::Isometry3d::Identity(), 0).empty());
  auto freed = updater.clearObservedFree(wall, Eigen::Isometry3d::Identity(), .1);
  REQUIRE(!freed.empty());
  updater.eraseCleared(old, freed); REQUIRE(old.empty());
  // A foreground return must preserve the hidden old obstacle behind it.
  updater.observeStatic(wall, Eigen::Isometry3d::Identity());
  Cloud occluder; p.x = 2.05; occluder.push_back(p);
  updater.clearObservedFree(occluder, Eigen::Isometry3d::Identity(), .2);
  REQUIRE(updater.clearObservedFree(occluder, Eigen::Isometry3d::Identity(), .3).empty());
  REQUIRE(updater.occupiedVoxels() == 1);
  // Seeing the existing surface, or receiving no returns, does not erase it.
  REQUIRE(updater.clearObservedFree(wall, Eigen::Isometry3d::Identity(), .4).empty());
  REQUIRE(updater.clearObservedFree(Cloud(), Eigen::Isometry3d::Identity(), .5).empty());
  // History is measured before the current scan, with distinct temporal evidence.
  dynamic_lio::VisibilityFilter visibility;
  auto ray_cloud=[](float x) { Cloud c; Point p{};p.x=x;p.y=.05;p.z=.05;p.curvature=7.;p.intensity=42.;c.push_back(p);return c; };
  const auto identity=Eigen::Isometry3d::Identity();
  for(int i=0;i<3;++i) { Cloud far=ray_cloud(4.05);visibility.process(far,far,identity,i*.2);REQUIRE(far.size()==1); }
  Cloud near=ray_cloud(2.05);const auto raw_near=near;
  REQUIRE(visibility.process(near,raw_near,identity,.5).empty());
  REQUIRE(near.size()==1 && visibility.weight(Eigen::Vector3d(2.05,.05,.05))<1.);
  REQUIRE(!visibility.backendStatic(Eigen::Vector3d(2.05,.05,.05)));
  REQUIRE(near[0].curvature==7. && near[0].intensity==42.);
  near=raw_near;visibility.process(near,raw_near,identity,.6);REQUIRE(near.size()==1);
  near=raw_near;REQUIRE(visibility.process(near,raw_near,identity,.7).empty());REQUIRE(near.size()==1);
  // Evidence follows a moving surface across neighboring voxels without
  // allowing multiple rays in the same scan to increase the score repeatedly.
  dynamic_lio::VisibilityFilter crossing;
  for(int i=0;i<3;++i) { const Cloud raw=ray_cloud(4.05);Cloud c=raw;crossing.process(c,raw,identity,i*.2); }
  for(int i=0;i<3;++i) {
    Cloud raw=ray_cloud(2.05f+i*.2f),c=raw;
    crossing.process(c,raw,identity,.5+i*.1);
    REQUIRE((i<2 && c.size()==1) || (i==2 && c.empty()));
  }
  // Occluded space behind foreground returns is unknown, not free.
  dynamic_lio::VisibilityFilter occlusion;
  for(int i=0;i<6;++i) { Cloud c=raw_near;occlusion.process(c,raw_near,identity,i*.1);REQUIRE(c.size()==1); }
  Cloud behind=ray_cloud(4.05);const Cloud raw_behind=behind;
  occlusion.process(behind,raw_behind,identity,.6);REQUIRE(behind.size()==1);
  // Duplicate rays in one scan cannot manufacture temporal confidence.
  dynamic_lio::VisibilityFilter one_scan;
  Cloud duplicate;for(int i=0;i<100;++i) duplicate+=raw_behind;
  one_scan.process(duplicate,duplicate,identity,0.);
  near=raw_near;one_scan.process(near,raw_near,identity,.1);REQUIRE(near.size()==1);
  // A surface newly occupying previously free space is not sustained motion.
  dynamic_lio::VisibilityFilter stationary_return;
  for(int i=0;i<3;++i) { const Cloud raw=ray_cloud(4.05);Cloud c=raw;stationary_return.process(c,raw,identity,i*.2); }
  for(int i=0;i<10;++i) {
    Cloud c=raw_near;stationary_return.process(c,raw_near,identity,.5+i*.1);
    REQUIRE(c.size()==1);
  }
  REQUIRE(stationary_return.score(Eigen::Vector3d(2.05,.05,.05))==0.);
  // Moving sensor with a sliding view of a stationary wall: use world-frame
  // rays only, without cluster centroid decisions or simulation truth input.
  dynamic_lio::VisibilityFilter wall_visibility;
  for(int i=0;i<20;++i) {
    Eigen::Isometry3d pose=identity;
    pose.translation().x()=i*.03;
    pose.linear()=Eigen::AngleAxisd(i*.01,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Cloud wall;
    for(int y=-10+i/2;y<=10+i/2;++y) for(int z=-2;z<=8;++z) {
      Point p{};Eigen::Vector3d q=pose.inverse()*Eigen::Vector3d(4.,y*.1,z*.1);
      p.x=q.x();p.y=q.y();p.z=q.z();wall.push_back(p);
    }
    const Cloud raw=wall;
    REQUIRE(wall_visibility.process(wall,raw,pose,i*.1).empty());
    REQUIRE(wall.size()==raw.size());
  }
  // A clock reset forgets old dynamic evidence; empty scans do not assert free space.
  near=raw_near;visibility.process(near,raw_near,identity,0.);REQUIRE(near.size()==1);
  dynamic_lio::VisibilityFilter unknown;
  Cloud nothing;unknown.process(nothing,nothing,identity,0.);
  near=raw_near;unknown.process(near,raw_near,identity,.5);REQUIRE(near.size()==1);
  std::cout << "Visibility confidence, occlusion, temporal evidence and field preservation passed\n";
  std::cout << "Dynamic tracking, ego compensation, field preservation, map cleanup and resets passed\n";
}
