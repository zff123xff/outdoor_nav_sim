#include "region_map.hpp"
#include <pcl/point_types.h>
#include <iostream>
#define REQUIRE(c) do {if(!(c)) {std::cerr<<"Failed: " #c <<" at "<<__LINE__<<'\n';return 1;}} while(0)
using Point=pcl::PointXYZI;
using Cloud=pcl::PointCloud<Point>;
Cloud surface(double x=2.05) {Cloud c;Point p;p.x=x;p.y=.05;p.z=.05;c.push_back(p);return c;}
int main() {
  dynamic_lio::RegionConfig config;
  config.dynamic_hold=.2;config.static_time=.5;config.static_observations=4;
  config.observation_gap=.25;config.history=1.;config.surface_voxel=.025;
  const auto pose=Eigen::Isometry3d::Identity();const Eigen::Vector3d p(2.05,.05,.05);
  using State=dynamic_lio::RegionMap::State;
  dynamic_lio::RegionMap regions(config);
  REQUIRE(regions.weight(p)==1. && regions.eligible(p));
  // Bootstrap/exploration remains possible even before static confirmation.
  regions.process(surface(),pose,{}, {},0.);
  REQUIRE(regions.state(p)==State::Unknown && regions.eligible(p));
  for(int i=1;i<=6;++i) regions.process(surface(),pose,{}, {},i*.1);
  REQUIRE(regions.state(p)==State::Static);
  const dynamic_lio::Box unrelated_free{Eigen::Vector3d(2.2,0.,0.),Eigen::Vector3d(2.3,.1,.1)};
  regions.process(Cloud{},pose,{}, {unrelated_free},.65);
  REQUIRE(regions.state(p)==State::Static); // A free ray elsewhere in the cell is not a contradiction.
  regions.process(surface(),pose,{p},{},.7);
  REQUIRE(regions.state(p)==State::Dynamic && !regions.eligible(p));
  REQUIRE(regions.weight(p)==config.dynamic_weight);
  // Occlusion must not silently rehabilitate an old dynamic region, even
  // after the ordinary-region expiry period, while the sensor stream continues.
  for(int i=8;i<=30;++i) regions.process(Cloud{},pose,{}, {},i*.1);
  REQUIRE(regions.state(p)==State::Dynamic && !regions.eligible(p));
  regions.process(surface(),pose,{}, {},3.1);
  REQUIRE(regions.state(p)==State::Recovering && !regions.eligible(p));
  const double low=regions.weight(p);
  for(int i=32;i<=34;++i) regions.process(surface(),pose,{}, {},i*.1);
  REQUIRE(regions.weight(p)>low && regions.weight(p)<1. && !regions.eligible(p));
  for(int i=35;i<=37;++i) regions.process(surface(),pose,{}, {},i*.1);
  REQUIRE(regions.state(p)==State::Static && regions.weight(p)==1. && regions.eligible(p));
  // Confirmed free rays through its fixed surface anchor invalidate stability.
  const dynamic_lio::Box free_box{Eigen::Vector3d(2.,0.,0.),Eigen::Vector3d(2.1,.1,.1)};
  regions.process(Cloud{},pose,{}, {free_box},3.8);
  REQUIRE(regions.state(p)==State::Dynamic);
  // A new disturbance restarts recovery; short quiet periods are insufficient.
  for(int i=39;i<=42;++i) regions.process(surface(),pose,{}, {},i*.1);
  regions.process(surface(),pose,{p},{},4.3);
  REQUIRE(regions.state(p)==State::Dynamic);
  // Slowly sliding geometry cannot pass by matching only adjacent frames.
  dynamic_lio::RegionMap moving(config);
  moving.process(surface(),pose,{p},{},0.);
  for(int i=1;i<=15;++i) moving.process(surface(2.05+i*.02),pose,{}, {},i*.1);
  REQUIRE(moving.state(p)!=State::Static && !moving.eligible(p));
  // A visibility gap restarts the static-duration timer.
  dynamic_lio::RegionMap gap(config);
  gap.process(surface(),pose,{p},{},0.);
  gap.process(surface(),pose,{}, {},.3);
  gap.process(surface(),pose,{}, {},.4);
  gap.process(Cloud{},pose,{}, {},.5);
  gap.process(Cloud{},pose,{}, {},.6);
  gap.process(surface(),pose,{}, {},.7);
  gap.process(surface(),pose,{}, {},.8);
  REQUIRE(!gap.eligible(p));
  // Duplicate scans do not advance state or erase history.
  gap.process(surface(),pose,{}, {},.8);REQUIRE(!gap.eligible(p));
  // Motion of the sensor alone leaves a fixed world surface stable.
  dynamic_lio::RegionMap stationary(config);
  for(int i=0;i<15;++i) {
    auto sensor=pose;sensor.translation().x()=i*.01;
    const auto q=sensor.inverse()*p;Cloud c;Point a;a.x=q.x();a.y=q.y();a.z=q.z();c.push_back(a);
    stationary.process(c,sensor,{}, {},i*.1);
  }
  REQUIRE(stationary.state(p)==State::Static);
  // Ordinary view changes are not a new dynamic classification.
  stationary.process(surface(2.4),pose,{}, {},1.5);
  REQUIRE(stationary.state(p)==State::Static);
  // Time reset clears metadata instead of mixing mapping epochs.
  stationary.process(surface(),pose,{}, {},0.);
  REQUIRE(stationary.state(p)==State::Unknown);
  bool invalid=false;
  try {auto bad=config;bad.surface_voxel=1.;dynamic_lio::RegionMap rejected(bad);}
  catch(const std::invalid_argument&){invalid=true;}REQUIRE(invalid);
  config.capacity=1;dynamic_lio::RegionMap limited(config);
  limited.process(surface(),pose,{p},{},0.);
  limited.process(surface(3.05),pose,{}, {},.1);
  REQUIRE(limited.stats().at_capacity && limited.stats().dynamic==1);
  std::cout<<"Region history, recovery, map admission, occlusion and fixed surface checks passed\n";
}
