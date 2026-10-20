#include "preprocess.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <limits>
#include <stdexcept>

int main() {
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*msg);
  modifier.setPointCloud2Fields(5,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "time", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(4);
  sensor_msgs::PointCloud2Iterator<float> x(*msg,"x"), y(*msg,"y"),
      z(*msg,"z"), intensity(*msg,"intensity"), time(*msg,"time");
  const float distances[] = {2, 3, 0.01f, std::numeric_limits<float>::quiet_NaN()};
  const float times[] = {0.09f, 0.02f, 0.04f, 0.05f};
  for (int i=0;i<4;++i,++x,++y,++z,++intensity,++time) {
    *x=distances[i]; *y=0; *z=0; *intensity=10+i; *time=times[i];
  }
  Preprocess pre;
  pre.set(false, SIMULATED, 0.05, 1);
  pre.time_unit=SEC;
  PointCloudXYZI::Ptr out(new PointCloudXYZI);
  pre.process(msg,out);
  if (out->size()!=2 || std::abs(out->points[0].curvature-20)>0.001 ||
      std::abs(out->points[1].curvature-90)>0.001 || out->points[0].x!=3 ||
      out->points[0].intensity!=11)
    throw std::runtime_error("Go2 time conversion, ordering or invalid-point filtering failed");
}
