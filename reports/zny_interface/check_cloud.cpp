#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <iostream>
#include <cstring>
int main(){
  sensor_msgs::msg::PointCloud2 cloud,out;
  cloud.header.frame_id="robot_1/livox_frame";cloud.width=1;cloud.height=1;
  cloud.fields.resize(5);
  const char *names[]={"x","y","z","intensity","timestamp"};
  for(int i=0;i<5;++i){cloud.fields[i].name=names[i];cloud.fields[i].offset=i*4;cloud.fields[i].datatype=i==4?8:7;cloud.fields[i].count=1;}
  cloud.point_step=24;cloud.row_step=24;cloud.data.resize(24);cloud.is_dense=true;
  const float xyz[]={1,2,3,0};std::memcpy(cloud.data.data(),xyz,sizeof(xyz));
  geometry_msgs::msg::TransformStamped tf;tf.header.frame_id="robot_1/map";tf.transform.rotation.w=1;tf.transform.translation.x=1;
  tf2::doTransform(cloud,out,tf);
  sensor_msgs::PointCloud2ConstIterator<float> x(out,"x");
  if(*x!=2)return 1;
  std::cout<<"PASS: xyz+intensity+timestamp, point_step=24 transformed correctly\n";
  cloud.fields.resize(1);cloud.point_step=4;cloud.row_step=4;cloud.data.resize(4);
  try{tf2::doTransform(cloud,out,tf);}
  catch(const std::exception &e){std::cout<<"Malformed x-only cloud: "<<e.what()<<"\n";return std::string(e.what()).find("y")!=std::string::npos?0:3;}
  return 2;
}
