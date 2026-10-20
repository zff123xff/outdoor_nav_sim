// Classic backend for the existing PX4 sensor model. Navigation never subscribes to truth.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geo/geo.h>
#include <geo_lookup/geo_mag_declination.h>
#include <random>

namespace px4_gps_imu {
class ClassicSensorSystem : public gazebo::ModelPlugin {
 public:
  ~ClassicSensorSystem() override { connection_.reset(); }
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    const auto ns=sdf->Get<std::string>("robot_namespace","robot").first;
    lat_=sdf->Get<double>("reference_latitude",40.9445353).first;
    lon_=sdf->Get<double>("reference_longitude",113.172819).first;
    alt_=sdf->Get<double>("reference_altitude",1298.7345711102703).first;
    pos_sigma_=sdf->Get<double>("gps_position_stddev",0.15).first;
    vel_sigma_=sdf->Get<double>("gps_velocity_stddev",0.03).first;
    const double gps_rate=sdf->Get<double>("gps_update_rate",10.0).first;
    if(!std::isfinite(gps_rate) || gps_rate<=0 || gps_rate>200)
      throw std::invalid_argument("gps_update_rate must be in (0, 200]");
    gps_period_ns_=static_cast<int64_t>(1e9/gps_rate);
    mag_sigma_=sdf->Get<double>("magnetic_stddev",1e-7).first;
    if(!std::isfinite(lat_) || !std::isfinite(lon_) || !std::isfinite(alt_) ||
       std::abs(lat_)>90 || std::abs(lon_)>180 || pos_sigma_<0 || vel_sigma_<0 || mag_sigma_<0)
      throw std::invalid_argument("invalid simulated sensor parameters");
    base_=model->GetLink("base_link");
    if(!base_) throw std::runtime_error("Classic Go2-W base_link missing");
    map_projection_init(&projection_,lat_,lon_);
    const double decl=get_mag_declination_radians(lat_,lon_);
    const double incl=get_mag_inclination_radians(lat_,lon_);
    const double field=get_mag_strength_tesla(lat_,lon_);
    magnetic_world_={field*std::cos(incl)*std::sin(decl),
                     field*std::cos(incl)*std::cos(decl),-field*std::sin(incl)};
    node_=gazebo_ros::Node::Get(sdf);
    fix_=node_->create_publisher<sensor_msgs::msg::NavSatFix>("sensors/gps/fix",10);
    vel_=node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("sensors/gps/velocity",10);
    mag_=node_->create_publisher<sensor_msgs::msg::MagneticField>("sensors/mag",10);
    baro_=node_->create_publisher<sensor_msgs::msg::FluidPressure>("sensors/baro",10);
    truth_=node_->create_publisher<nav_msgs::msg::Odometry>("simulation/truth/odom",10);
    frame_prefix_=ns+"/";
    connection_=gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo &info){OnUpdate(info);});
  }
  void OnUpdate(const gazebo::common::UpdateInfo &info) {
    if(!node_) return;
    const int64_t time=static_cast<int64_t>(info.simTime.sec)*1000000000LL+info.simTime.nsec;
    if(time<last_mag_ || time<last_gps_) {last_mag_=last_gps_=next_gps_ns_=0;}
    const auto pose=base_->WorldPose();
    const builtin_interfaces::msg::Time stamp=rclcpp::Time(time);
    if(time-last_mag_>=10000000) {
      auto field=pose.Rot().RotateVectorReverse(magnetic_world_);
      sensor_msgs::msg::MagneticField m;
      m.header.stamp=stamp;m.header.frame_id=frame_prefix_+"imu_link";
      m.magnetic_field.x=field.X()+noise()*mag_sigma_;
      m.magnetic_field.y=field.Y()+noise()*mag_sigma_;
      m.magnetic_field.z=field.Z()+noise()*mag_sigma_;
      for(int i:{0,4,8}) m.magnetic_field_covariance[i]=mag_sigma_*mag_sigma_;
      mag_->publish(m);last_mag_=time;
      sensor_msgs::msg::FluidPressure pressure;
      pressure.header=m.header;
      pressure.fluid_pressure=101325.*std::exp(-(alt_+pose.Pos().Z())/8434.5)+noise()*0.5;
      pressure.variance=0.25;baro_->publish(pressure);
    }
    if(time<next_gps_ns_) return;
    const auto velocity=base_->WorldLinearVel();
    last_gps_=time;
    next_gps_ns_=(time/gps_period_ns_+1)*gps_period_ns_;
    const auto &p=pose.Pos();
    sensor_msgs::msg::NavSatFix gps;
    gps.header.stamp=stamp;gps.header.frame_id=frame_prefix_+"base_link";
    // World axes are ENU; inverse of PX4's own spherical map projection.
    map_projection_reproject(&projection_,p.Y()+noise()*pos_sigma_,p.X()+noise()*pos_sigma_,
      &gps.latitude,&gps.longitude);
    gps.altitude=alt_+p.Z()+noise()*pos_sigma_;
    gps.status.status=sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    gps.status.service=sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    for(int i:{0,4,8}) gps.position_covariance[i]=pos_sigma_*pos_sigma_;
    gps.position_covariance_type=sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    geometry_msgs::msg::TwistWithCovarianceStamped vel;
    vel.header.stamp=stamp;vel.header.frame_id=frame_prefix_+"map";
    vel.twist.twist.linear.x=velocity.X()+noise()*vel_sigma_;
    vel.twist.twist.linear.y=velocity.Y()+noise()*vel_sigma_;
    vel.twist.twist.linear.z=velocity.Z()+noise()*vel_sigma_;
    for(int i:{0,7,14}) vel.twist.covariance[i]=vel_sigma_*vel_sigma_;
    fix_->publish(gps);vel_->publish(vel);
    // Evaluation only. The estimator has no truth subscription.
    nav_msgs::msg::Odometry truth;
    truth.header.stamp=stamp;truth.header.frame_id=frame_prefix_+"map";
    truth.child_frame_id=frame_prefix_+"base_link";
    truth.pose.pose.position.x=p.X();truth.pose.pose.position.y=p.Y();truth.pose.pose.position.z=p.Z();
    const auto &q=pose.Rot();
    truth.pose.pose.orientation.w=q.W();truth.pose.pose.orientation.x=q.X();
    truth.pose.pose.orientation.y=q.Y();truth.pose.pose.orientation.z=q.Z();
    const auto body_vel=q.RotateVectorReverse(velocity);
    truth.twist.twist.linear.x=body_vel.X();truth.twist.twist.linear.y=body_vel.Y();truth.twist.twist.linear.z=body_vel.Z();
    const auto angular=q.RotateVectorReverse(base_->WorldAngularVel());
    truth.twist.twist.angular.x=angular.X();truth.twist.twist.angular.y=angular.Y();truth.twist.twist.angular.z=angular.Z();
    truth_->publish(truth);
  }
 private:
  double noise(){return gaussian_(rng_);}
  double lat_{},lon_{},alt_{},pos_sigma_{},vel_sigma_{},mag_sigma_{};
  int64_t last_mag_{0},last_gps_{0},gps_period_ns_{100000000},next_gps_ns_{0};
  map_projection_reference_s projection_{};
  gazebo::physics::LinkPtr base_;
  gazebo::event::ConnectionPtr connection_;
  ignition::math::Vector3d magnetic_world_;
  std::string frame_prefix_;
  std::mt19937 rng_{42};std::normal_distribution<double> gaussian_{0.,1.};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr baro_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_;
};
}
GZ_REGISTER_MODEL_PLUGIN(px4_gps_imu::ClassicSensorSystem)
