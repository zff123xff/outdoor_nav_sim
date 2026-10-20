#include <px4_gps_imu/fusion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <deque>
#include <memory>
#include <sstream>

using sensor_msgs::msg::NavSatFix;
using geometry_msgs::msg::TwistWithCovarianceStamped;
namespace px4_gps_imu {
static uint64_t micros(const builtin_interfaces::msg::Time &t) {
  if (t.sec<0) return 0;
  return uint64_t(t.sec)*1000000+t.nanosec/1000;
}
class EkfNode : public rclcpp::Node {
 public:
  EkfNode():Node("px4_ekf") {
    lat_=declare_parameter("reference_latitude", 40.9445353);
    lon_=declare_parameter("reference_longitude", 113.172819);
    alt_=declare_parameter("reference_altitude", 1298.7345711102703);
    const auto offset=declare_parameter("imu_xyz", std::vector<double>{-0.02557,0.,0.04232});
    if(offset.size()!=3) throw std::invalid_argument("imu_xyz must contain three values");
    imu_offset_={float(offset[0]),float(offset[1]),float(offset[2])};
    const auto prefix=std::string(get_namespace()).substr(1);
    odom_frame_=prefix.empty()?"odom":prefix+"/odom";
    body_frame_=prefix.empty()?"base_link":prefix+"/base_link";
    reset();
    // Reliable outputs match both Nav2 and the MAVROS-compatible input bridge.
    odom_=create_publisher<nav_msgs::msg::Odometry>("odom",10);
    mavros_=create_publisher<nav_msgs::msg::Odometry>("mavros/odom",10);
    status_=create_publisher<std_msgs::msg::String>("ekf/status",rclcpp::QoS(1).transient_local());
    auto qos=rclcpp::SensorDataQoS().keep_last(2000);
    imu_=create_subscription<sensor_msgs::msg::Imu>("livox/imu",qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m){on_imu(*m);});
    fix_=create_subscription<NavSatFix>("sensors/gps/fix",rclcpp::SensorDataQoS(),
      [this](NavSatFix::ConstSharedPtr m){fixes_.push_back(m);match_gps();});
    velocity_=create_subscription<TwistWithCovarianceStamped>("sensors/gps/velocity",rclcpp::SensorDataQoS(),
      [this](TwistWithCovarianceStamped::ConstSharedPtr m){velocities_.push_back(m);match_gps();});
    mag_=create_subscription<sensor_msgs::msg::MagneticField>("sensors/mag",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::MagneticField::ConstSharedPtr m){
        const auto t=micros(m->header.stamp);
        if(!fusion_->started() || t<=last_mag_) return;
        const auto &v=m->magnetic_field;
        matrix::Vector3f flu{float(v.x*1e4),float(v.y*1e4),float(v.z*1e4)};
        if(!finite(flu)) return;
        fusion_->ekf.setMagData(magSample{t,flu_to_frd(flu)});
        last_mag_=t;
      });
    baro_=create_subscription<sensor_msgs::msg::FluidPressure>("sensors/baro",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::FluidPressure::ConstSharedPtr m){
        const auto t=micros(m->header.stamp);
        if(!fusion_->started() || t<=last_baro_ || !std::isfinite(m->fluid_pressure) || m->fluid_pressure<=0) return;
        fusion_->ekf.setBaroData(baroSample{t,float(-8434.5*std::log(m->fluid_pressure/101325.))});
        last_baro_=t;
      });
    RCLCPP_INFO(get_logger(),"Original PX4 ECL EKF: GPS + IMU + magnetic heading; ENU origin %.8f %.8f %.3f",lat_,lon_,alt_);
  }
 private:
  void reset() {
    fusion_=std::make_unique<Fusion>(lat_,lon_,alt_,imu_offset_);
    fixes_.clear(); velocities_.clear();
    last_imu_=last_gps_=last_mag_=last_baro_=last_status_=last_publish_=0;
  }
  void match_gps() {
    while(fixes_.size()>20) fixes_.pop_front();
    while(velocities_.size()>20) velocities_.pop_front();
    while(!fixes_.empty() && !velocities_.empty()) {
      auto f=fixes_.front(); auto v=velocities_.front();
      const auto ft=micros(f->header.stamp), vt=micros(v->header.stamp);
      if(ft<vt){fixes_.pop_front();continue;}
      if(vt<ft){velocities_.pop_front();continue;}
      fixes_.pop_front(); velocities_.pop_front();
      if(!fusion_->started() || ft<=last_gps_ || f->status.status<0 ||
         !std::isfinite(f->latitude) || !std::isfinite(f->longitude) ||
         !std::isfinite(f->altitude) || std::abs(f->latitude)>90 || std::abs(f->longitude)>180) continue;
      const auto &linear=v->twist.twist.linear;
      matrix::Vector3f enu{float(linear.x),float(linear.y),float(linear.z)};
      if(!finite(enu)) continue;
      gps_message gps{};
      gps.time_usec=ft;
      gps.lat=std::llround(f->latitude*1e7); gps.lon=std::llround(f->longitude*1e7);
      gps.alt=std::llround(f->altitude*1e3);
      gps.fix_type=3; gps.nsats=16; gps.pdop=1.;
      gps.yaw=NAN; // single-antenna GPS does NOT supply robot orientation
      gps.eph=std::sqrt(std::max({f->position_covariance[0],f->position_covariance[4],0.01}));
      gps.epv=std::sqrt(std::max(f->position_covariance[8],0.01));
      gps.sacc=std::sqrt(std::max({v->twist.covariance[0],v->twist.covariance[7],v->twist.covariance[14],0.0001}));
      gps.vel_m_s=std::hypot(enu(0),enu(1));
      gps.vel_ned={enu(1),enu(0),-enu(2)}; gps.vel_ned_valid=true;
      fusion_->ekf.setGpsData(gps); last_gps_=ft;
    }
  }
  void on_imu(const sensor_msgs::msg::Imu &m) {
    const auto t=micros(m.header.stamp);
    if(last_imu_ && t+1000000<last_imu_) {
      reset();
      RCLCPP_WARN(get_logger(),"Simulation clock reset: restarting EKF; waiting for GPS health and alignment");
    }
    if(t<=last_imu_) return;
    const auto &w=m.angular_velocity; const auto &a=m.linear_acceleration;
    matrix::Vector3f gyro{float(w.x),float(w.y),float(w.z)}, accel{float(a.x),float(a.y),float(a.z)};
    const bool updated=fusion_->imu(t,gyro,accel);
    last_imu_=t;
    if(t>=last_status_+1000000) {
      uint16_t checks=0; fusion_->ekf.get_gps_check_status(&checks);
      const auto &s=fusion_->ekf.control_status_flags();
      std::ostringstream out;
      out<<"ready="<<fusion_->ready()<<" tilt="<<s.tilt_align<<" yaw="<<s.yaw_align
         <<" gps="<<s.gps<<" gps_height="<<s.gps_hgt<<" gps_check_fail="<<checks;
      std_msgs::msg::String status; status.data=out.str();status_->publish(status);
      RCLCPP_INFO(get_logger(),"%s",status.data.c_str());last_status_=t;
    }
    if(!updated || !fusion_->ready() || t<last_publish_+20000) return;
    last_publish_=t;
    const auto rotation=attitude_enu_flu(fusion_->ekf.getQuaternion());
    const matrix::Quatf q(rotation);
    const auto pos=ned_to_enu(fusion_->ekf.getPosition());
    const auto world_vel=ned_to_enu(fusion_->ekf.getVelocity());
    const matrix::Vector3f body_vel(rotation.transpose()*world_vel);
    if(!finite(pos) || !finite(world_vel)) return;
    nav_msgs::msg::Odometry odom;
    odom.header=m.header; odom.header.frame_id=odom_frame_;odom.child_frame_id=body_frame_;
    odom.pose.pose.position.x=pos(0);odom.pose.pose.position.y=pos(1);odom.pose.pose.position.z=pos(2);
    auto &orientation=odom.pose.pose.orientation;
    orientation.w=q(0);orientation.x=q(1);orientation.y=q(2);orientation.z=q(3);
    odom.twist.twist.linear.x=body_vel(0);odom.twist.twist.linear.y=body_vel(1);odom.twist.twist.linear.z=body_vel(2);
    odom.twist.twist.angular=m.angular_velocity;
    matrix::Dcmf enu_ned;enu_ned.setZero();enu_ned(0,1)=1;enu_ned(1,0)=1;enu_ned(2,2)=-1;
    const auto pcov=enu_ned*fusion_->ekf.position_covariances()*enu_ned.transpose();
    const auto body_ned=rotation.transpose()*enu_ned;
    const auto vcov=body_ned*fusion_->ekf.velocity_covariances()*body_ned.transpose();
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) {
      odom.pose.covariance[i*6+j]=pcov(i,j);odom.twist.covariance[i*6+j]=vcov(i,j);
    }
    // Quaternion covariance is not an Euler covariance: leave unavailable
    // orientation/rate covariance explicitly large rather than inventing certainty.
    for(int i=3;i<6;++i){odom.pose.covariance[i*6+i]=1.;odom.twist.covariance[i*6+i]=1.;}
    odom_->publish(odom);mavros_->publish(odom);
  }
  double lat_,lon_,alt_;
  matrix::Vector3f imu_offset_;
  std::unique_ptr<Fusion> fusion_;
  std::string odom_frame_,body_frame_;
  uint64_t last_imu_{0},last_gps_{0},last_mag_{0},last_baro_{0},last_status_{0},last_publish_{0};
  std::deque<NavSatFix::ConstSharedPtr> fixes_;
  std::deque<TwistWithCovarianceStamped::ConstSharedPtr> velocities_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_,mavros_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Subscription<NavSatFix>::SharedPtr fix_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr velocity_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_;
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<px4_gps_imu::EkfNode>());
  rclcpp::shutdown();
}
