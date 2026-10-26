#pragma once

#include <cmath>
#include <stdexcept>
#include <EKF/ekf.h>

namespace px4_gps_imu {
inline bool finite(const matrix::Vector3f &v) {
  return std::isfinite(v(0)) && std::isfinite(v(1)) && std::isfinite(v(2));
}
inline matrix::Vector3f flu_to_frd(const matrix::Vector3f &v) {
  return {v(0), -v(1), -v(2)};
}
inline matrix::Vector3f ned_to_enu(const matrix::Vector3f &v) {
  return {v(1), v(0), -v(2)};
}
inline matrix::Dcmf attitude_enu_flu(const matrix::Quatf &q) {
  matrix::Dcmf enu_ned;
  enu_ned.setZero(); enu_ned(0,1)=1; enu_ned(1,0)=1; enu_ned(2,2)=-1;
  matrix::Dcmf frd_flu;
  frd_flu.setIdentity(); frd_flu(1,1)=-1; frd_flu(2,2)=-1;
  return matrix::Dcmf(enu_ned * matrix::Dcmf(q) * frd_flu);
}

class Fusion {
 public:
  Fusion(double lat, double lon, double alt, const matrix::Vector3f &imu_flu)
      : lat_(lat), lon_(lon), alt_(alt) {
    if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(alt) ||
        std::abs(lat)>90 || std::abs(lon)>180) {
      throw std::invalid_argument("invalid GPS origin");
    }
    auto *p = ekf.getParamHandle();
    p->fusion_mode = MASK_USE_GPS;
    p->vdist_sensor_type = VDIST_SENSOR_GPS;
    p->gps_delay_ms = 0;  // simulated messages stamped at measurement time
    p->mag_fusion_type = MAG_FUSE_TYPE_HEADING;
    p->imu_pos_body = flu_to_frd(imu_flu);
    p->gps_pos_body.setZero();
  }
  bool imu(uint64_t stamp, const matrix::Vector3f &gyro_flu,
           const matrix::Vector3f &accel_flu) {
    if (!finite(gyro_flu) || !finite(accel_flu) || stamp==0) return false;
    if (!started_) {
      ekf.init(stamp);
      ekf.setEkfGlobalOrigin(lat_, lon_, static_cast<float>(alt_));
      started_=true;
      previous_=stamp;
      gyro_=gyro_flu; accel_=accel_flu;
      return false;
    }
    if (stamp<=previous_) return false;
    const float dt=(stamp-previous_)*1e-6f;
    previous_=stamp;
    if (dt>0.05f) {
      gyro_=gyro_flu; accel_=accel_flu;
      return false;  // never integrate a large missing IMU interval
    }
    imuSample sample{};
    sample.time_us=stamp;
    sample.delta_ang=flu_to_frd((gyro_+gyro_flu)*(0.5f*dt));
    sample.delta_vel=flu_to_frd((accel_+accel_flu)*(0.5f*dt));
    sample.delta_ang_dt=dt; sample.delta_vel_dt=dt;
    gyro_=gyro_flu; accel_=accel_flu;
    ekf.set_in_air_status(false); // ground vehicle; no landed-state inference from commands
    ekf.setIMUData(sample);
    const bool updated=ekf.update();
    // Upstream sets the vertical reference when GPS first passes its checks.
    // Reapply the configured common origin once, through its public API.
    if(ekf.global_origin_valid() && !origin_finalized_) {
      ekf.setEkfGlobalOrigin(lat_,lon_,static_cast<float>(alt_));
      origin_finalized_=true;
    }
    return updated;
  }
  bool ready() const {
    const auto &s=ekf.control_status_flags();
    return started_ && ekf.attitude_valid() && ekf.local_position_is_valid() &&
      s.yaw_align && s.gps;
  }
  bool started() const { return started_; }
  Ekf ekf;
 private:
  double lat_, lon_, alt_;
  bool started_{false};
  bool origin_finalized_{false};
  uint64_t previous_{0};
  matrix::Vector3f gyro_{}, accel_{};
};
}  // namespace px4_gps_imu
