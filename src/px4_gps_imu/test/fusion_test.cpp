#include <px4_gps_imu/fusion.hpp>
#include <geo/geo.h>
#include <geo_lookup/geo_mag_declination.h>
#include <iostream>

static void require(bool condition,const char *message) {
  if(!condition) throw std::runtime_error(message);
}
int main() {
  using namespace px4_gps_imu;
  const double lat=40.9445353,lon=113.172819,alt=1298.734571;
  Fusion fusion(lat,lon,alt,{0,0,0});
  map_projection_reference_s projection{};map_projection_init(&projection,lat,lon);
  const float d=get_mag_declination_radians(lat,lon),i=get_mag_inclination_radians(lat,lon);
  const float b=get_mag_strength_gauss(lat,lon);
  // FLU body x points east in ENU. Convert the geographic magnetic field
  // to body FRD. No orientation measurement is injected into the estimator.
  const matrix::Vector3f mag{b*std::cos(i)*std::sin(d),-b*std::cos(i)*std::cos(d),b*std::sin(i)};
  double position=0,velocity=0;
  for(int step=1;step<=6000;++step) {
    const uint64_t t=uint64_t(step)*5000;
    const double time=t*1e-6;
    double accel=0;
    if(time>=18 && time<20) accel=0.25;
    if(time>=24 && time<26) accel=-0.25;
    position+=velocity*0.005+accel*0.005*0.005/2;
    velocity+=accel*0.005;
    fusion.imu(t,{0,0,0},{float(accel),0,9.80665f});
    if(step%4==0) {
      fusion.ekf.setMagData(magSample{t,mag});
      fusion.ekf.setBaroData(baroSample{t,float(alt)});
    }
    if(step%20==0) {
      gps_message g{};g.time_usec=t;
      double glat,glon;map_projection_reproject(&projection,0,position,&glat,&glon);
      g.lat=std::llround(glat*1e7);g.lon=std::llround(glon*1e7);g.alt=std::llround(alt*1000);
      g.yaw=NAN;g.fix_type=3;g.nsats=16;g.pdop=1.;g.eph=.15;g.epv=.15;g.sacc=.03;
      g.vel_ned={0,float(velocity),0};g.vel_ned_valid=true;g.vel_m_s=std::abs(velocity);
      fusion.ekf.setGpsData(g);
    }
    if(step==3400) {
      require(fusion.ready(),"GPS fusion did not initialize");
      require(ned_to_enu(fusion.ekf.getPosition()).norm()<.1,"stationary position drift");
      const auto r=attitude_enu_flu(fusion.ekf.getQuaternion());
      require(std::abs(r(0,0)-1)<.02 && std::abs(r(1,0))<.02,"ENU/FRD attitude conversion");
    }
  }
  const auto p=ned_to_enu(fusion.ekf.getPosition());
  std::cout<<"truth east="<<position<<" estimate="<<p(0)<<","<<p(1)<<","<<p(2)<<"\n";
  require(fusion.ready(),"GPS fusion lost during motion");
  require(std::abs(p(0)-position)<.15 && std::abs(p(1))<.1,"GPS/IMU motion estimate inaccurate");
  require(fusion.ekf.getVelocity().norm()<.1,"estimated speed did not settle after stop");
  require(!fusion.imu(30000000,{0,0,0},{0,0,9.80665f}),"duplicate IMU accepted");
  require(!fusion.imu(29990000,{0,0,0},{0,0,9.80665f}),"backwards IMU accepted");
  require(!fusion.imu(30005000,{NAN,0,0},{0,0,9.80665f}),"invalid IMU accepted");
  for(int step=6001;step<=8600;++step) {
    const uint64_t t=uint64_t(step)*5000;
    fusion.imu(t,{0,0,0},{0,0,9.80665f});
    if(step%4==0) fusion.ekf.setMagData(magSample{t,mag});
  }
  require(!fusion.ready(),"GPS outage did not invalidate navigation output");
  std::cout<<"PX4 EKF stationary, motion, coordinate conversion, timestamps and GPS outage passed\n";
}
