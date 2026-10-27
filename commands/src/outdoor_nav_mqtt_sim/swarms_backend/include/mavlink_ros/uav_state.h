//
// Created by libo on 2024/08/27.
//

#ifndef UAV_STATE_H
#define UAV_STATE_H

#include <mavros_msgs/msg/state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cmath>
#include <array>

struct RobotMajorState
{
    int32_t state;
    bool isUpdated;

    RobotMajorState() : state(0), isUpdated(false) {}
};

struct PX4State
{
    bool connected;
    bool armed;
    bool guided;
    bool manual_input;
    std::string mode;
    bool isUpdated;

    PX4State() : connected(false), armed(false), guided(false), manual_input(false), mode(""), isUpdated(false) {}
};

struct BatteryState
{
    float voltage;
    bool isUpdated;

    BatteryState() : voltage(0), isUpdated(false) {}
};

struct Satellite
{
    int satellite_count;
    bool isUpdated;

    Satellite() : satellite_count(0), isUpdated(false) {}
};

struct PX4Pos
{
    float x;
    float y;
    float z;
    double yaw;
    double pitch;
    double roll;
    float dis;
    float high;
    bool isUpdated;

    PX4Pos() : x(0), y(0), z(0), yaw(0), roll(0), pitch(0), dis(0), high(0), isUpdated(false) {}
};

struct UsmDataInfo
{
    std::array<float, 3> ref_pos;
    std::array<float, 3> ref_gps;
    bool isUpdated;
    UsmDataInfo() : ref_pos{0, 0, 0}, ref_gps{0, 0, 0}, isUpdated(false) {}
};

struct PX4Vel
{
    float vx;
    float vy;
    float vz;
    bool isUpdated;
    float dis_vel;
    PX4Vel() : vx(0), vy(0), vz(0), dis_vel(0), isUpdated(false) {}
};

struct GlobalPosition
{
    int8_t status;
    uint16_t service;
    double latitude;
    double longitude;
    double altitude;
    bool isUpdated;

    GlobalPosition() : status(0), service(0), latitude(0), longitude(0), altitude(0), isUpdated(false) {}
};

struct UAVState
{
    RobotMajorState robotMajorState_;
    PX4State px4State_;
    GlobalPosition globalPosition_;
    BatteryState batteryState_;
    Satellite satellite_;
    PX4Pos px4Pos_;
    PX4Vel px4Vel_;

    UAVState() : robotMajorState_(), px4State_(), globalPosition_(), batteryState_(), satellite_(), px4Pos_(), px4Vel_() {}
};

#endif // UAV_STATE_H