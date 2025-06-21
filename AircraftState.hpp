// AircraftState.hpp
#ifndef AIRCRAFT_STATE_HPP
#define AIRCRAFT_STATE_HPP

#include "OeBase.hpp"

struct AircraftState {
    // --- 姿态 ---
    double roll  = 0.0; // 滚转角 (phi), 弧度
    double pitch = 0.0; // 俯仰角 (tht), 弧度
    double yaw   = 0.0; // 偏航角 (psi), 弧度

    // --- 位置 ---
    // 使用简单的笛卡尔坐标系，而非经纬度
    oe_base::Vec3d position; // 位置 (x, y, z), 米

    // --- 速度 ---
    oe_base::Vec3d velocity;      // 世界坐标系速度 (北-东-地), m/s
    oe_base::Vec3d bodyVelocity;  // 机体坐标系速度 (u,v,w), m/s

    // --- 角速度 ---
    oe_base::Vec3d angularVelocity; // 机体坐标系角速度 (p,q,r), rad/s
};

#endif // AIRCRAFT_STATE_HPP