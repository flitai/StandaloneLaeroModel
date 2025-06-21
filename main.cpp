// main.cpp
// 编译指令: g++ main.cpp StandaloneLaeroModel.cpp -o ManeuverSim -std=c++17 -I.

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include "StandaloneLaeroModel.hpp"

// 定义轨迹点结构体，包含时间戳
struct TrajectoryPoint {
    double timestamp;         // 时间戳 (秒)
    oe_base::Vec3d position;  // 期望位置 (x, y, z), z为负表示高度
    double velocityKts;       // 期望速度大小 (节)
    double headingDeg;        // 期望航向 (度)
};

// 函数：生成一条平滑的S型转弯爬升机动轨迹
// 采样周期为0.1秒
std::vector<TrajectoryPoint> createManeuverTrajectory(double duration = 120.0, double Ts = 0.1) {
    std::vector<TrajectoryPoint> trajectory;
    const double R2D = oe_base::angle::R2DCC;

    for (double t = 0.0; t <= duration; t += Ts) {
        TrajectoryPoint p;
        p.timestamp = t;

        // --- 定义速度和高度剖面 ---
        // 0-20秒: 加速并爬升
        // 20-100秒: 保持速度和高度
        // 100-120秒: 减速并下降
        if (t < 20.0) {
            p.velocityKts = 200.0 + (t / 20.0) * 150.0; // 200 -> 350 kts
            p.position.set(0, 0, -2000.0 - (t / 20.0) * 2000.0); // 2000m -> 4000m
        } else if (t <= 100.0) {
            p.velocityKts = 350.0;
            p.position.set(0, 0, -4000.0);
        } else {
            p.velocityKts = 350.0 - ((t - 100.0) / 20.0) * 100.0; // 350 -> 250 kts
            p.position.set(0, 0, -4000.0 + ((t - 100.0) / 20.0) * 1000.0); // 4000m -> 3000m
        }

        // --- 定义航向剖面 (S型转弯) ---
        // 30-60秒: 右转90度
        // 60-90秒: 左转90度回到原航向
        if (t > 30.0 && t <= 60.0) {
            double turn_progress = (t - 30.0) / 30.0;
            p.headingDeg = turn_progress * 90.0;
        } else if (t > 60.0 && t <= 90.0) {
            double turn_progress = (t - 60.0) / 30.0;
            p.headingDeg = 90.0 - turn_progress * 90.0;
        } else if (t > 90.0) {
            p.headingDeg = 0.0;
        } else {
            p.headingDeg = 0.0;
        }

        // --- 通过积分计算位置 ---
        // (为了简化，这里只积分前一个点的位置，实际样条曲线会更复杂)
        if (!trajectory.empty()) {
            const TrajectoryPoint& last_p = trajectory.back();
            double avg_vel_mps = (last_p.velocityKts + p.velocityKts) / 2.0 * (1852.0 / 3600.0);
            double avg_hdg_rad = (last_p.headingDeg + p.headingDeg) / 2.0 * oe_base::angle::D2RCC;
            
            p.position.set(
                last_p.position.x() + avg_vel_mps * std::cos(avg_hdg_rad) * Ts,
                last_p.position.y() + avg_vel_mps * std::sin(avg_hdg_rad) * Ts,
                p.position.z() // 高度已在上面定义
            );
        }

        trajectory.push_back(p);
    }
    return trajectory;
}

int main() {
    StandaloneLaeroModel aircraft;
    std::vector<TrajectoryPoint> trajectory = createManeuverTrajectory();

    // --- 初始化飞机状态，使其与轨迹起点完全一致 ---
    if (trajectory.empty()) {
        std::cerr << "Error: Trajectory is empty." << std::endl;
        return 1;
    }
    const TrajectoryPoint& startPoint = trajectory.front();
    AircraftState initialState;
    initialState.position = startPoint.position;
    initialState.yaw = startPoint.headingDeg * oe_base::angle::D2RCC;
    double startVelMps = startPoint.velocityKts * (1852.0 / 3600.0);
    initialState.bodyVelocity.set(startVelMps, 0, 0);
    initialState.velocity.set(startVelMps * std::cos(initialState.yaw), startVelMps * std::sin(initialState.yaw), 0);
    aircraft.setInitialState(initialState);
    
    // --- 打开输出文件并写入文件头 ---
    std::ofstream outputFile("maneuver_log.csv");
    if (!outputFile.is_open()) {
        std::cerr << "Error: Could not open output file." << std::endl;
        return 1;
    }
    outputFile << "Time,PosX,PosY,Alt,Roll,Pitch,Yaw,VelKts,"
               << "TargetPosX,TargetPosY,TargetAlt,TargetHdg,TargetVelKts,"
               << "ErrorDist,ErrorAlt,ErrorHdg,ErrorVel\n";
    
    std::cout << "Simulation Started. Following maneuver trajectory..." << std::endl;
    std::cout << "Data will be saved to maneuver_log.csv" << std::endl;
    
    // --- 仿真循环 ---
    const double dt = 1.0 / 60.0; // 仿真步长
    size_t trajectoryIndex = 0;
    
    for (double simTime = 0.0; simTime <= trajectory.back().timestamp; simTime += dt) {
        // --- 时间同步的轨迹跟随逻辑 ---
        // 1. 查找与当前仿真时间对应的期望轨迹点
        while (trajectoryIndex < trajectory.size() - 1 && trajectory[trajectoryIndex].timestamp < simTime) {
            trajectoryIndex++;
        }
        const TrajectoryPoint& targetPoint = trajectory[trajectoryIndex];

        // 2. 从期望轨迹点获取控制指令
        double commandedAltitude = -targetPoint.position.z();
        double commandedVelocity = targetPoint.velocityKts;
        double commandedHeading = targetPoint.headingDeg;

        // 3. 发送控制指令给飞机模型
        aircraft.setCommandedAltitude(commandedAltitude);
        aircraft.setCommandedVelocityKts(commandedVelocity);
        aircraft.setCommandedHeadingD(commandedHeading);
        
        // --- 更新动力学 ---
        aircraft.update(dt);

        // --- 计算误差 ---
        const AircraftState& currentState = aircraft.getState();
        const oe_base::Vec3d posErrorVec(
            currentState.position.x() - targetPoint.position.x(),
            currentState.position.y() - targetPoint.position.y(),
            currentState.position.z() - targetPoint.position.z()
        );
        double errorDist = posErrorVec.length();
        double currentAlt = -currentState.position.z();
        double errorAlt = currentAlt - commandedAltitude;
        double currentHdg = oe_base::aepcdDeg(currentState.yaw * oe_base::angle::R2DCC);
        double errorHdg = oe_base::aepcdDeg(currentHdg - commandedHeading);
        double currentKts = currentState.bodyVelocity.length() * (3600.0 / 1852.0);
        double errorVel = currentKts - commandedVelocity;

        // --- 写入数据到文件 ---
        outputFile << std::fixed << std::setprecision(4)
                   << simTime << ","
                   << currentState.position.x() << "," << currentState.position.y() << "," << currentAlt << ","
                   << currentState.roll * oe_base::angle::R2DCC << "," << currentState.pitch * oe_base::angle::R2DCC << "," << currentHdg << ","
                   << currentKts << ","
                   << targetPoint.position.x() << "," << targetPoint.position.y() << "," << commandedAltitude << ","
                   << commandedHeading << "," << commandedVelocity << ","
                   << errorDist << "," << errorAlt << "," << errorHdg << "," << errorVel
                   << "\n";
    }

    outputFile.close();
    std::cout << "Simulation Finished. Log file 'maneuver_log.csv' has been saved." << std::endl;

    return 0;
}