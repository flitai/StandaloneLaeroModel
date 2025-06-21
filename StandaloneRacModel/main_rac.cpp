// main_rac.cpp
// 编译指令: g++ main_rac.cpp StandaloneRacModel.cpp -o RacSim -std=c++17 -I.

#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>
#include "StandaloneRacModel.hpp"

// 轨迹点定义 (与之前相同)
struct TrajectoryPoint {
    double timestamp;
    oe_base::Vec3d position;
    double velocityKts;
    double headingDeg;
};

// 轨迹生成函数 (与之前相同,生成一条平滑的S型转弯爬升机动轨迹)
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
    StandaloneRacModel aircraft;
    
    // --- 配置飞机性能 ---
    // setPerformanceLimits(minSpeedKts, maxG, speedAtMaxG_Kts, maxAccel_mps2)
    aircraft.setPerformanceLimits(100.0, 9.0, 400.0, 25.0);

    // --- 轨迹和文件初始化 ---
    std::vector<TrajectoryPoint> trajectory = createManeuverTrajectory();
    if (trajectory.empty()) return 1;

    // 设置初始状态
    const TrajectoryPoint& startPoint = trajectory.front();
    AircraftState initialState;
    initialState.position = startPoint.position;
    initialState.yaw = startPoint.headingDeg * oe_base::angle::D2RCC;
    double startVelMps = startPoint.velocityKts * (1852.0 / 3600.0);
    initialState.bodyVelocity.set(startVelMps, 0, 0);
    aircraft.setInitialState(initialState);
    
    std::ofstream outputFile("rac_model_log.csv");
    outputFile << "Time,PosX,PosY,Alt,Roll,Pitch,Yaw,VelKts,TargetAlt,TargetHdg,TargetVelKts,ErrorDist\n";
    
    // --- 仿真循环 ---
    const double dt = 1.0 / 60.0;
    size_t trajectoryIndex = 0;
    
    std::cout << "RacModel Simulation Started..." << std::endl;

    for (double simTime = 0.0; simTime <= trajectory.back().timestamp; simTime += dt) {
        while (trajectoryIndex < trajectory.size() - 1 && trajectory[trajectoryIndex].timestamp < simTime) {
            trajectoryIndex++;
        }
        const TrajectoryPoint& targetPoint = trajectory[trajectoryIndex];

        // --- 引导逻辑: 计算并下达指令 ---
        double commandedAltitude = -targetPoint.position.z();
        double commandedVelocity = targetPoint.velocityKts;
        
        // RacModel的航向指令是直接给目标航向，而不是计算方位角
        double commandedHeading = targetPoint.headingDeg;

        aircraft.setCommandedAltitude(commandedAltitude);
        aircraft.setCommandedVelocityKts(commandedVelocity);
        aircraft.setCommandedHeadingD(commandedHeading);
        
        // --- 更新动力学 ---
        aircraft.update(dt);

        // --- 记录状态和误差 ---
        const AircraftState& currentState = aircraft.getState();
        double errorDist = (currentState.position - targetPoint.position).length();
        
        outputFile << std::fixed << std::setprecision(4)
                   << simTime << ","
                   << currentState.position.x() << "," << currentState.position.y() << "," << -currentState.position.z() << ","
                   << currentState.roll * oe_base::angle::R2DCC << "," << currentState.pitch * oe_base::angle::R2DCC << "," << currentState.yaw * oe_base::angle::R2DCC << ","
                   << currentState.bodyVelocity.length() * (3600.0 / 1852.0) << ","
                   << commandedAltitude << "," << commandedHeading << "," << commandedVelocity << ","
                   << errorDist << "\n";
    }

    outputFile.close();
    std::cout << "Simulation Finished. Log file 'rac_model_log.csv' has been saved." << std::endl;

    return 0;
}