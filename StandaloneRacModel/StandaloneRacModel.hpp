// StandaloneRacModel.hpp
#ifndef STANDALONE_RAC_MODEL_HPP
#define STANDALONE_RAC_MODEL_HPP

#include "AircraftState.hpp"

class StandaloneRacModel {
public:
    StandaloneRacModel();

    // --- 公共接口 ---
    void update(const double dt);
    
    // 设置飞行指令
    void setCommandedHeadingD(double degs);
    void setCommandedAltitude(double meters);
    void setCommandedVelocityKts(double kts);

    // 设置飞机性能限制 (替代原有的Slots配置)
    void setPerformanceLimits(double minSpeedKts, double maxG, double speedAtMaxG_Kts, double maxAccel_mps2);

    // 获取当前状态
    const AircraftState& getState() const { return m_state; }
    void setInitialState(const AircraftState& initialState);

private:
    // --- 私有辅助函数 (移植自RacModel) ---
    void updateRac(const double dt);

private:
    // --- 模型状态 ---
    AircraftState m_state;

    // --- 性能限制参数 (原槽位变量) ---
    double vpMinKts   = 80.0;    // 最小速度 (节)
    double vpMaxG_Kts = 350.0;   // 达到最大G值的速度 (节)
    double gMax       = 7.0;     // 最大G值
    double maxAccel   = 20.0;    // 最大加速度 (米/秒^2)

    // --- 指令变量 ---
    double cmdAltitude = -9999.0;
    double cmdHeading  = -9999.0;
    double cmdVelocity = -9999.0;
    
    // --- 内部状态变量 ---
    double qa1 = 0.0;
    double ra1 = 0.0;
};

#endif // STANDALONE_RAC_MODEL_HPP