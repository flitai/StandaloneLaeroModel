// StandaloneRacModel.cpp
#include "StandaloneRacModel.hpp"
#include <iostream>

StandaloneRacModel::StandaloneRacModel() {
    // 构造函数初始化
}

void StandaloneRacModel::setInitialState(const AircraftState& initialState) {
    m_state = initialState;
}

void StandaloneRacModel::update(const double dt) {
    updateRac(dt);
}

void StandaloneRacModel::setPerformanceLimits(double minSpeedKts, double maxG_val, double speedAtMaxG_Kts, double maxAccel_mps2) {
    vpMinKts = minSpeedKts;
    gMax = maxG_val;
    vpMaxG_Kts = speedAtMaxG_Kts;
    maxAccel = maxAccel_mps2;
}

void StandaloneRacModel::setCommandedHeadingD(double degs) {
    cmdHeading = degs;
}

void StandaloneRacModel::setCommandedAltitude(double meters) {
    cmdAltitude = meters;
}

void StandaloneRacModel::setCommandedVelocityKts(double kts) {
    cmdVelocity = kts;
}

void StandaloneRacModel::updateRac(const double dt) {
    // --- 常量转换 ---
    const double KTS2MPS = 1852.0 / 3600.0;
    const double MPS2KTS = 3600.0 / 1852.0;
    const double R2D = oe_base::angle::R2DCC;
    const double D2R = oe_base::angle::D2RCC;

    // 获取当前状态
    double currentAltitudeM = -m_state.position.z();
    double currentHeadingD = m_state.yaw * R2D;
    double currentVelocityKts = m_state.bodyVelocity.length() * MPS2KTS;
    double currentVelocityMps = m_state.bodyVelocity.length();

    // 如果指令未设置，则保持当前状态
    if (cmdAltitude < -9000.0) cmdAltitude = currentAltitudeM;
    if (cmdHeading < -9000.0) cmdHeading = currentHeadingD;
    if (cmdVelocity < -9000.0) cmdVelocity = currentVelocityKts;

    // --- 计算高度差、期望垂直速度和期望俯仰角 ---
    double maxAltRate = (3000.0 / 60.0) * (3.28084 / 3.28084); // 3000 ft/min in m/s
    double cmdAltRate = cmdAltitude - currentAltitudeM;
    cmdAltRate = std::max(-maxAltRate, std::min(maxAltRate, cmdAltRate));
    
    double cmdPitchRad = 0.0;
    if (currentVelocityMps > 1.0) {
        cmdPitchRad = std::asin(cmdAltRate / currentVelocityMps);
    }
    
    // --- 计算最大G值 ---
    double gmax_now = gMax;
    if (currentVelocityKts < vpMaxG_Kts && vpMaxG_Kts > vpMinKts) {
        gmax_now = 1.0 + (gMax - 1.0) * (currentVelocityKts - vpMinKts) / (vpMaxG_Kts - vpMinKts);
    }
    if (gmax_now < 1.0) gmax_now = 1.0;

    // --- 计算最大转弯率和俯仰率 ---
    double ra_max = (gmax_now * oe_base::ETHGM) / currentVelocityMps;
    double qa_max = ra_max;
    double qa_min = -ra_max;
    if (gmax_now > 2.0) {
        qa_min = -(2.0f * oe_base::ETHGM / currentVelocityMps);
    }

    // --- 计算期望角速度 ---
    double qa = oe_base::aepcdRad(cmdPitchRad - m_state.pitch) * 0.5; // 增加增益使其响应更快
    qa = std::max(qa_min, std::min(qa_max, qa));

    double ra = oe_base::aepcdRad((cmdHeading * D2R) - m_state.yaw) * 0.5; // 增加增益
    ra = std::max(-ra_max, std::min(ra_max, ra));

    // --- 积分计算新姿态 ---
    double newTheta = m_state.pitch + (qa + qa1) * dt / 2.0;
    double newPsi = oe_base::aepcdRad(m_state.yaw + (ra + ra1) * dt / 2.0);
    
    // 滚转角与转弯率成正比 (为了视觉效果)
    double newPhi = 0.98 * m_state.roll + 0.02 * (ra / ra_max * (D2R * 60.0));

    // --- 计算期望加速度和新速度 ---
    double cmdVelMPS = cmdVelocity * KTS2MPS;
    double vpdot = (cmdVelMPS - currentVelocityMps) * 0.1; // 增加增益
    vpdot = std::max(-maxAccel, std::min(maxAccel, vpdot));

    double newVP_mps = currentVelocityMps + vpdot * dt;
    if (newVP_mps < vpMinKts * KTS2MPS) newVP_mps = vpMinKts * KTS2MPS;

    // --- 更新状态 ---
    m_state.roll = newPhi;
    m_state.pitch = newTheta;
    m_state.yaw = newPsi;
    
    m_state.angularVelocity.set(0.0, qa, ra); // pa (滚转率)简化为0
    qa1 = qa;
    ra1 = ra;

    m_state.bodyVelocity.set(newVP_mps, 0.0, 0.0); // 简化: 无侧滑和垂直机体速度
    
    // 通过姿态和机体速度计算世界速度和位置
    double l1 = std::cos(newTheta) * std::cos(newPsi);
    double l2 = std::cos(newTheta) * std::sin(newPsi);
    double l3 = -std::sin(newTheta);
    // ... (m, n分量计算，为简化省略v,w的影响)
    double velN = l1 * newVP_mps;
    double velE = l2 * newVP_mps;
    double velD = l3 * newVP_mps;
    m_state.velocity.set(velN, velE, velD);
    m_state.position.set(
        m_state.position.x() + velN * dt,
        m_state.position.y() + velE * dt,
        m_state.position.z() + velD * dt
    );
}