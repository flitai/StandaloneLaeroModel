// StandaloneLaeroModel.cpp
#include "StandaloneLaeroModel.hpp"
#include <iostream>

const double StandaloneLaeroModel::HALF_PI = oe_base::PI / 2.0;
const double StandaloneLaeroModel::EPSILON = 1.0E-10;

StandaloneLaeroModel::StandaloneLaeroModel() {
    // 构造函数中可以设置初始状态
}

void StandaloneLaeroModel::setInitialVelocityKts(double kts) {
    const double KTS2MPS = 1852.0 / 3600.0;
    u = kts * KTS2MPS;
    m_state.bodyVelocity.set(u, 0, 0);
}

void StandaloneLaeroModel::setInitialState(const AircraftState& initialState) {
    m_state = initialState;
    // 关键: 同时初始化内部使用的机体速度u, 否则控制律会出错
    u = m_state.bodyVelocity.length();
}


void StandaloneLaeroModel::update(const double dt) {
    dT = dt;
    updateModel(dt);
}

void StandaloneLaeroModel::updateModel(const double dt) {
    // ==============================================================
    // 旋转方程 EOM
    // ==============================================================
    double& phi = m_state.roll;
    double& tht = m_state.pitch;
    double& psi = m_state.yaw;

    phi += 0.5 * (3.0 * phiDot - phiDot1) * dT;
    phi = oe_base::aepcdRad(phi);

    tht += 0.5 * (3.0 * thtDot - thtDot1) * dT;
    if (tht >= HALF_PI) tht = (HALF_PI - EPSILON);
    if (tht <= -HALF_PI) tht = -(HALF_PI - EPSILON);

    psi += 0.5 * (3.0 * psiDot - psiDot1) * dT;
    psi = oe_base::aepcdRad(psi);

    phiDot1 = phiDot;
    thtDot1 = thtDot;
    psiDot1 = psiDot;
    
    double sinPhi = std::sin(phi), cosPhi = std::cos(phi);
    double sinTht = std::sin(tht), cosTht = std::cos(tht);
    double sinPsi = std::sin(psi), cosPsi = std::cos(psi);

    double l1 = cosTht * cosPsi;
    double l2 = cosTht * sinPsi;
    double l3 = -sinTht;
    double m1 = sinPhi * sinTht * cosPsi - cosPhi * sinPsi;
    double m2 = sinPhi * sinTht * sinPsi + cosPhi * cosPsi;
    double m3 = sinPhi * cosTht;
    double n1 = cosPhi * sinTht * cosPsi + sinPhi * sinPsi;
    double n2 = cosPhi * sinTht * sinPsi - sinPhi * cosPsi;
    double n3 = cosPhi * cosTht;

    p = phiDot - sinTht * psiDot;
    q = cosPhi * thtDot + cosTht * sinPhi * psiDot;
    r = -sinPhi * thtDot + cosTht * cosPhi * psiDot;
    m_state.angularVelocity.set(p, q, r);

    // ==============================================================
    // 平移方程 EOM
    // ==============================================================
    u += 0.5 * (3.0 * uDot - uDot1) * dT;
    v += 0.5 * (3.0 * vDot - vDot1) * dT;
    w += 0.5 * (3.0 * wDot - wDot1) * dT;
    m_state.bodyVelocity.set(u, v, w);

    uDot1 = uDot;
    vDot1 = vDot;
    wDot1 = wDot;

    double velN = l1 * u + m1 * v + n1 * w;
    double velE = l2 * u + m2 * v + n2 * w;
    double velD = l3 * u + m3 * v + n3 * w;
    m_state.velocity.set(velN, velE, velD);

    // 更新位置 (简单的欧拉积分)
    double posX = m_state.position.x() + velN * dt;
    double posY = m_state.position.y() + velE * dt;
    double posZ = m_state.position.z() + velD * dt;
    m_state.position.set(posX, posY, posZ);
}

// --- 控制律实现 ---
bool StandaloneLaeroModel::flyPhi(double phiCmdDeg, double phiDotCmdDps) {
    double phiCmdRad = phiCmdDeg * oe_base::angle::D2RCC;
    double phiDotCmdRps = phiDotCmdDps * oe_base::angle::D2RCC;

    double phiErrRad = oe_base::aepcdRad(phiCmdRad - m_state.roll);

    const double TAU = 1.0;
    double phiErrBrkRad = phiDotCmdRps * TAU;
    
    double phiDotRps = oe_base::sign(phiErrRad) * phiDotCmdRps;
    if (std::abs(phiErrRad) < phiErrBrkRad) {
        phiDotRps = (phiErrRad / phiErrBrkRad) * phiDotCmdRps;
    }
    phiDot = phiDotRps;
    return true;
}

bool StandaloneLaeroModel::flyTht(double thtCmdDeg, double thtDotCmdDps) {
    double thtCmdRad = thtCmdDeg * oe_base::angle::D2RCC;
    double thtDotCmdRps = thtDotCmdDps * oe_base::angle::D2RCC;
    
    double thtErrRad = thtCmdRad - m_state.pitch;
    
    const double TAU = 1.0;
    double thtErrBrkRad = thtDotCmdRps * TAU;

    double thtDotRps = oe_base::sign(thtErrRad) * thtDotCmdRps;
    if (std::abs(thtErrRad) < thtErrBrkRad) {
        thtDotRps = (thtErrRad / thtErrBrkRad) * thtDotCmdRps;
    }
    thtDot = thtDotRps;
    return true;
}

bool StandaloneLaeroModel::flyPsi(double psiCmdDeg, double psiDotCmdDps) {
    // 此函数在原始代码中存在，但高层指令未使用，为完整性保留
    double psiCmdRad = psiCmdDeg * oe_base::angle::D2RCC;
    double psiDotCmdRps = psiDotCmdDps * oe_base::angle::D2RCC;
    
    double psiErrRad = oe_base::aepcdRad(psiCmdRad - m_state.yaw);
    
    const double TAU = 1.0;
    double psiErrBrkRad = psiDotCmdRps * TAU;

    double psiDotRps = oe_base::sign(psiErrRad) * psiDotCmdRps;
    if (std::abs(psiErrRad) < psiErrBrkRad) {
        psiDotRps = (psiErrRad / psiErrBrkRad) * psiDotCmdRps;
    }
    psiDot = psiDotRps;
    return true;
}

// --- 高层指令接口 ---
void StandaloneLaeroModel::setCommandedHeadingD(double h, double hDps, double maxBank) {
    const double MAX_BANK_RAD = maxBank * oe_base::angle::D2RCC;
    const double TAU = 1.0;

    double velMps = m_state.bodyVelocity.length();
    if (velMps < 1.0) velMps = 1.0; // 避免除零

    double hdgDeg = m_state.yaw * oe_base::angle::R2DCC;
    double hdgErrDeg = oe_base::aepcdDeg(h - hdgDeg);

    double hdgDotMaxAbsRps = oe_base::ETHGM * std::tan(MAX_BANK_RAD) / velMps;
    double hdgDotMaxAbsDps = hdgDotMaxAbsRps * oe_base::angle::R2DCC;

    double hdgDotAbsDps = std::min(hDps, hdgDotMaxAbsDps);
    
    double hdgErrBrkAbsDeg = TAU * hdgDotAbsDps;
    if (std::abs(hdgErrDeg) < hdgErrBrkAbsDeg) {
        hdgDotAbsDps = std::abs(hdgErrDeg) / TAU;
    }

    double hdgDotDps = oe_base::sign(hdgErrDeg) * hdgDotAbsDps;
    psiDot = hdgDotDps * oe_base::angle::D2RCC;

    double phiCmdDeg = std::atan2(psiDot * velMps, oe_base::ETHGM) * oe_base::angle::R2DCC;
    flyPhi(phiCmdDeg);
}

void StandaloneLaeroModel::setCommandedAltitude(double a, double aMps, double maxPitch) {
    const double TAU = 4.0;
    double altMtr = -m_state.position.z(); // 假设Z轴朝下（NED坐标系）
    double altErrMtr = a - altMtr;
    
    double altDotCmdMps = aMps;
    double altErrBrkMtr = altDotCmdMps * TAU;

    double altDotMps = oe_base::sign(altErrMtr) * altDotCmdMps;
    if (std::abs(altErrMtr) < altErrBrkMtr) {
        altDotMps = altErrMtr * (altDotCmdMps / altErrBrkMtr);
    }
    
    double velU = m_state.bodyVelocity.x();
    if (std::abs(velU) < 1.0) velU = 1.0;

    double thtCmdRad = std::asin(altDotMps / velU);
    double thtCmdDeg = thtCmdRad * oe_base::angle::R2DCC;
    
    // 限制最大俯仰角
    thtCmdDeg = std::max(-maxPitch, std::min(maxPitch, thtCmdDeg));
    
    flyTht(thtCmdDeg);
}

void StandaloneLaeroModel::setCommandedVelocityKts(double v, double vNps) {
    const double KTS2MPS = 1852.0 / 3600.0;
    double velCmdMps = v * KTS2MPS;
    double velDotCmdMps2 = vNps * KTS2MPS;
    
    double velMps = m_state.bodyVelocity.x(); // 只考虑前向速度
    double velErrMps = velCmdMps - velMps;
    
    const double TAU = 1.0;
    double velErrBrkMps = velDotCmdMps2 * TAU;

    double velDotMps2 = oe_base::sign(velErrMps) * velDotCmdMps2;
    if (std::abs(velErrMps) < velErrBrkMps) {
        velDotMps2 = (velErrMps / velErrBrkMps) * velDotCmdMps2;
    }
    uDot = velDotMps2;
}