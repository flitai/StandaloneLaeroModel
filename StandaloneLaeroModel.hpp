// StandaloneLaeroModel.hpp
#ifndef STANDALONE_LAERO_MODEL_HPP
#define STANDALONE_LAERO_MODEL_HPP

#include "AircraftState.hpp"

class StandaloneLaeroModel {
public:
    StandaloneLaeroModel();

    // --- 公共接口 ---
    void update(const double dt);
    
    void setCommandedHeadingD(double degs, double hDps = 20.0, double maxBankD = 30.0);
    void setCommandedAltitude(double meters, double aMps = 150.0, double maxPitchD = 15.0);
    void setCommandedVelocityKts(double kts, double vNps = 5.0);

    const AircraftState& getState() const { return m_state; }
    void setInitialState(const AircraftState& initialState);
    void setInitialVelocityKts(double kts);

private:
    // --- 私有辅助函数 (移植自LaeroModel) ---
    void updateModel(const double dt);
    bool flyPhi(double phiCmdDeg, double phiDotCmdDps = 30.0);
    bool flyTht(double thtCmdDeg, double thtDotCmdDps = 10.0);
    bool flyPsi(double psiCmdDeg, double psiDotCmdDps = 20.0);

private:
    // --- 模型状态和内部变量 ---
    AircraftState m_state;

    // --- LaeroModel的内部变量 ---
    static const double HALF_PI;
    static const double EPSILON;
    
    double dT = 0.0;
    
    // 机体角速度分量 (p, q, r)
    double p = 0.0, q = 0.0, r = 0.0;
    
    // 欧拉角速率 (phiDot, thtDot, psiDot)
    double phiDot = 0.0, thtDot = 0.0, psiDot = 0.0;
    
    // 机体线速度分量 (u, v, w)
    double u = 0.0, v = 0.0, w = 0.0;

    // 机体线加速度分量 (uDot, vDot, wDot)
    double uDot = 0.0, vDot = 0.0, wDot = 0.0;

    // Adams-Bashforth积分的历史值
    double phiDot1 = 0.0, thtDot1 = 0.0, psiDot1 = 0.0;
    double uDot1 = 0.0, vDot1 = 0.0, wDot1 = 0.0;
};

#endif // STANDALONE_LAERO_MODEL_HPP