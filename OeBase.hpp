// OeBase.hpp
#ifndef OE_BASE_HPP
#define OE_BASE_HPP

#include <cmath>
#include <algorithm> // For std::max/min if needed

namespace oe_base {

// --- 数学常量 ---
const double PI = 3.14159265358979323846;
const double ETHGM = 9.80665; // 地球重力加速度 m/s^2

// --- 角度转换 ---
namespace angle {
    const double D2RCC = PI / 180.0; // 度转弧度
    const double R2DCC = 180.0 / PI; // 弧度转度
}

// --- 简单的三维向量类 ---
class Vec3d {
public:
    Vec3d() : v_x(0.0), v_y(0.0), v_z(0.0) {}
    Vec3d(double x, double y, double z) : v_x(x), v_y(y), v_z(z) {}

    void set(double x, double y, double z) {
        v_x = x; v_y = y; v_z = z;
    }

    double x() const { return v_x; }
    double y() const { return v_y; }
    double z() const { return v_z; }
    
    double length() const {
        return std::sqrt(v_x * v_x + v_y * v_y + v_z * v_z);
    }
    
    double length2() const {
        return v_x * v_x + v_y * v_y + v_z * v_z;
    }

private:
    double v_x, v_y, v_z;
};

// --- 工具函数 ---
template<class T>
inline T sign(const T& x) {
    if (x > 0) return T(1);
    if (x < 0) return T(-1);
    return T(0);
}

// 确保角度在 -PI 到 +PI 之间
inline double aepcdRad(double angle) {
    while (angle > PI) angle -= (2.0 * PI);
    while (angle < -PI) angle += (2.0 * PI);
    return angle;
}

// 确保角度在 -180 到 +180 之间
inline double aepcdDeg(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

} // namespace oe_base

#endif // OE_BASE_HPP