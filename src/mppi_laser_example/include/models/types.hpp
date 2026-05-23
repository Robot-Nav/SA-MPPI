// ============================================================================
// 文件：types.hpp
// 功能：基础数据类型定义（点、位姿、速度、控制量、障碍物预测等）
// ============================================================================
#ifndef MPPI_MODELS_TYPES_HPP_
#define MPPI_MODELS_TYPES_HPP_

#include <cmath>
#include <limits>
#include <vector>

namespace mppi
{

/**
 * @brief 二维点结构
 */
struct Point2D
{
    float x = 0.0f;
    float y = 0.0f;
    Point2D() = default;
    Point2D(float x_, float y_) : x(x_), y(y_) {}
};

/**
 * @brief 二维位姿（位置 + 朝向）
 */
struct Pose2D
{
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
    Pose2D() = default;
    Pose2D(float x_, float y_, float theta_) : x(x_), y(y_), theta(theta_) {}
};

/**
 * @brief 二维速度（线速度 + 角速度）
 */
struct Twist2D
{
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    Twist2D() = default;
    Twist2D(float vx_, float vy_, float wz_) : vx(vx_), vy(vy_), wz(wz_) {}
};

/**
 * @brief 单步控制量
 */
struct Control
{
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
};

/**
 * @brief 动态障碍物预测结构
 */
struct ObstaclePrediction
{
    float x, y;           // 初始位置
    float vx, vy;         // 速度
    float radius;         // 膨胀半径

    /**
     * @brief 获取某时刻的位置
     * @param t 时间
     * @return 预测位置
     */
    Point2D positionAt(float t) const {
        return Point2D(x + vx * t, y + vy * t);
    }
};

} // namespace mppi

#endif // MPPI_MODELS_TYPES_HPP_
