// ============================================================================
// 文件：constraints.hpp
// 功能：控制量约束与采样噪声参数定义
// ============================================================================
#ifndef MPPI_MODELS_CONSTRAINTS_HPP_
#define MPPI_MODELS_CONSTRAINTS_HPP_

namespace mppi
{

/**
 * @brief 控制量约束
 */
struct ControlConstraints
{
    float vx_max = 0.5f;
    float vx_min = -0.35f;
    float vy_max = 0.5f;
    float wz_max = 1.9f;
    // 加速度限制
    float ax_max = 2.0f;
    float ay_max = 2.0f;
    float az_max = 3.0f;
    // 碰撞代价阈值（用于判断最优轨迹是否发生碰撞）
    float collision_cost_threshold = 5000.0f;
};

/**
 * @brief 采样噪声标准差
 */
struct SamplingStd
{
    float vx = 0.2f;
    float vy = 0.2f;
    float wz = 0.4f;
};

} // namespace mppi

#endif // MPPI_MODELS_CONSTRAINTS_HPP_
