// ============================================================================
// 文件：velocity_deadband_critic.hpp
// 功能：速度死区代价，惩罚速度低于死区阈值的情况，避免低速不稳定行为
//       对齐 Nav2：始终激活（无距离阈值），使用实际速度 vx/vy/wz
// ============================================================================
#ifndef MPPI_CRITICS_VELOCITY_DEADBAND_CRITIC_HPP_
#define MPPI_CRITICS_VELOCITY_DEADBAND_CRITIC_HPP_

#include <cmath>

#include <xtensor/xtensor.hpp>

#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"

namespace mppi
{

/**
 * @brief 速度死区代价：惩罚速度低于死区阈值的情况
 * 对齐 Nav2：始终激活，使用实际轨迹速度
 */
class VelocityDeadbandCritic : public CriticFunction
{
public:
    void initialize() override {}

    /**
     * @param weight 代价权重（Nav2 默认 35.0）
     * @param deadband_vx 线速度死区阈值
     * @param deadband_vy 横向速度死区阈值（全向模型）
     * @param deadband_wz 角速度死区阈值
     */
    void setParams(float weight, float deadband_vx, float deadband_vy, float deadband_wz)
    {
        weight_ = weight;
        deadband_vx_ = deadband_vx;
        deadband_vy_ = deadband_vy;
        deadband_wz_ = deadband_wz;
    }

    void score(CriticData & data) override
    {
        if (!enabled_) return;

        const size_t batch_size = data.state.vx.shape(0);
        const size_t traj_len = data.state.vx.shape(1);

        for (size_t i = 0; i < batch_size; ++i) {
            float cost = 0.0f;
            for (size_t j = 0; j < traj_len; ++j) {
                float vx = data.state.vx(i, j);
                float vy = data.state.vy(i, j);
                float wz = data.state.wz(i, j);

                // 惩罚速度低于死区阈值的情况
                float vx_penalty = std::max(0.0f, deadband_vx_ - std::abs(vx));
                float vy_penalty = std::max(0.0f, deadband_vy_ - std::abs(vy));
                float wz_penalty = std::max(0.0f, deadband_wz_ - std::abs(wz));

                cost += (vx_penalty + vy_penalty + wz_penalty) * data.model_dt;
            }
            data.costs(i) += weight_ * cost;
        }
    }

private:
    float weight_ = 35.0f;
    float deadband_vx_ = 0.05f;  // 线速度死区 (m/s)
    float deadband_vy_ = 0.05f;  // 横向速度死区 (m/s)
    float deadband_wz_ = 0.1f;   // 角速度死区 (rad/s)
};

} // namespace mppi

#endif // MPPI_CRITICS_VELOCITY_DEADBAND_CRITIC_HPP_
