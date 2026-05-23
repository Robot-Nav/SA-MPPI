// ============================================================================
// 文件：twirling_critic.hpp
// 功能：旋转惩罚代价，大幅惩罚"低前进+高旋转"行为，阻止原地打转
// ============================================================================
#ifndef MPPI_CRITICS_TWIRLING_CRITIC_HPP_
#define MPPI_CRITICS_TWIRLING_CRITIC_HPP_

#include <cmath>

#include <xtensor/xtensor.hpp>

#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"
#include "tools/math_utils.hpp"

namespace mppi
{

/**
 * @brief 旋转惩罚代价（TwirlingCritic）：大幅惩罚"低前进+高旋转"行为
 * 当轨迹的前进速度很小但角速度很大时，施加高昂代价。
 * 在接近目标点一定距离内自动关闭，允许机器人在目标点附近调整朝向。
 */
class TwirlingCritic : public CriticFunction
{
public:
    void initialize() override {}

    /**
     * @param weight 代价权重
     * @param vx_max 最大前进速度（用于归一化）
     * @param threshold 距离阈值，小于该值时关闭TwirlingCritic
     */
    void setParams(float weight, float vx_max, float threshold = 0.5f)
    {
        weight_ = weight;
        vx_max_ = std::max(vx_max, EPSILON);
        threshold_to_consider_ = threshold;
    }

    void score(CriticData & data) override
    {
        if (!enabled_) return;

        // 在接近目标点一定距离内关闭TwirlingCritic
        Pose2D goal = data.path.getGoal();
        float dist_to_goal = std::hypot(data.state.pose.x - goal.x, data.state.pose.y - goal.y);
        if (dist_to_goal < threshold_to_consider_) return;

        const size_t batch_size = data.state.cvx.shape(0);
        const size_t traj_len = data.state.cvx.shape(1);

        for (size_t i = 0; i < batch_size; ++i) {
            float cost = 0.0f;
            for (size_t j = 0; j < traj_len; ++j) {
                float vx = data.state.cvx(i, j);
                float wz = data.state.cwz(i, j);
                // 前进速度越小，旋转惩罚越大（归一化到 [0,1]）
                float forward_ratio = clamp(vx / vx_max_, 0.0f, 1.0f);
                float spin_penalty = (1.0f - forward_ratio) * std::abs(wz);
                cost += spin_penalty;
            }
            data.costs(i) += weight_ * cost / traj_len;
        }
    }

private:
    float weight_ = 10.0f;
    float vx_max_ = 0.3f;
    float threshold_to_consider_ = 0.5f;
};

} // namespace mppi

#endif // MPPI_CRITICS_TWIRLING_CRITIC_HPP_
