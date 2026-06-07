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
     * @param threshold 距离阈值，小于该值时关闭TwirlingCritic
     */
    void setParams(float weight, float threshold = 0.5f)
    {
        weight_ = weight;
        threshold_to_consider_ = threshold;
    }

    void score(CriticData & data) override
    {
        if (!enabled_) return;

        Pose2D goal = data.path.getGoal();
        float dist_to_goal = std::hypot(data.state.pose.x - goal.x, data.state.pose.y - goal.y);
        if (dist_to_goal < threshold_to_consider_) return;

        const size_t batch_size = data.state.wz.shape(0);
        const size_t traj_len = data.state.wz.shape(1);

        for (size_t i = 0; i < batch_size; ++i) {
            float avg_abs_wz = 0.0f;
            for (size_t j = 0; j < traj_len; ++j) {
                avg_abs_wz += std::abs(data.state.wz(i, j));
            }
            avg_abs_wz /= traj_len;
            data.costs(i) += weight_ * avg_abs_wz;
        }
    }

private:
    float weight_ = 10.0f;
    float threshold_to_consider_ = 0.5f;
};

} // namespace mppi

#endif // MPPI_CRITICS_TWIRLING_CRITIC_HPP_
