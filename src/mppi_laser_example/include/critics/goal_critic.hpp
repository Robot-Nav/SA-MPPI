// ============================================================================
// 文件：goal_critic.hpp
// 功能：目标点代价，当接近目标时惩罚轨迹终点与目标点的距离平方
// ============================================================================
#ifndef MPPI_CRITICS_GOAL_CRITIC_HPP_
#define MPPI_CRITICS_GOAL_CRITIC_HPP_

#include <cmath>

#include <xtensor/xtensor.hpp>

#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"

namespace mppi
{

/**
 * @brief 目标点代价：当接近目标时，惩罚轨迹终点与目标点的距离平方
 */
class GoalCritic : public CriticFunction
{
public:
    void initialize() override {}

    /**
     * @param weight 代价权重
     * @param threshold 距离阈值，小于该值才激活代价
     */
    void setParams(float weight, float threshold)
    {
        weight_ = weight;
        threshold_to_consider_ = threshold;
    }

    void score(CriticData & data) override
    {
        if (!enabled_) return;
        Pose2D goal = data.path.getGoal();
        float dist_to_goal = std::hypot(data.state.pose.x - goal.x, data.state.pose.y - goal.y);
        if (dist_to_goal > threshold_to_consider_) return;

        const size_t batch_size = data.trajectories.x.shape(0);
        const size_t traj_len = data.trajectories.x.shape(1);

        for (size_t i = 0; i < batch_size; ++i) {
            float last_x = data.trajectories.x(i, traj_len - 1);
            float last_y = data.trajectories.y(i, traj_len - 1);
            float dist = std::hypot(last_x - goal.x, last_y - goal.y);
            data.costs(i) += weight_ * dist * dist;
        }
    }

private:
    float weight_ = 5.0f;
    float threshold_to_consider_ = 1.0f;
};

} // namespace mppi

#endif // MPPI_CRITICS_GOAL_CRITIC_HPP_
