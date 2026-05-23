// ============================================================================
// 文件：goal_angle_critic.hpp
// 功能：目标朝向代价，当接近目标时惩罚轨迹终点朝向与目标朝向的偏差平方
// ============================================================================
#ifndef MPPI_CRITICS_GOAL_ANGLE_CRITIC_HPP_
#define MPPI_CRITICS_GOAL_ANGLE_CRITIC_HPP_

#include <cmath>

#include <xtensor/xtensor.hpp>

#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"
#include "tools/math_utils.hpp"

namespace mppi
{

/**
 * @brief 目标朝向代价：当接近目标时，惩罚轨迹终点朝向与目标朝向的偏差平方
 */
class GoalAngleCritic : public CriticFunction
{
public:
    void initialize() override {}

    /**
     * @param weight 代价权重
     * @param threshold 距离阈值
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

        const size_t batch_size = data.trajectories.yaws.shape(0);
        const size_t traj_len = data.trajectories.yaws.shape(1);

        for (size_t i = 0; i < batch_size; ++i) {
            float last_yaw = data.trajectories.yaws(i, traj_len - 1);
            float angle_diff = std::abs(shortestAngularDistance(last_yaw, goal.theta));
            data.costs(i) += weight_ * angle_diff * angle_diff;
        }
    }

private:
    float weight_ = 3.0f;
    float threshold_to_consider_ = 0.4f;
};

} // namespace mppi

#endif // MPPI_CRITICS_GOAL_ANGLE_CRITIC_HPP_
