// ============================================================================
// 文件：path_follow_critic.hpp
// 功能：路径跟随代价，计算轨迹终点与路径参考点之间的距离
// ============================================================================
#ifndef MPPI_CRITICS_PATH_FOLLOW_CRITIC_HPP_
#define MPPI_CRITICS_PATH_FOLLOW_CRITIC_HPP_

#include <cmath>
#include <algorithm>

#include <xtensor/xtensor.hpp>

#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"
#include "tools/math_utils.hpp"

namespace mppi
{

/**
 * @brief 路径跟随代价：计算每条轨迹终点与路径上参考点之间的距离
 */
class PathFollowCritic : public CriticFunction
{
public:
    void initialize() override {}

    /**
     * @param weight 代价权重
     * @param offset 从最远路径点向前偏移的步数
     * @param threshold 距离阈值，当距离小于该阈值时不计算代价
     */
    void setParams(float weight, int offset, float threshold)
    {
        weight_ = weight;
        offset_from_furthest_ = offset;
        threshold_to_consider_ = threshold;
    }

    void score(CriticData & data) override
    {
        if (!enabled_ || data.path.size() < 2) return;
        Pose2D goal = data.path.getGoal();
        float dist_to_goal = std::hypot(data.state.pose.x - goal.x, data.state.pose.y - goal.y);
        if (dist_to_goal < threshold_to_consider_) return;

        const size_t batch_size = data.trajectories.x.shape(0);
        const size_t traj_len = data.trajectories.x.shape(1);

        // 路径已裁剪，使用furthest_reached_path_point + offset
        size_t ref_idx = data.path.size() - 1;
        if (data.furthest_reached_path_point) {
            ref_idx = std::min(*data.furthest_reached_path_point + offset_from_furthest_,
                               data.path.size() - 1);
        }

        // 跳过被障碍物阻挡的路径点
        ref_idx = findFirstValidPathPoint(data, ref_idx);

        float target_x = data.path.x(ref_idx);
        float target_y = data.path.y(ref_idx);

        for (size_t i = 0; i < batch_size; ++i) {
            float last_x = data.trajectories.x(i, traj_len - 1);
            float last_y = data.trajectories.y(i, traj_len - 1);
            float dist = std::hypot(last_x - target_x, last_y - target_y);
            data.costs(i) += weight_ * dist;
        }
    }

    /**
     * @brief 查找第一个有效的路径点（跳过被障碍物阻挡的点）
     */
    size_t findFirstValidPathPoint(CriticData & data, size_t start_idx)
    {
        const size_t path_size = data.path.size() - 1;
        size_t idx = std::min(start_idx, path_size);

        if (data.path_pts_valid && data.path_pts_valid->size() > idx) {
            while (idx < path_size - 1 && !(*data.path_pts_valid)[idx]) {
                idx++;
            }
        }
        else if (data.obstacles && !data.obstacles->empty()) {
            const float check_radius = 0.15f;
            while (idx < path_size - 1) {
                float px = data.path.x(idx);
                float py = data.path.y(idx);
                bool blocked = false;

                for (const auto& obs : *data.obstacles) {
                    float dist = std::hypot(px - obs.x, py - obs.y);
                    if (dist < check_radius) {
                        blocked = true;
                        break;
                    }
                }

                if (!blocked) break;
                idx++;
            }
        }

        return std::min(idx, path_size);
    }

private:
    float weight_ = 5.0f;
    float threshold_to_consider_ = 1.4f;
    int offset_from_furthest_ = 6;
};

} // namespace mppi

#endif // MPPI_CRITICS_PATH_FOLLOW_CRITIC_HPP_
