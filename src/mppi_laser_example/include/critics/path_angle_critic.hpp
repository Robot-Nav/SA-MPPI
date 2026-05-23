// ============================================================================
// 文件：path_angle_critic.hpp
// 功能：路径角度代价，惩罚轨迹终点与路径参考点之间的朝向偏差
//       支持多种角度模式：偏好前进、无偏好、考虑路径朝向
// ============================================================================
#ifndef MPPI_CRITICS_PATH_ANGLE_CRITIC_HPP_
#define MPPI_CRITICS_PATH_ANGLE_CRITIC_HPP_

#include <cmath>
#include <algorithm>

#include <xtensor/xtensor.hpp>

#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"
#include "tools/math_utils.hpp"

namespace mppi
{

/**
 * @brief 路径角度模式枚举
 */
enum class PathAngleMode {
    FORWARD_PREFERENCE = 0,              // 偏好前进方向
    NO_DIRECTIONAL_PREFERENCE = 1,       // 无方向偏好(允许倒车)
    CONSIDER_FEASIBLE_PATH_ORIENTATIONS = 2  // 考虑路径朝向
};

/**
 * @brief 路径角度代价：惩罚轨迹终点与路径参考点之间的朝向偏差
 */
class PathAngleCritic : public CriticFunction
{
public:
    void initialize() override {}

    /**
     * @param weight 代价权重
     * @param offset 从最远路径点向前偏移的步数
     * @param threshold 距离阈值
     * @param max_angle 最大允许角度偏差（超过此值才惩罚）
     * @param mode 路径角度模式（0=偏好前进，1=无偏好，2=考虑路径朝向）
     */
    void setParams(float weight, int offset, float threshold, float max_angle, int mode = 0)
    {
        weight_ = weight;
        offset_from_furthest_ = offset;
        threshold_to_consider_ = threshold;
        max_angle_ = max_angle;
        mode_ = static_cast<PathAngleMode>(mode);
    }

    void score(CriticData & data) override
    {
        if (!enabled_ || data.path.size() < 2) return;

        Pose2D goal = data.path.getGoal();
        float dist_to_goal = std::hypot(data.state.pose.x - goal.x, data.state.pose.y - goal.y);
        if (dist_to_goal < threshold_to_consider_) return;

        const size_t batch_size = data.trajectories.yaws.shape(0);
        const size_t traj_len = data.trajectories.yaws.shape(1);

        size_t path_end_idx = data.path.size() - 1;
        if (data.furthest_reached_path_point) {
            path_end_idx = std::min(*data.furthest_reached_path_point + offset_from_furthest_,
                                    data.path.size() - 1);
        }

        float target_x = data.path.x(path_end_idx);
        float target_y = data.path.y(path_end_idx);

        for (size_t i = 0; i < batch_size; ++i) {
            float last_x = data.trajectories.x(i, traj_len - 1);
            float last_y = data.trajectories.y(i, traj_len - 1);
            float last_yaw = data.trajectories.yaws(i, traj_len - 1);

            float angle_to_target = std::atan2(target_y - last_y, target_x - last_x);
            float angle_diff = 0.0f;

            switch (mode_) {
                case PathAngleMode::FORWARD_PREFERENCE:
                    angle_diff = std::abs(shortestAngularDistance(last_yaw, angle_to_target));
                    break;

                case PathAngleMode::NO_DIRECTIONAL_PREFERENCE:
                    {
                        float diff1 = std::abs(shortestAngularDistance(last_yaw, angle_to_target));
                        float diff2 = std::abs(shortestAngularDistance(last_yaw, normalizeAngle(angle_to_target + PI)));
                        angle_diff = std::min(diff1, diff2);
                    }
                    break;

                case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS:
                    {
                        float avg_vx = 0.0f;
                        for (size_t j = 0; j < traj_len; ++j) {
                            avg_vx += data.state.cvx(i, j);
                        }
                        avg_vx /= traj_len;

                        float effective_angle_to_target = (avg_vx < 0.0f) ?
                            normalizeAngle(angle_to_target + PI) : angle_to_target;
                        angle_diff = std::abs(shortestAngularDistance(last_yaw, effective_angle_to_target));
                    }
                    break;
            }

            if (angle_diff > max_angle_) {
                data.costs(i) += weight_ * angle_diff;
            }
        }
    }

private:
    float weight_ = 2.0f;
    float threshold_to_consider_ = 0.4f;
    float max_angle_ = 0.8f;
    int offset_from_furthest_ = 4;
    PathAngleMode mode_ = PathAngleMode::FORWARD_PREFERENCE;
};

} // namespace mppi

#endif // MPPI_CRITICS_PATH_ANGLE_CRITIC_HPP_
