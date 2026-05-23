// ============================================================================
// 文件：path_align_critic.hpp
// 功能：路径对齐代价，计算轨迹上每个点与路径上最近点距离的平均值
//       支持路径阻塞检测和路径朝向考虑
// ============================================================================
#ifndef MPPI_CRITICS_PATH_ALIGN_CRITIC_HPP_
#define MPPI_CRITICS_PATH_ALIGN_CRITIC_HPP_

#include <cmath>
#include <vector>
#include <algorithm>

#include <xtensor/xtensor.hpp>

#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"
#include "tools/math_utils.hpp"

namespace mppi
{

/**
 * @brief 路径对齐代价：计算轨迹上每个点与路径上最近点的距离的平均值
 * 使用路径点缓存减少搜索范围，支持路径阻塞检测和路径朝向考虑
 */
class PathAlignCritic : public CriticFunction
{
public:
    void initialize() override {}

    /**
     * @param weight 代价权重
     * @param offset 从最远路径点向前偏移的步数
     * @param threshold 距离阈值
     * @param step 轨迹点采样步长（每隔 step 个点计算一次）
     * @param max_path_occupancy_ratio 路径阻塞比例阈值
     * @param use_path_orientations 是否考虑路径朝向
     */
    void setParams(float weight, int offset, float threshold, int step,
                   float max_path_occupancy_ratio = 0.07f,
                   bool use_path_orientations = false)
    {
        weight_ = weight;
        offset_from_furthest_ = offset;
        threshold_to_consider_ = threshold;
        traj_point_step_ = step;
        max_path_occupancy_ratio_ = max_path_occupancy_ratio;
        use_path_orientations_ = use_path_orientations;
    }

    /**
     * @brief 设置障碍物点用于路径阻塞检测
     */
    void setObstacles(const std::vector<Point2D>* obstacles, float check_radius = 0.15f)
    {
        obstacles_ = obstacles;
        obstacle_check_radius_ = check_radius;
    }

    void score(CriticData & data) override
    {
        if (!enabled_ || data.path.size() < 2) return;

        // 检查路径是否被阻塞
        float occupancy_ratio = computePathOccupancyRatio(data.path);
        if (occupancy_ratio > max_path_occupancy_ratio_) return;

        Pose2D goal = data.path.getGoal();
        float dist_to_goal = std::hypot(data.state.pose.x - goal.x, data.state.pose.y - goal.y);
        if (dist_to_goal < threshold_to_consider_) return;

        const size_t batch_size = data.trajectories.x.shape(0);
        const size_t traj_len = data.trajectories.x.shape(1);

        // 路径已裁剪，索引0对应机器人当前位置
        size_t path_segments_count = data.path.size() - 1;
        if (data.furthest_reached_path_point) {
            path_segments_count = *data.furthest_reached_path_point;
        }

        if (path_segments_count < static_cast<size_t>(offset_from_furthest_)) return;

        // 计算路径累计距离
        std::vector<float> path_integrated_distances(path_segments_count, 0.0f);
        for (size_t i = 1; i < path_segments_count; ++i) {
            float dx = data.path.x(i) - data.path.x(i - 1);
            float dy = data.path.y(i) - data.path.y(i - 1);
            path_integrated_distances[i] = path_integrated_distances[i - 1] + std::hypot(dx, dy);
        }

        for (size_t i = 0; i < batch_size; ++i) {
            float cost = 0.0f;
            int count = 0;
            float traj_integrated_distance = 0.0f;
            size_t path_pt = 0;

            for (size_t j = 0; j < traj_len; j += traj_point_step_) {
                float px = data.trajectories.x(i, j);
                float py = data.trajectories.y(i, j);

                if (j > 0) {
                    float prev_x = data.trajectories.x(i, j - traj_point_step_);
                    float prev_y = data.trajectories.y(i, j - traj_point_step_);
                    traj_integrated_distance += std::hypot(px - prev_x, py - prev_y);
                }

                path_pt = findClosestPathPointByDistance(path_integrated_distances, traj_integrated_distance, path_pt);

                if (path_pt >= path_segments_count) path_pt = path_segments_count - 1;

                float path_x = data.path.x(path_pt);
                float path_y = data.path.y(path_pt);
                float point_dist = std::hypot(px - path_x, py - path_y);

                if (use_path_orientations_) {
                    float traj_yaw = data.trajectories.yaws(i, j);
                    float path_yaw = data.path.yaws(path_pt);
                    float angle_diff = std::abs(shortestAngularDistance(traj_yaw, path_yaw));
                    point_dist *= (1.0f + 0.5f * angle_diff);
                }

                cost += point_dist;
                count++;
            }
            if (count > 0) data.costs(i) += weight_ * (cost / count);
        }
    }

    /**
     * @brief 计算路径被障碍物占据的比例
     */
    float computePathOccupancyRatio(const Path& path) const
    {
        if (!obstacles_ || obstacles_->empty() || path.empty()) return 0.0f;

        int occupied_count = 0;
        for (size_t i = 0; i < path.size(); ++i) {
            for (const auto& obs : *obstacles_) {
                float dist = std::hypot(path.x(i) - obs.x, path.y(i) - obs.y);
                if (dist < obstacle_check_radius_) {
                    occupied_count++;
                    break;
                }
            }
        }

        return static_cast<float>(occupied_count) / path.size();
    }

    /**
     * @brief 基于累计距离找到最近的路径点
     */
    size_t findClosestPathPointByDistance(const std::vector<float>& path_distances,
                                          float target_distance,
                                          size_t start_idx) const
    {
        size_t closest_idx = start_idx;
        float min_diff = std::abs(path_distances[start_idx] - target_distance);

        for (size_t i = start_idx + 1; i < path_distances.size(); ++i) {
            float diff = std::abs(path_distances[i] - target_distance);
            if (diff < min_diff) {
                min_diff = diff;
                closest_idx = i;
            } else if (diff > min_diff) {
                break;
            }
        }

        return closest_idx;
    }

private:
    float weight_ = 14.0f;
    float threshold_to_consider_ = 0.4f;
    int offset_from_furthest_ = 20;
    int traj_point_step_ = 4;
    float max_path_occupancy_ratio_ = 0.07f;
    bool use_path_orientations_ = false;
    const std::vector<Point2D>* obstacles_ = nullptr;
    float obstacle_check_radius_ = 0.15f;
};

} // namespace mppi

#endif // MPPI_CRITICS_PATH_ALIGN_CRITIC_HPP_
