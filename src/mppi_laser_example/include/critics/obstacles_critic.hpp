// ============================================================================
// 文件：obstacles_critic.hpp
// 功能：障碍物代价函数，使用局部栅格距离场加速静态障碍物距离查询，
//       并支持动态障碍物预测和非圆形机器人足迹检测
// ============================================================================
#ifndef MPPI_CRITICS_OBSTACLES_CRITIC_HPP_
#define MPPI_CRITICS_OBSTACLES_CRITIC_HPP_

#include <queue>
#include <limits>
#include <algorithm>
#include <vector>

#include <xtensor/xtensor.hpp>

#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"
#include "models/types.hpp"
#include "tools/math_utils.hpp"

namespace mppi
{

/**
 * @brief 障碍物代价函数，使用局部栅格距离场加速静态障碍物距离查询，并支持动态障碍物预测
 */
class ObstaclesCritic : public CriticFunction
{
public:
    void initialize() override {}

    /**
     * @brief 从参数设置代价权重和栅格参数
     * @param repulsion_weight 排斥权重
     * @param collision_cost 碰撞代价
     * @param collision_margin 碰撞边界距离
     * @param inflation_radius 膨胀半径
     * @param cost_scaling 代价缩放因子
     * @param near_goal_distance 接近目标时的距离阈值
     * @param robot_radius 机器人半径
     * @param grid_resolution 栅格分辨率
     * @param grid_width 栅格宽度（像素）
     * @param grid_height 栅格高度（像素）
     * @param consider_footprint 是否考虑机器人足迹
     * @param footprint 机器人足迹点集合
     */
    void setParams(float repulsion_weight, float collision_cost,
                   float collision_margin, float inflation_radius, float cost_scaling,
                   float near_goal_distance, float robot_radius,
                   float grid_resolution, int grid_width, int grid_height,
                   bool consider_footprint = false,
                   const std::vector<Point2D>& footprint = {})
    {
        repulsion_weight_ = repulsion_weight;
        collision_cost_ = collision_cost;
        collision_margin_distance_ = collision_margin;
        inflation_radius_ = inflation_radius;
        cost_scaling_factor_ = cost_scaling;
        near_goal_distance_ = near_goal_distance;
        robot_radius_ = robot_radius;
        grid_resolution_ = grid_resolution;
        grid_width_ = grid_width;
        grid_height_ = grid_height;
        consider_footprint_ = consider_footprint;
        footprint_ = footprint;
        grid_distance_field_.resize(grid_width_ * grid_height_, std::numeric_limits<float>::max());
    }

    /**
     * @brief 设置机器人足迹（用于非圆形机器人碰撞检测）
     * @param footprint 足迹点集合（相对于机器人中心）
     */
    void setFootprint(const std::vector<Point2D>& footprint)
    {
        footprint_ = footprint;
    }

    /**
     * @brief 更新静态障碍物点云，并以当前机器人位置为中心重建局部栅格距离场
     * @param points 世界坐标系下的障碍物点集
     * @param robot_pose 当前机器人位姿（用于确定栅格中心）
     */
    void setLaserPoints(const std::vector<Point2D> & points, const Pose2D & robot_pose)
    {
        static_obstacles_ = points;
        rebuildGrid(robot_pose);
    }

    /**
     * @brief 设置动态障碍物列表指针
     * @param obstacles 动态障碍物预测列表
     */
    void setDynamicObstacles(const std::vector<ObstaclePrediction> & obstacles) { dynamic_obstacles_ = &obstacles; }

    /**
     * @brief 对每条轨迹逐点计算障碍物代价（静态 + 动态）
     */
    void score(CriticData & data) override;

private:
    /**
     * @brief 查询栅格距离（若超出范围返回一个大值）
     */
    float getGridDistance(float x, float y) const
    {
        int ix = static_cast<int>((x - grid_origin_x_) / grid_resolution_);
        int iy = static_cast<int>((y - grid_origin_y_) / grid_resolution_);
        if (ix < 0 || ix >= grid_width_ || iy < 0 || iy >= grid_height_)
            return std::numeric_limits<float>::max();
        return grid_distance_field_[iy * grid_width_ + ix];
    }

    /**
     * @brief 以机器人位置为中心重建局部栅格距离场（BFS 四邻域传播）
     */
    void rebuildGrid(const Pose2D & robot_pose)
    {
        grid_origin_x_ = robot_pose.x - grid_width_ * grid_resolution_ * 0.5f;
        grid_origin_y_ = robot_pose.y - grid_height_ * grid_resolution_ * 0.5f;

        std::fill(grid_distance_field_.begin(), grid_distance_field_.end(), std::numeric_limits<float>::max());

        // 标记障碍物栅格距离为0
        for (const auto & p : static_obstacles_)
        {
            int ix = static_cast<int>((p.x - grid_origin_x_) / grid_resolution_);
            int iy = static_cast<int>((p.y - grid_origin_y_) / grid_resolution_);
            if (ix >= 0 && ix < grid_width_ && iy >= 0 && iy < grid_height_)
                grid_distance_field_[iy * grid_width_ + ix] = 0.0f;
        }

        // BFS 传播距离（四邻域）
        std::queue<std::pair<int,int>> q;
        for (int i = 0; i < grid_height_; ++i)
            for (int j = 0; j < grid_width_; ++j)
                if (grid_distance_field_[i * grid_width_ + j] == 0.0f)
                    q.push({j, i});

        const int dx[4] = {1, 0, -1, 0};
        const int dy[4] = {0, 1, 0, -1};
        while (!q.empty())
        {
            auto [x, y] = q.front(); q.pop();
            float cur_dist = grid_distance_field_[y * grid_width_ + x];
            for (int k = 0; k < 4; ++k)
            {
                int nx = x + dx[k];
                int ny = y + dy[k];
                if (nx < 0 || nx >= grid_width_ || ny < 0 || ny >= grid_height_)
                    continue;
                float nd = cur_dist + grid_resolution_;
                if (nd < grid_distance_field_[ny * grid_width_ + nx])
                {
                    grid_distance_field_[ny * grid_width_ + nx] = nd;
                    q.push({nx, ny});
                }
            }
        }
    }

    /**
     * @brief 足迹检查结果结构
     */
    struct FootprintCheckResult {
        bool is_collision = false;
        float min_distance = std::numeric_limits<float>::max();
    };

    /**
     * @brief 检查机器人足迹并返回最小距离（非圆形机器人模式）
     */
    FootprintCheckResult checkFootprintWithDistance(float x, float y, float theta) const
    {
        FootprintCheckResult result;

        if (footprint_.empty()) {
            float dist = getGridDistance(x, y);
            result.min_distance = dist;
            result.is_collision = (dist < collision_margin_distance_);
            return result;
        }

        float cos_theta = std::cos(theta);
        float sin_theta = std::sin(theta);

        for (const auto& point : footprint_) {
            float world_x = x + point.x * cos_theta - point.y * sin_theta;
            float world_y = y + point.x * sin_theta + point.y * cos_theta;

            float dist = getGridDistance(world_x, world_y);
            if (dist < result.min_distance) {
                result.min_distance = dist;
            }

            if (dist < collision_margin_distance_) {
                result.is_collision = true;
            }
        }

        return result;
    }

    std::vector<Point2D> static_obstacles_;
    const std::vector<ObstaclePrediction> * dynamic_obstacles_ = nullptr;
    std::vector<Point2D> footprint_;

    float repulsion_weight_ = 2.0f;
    float collision_cost_ = 100000.0f;
    float collision_margin_distance_ = 0.15f;
    float inflation_radius_ = 1.0f;
    float cost_scaling_factor_ = 5.0f;
    float near_goal_distance_ = 0.5f;
    float robot_radius_ = 0.3f;
    bool consider_footprint_ = false;

    // 栅格参数
    float grid_resolution_ = 0.1f;
    int grid_width_ = 100;
    int grid_height_ = 100;
    float grid_origin_x_ = 0.0f;
    float grid_origin_y_ = 0.0f;
    std::vector<float> grid_distance_field_;
};

inline void ObstaclesCritic::score(CriticData & data)
{
    if (!enabled_) return;

    const size_t batch_size = data.trajectories.x.shape(0);
    const size_t traj_len = data.trajectories.x.shape(1);
    Pose2D goal = data.path.getGoal();
    float dist_to_goal = std::hypot(data.state.pose.x - goal.x, data.state.pose.y - goal.y);
    // 接近目标时跳过障碍物代价，便于最终收敛到目标点
    if (dist_to_goal < near_goal_distance_) return;

    int collision_count = 0;

    for (size_t i = 0; i < batch_size; ++i)
    {
        float traj_cost = 0.0f;
        bool trajectory_collide = false;

        for (size_t j = 0; j < traj_len; ++j)
        {
            float px = data.trajectories.x(i, j);
            float py = data.trajectories.y(i, j);
            float ptheta = data.trajectories.yaws(i, j);
            float pt = data.trajectories.times(i, j);

            if (consider_footprint_) {
                // 非圆形机器人 - 检查整个足迹
                FootprintCheckResult result = checkFootprintWithDistance(px, py, ptheta);

                if (result.is_collision) {
                    trajectory_collide = true;
                    break;
                }

                if (result.min_distance < inflation_radius_) {
                    float d = result.min_distance - collision_margin_distance_;
                    if (d > 0) {
                        float exp_cost = repulsion_weight_ * std::exp(-cost_scaling_factor_ * d);
                        traj_cost += exp_cost;
                    }
                }

                // 动态障碍物（足迹模式）
                if (data.dynamic_obstacles && !data.dynamic_obstacles->empty())
                {
                    for (const auto & obs : *data.dynamic_obstacles)
                    {
                        Point2D pred = obs.positionAt(pt);
                        float dist = std::hypot(px - pred.x, py - pred.y) - obs.radius - robot_radius_;
                        if (dist < collision_margin_distance_)
                        {
                            trajectory_collide = true;
                            break;
                        }
                        else if (dist < inflation_radius_)
                        {
                            float d = dist - collision_margin_distance_;
                            float exp_cost = repulsion_weight_ * std::exp(-cost_scaling_factor_ * d);
                            traj_cost += exp_cost;
                        }
                    }
                    if (trajectory_collide) break;
                }
            } else {
                // 圆形机器人 - 只检查中心点
                float static_dist = getGridDistance(px, py);
                float min_dist = static_dist;

                // 动态障碍物
                if (data.dynamic_obstacles && !data.dynamic_obstacles->empty())
                {
                    for (const auto & obs : *data.dynamic_obstacles)
                    {
                        Point2D pred = obs.positionAt(pt);
                        float dist = std::hypot(px - pred.x, py - pred.y) - obs.radius;
                        if (dist < min_dist) min_dist = dist;
                    }
                }

                min_dist -= robot_radius_;

                if (min_dist < collision_margin_distance_)
                {
                    trajectory_collide = true;
                }
                else if (min_dist < inflation_radius_)
                {
                    float d = min_dist - collision_margin_distance_;
                    float exp_cost = repulsion_weight_ * std::exp(-cost_scaling_factor_ * d);
                    traj_cost += exp_cost;
                }
            }

            if (trajectory_collide) {
                break;
            }
        }

        if (trajectory_collide) {
            traj_cost = collision_cost_;
            collision_count++;
        }

        data.costs(i) += traj_cost;
    }

    if (collision_count == static_cast<int>(batch_size)) {
        data.fail_flag = true;
    }
}

} // namespace mppi

#endif // MPPI_CRITICS_OBSTACLES_CRITIC_HPP_
