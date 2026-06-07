// ============================================================================
// 文件：esdf_critic.hpp
// 功能：基于 RC-ESDF (Robo-Centric ESDF) 的形状感知避障代价函数
//       支持任意多边形 footprint 的精确碰撞检测，解析梯度引导代价调制
//
// 初始化机制：
//   采用延迟构建模式 (Lazy Build)，ESDF 地图在所有前置条件满足后自动生成：
//   - initialize()    → 标记初始化请求，若前置条件已满足则立即构建
//   - setFootprint()  → 设置轮廓后，若已请求初始化且参数已就绪则自动构建
//   - setParams()     → 设置参数后，若已请求初始化且轮廓已就绪则自动构建
//   无论三者的调用顺序如何，ESDF 都会在条件齐备时自动生成，无需手动二次调用。
// ============================================================================
#ifndef MPPI_CRITICS_ESDF_CRITIC_HPP_
#define MPPI_CRITICS_ESDF_CRITIC_HPP_

#include <cmath>
#include <limits>
#include <algorithm>
#include <vector>
#include <queue>
#include <unordered_map>

#include <xtensor/xtensor.hpp>
#include <Eigen/Core>

#include "rc_esdf.h"
#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"
#include "models/types.hpp"

namespace mppi
{

class EsdfCritic : public CriticFunction
{
public:
    void initialize() override
    {
        init_requested_ = true;
        tryBuildEsdf();
    }

    void setParams(float weight, float collision_cost,
                   float d_hard, float d_margin, float cost_scaling,
                   float near_goal_distance,
                   double esdf_width, double esdf_height, double esdf_resolution,
                   float obstacle_range, int check_step,
                   float grad_weight = 0.5f,
                   float robot_radius = 0.25f,
                   float prefilter_threshold = 2.0f)
    {
        weight_ = weight;
        collision_cost_ = collision_cost;
        d_hard_ = d_hard;
        d_margin_ = d_margin;
        cost_scaling_ = cost_scaling;
        near_goal_distance_ = near_goal_distance;
        esdf_width_ = esdf_width;
        esdf_height_ = esdf_height;
        esdf_resolution_ = esdf_resolution;
        obstacle_range_ = obstacle_range;
        check_step_ = std::max(1, check_step);
        grad_weight_ = grad_weight;
        robot_radius_ = robot_radius;
        prefilter_threshold_ = prefilter_threshold;
        params_set_ = true;
        tryBuildEsdf();
    }

    void setFootprint(const std::vector<Point2D>& footprint)
    {
        footprint_eigen_.clear();
        footprint_eigen_.reserve(footprint.size());
        for (const auto& pt : footprint) {
            footprint_eigen_.emplace_back(pt.x, pt.y);
        }

        // 计算外接圆半径（回退用）
        fallback_radius_ = 0.0;
        for (const auto& pt : footprint) {
            float r = std::hypot(pt.x, pt.y);
            if (r > fallback_radius_) fallback_radius_ = r;
        }

        tryBuildEsdf();
    }

    void setObstacles(const std::vector<Point2D>& obstacles)
    {
        if (voxel_size_ <= 0.0f) {
            obstacles_ = obstacles;
            return;
        }

        obstacles_.clear();
        voxel_set_.clear();

        for (const auto& pt : obstacles) {
            int vx = static_cast<int>(std::floor(pt.x / voxel_size_));
            int vy = static_cast<int>(std::floor(pt.y / voxel_size_));
            uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(vx)) << 32)
                         | static_cast<uint64_t>(static_cast<uint32_t>(vy));

            auto it = voxel_set_.find(key);
            if (it == voxel_set_.end()) {
                voxel_set_[key] = obstacles_.size();
                obstacles_.push_back(pt);
            }
        }
    }

    void score(CriticData & data) override
    {
        if (!enabled_ || !esdf_initialized_) return;
        if (obstacles_.empty() && (!data.dynamic_obstacles || data.dynamic_obstacles->empty())) return;

        buildPreFilterGrid(data.state.pose.x, data.state.pose.y);

        const size_t batch_size = data.trajectories.x.shape(0);
        const size_t traj_len = data.trajectories.x.shape(1);

        Pose2D goal = data.path.getGoal();
        float dist_to_goal = std::hypot(data.state.pose.x - goal.x, data.state.pose.y - goal.y);
        if (dist_to_goal < near_goal_distance_) return;

        int collision_count = 0;

        for (size_t i = 0; i < batch_size; ++i)
        {
            float traj_cost = 0.0f;
            bool trajectory_collide = false;

            for (size_t j = 0; j < traj_len; j += static_cast<size_t>(check_step_))
            {
                float px = data.trajectories.x(i, j);
                float py = data.trajectories.y(i, j);
                float ptheta = data.trajectories.yaws(i, j);

                float grid_dist = getPreFilterDistance(px, py);
                if (grid_dist > prefilter_threshold_ + robot_radius_) continue;

                float cos_theta = std::cos(ptheta);
                float sin_theta = std::sin(ptheta);

                double min_dist = std::numeric_limits<double>::max();
                Eigen::Vector2d worst_grad = Eigen::Vector2d::Zero();

                for (const auto& obs : obstacles_)
                {
                    float dx = obs.x - px;
                    float dy = obs.y - py;

                    if (dx * dx + dy * dy > obstacle_range_sq_) continue;

                    double body_x = dx * cos_theta + dy * sin_theta;
                    double body_y = -dx * sin_theta + dy * cos_theta;

                    double dist;
                    Eigen::Vector2d grad;
                    if (esdf_map_.query(Eigen::Vector2d(body_x, body_y), dist, grad)) {
                        // ESDF query succeeded — use precise distance + gradient
                    } else {
                        // 回退到外接圆半径（而非内切圆），保守估计碰撞风险
                        dist = std::hypot(body_x, body_y) - fallback_radius_;
                        grad.setZero();
                    }

                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        worst_grad = grad;
                    }

                    if (dist < d_hard_)
                    {
                        trajectory_collide = true;
                        break;
                    }
                }

                if (!trajectory_collide && data.dynamic_obstacles && !data.dynamic_obstacles->empty())
                {
                    float pt = data.trajectories.times(i, j);
                    for (const auto& dyn_obs : *data.dynamic_obstacles)
                    {
                        Point2D pred = dyn_obs.positionAt(pt);
                        float dx = pred.x - px;
                        float dy = pred.y - py;
                        if (dx * dx + dy * dy > obstacle_range_sq_) continue;

                        double body_x = dx * cos_theta + dy * sin_theta;
                        double body_y = -dx * sin_theta + dy * cos_theta;

                        double dist;
                        Eigen::Vector2d grad;
                        if (esdf_map_.query(Eigen::Vector2d(body_x, body_y), dist, grad)) {
                            dist -= dyn_obs.radius;
                        } else {
                            dist = std::hypot(body_x, body_y) - fallback_radius_ - dyn_obs.radius;
                            grad.setZero();
                        }

                        if (dist < min_dist)
                        {
                            min_dist = dist;
                            worst_grad = grad;
                        }

                        if (dist < d_hard_)
                        {
                            trajectory_collide = true;
                            break;
                        }
                    }
                }

                if (trajectory_collide) break;

                if (min_dist < d_margin_ && min_dist >= d_hard_)
                {
                    double d = min_dist - d_hard_;
                    double penalty = weight_ * std::exp(-cost_scaling_ * d);

                    float vx_body = data.state.vx(i, j);
                    float vy_body = data.state.vy(i, j);
                    double vel_norm = std::hypot(vx_body, vy_body);

                    double grad_norm = worst_grad.norm();
                    if (vel_norm > 1e-6 && grad_norm > 1e-6)
                    {
                        // 梯度指向"障碍物远离机器人轮廓"的方向（ESDF距离增大的方向）
                        // 机器人速度 vx_body/vy_body 是机器人大身运动，取反方向才是
                        // 障碍物在Body Frame中的视在运动方向
                        //   dim-  如果机器人向前(vx>0)，障碍物在Body Frame中向后(-dx)
                        //         即障碍物向机器人轮廓靠近 → ESDF距离减小 → 更危险
                        //   curve  所以alignment = -(v_body · grad)：
                        //   dim-  值>0 表示机器人运动方向与逃逸方向一致→远离碰撞
                        //   curve       值<0 表示机器人向障碍物方向运动→靠近碰撞
                        double alignment = -(vx_body * worst_grad.x() + vy_body * worst_grad.y())
                                          / (vel_norm * grad_norm);
                        alignment = std::clamp(alignment, -1.0, 1.0);

                        double modulation = 1.0 - grad_weight_ * alignment;
                        if (modulation < 0.1) modulation = 0.1;
                        penalty *= modulation;
                    }

                    traj_cost += static_cast<float>(penalty);
                }
            }

            if (trajectory_collide)
            {
                traj_cost = collision_cost_;
                collision_count++;
            }

            data.costs(i) += traj_cost;
        }

        if (collision_count == static_cast<int>(batch_size))
        {
            data.fail_flag = true;
        }
    }

    bool isEsdfInitialized() const { return esdf_initialized_; }

    void setVoxelSize(float voxel_size) { voxel_size_ = voxel_size; }
    void setRobotRadius(float radius) { robot_radius_ = radius; }

private:
    bool canBuildEsdf() const
    {
        return init_requested_ && !footprint_eigen_.empty() && params_set_;
    }

    void tryBuildEsdf()
    {
        if (esdf_initialized_ || !canBuildEsdf()) return;

        esdf_map_.initialize(esdf_width_, esdf_height_, esdf_resolution_);
        esdf_map_.generateFromPolygon(footprint_eigen_);
        esdf_initialized_ = true;

        obstacle_range_sq_ = obstacle_range_ * obstacle_range_;

        prefilter_grid_.resize(static_cast<size_t>(prefilter_w_) * prefilter_h_,
                               std::numeric_limits<float>::max());
    }

    void buildPreFilterGrid(float robot_x, float robot_y)
    {
        prefilter_ox_ = robot_x - prefilter_w_ * prefilter_res_ * 0.5f;
        prefilter_oy_ = robot_y - prefilter_h_ * prefilter_res_ * 0.5f;

        std::fill(prefilter_grid_.begin(), prefilter_grid_.end(),
                  std::numeric_limits<float>::max());

        for (const auto& obs : obstacles_)
        {
            int ix = static_cast<int>((obs.x - prefilter_ox_) / prefilter_res_);
            int iy = static_cast<int>((obs.y - prefilter_oy_) / prefilter_res_);
            if (ix >= 0 && ix < prefilter_w_ && iy >= 0 && iy < prefilter_h_)
                prefilter_grid_[static_cast<size_t>(iy) * prefilter_w_ + ix] = 0.0f;
        }

        std::queue<std::pair<int,int>> q;
        for (int iy = 0; iy < prefilter_h_; ++iy)
            for (int ix = 0; ix < prefilter_w_; ++ix)
                if (prefilter_grid_[static_cast<size_t>(iy) * prefilter_w_ + ix] == 0.0f)
                    q.push({ix, iy});

        static const int dx[4] = {1, 0, -1, 0};
        static const int dy[4] = {0, 1, 0, -1};
        while (!q.empty())
        {
            auto [x, y] = q.front(); q.pop();
            float cur_dist = prefilter_grid_[static_cast<size_t>(y) * prefilter_w_ + x];
            for (int k = 0; k < 4; ++k)
            {
                int nx = x + dx[k];
                int ny = y + dy[k];
                if (nx < 0 || nx >= prefilter_w_ || ny < 0 || ny >= prefilter_h_)
                    continue;
                float nd = cur_dist + prefilter_res_;
                size_t idx = static_cast<size_t>(ny) * prefilter_w_ + nx;
                if (nd < prefilter_grid_[idx])
                {
                    prefilter_grid_[idx] = nd;
                    q.push({nx, ny});
                }
            }
        }
    }

    float getPreFilterDistance(float wx, float wy) const
    {
        int ix = static_cast<int>((wx - prefilter_ox_) / prefilter_res_);
        int iy = static_cast<int>((wy - prefilter_oy_) / prefilter_res_);
        if (ix < 0 || ix >= prefilter_w_ || iy < 0 || iy >= prefilter_h_)
            return std::numeric_limits<float>::max();
        return prefilter_grid_[static_cast<size_t>(iy) * prefilter_w_ + ix];
    }

    RcEsdfMap esdf_map_;
    std::vector<Eigen::Vector2d> footprint_eigen_;
    std::vector<Point2D> obstacles_;
    std::unordered_map<uint64_t, size_t> voxel_set_;

    std::vector<float> prefilter_grid_;
    float prefilter_res_ = 0.1f;
    int prefilter_w_ = 60;
    int prefilter_h_ = 60;
    float prefilter_ox_ = 0.0f;
    float prefilter_oy_ = 0.0f;

    float weight_ = 5.0f;
    float collision_cost_ = 100000.0f;
    float d_hard_ = 0.0f;
    float d_margin_ = 0.3f;
    float cost_scaling_ = 5.0f;
    float near_goal_distance_ = 0.3f;
    float obstacle_range_ = 5.0f;
    float obstacle_range_sq_ = 25.0f;
    int check_step_ = 1;
    float voxel_size_ = 0.1f;
    float grad_weight_ = 0.5f;
    float robot_radius_ = 0.25f;
    float fallback_radius_ = 0.375f;  // 外接圆半径，ESDF查询越界时保守回退
    float prefilter_threshold_ = 2.0f;

    double esdf_width_ = 10.0;
    double esdf_height_ = 10.0;
    double esdf_resolution_ = 0.05;

    bool esdf_initialized_ = false;
    bool init_requested_ = false;
    bool params_set_ = false;
};

} // namespace mppi

#endif // MPPI_CRITICS_ESDF_CRITIC_HPP_
