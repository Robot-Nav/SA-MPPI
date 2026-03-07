// ============================================================================
// 文件：mppi_controller.hpp
// 功能：纯C++版MPPI控制器 (工业级完善版) – 集成栅格化障碍物距离场
// 版本：适配 xtensor 0.21.0
// 特性：加速度限制、动态障碍物预测、多线程优化、数值稳定算法、连续代价函数
// ============================================================================
#ifndef MPPI_CONTROLLER_HPP_
#define MPPI_CONTROLLER_HPP_

#include <cmath>
#include <vector>
#include <array>
#include <memory>
#include <string>
#include <functional>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <random>
#include <iostream>
#include <optional>
#include <atomic>
#include <future>
#include <queue>
#include <deque>

#include <xtensor/xtensor.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xsort.hpp>

namespace mppi
{

// ==================== 常量与基础数学工具 ====================
constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;
constexpr float EPSILON = 1e-6f;

/**
 * @brief 将角度规范化到 [-PI, PI] 区间
 * @param angle 输入角度（弧度）
 * @return 规范化后的角度
 */
inline float normalizeAngle(float angle)
{
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

/**
 * @brief 计算两个角度之间的最短有符号距离
 * @param from 起始角度
 * @param to 目标角度
 * @return 从 from 到 to 的最短角度差（弧度，范围 [-PI, PI]）
 */
inline float shortestAngularDistance(float from, float to)
{
    return normalizeAngle(to - from);
}

/**
 * @brief 将值限制在指定范围内
 * @param val 输入值
 * @param min_val 最小值
 * @param max_val 最大值
 * @return 限制后的值
 */
inline float clamp(float val, float min_val, float max_val)
{
    return std::max(min_val, std::min(val, max_val));
}

// ==================== 数据结构定义 ====================

/**
 * @brief 二维点结构
 */
struct Point2D
{
    float x = 0.0f;
    float y = 0.0f;
    Point2D() = default;
    Point2D(float x_, float y_) : x(x_), y(y_) {}
};

/**
 * @brief 二维位姿（位置 + 朝向）
 */
struct Pose2D
{
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
    Pose2D() = default;
    Pose2D(float x_, float y_, float theta_) : x(x_), y(y_), theta(theta_) {}
};

/**
 * @brief 二维速度（线速度 + 角速度）
 */
struct Twist2D
{
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    Twist2D() = default;
    Twist2D(float vx_, float vy_, float wz_) : vx(vx_), vy(vy_), wz(wz_) {}
};

/**
 * @brief 控制量约束
 */
struct ControlConstraints
{
    float vx_max = 0.5f;
    float vx_min = -0.35f;
    float vy_max = 0.5f;
    float wz_max = 1.9f;
    // 加速度限制
    float ax_max = 2.0f;
    float ay_max = 2.0f;
    float az_max = 3.0f;
    // 碰撞代价阈值（用于判断最优轨迹是否发生碰撞）
    // 注意：此值应低于 obstacle_collision_cost (默认10000.0)，建议设为5000.0
    float collision_cost_threshold = 5000.0f;
};

/**
 * @brief 采样噪声标准差
 */
struct SamplingStd
{
    float vx = 0.2f;
    float vy = 0.2f;
    float wz = 0.4f;
};

/**
 * @brief 优化器配置参数
 */
struct OptimizerSettings
{
    ControlConstraints base_constraints;
    ControlConstraints constraints;
    SamplingStd sampling_std;
    float model_dt = 0.05f;
    float temperature = 0.3f;
    float gamma = 0.015f;
    unsigned int batch_size = 1000;
    unsigned int time_steps = 56;
    unsigned int iteration_count = 1;
    bool shift_control_sequence = false;
    size_t retry_attempt_limit = 1;
    unsigned int thread_count = 4; // 并行线程数

    // 阻塞检测参数（当所有轨迹都被障碍物阻挡时，输出零速度而非原地旋转）
    float spinning_ratio_threshold = 5.0f;  // wz/vx比率阈值：超此值视为打转（如wz=1.5,vx=0.1,比率=15>>5）
    int spinning_detect_frames = 2;         // 连续检测帧数：连续N帧打转才触发阻塞（2帧=100ms即可确认）
    float twirling_weight = 20.0f;          // TwirlingCritic权重
    
    // 自适应温度参数
    bool adaptive_temperature = false;       // 是否启用自适应温度
    float adaptive_temperature_min = 0.1f;   // 自适应温度下限
    float adaptive_temperature_max = 1.0f;   // 自适应温度上限
    
    // 归一化模式
    bool use_mean_normalization = false;     // 是否使用均值归一化（默认使用min归一化）
    
    // Savitzky-Golay 滤波器参数
    bool use_sg_filter = false;              // 是否启用 Savitzky-Golay 滤波
};

/**
 * @brief 单步控制量
 */
struct Control
{
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
};

/**
 * @brief 控制序列（每个时间步的控制量）
 */
struct ControlSequence
{
    xt::xtensor<float, 1> vx;
    xt::xtensor<float, 1> vy;
    xt::xtensor<float, 1> wz;

    /**
     * @brief 重置控制序列大小并清零
     * @param time_steps 时间步数
     */
    void reset(unsigned int time_steps)
    {
        vx = xt::zeros<float>({time_steps});
        vy = xt::zeros<float>({time_steps});
        wz = xt::zeros<float>({time_steps});
    }
};

/**
 * @brief 状态容器，存储所有采样轨迹的控制量和速度
 */
struct State
{
    xt::xtensor<float, 2> vx;   ///< 速度 x (batch, time)
    xt::xtensor<float, 2> vy;   ///< 速度 y (batch, time)
    xt::xtensor<float, 2> wz;   ///< 角速度 (batch, time)
    xt::xtensor<float, 2> cvx;  ///< 含噪声的控制量 vx
    xt::xtensor<float, 2> cvy;  ///< 含噪声的控制量 vy
    xt::xtensor<float, 2> cwz;  ///< 含噪声的控制量 wz
    Pose2D pose;                 ///< 当前机器人位姿
    Twist2D speed;               ///< 当前机器人速度

    /**
     * @brief 重置状态容器大小并清零
     * @param batch_size 采样批次数
     * @param time_steps 时间步数
     */
    void reset(unsigned int batch_size, unsigned int time_steps)
    {
        vx = xt::zeros<float>({batch_size, time_steps});
        vy = xt::zeros<float>({batch_size, time_steps});
        wz = xt::zeros<float>({batch_size, time_steps});
        cvx = xt::zeros<float>({batch_size, time_steps});
        cvy = xt::zeros<float>({batch_size, time_steps});
        cwz = xt::zeros<float>({batch_size, time_steps});
    }
};

/**
 * @brief 轨迹容器，存储所有采样轨迹的位置和朝向
 */
struct Trajectories
{
    xt::xtensor<float, 2> x;      ///< x 坐标 (batch, time)
    xt::xtensor<float, 2> y;      ///< y 坐标 (batch, time)
    xt::xtensor<float, 2> yaws;   ///< 朝向角 (batch, time)
    xt::xtensor<float, 2> times;  ///< 每个点的时间戳 (batch, time)

    /**
     * @brief 重置轨迹容器大小并清零
     * @param batch_size 采样批次数
     * @param time_steps 时间步数
     */
    void reset(unsigned int batch_size, unsigned int time_steps)
    {
        x = xt::zeros<float>({batch_size, time_steps});
        y = xt::zeros<float>({batch_size, time_steps});
        yaws = xt::zeros<float>({batch_size, time_steps});
        times = xt::zeros<float>({batch_size, time_steps});
    }
};

/**
 * @brief 全局路径（离散点序列）
 */
struct Path
{
    xt::xtensor<float, 1> x;      ///< x 坐标
    xt::xtensor<float, 1> y;      ///< y 坐标
    xt::xtensor<float, 1> yaws;   ///< 朝向角

    /**
     * @brief 重置路径大小并清零
     * @param size 路径点数
     */
    void reset(unsigned int size)
    {
        x = xt::zeros<float>({size});
        y = xt::zeros<float>({size});
        yaws = xt::zeros<float>({size});
    }

    /** @return 路径点数 */
    size_t size() const { return x.shape(0); }
    /** @return 路径是否为空 */
    bool empty() const { return x.shape(0) == 0; }

    /**
     * @brief 获取路径终点位姿
     * @return 终点位姿（若路径为空则返回原点）
     */
    Pose2D getGoal() const
    {
        if (empty()) return Pose2D();
        size_t last = size() - 1;
        return Pose2D(x(last), y(last), yaws(last));
    }
};

/**
 * @brief 动态障碍物预测结构
 */
struct ObstaclePrediction
{
    float x, y;           // 初始位置
    float vx, vy;         // 速度
    float radius;         // 膨胀半径

    /**
     * @brief 获取某时刻的位置
     * @param t 时间
     * @return 预测位置
     */
    Point2D positionAt(float t) const {
        return Point2D(x + vx * t, y + vy * t);
    }
};

/**
 * @brief 传递给代价函数的数据集合
 */
struct CriticData
{
    const State & state;                        ///< 当前状态
    const Trajectories & trajectories;          ///< 采样轨迹
    const Path & path;                          ///< 全局路径
    xt::xtensor<float, 1> & costs;              ///< 代价向量（将被修改）
    float & model_dt;                            ///< 模型时间步长
    bool fail_flag = false;                      ///< 是否失败（可由代价函数设置）
    std::shared_ptr<class MotionModel> motion_model; ///< 运动模型
    std::optional<size_t> furthest_reached_path_point; ///< 路径上最远已到达点索引
    const std::vector<ObstaclePrediction> * dynamic_obstacles = nullptr; ///< 动态障碍物列表
};

// ==================== 运动模型基类及派生类 ====================

/**
 * @brief 运动模型基类，定义运动学预测接口
 */
class MotionModel
{
public:
    MotionModel() = default;
    virtual ~MotionModel() = default;

    /**
     * @brief 根据控制量预测速度（包含加速度约束）
     * @param state 状态容器，其中 cvx/cvy/cwz 为含噪声控制量，本函数将 vx/vy/wz 后移一位
     */
    virtual void predict(State & state)
    {
        using namespace xt::placeholders;
        
        // 第一时刻使用当前速度
        for (size_t i = 0; i < state.vx.shape(0); ++i) {
            state.vx(i, 0) = state.speed.vx;
            state.wz(i, 0) = state.speed.wz;
            if (isHolonomic()) {
                state.vy(i, 0) = state.speed.vy;
            }
        }
        
        // 后续时刻：应用加速度约束的速度传播
        for (size_t i = 0; i < state.vx.shape(0); ++i) {
            for (size_t j = 1; j < state.vx.shape(1); ++j) {
                // 获取上一时刻的速度
                float prev_vx = state.vx(i, j - 1);
                float prev_vy = isHolonomic() ? state.vy(i, j - 1) : 0.0f;
                float prev_wz = state.wz(i, j - 1);
                
                // 获取当前时刻的控制量
                float ctrl_vx = state.cvx(i, j - 1);
                float ctrl_vy = isHolonomic() ? state.cvy(i, j - 1) : 0.0f;
                float ctrl_wz = state.cwz(i, j - 1);
                
                // 应用加速度约束
                state.vx(i, j) = applyAccelerationConstraint(ctrl_vx, prev_vx, ax_max_, model_dt_);
                state.wz(i, j) = applyAccelerationConstraint(ctrl_wz, prev_wz, az_max_, model_dt_);
                if (isHolonomic()) {
                    state.vy(i, j) = applyAccelerationConstraint(ctrl_vy, prev_vy, ay_max_, model_dt_);
                }
            }
        }
    }

    /** @return 是否为全向移动模型 */
    virtual bool isHolonomic() const = 0;

    /**
     * @brief 对含噪声控制量施加运动学约束（如最小转弯半径）
     * @param cvx 控制量 vx (batch, time)
     * @param cvy 控制量 vy (batch, time)
     * @param cwz 控制量 wz (batch, time)
     */
    virtual void applyConstraints(xt::xtensor<float, 2> & /*cvx*/,
                                   xt::xtensor<float, 2> & /*cvy*/,
                                   xt::xtensor<float, 2> & /*cwz*/) {}
    
    /**
     * @brief 设置加速度约束参数
     */
    void setAccelerationConstraints(float ax_max, float ay_max, float az_max, float model_dt)
    {
        ax_max_ = ax_max;
        ay_max_ = ay_max;
        az_max_ = az_max;
        model_dt_ = model_dt;
    }

protected:
    /**
     * @brief 应用加速度约束
     * @param desired 期望速度
     * @param current 当前速度
     * @param max_accel 最大加速度
     * @param dt 时间步长
     * @return 约束后的速度
     */
    float applyAccelerationConstraint(float desired, float current, float max_accel, float dt) const
    {
        float max_delta = max_accel * dt;
        float delta = desired - current;
        delta = clamp(delta, -max_delta, max_delta);
        return current + delta;
    }
    
    float ax_max_ = 2.0f;   // 最大线加速度
    float ay_max_ = 2.0f;   // 最大横向加速度
    float az_max_ = 3.0f;   // 最大角加速度
    float model_dt_ = 0.05f; // 时间步长
};

/**
 * @brief 差速运动模型（非全向）
 */
class DiffDriveMotionModel : public MotionModel
{
public:
    DiffDriveMotionModel() = default;
    bool isHolonomic() const override { return false; }
};

/**
 * @brief 全向运动模型
 */
class OmniMotionModel : public MotionModel
{
public:
    OmniMotionModel() = default;
    bool isHolonomic() const override { return true; }
};

/**
 * @brief 阿克曼运动模型（非全向，带最小转弯半径约束）
 */
class AckermannMotionModel : public MotionModel
{
public:
    /**
     * @brief 构造函数
     * @param min_turning_r 最小转弯半径
     */
    explicit AckermannMotionModel(float min_turning_r = 0.2f)
        : min_turning_r_(min_turning_r) {}

    bool isHolonomic() const override { return false; }

    /**
     * @brief 施加最小转弯半径约束：当 vx 非零且 wz 非零时，若半径小于阈值，则调整 wz 使其满足最小半径
     * @param cvx 控制量 vx
     * @param cvy 控制量 vy（阿克曼模型中 vy 通常为 0，这里忽略）
     * @param cwz 控制量 wz
     */
    void applyConstraints(xt::xtensor<float, 2> & cvx,
                          xt::xtensor<float, 2> & /*cvy*/,
                          xt::xtensor<float, 2> & cwz) override
    {
        for (size_t i = 0; i < cvx.shape(0); ++i) {
            for (size_t j = 0; j < cvx.shape(1); ++j) {
                float vx_val = cvx(i, j);
                float wz_val = cwz(i, j);
                if (std::abs(vx_val) > EPSILON && std::abs(wz_val) > EPSILON) {
                    float radius = std::abs(vx_val / wz_val);
                    if (radius < min_turning_r_) {
                        cwz(i, j) = (wz_val > 0 ? 1.0f : -1.0f) * std::abs(vx_val) / min_turning_r_;
                    }
                }
            }
        }
    }

private:
    float min_turning_r_{0.2f};
};

// ==================== 代价函数基类 ====================

/**
 * @brief 代价函数基类，所有具体代价函数需继承并实现 score 方法
 */
class CriticFunction
{
public:
    CriticFunction() = default;
    virtual ~CriticFunction() = default;

    /** @brief 初始化代价函数（如加载参数） */
    virtual void initialize() = 0;

    /**
     * @brief 对采样轨迹计算代价，并累加到 data.costs 中
     * @param data 包含状态、轨迹、路径等信息的结构体
     */
    virtual void score(CriticData & data) = 0;

    /** @brief 启用/禁用该代价函数 */
    virtual void setEnabled(bool enabled) { enabled_ = enabled; }
    /** @return 是否启用 */
    bool isEnabled() const { return enabled_; }
    /** @brief 设置代价函数名称 */
    void setName(const std::string & name) { name_ = name; }
    /** @return 代价函数名称 */
    const std::string & getName() const { return name_; }

protected:
    bool enabled_ = true;
    std::string name_;
};

// ==================== 障碍物代价函数（栅格加速版） ====================

/**
 * @brief 障碍物代价函数，使用局部栅格距离场加速静态障碍物距离查询，并支持动态障碍物预测
 */
class ObstaclesCritic : public CriticFunction
{
public:
    void initialize() override
    {
        // 参数将由外部通过 setParams 设置，避免硬编码
    }

    /**
     * @brief 从 YAML 参数设置代价权重和栅格参数
     * @param repulsion_weight 排斥权重
     * @param critical_weight 临界区权重（未使用）
     * @param collision_cost 碰撞代价
     * @param collision_margin 碰撞边界距离
     * @param inflation_radius 膨胀半径
     * @param cost_scaling 代价缩放因子
     * @param near_goal_distance 接近目标时的距离阈值（未使用）
     * @param robot_radius 机器人半径
     * @param grid_resolution 栅格分辨率
     * @param grid_width 栅格宽度（像素）
     * @param grid_height 栅格高度（像素）
     * @param consider_footprint 是否考虑机器人足迹（false=圆形机器人，true=非圆形机器人）
     */
    void setParams(float repulsion_weight, float critical_weight, float collision_cost,
                   float collision_margin, float inflation_radius, float cost_scaling,
                   float near_goal_distance, float robot_radius,
                   float grid_resolution, int grid_width, int grid_height,
                   bool consider_footprint = false,
                   const std::vector<Point2D>& footprint = {})
    {
        repulsion_weight_ = repulsion_weight;
        critical_weight_ = critical_weight;
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
     * @param data 包含轨迹和时间的结构体
     */
    void score(CriticData & data) override
    {
        if (!enabled_) return;

        const size_t batch_size = data.trajectories.x.shape(0);
        const size_t traj_len = data.trajectories.x.shape(1);
        Pose2D goal = data.path.getGoal();
        float dist_to_goal = std::hypot(data.state.pose.x - goal.x, data.state.pose.y - goal.y);
        // 接近目标时跳过障碍物代价，便于最终收敛到目标点
        if (dist_to_goal < near_goal_distance_) return;

        int collision_count = 0;  // 碰撞轨迹计数

        for (size_t i = 0; i < batch_size; ++i)
        {
            float traj_cost = 0.0f;
            bool trajectory_collide = false;  // 当前轨迹是否碰撞

            for (size_t j = 0; j < traj_len; ++j)
            {
                float px = data.trajectories.x(i, j);
                float py = data.trajectories.y(i, j);
                float ptheta = data.trajectories.yaws(i, j);
                float pt = data.trajectories.times(i, j);

                // 根据机器人类型选择碰撞检测方式
                if (consider_footprint_) {
                    // 模式2: 非圆形机器人 - 检查整个足迹
                    FootprintCheckResult result = checkFootprintWithDistance(px, py, ptheta);
                    
                    if (result.is_collision) {
                        trajectory_collide = true;
                        break;
                    }
                    
                    // 计算距离代价（使用足迹的最小距离）
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
                    // 模式1: 圆形机器人 - 只检查中心点
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

                    // 考虑机器人半径
                    min_dist -= robot_radius_;

                    // 碰撞判断：距离小于碰撞边界
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

                // 如果检测到碰撞，提前结束当前轨迹的检查
                if (trajectory_collide) {
                    break;
                }
            }

            // 如果轨迹碰撞，赋予极高的碰撞代价
            if (trajectory_collide) {
                traj_cost = collision_cost_;
                collision_count++;
            }

            data.costs(i) += traj_cost;
        }

        // 如果所有轨迹都碰撞，设置fail_flag
        if (collision_count == static_cast<int>(batch_size)) {
            data.fail_flag = true;
        }
    }

private:
    /**
     * @brief 查询栅格距离（若超出范围返回一个大值）
     * @param x 世界坐标 x
     * @param y 世界坐标 y
     * @return 到最近障碍物的欧几里得距离（栅格值）
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
     * @param robot_pose 当前机器人位姿（用于确定栅格原点）
     */
    void rebuildGrid(const Pose2D & robot_pose)
    {
        // 设置栅格原点为机器人左下角
        grid_origin_x_ = robot_pose.x - grid_width_ * grid_resolution_ * 0.5f;
        grid_origin_y_ = robot_pose.y - grid_height_ * grid_resolution_ * 0.5f;

        // 初始化距离场为大值
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
     * @brief 检查机器人足迹是否碰撞（非圆形机器人模式）
     * @param x 机器人中心x坐标
     * @param y 机器人中心y坐标
     * @param theta 机器人朝向
     * @return true 如果足迹任何部分与障碍物碰撞
     */
    bool checkFootprintCollision(float x, float y, float theta) const
    {
        if (footprint_.empty()) {
            // 如果没有设置足迹，退化为圆形机器人检测
            float dist = getGridDistance(x, y);
            return (dist < collision_margin_distance_);
        }

        float cos_theta = std::cos(theta);
        float sin_theta = std::sin(theta);

        // 检查足迹的每个点
        for (const auto& point : footprint_) {
            // 将足迹点从机器人坐标系转换到世界坐标系
            float world_x = x + point.x * cos_theta - point.y * sin_theta;
            float world_y = y + point.x * sin_theta + point.y * cos_theta;

            // 检查该点是否与障碍物碰撞
            float dist = getGridDistance(world_x, world_y);
            if (dist < collision_margin_distance_) {
                return true;  // 碰撞
            }
        }

        return false;  // 无碰撞
    }

    /**
     * @brief 足迹检查结果结构
     */
    struct FootprintCheckResult {
        bool is_collision = false;    // 是否碰撞
        float min_distance = std::numeric_limits<float>::max();  // 到障碍物的最小距离
    };

    /**
     * @brief 检查机器人足迹并返回最小距离（非圆形机器人模式）
     * @param x 机器人中心x坐标
     * @param y 机器人中心y坐标
     * @param theta 机器人朝向
     * @return 足迹检查结果（是否碰撞 + 最小距离）
     */
    FootprintCheckResult checkFootprintWithDistance(float x, float y, float theta) const
    {
        FootprintCheckResult result;
        
        if (footprint_.empty()) {
            // 如果没有设置足迹，退化为圆形机器人检测
            float dist = getGridDistance(x, y);
            result.min_distance = dist;
            result.is_collision = (dist < collision_margin_distance_);
            return result;
        }

        float cos_theta = std::cos(theta);
        float sin_theta = std::sin(theta);

        // 检查足迹的每个点，记录最小距离
        for (const auto& point : footprint_) {
            // 将足迹点从机器人坐标系转换到世界坐标系
            float world_x = x + point.x * cos_theta - point.y * sin_theta;
            float world_y = y + point.x * sin_theta + point.y * cos_theta;

            // 获取该点到障碍物的距离
            float dist = getGridDistance(world_x, world_y);
            if (dist < result.min_distance) {
                result.min_distance = dist;
            }
            
            // 检查是否碰撞
            if (dist < collision_margin_distance_) {
                result.is_collision = true;
            }
        }

        return result;
    }

    std::vector<Point2D> static_obstacles_;
    const std::vector<ObstaclePrediction> * dynamic_obstacles_ = nullptr;
    std::vector<Point2D> footprint_;  // 机器人足迹（用于非圆形机器人）

    float repulsion_weight_ = 2.0f;
    float critical_weight_ = 5.0f;
    float collision_cost_ = 10000.0f;
    float collision_margin_distance_ = 0.15f;
    float inflation_radius_ = 1.0f;
    float cost_scaling_factor_ = 5.0f;
    float near_goal_distance_ = 0.5f;
    float robot_radius_ = 0.3f;
    bool consider_footprint_ = false;  // 是否考虑机器人足迹

    // 栅格参数
    float grid_resolution_ = 0.1f;
    int grid_width_ = 100;
    int grid_height_ = 100;
    float grid_origin_x_ = 0.0f;
    float grid_origin_y_ = 0.0f;
    std::vector<float> grid_distance_field_;
};

// ==================== 路径跟踪代价函数 ====================

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

        size_t ref_idx = data.path.size() - 1;
        if (data.furthest_reached_path_point) {
            ref_idx = std::min(*data.furthest_reached_path_point + offset_from_furthest_,
                               data.path.size() - 1);
        }

        float target_x = data.path.x(ref_idx);
        float target_y = data.path.y(ref_idx);

        for (size_t i = 0; i < batch_size; ++i) {
            float last_x = data.trajectories.x(i, traj_len - 1);
            float last_y = data.trajectories.y(i, traj_len - 1);
            float dist = std::hypot(last_x - target_x, last_y - target_y);
            data.costs(i) += weight_ * dist;
        }
    }
private:
    float weight_ = 5.0f;
    float threshold_to_consider_ = 0.6f;
    int offset_from_furthest_ = 5;
};

/**
 * @brief 路径对齐代价：计算轨迹上每个点与路径上最近点的距离的平均值
 * 支持路径阻塞检测和路径朝向考虑
 * 优化：使用路径点缓存减少搜索范围，时间复杂度从O(batch*time_steps*path_length)降低
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
     * @param max_path_occupancy_ratio 路径阻塞比例阈值，大于此值时失效
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
     * @param obstacles 障碍物点云指针
     * @param check_radius 路径点检查半径
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
        if (occupancy_ratio > max_path_occupancy_ratio_) {
            // 路径被阻塞，此critic失效
            return;
        }
        
        Pose2D goal = data.path.getGoal();
        float dist_to_goal = std::hypot(data.state.pose.x - goal.x, data.state.pose.y - goal.y);
        if (dist_to_goal < threshold_to_consider_) return;

        const size_t batch_size = data.trajectories.x.shape(0);
        const size_t traj_len = data.trajectories.x.shape(1);

        size_t path_end_idx = data.path.size() - 1;
        size_t search_start = 0;
        if (data.furthest_reached_path_point) {
            path_end_idx = std::min(*data.furthest_reached_path_point + offset_from_furthest_,
                                    data.path.size() - 1);
            search_start = *data.furthest_reached_path_point;
        }

        // 优化：预计算路径段的起始位置，用于快速定位
        // 使用上一次的最近点作为搜索起点，大幅减少搜索范围
        size_t last_closest_idx = search_start;
        
        for (size_t i = 0; i < batch_size; ++i) {
            float cost = 0.0f;
            int count = 0;
            last_closest_idx = search_start;  // 每条轨迹重置搜索起点
            
            for (size_t j = 0; j < traj_len; j += traj_point_step_) {
                float px = data.trajectories.x(i, j);
                float py = data.trajectories.y(i, j);
                float min_dist = std::numeric_limits<float>::max();
                
                // 优化：限制搜索范围，从上一次最近点附近开始搜索
                // 搜索范围：前后各扩展一定距离（基于轨迹步长估算）
                size_t search_range = static_cast<size_t>(traj_point_step_ * 2) + 5;
                size_t local_start = (last_closest_idx > search_range) ? 
                                      last_closest_idx - search_range : search_start;
                size_t local_end = std::min(last_closest_idx + search_range + 1, path_end_idx + 1);
                
                // 确保搜索范围有效
                local_start = std::max(local_start, search_start);
                
                size_t closest_path_idx = last_closest_idx;
                for (size_t k = local_start; k <= local_end && k < data.path.size(); ++k) {
                    float dist = std::hypot(px - data.path.x(k), py - data.path.y(k));
                    if (dist < min_dist) {
                        min_dist = dist;
                        closest_path_idx = k;
                    }
                }
                
                // 更新最近点缓存
                last_closest_idx = closest_path_idx;
                
                // 如果考虑路径朝向，根据朝向偏差调整代价
                if (use_path_orientations_ && closest_path_idx < data.path.size()) {
                    float traj_yaw = data.trajectories.yaws(i, j);
                    float path_yaw = data.path.yaws(closest_path_idx);
                    float angle_diff = std::abs(shortestAngularDistance(traj_yaw, path_yaw));
                    // 朝向偏差越大，距离代价越大
                    min_dist *= (1.0f + 0.5f * angle_diff);
                }
                
                cost += min_dist;
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
    
private:
    float weight_ = 14.0f;
    float threshold_to_consider_ = 0.4f;
    int offset_from_furthest_ = 20;
    int traj_point_step_ = 4;
    float max_path_occupancy_ratio_ = 0.07f;  // 路径阻塞比例阈值
    bool use_path_orientations_ = false;       // 是否考虑路径朝向
    const std::vector<Point2D>* obstacles_ = nullptr;  // 障碍物点云
    float obstacle_check_radius_ = 0.15f;      // 障碍物检查半径
};

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
 * 支持多种角度模式：偏好前进、无偏好、考虑路径朝向
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

        float target_yaw = data.path.yaws(path_end_idx);

        for (size_t i = 0; i < batch_size; ++i) {
            float last_yaw = data.trajectories.yaws(i, traj_len - 1);
            float angle_diff = 0.0f;
            
            switch (mode_) {
                case PathAngleMode::FORWARD_PREFERENCE:
                    // 偏好前进方向：惩罚与目标朝向的偏差
                    angle_diff = std::abs(shortestAngularDistance(last_yaw, target_yaw));
                    break;
                    
                case PathAngleMode::NO_DIRECTIONAL_PREFERENCE:
                    // 无方向偏好：允许倒车，取最小角度差（考虑反向）
                    {
                        float diff1 = std::abs(shortestAngularDistance(last_yaw, target_yaw));
                        float diff2 = std::abs(shortestAngularDistance(last_yaw, normalizeAngle(target_yaw + PI)));
                        angle_diff = std::min(diff1, diff2);
                    }
                    break;
                    
                case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS:
                    // 考虑路径朝向：根据轨迹速度方向选择参考朝向
                    {
                        // 获取轨迹的平均vx来判断是前进还是后退
                        float avg_vx = 0.0f;
                        for (size_t j = 0; j < traj_len; ++j) {
                            avg_vx += data.state.cvx(i, j);
                        }
                        avg_vx /= traj_len;
                        
                        // 如果vx为负（后退），使用反向目标朝向
                        float effective_target_yaw = (avg_vx < 0.0f) ? 
                            normalizeAngle(target_yaw + PI) : target_yaw;
                        angle_diff = std::abs(shortestAngularDistance(last_yaw, effective_target_yaw));
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

/**
 * @brief 偏好前进代价：惩罚控制序列中负速度（倒车）的出现
 */
class PreferForwardCritic : public CriticFunction
{
public:
    void initialize() override {}
    /**
     * @param weight 代价权重
     * @param threshold 距离阈值，大于该值才激活（即远处才惩罚倒车）
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
        if (dist_to_goal < threshold_to_consider_) return;

        const size_t batch_size = data.state.cvx.shape(0);
        const size_t traj_len = data.state.cvx.shape(1);

        for (size_t i = 0; i < batch_size; ++i) {
            float backward_cost = 0.0f;
            for (size_t j = 0; j < traj_len; ++j) {
                if (data.state.cvx(i, j) < 0.0f) {
                    backward_cost += std::abs(data.state.cvx(i, j));
                }
            }
            data.costs(i) += weight_ * backward_cost;
        }
    }
private:
    float weight_ = 5.0f;
    float threshold_to_consider_ = 0.4f;
};

/**
 * @brief 约束代价：激励机器人在运动学和动力学约束范围内移动
 * 支持差速驱动、全向、阿克曼等多种运动模型
 */
class ConstraintCritic : public CriticFunction
{
public:
    void initialize() override {}
    /**
     * @param weight 代价权重
     * @param vx_max 最大线速度
     * @param vx_min 最小线速度（允许负值表示倒车）
     * @param vy_max 最大横向速度（全向模型）
     * @param wz_max 最大角速度
     * @param min_turning_radius 最小转弯半径（阿克曼模型）
     * @param model_type 运动模型类型：0=DiffDrive, 1=Omni, 2=Ackermann
     */
    void setParams(float weight, float vx_max, float vx_min, float vy_max, float wz_max,
                   float min_turning_radius = 0.2f, int model_type = 0)
    {
        weight_ = weight;
        vx_max_ = vx_max;
        vx_min_ = vx_min;
        vy_max_ = vy_max;
        wz_max_ = wz_max;
        min_turning_radius_ = min_turning_radius;
        model_type_ = static_cast<MotionModelType>(model_type);
    }

    void score(CriticData & data) override
    {
        if (!enabled_) return;
        const size_t batch_size = data.state.cvx.shape(0);
        const size_t traj_len = data.state.cvx.shape(1);
        
        for (size_t i = 0; i < batch_size; ++i) {
            float cost = 0.0f;
            
            for (size_t j = 0; j < traj_len; ++j) {
                float vx = data.state.cvx(i, j);
                float vy = data.state.cvy(i, j);
                float wz = data.state.cwz(i, j);
                
                switch (model_type_) {
                    case MotionModelType::DIFF_DRIVE:
                        // 差速驱动模型：惩罚速度超出约束范围
                        {
                            float vx_violation = std::max(0.0f, vx - vx_max_) + 
                                                std::max(0.0f, vx_min_ - vx);
                            float wz_violation = std::max(0.0f, std::abs(wz) - wz_max_);
                            cost += (vx_violation + wz_violation) * data.model_dt;
                        }
                        break;
                        
                    case MotionModelType::OMNI:
                        // 全向模型：惩罚总速度超出约束
                        {
                            float vel_total = std::copysign(std::hypot(vx, vy), vx);
                            float vel_violation = std::max(0.0f, vel_total - vx_max_) + 
                                                 std::max(0.0f, vx_min_ - vel_total);
                            float wz_violation = std::max(0.0f, std::abs(wz) - wz_max_);
                            cost += (vel_violation + wz_violation) * data.model_dt;
                        }
                        break;
                        
                    case MotionModelType::ACKERMANN:
                        // 阿克曼模型：速度约束 + 转弯半径约束
                        {
                            float vx_violation = std::max(0.0f, vx - vx_max_) + 
                                                std::max(0.0f, vx_min_ - vx);
                            float wz_violation = std::max(0.0f, std::abs(wz) - wz_max_);
                            
                            // 转弯半径约束：当vx和wz都非零时检查
                            float turning_radius_violation = 0.0f;
                            if (std::abs(vx) > EPSILON && std::abs(wz) > EPSILON) {
                                float radius = std::abs(vx / wz);
                                turning_radius_violation = std::max(0.0f, min_turning_radius_ - radius);
                            }
                            
                            cost += (vx_violation + wz_violation + turning_radius_violation) * data.model_dt;
                        }
                        break;
                }
            }
            
            data.costs(i) += weight_ * cost / traj_len;
        }
    }
    
private:
    enum class MotionModelType {
        DIFF_DRIVE = 0,
        OMNI = 1,
        ACKERMANN = 2
    };
    
    float weight_ = 4.0f;
    float vx_max_ = 0.5f;
    float vx_min_ = -0.35f;
    float vy_max_ = 0.5f;
    float wz_max_ = 1.9f;
    float min_turning_radius_ = 0.2f;
    MotionModelType model_type_ = MotionModelType::DIFF_DRIVE;
};

/**
 * @brief 旋转惩罚代价（TwirlingCritic）：大幅惩罚"低前进+高旋转"行为
 * 这是阻止原地打转的核心代价函数，Nav2 MPPI中的对应实现。
 * 当轨迹的前进速度很小但角速度很大时，施加高昂代价。
 */
class TwirlingCritic : public CriticFunction
{
public:
    void initialize() override {}
    /**
     * @param weight 代价权重
     * @param vx_max 最大前进速度（用于归一化）
     */
    void setParams(float weight, float vx_max)
    {
        weight_ = weight;
        vx_max_ = std::max(vx_max, EPSILON);
    }

    void score(CriticData & data) override
    {
        if (!enabled_) return;
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
};

/**
 * @brief 速度死区批评：惩罚速度低于死区阈值的情况
 * 用于避免机器人在低速时的不稳定行为
 */
class VelocityDeadbandCritic : public CriticFunction
{
public:
    void initialize() override {}
    /**
     * @param weight 代价权重
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
        const size_t batch_size = data.state.cvx.shape(0);
        const size_t traj_len = data.state.cvx.shape(1);

        for (size_t i = 0; i < batch_size; ++i) {
            float cost = 0.0f;
            for (size_t j = 0; j < traj_len; ++j) {
                float vx = data.state.cvx(i, j);
                float vy = data.state.cvy(i, j);
                float wz = data.state.cwz(i, j);
                
                // 惩罚速度低于死区阈值的情况
                // max(|deadband| - |velocity|, 0) 当速度低于死区时惩罚
                float vx_penalty = std::max(0.0f, deadband_vx_ - std::abs(vx));
                float vy_penalty = std::max(0.0f, deadband_vy_ - std::abs(vy));
                float wz_penalty = std::max(0.0f, deadband_wz_ - std::abs(wz));
                
                cost += (vx_penalty + vy_penalty + wz_penalty) * data.model_dt;
            }
            data.costs(i) += weight_ * cost / traj_len;
        }
    }
private:
    float weight_ = 1.0f;
    float deadband_vx_ = 0.05f;  // 线速度死区 (m/s)
    float deadband_vy_ = 0.05f;  // 横向速度死区 (m/s)
    float deadband_wz_ = 0.1f;   // 角速度死区 (rad/s)
};

// ==================== 代价函数管理器 ====================

/**
 * @brief 代价函数管理器，负责存储所有代价函数并批量执行评分
 */
class CriticManager
{
public:
    CriticManager() = default;
    /** @brief 添加一个代价函数（接管所有权） */
    void addCritic(std::unique_ptr<CriticFunction> critic) { critics_.push_back(std::move(critic)); }
    /** @brief 初始化所有代价函数 */
    void initializeCritics() { for (auto & c : critics_) c->initialize(); }

    /**
     * @brief 对所有代价函数执行评分，累加代价到 data.costs
     * @param data 评分所需数据
     */
    void evalTrajectoriesScores(CriticData & data) const
    {
        for (size_t i = 0; i < data.costs.shape(0); ++i) data.costs(i) = 0.0f;
        data.fail_flag = false; // Reset

        for (const auto & critic : critics_) {
            if (critic->isEnabled()) critic->score(data);
        }
    }

    /**
     * @brief 根据名称获取代价函数指针（用于外部设置参数）
     * @param name 代价函数名称
     * @return 代价函数指针，若未找到返回 nullptr
     */
    CriticFunction* getCritic(const std::string & name) const
    {
        for (const auto & c : critics_)
            if (c->getName() == name) return c.get();
        return nullptr;
    }

private:
    std::vector<std::unique_ptr<CriticFunction>> critics_;
};

// ==================== 噪声生成器 ====================

/**
 * @brief 噪声生成器，为每条轨迹生成高斯噪声
 * 参考Nav2实现：使用xt::random和独立线程异步预生成噪声
 */
class NoiseGenerator
{
public:
    NoiseGenerator() = default;
    
    ~NoiseGenerator() { shutdown(); }

    /**
     * @brief 初始化噪声生成器
     * @param settings 优化器设置（含标准差、batch size、时间步数）
     * @param is_holonomic 是否为全向模型
     */
    void initialize(OptimizerSettings & settings, bool is_holonomic)
    {
        settings_ = settings;
        is_holonomic_ = is_holonomic;
        noises_vx_ = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
        noises_vy_ = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
        noises_wz_ = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
        
        active_ = true;
        ready_ = false;
        regenerate_noises_ = false;
        
        // 启动噪声生成线程，异步预生成噪声
        noise_thread_ = std::thread(&NoiseGenerator::noiseThread, this);
        
        // 触发第一次噪声生成
        generateNextNoises();
    }
    
    /**
     * @brief 关闭噪声生成线程
     */
    void shutdown()
    {
        active_ = false;
        ready_ = true;
        noise_cond_.notify_all();
        if (noise_thread_.joinable()) {
            noise_thread_.join();
        }
    }

    /** 
     * @brief 触发噪声线程生成下一组噪声（异步预生成）
     */
    void generateNextNoises()
    {
        {
            std::unique_lock<std::mutex> lock(noise_lock_);
            ready_ = true;
        }
        noise_cond_.notify_all();
    }

    /**
     * @brief 将噪声叠加到控制序列上，得到含噪声的控制量，存入 state
     * @param state 状态容器（cvx/cvy/cwz 将被写入）
     * @param control_sequence 原始控制序列
     */
    void setNoisedControls(State & state, const ControlSequence & control_sequence)
    {
        std::unique_lock<std::mutex> lock(noise_lock_);
        
        // 等待噪声生成完成（如果正在生成）
        noise_cond_.wait(lock, [this]() { return !ready_; });
        
        // 使用xt::noalias优化内存操作
        xt::noalias(state.cvx) = xt::view(control_sequence.vx, xt::newaxis(), xt::all()) + noises_vx_;
        xt::noalias(state.cwz) = xt::view(control_sequence.wz, xt::newaxis(), xt::all()) + noises_wz_;
        if (is_holonomic_) {
            xt::noalias(state.cvy) = xt::view(control_sequence.vy, xt::newaxis(), xt::all()) + noises_vy_;
        }
    }

    /** @brief 重置生成器（重新初始化） */
    void reset(OptimizerSettings & settings, bool is_holonomic)
    {
        settings_ = settings;
        is_holonomic_ = is_holonomic;
        
        {
            std::unique_lock<std::mutex> lock(noise_lock_);
            xt::noalias(noises_vx_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
            xt::noalias(noises_vy_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
            xt::noalias(noises_wz_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
            ready_ = true;
        }
        noise_cond_.notify_all();
    }

private:
    /**
     * @brief 噪声生成线程（后台异步预生成噪声）
     */
    void noiseThread()
    {
        do {
            std::unique_lock<std::mutex> lock(noise_lock_);
            noise_cond_.wait(lock, [this]() { return ready_; });
            ready_ = false;
            if (!active_) break;
            generateNoisedControls();
        } while (active_);
    }
    
    /**
     * @brief 生成高斯噪声矩阵
     * 使用xt::random::randn（比std::mt19937更快）
     */
    void generateNoisedControls()
    {
        auto & s = settings_;
        
        // 使用xt::random::randn生成高斯噪声（更快，且与xtensor集成更好）
        xt::noalias(noises_vx_) = xt::random::randn<float>(
            {s.batch_size, s.time_steps}, 0.0f, s.sampling_std.vx);
        xt::noalias(noises_wz_) = xt::random::randn<float>(
            {s.batch_size, s.time_steps}, 0.0f, s.sampling_std.wz);
        if (is_holonomic_) {
            xt::noalias(noises_vy_) = xt::random::randn<float>(
                {s.batch_size, s.time_steps}, 0.0f, s.sampling_std.vy);
        }
    }

    xt::xtensor<float, 2> noises_vx_, noises_vy_, noises_wz_;
    OptimizerSettings settings_;
    bool is_holonomic_ = false;
    
    // 线程相关成员
    std::thread noise_thread_;
    std::condition_variable noise_cond_;
    std::mutex noise_lock_;
    bool active_ = false;
    bool ready_ = false;
    bool regenerate_noises_ = false;
};

// ==================== 优化器核心 ====================

/**
 * @brief MPPI 优化器核心，执行采样、评分、加权更新
 */
class Optimizer
{
public:
    /**
     * @brief 初始化优化器
     * @param settings 优化器设置
     * @param motion_model 运动模型
     * @param critic_manager 代价函数管理器
     */
    void initialize(const OptimizerSettings & settings,
                    std::shared_ptr<MotionModel> motion_model,
                    CriticManager * critic_manager)
    {
        settings_ = settings;
        motion_model_ = motion_model;
        critic_manager_ = critic_manager;
        
        // 设置运动模型的加速度约束
        motion_model_->setAccelerationConstraints(
            settings_.base_constraints.ax_max,
            settings_.base_constraints.ay_max,
            settings_.base_constraints.az_max,
            settings_.model_dt
        );
        
        noise_generator_.initialize(settings_, isHolonomic());
        reset();
    }

    /** @brief 重置优化器内部状态 */
    void reset()
    {
        state_.reset(settings_.batch_size, settings_.time_steps);
        control_sequence_.reset(settings_.time_steps);
        settings_.constraints = settings_.base_constraints;
        costs_ = xt::zeros<float>({settings_.batch_size});
        generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);
        noise_generator_.reset(settings_, isHolonomic());
        last_control_ = {0.0f, 0.0f, 0.0f};
        // 重置控制历史
        control_history_[0] = {0.0f, 0.0f, 0.0f};
        control_history_[1] = {0.0f, 0.0f, 0.0f};
        control_history_[2] = {0.0f, 0.0f, 0.0f};
        control_history_[3] = {0.0f, 0.0f, 0.0f};
        // 注意：不清零 last_speed_，保留当前速度作为加速度约束的参考
        // last_speed_ = {0.0f, 0.0f, 0.0f};
    }

    /**
     * @brief 主接口：给定当前位姿、速度、全局路径，计算最优控制量
     * @param robot_pose 当前机器人位姿
     * @param robot_speed 当前机器人速度
     * @param path 全局路径
     * @return 最优控制量（Twist2D）
     */
    Twist2D evalControl(const Pose2D & robot_pose, const Twist2D & robot_speed, const Path & path)
    {
        prepare(robot_pose, robot_speed, path);

        float prev_min_cost = std::numeric_limits<float>::max();
        bool all_trajectories_collide = false;
        bool fail_flag = false;

        for (size_t iter = 0; iter < settings_.iteration_count; ++iter) {
            optimize(fail_flag);

            float current_min_cost = xt::amin(costs_)(0);
            if (std::abs(prev_min_cost - current_min_cost) < 0.01f && current_min_cost < 100.0f) {
                break;
            }
            prev_min_cost = current_min_cost;

            // 检查最优轨迹是否碰撞
            size_t best_idx = xt::argmin(costs_)(0);
            bool best_trajectory_collide = (costs_(best_idx) >= settings_.constraints.collision_cost_threshold);
            
            // 检查是否所有轨迹都碰撞（通过fail_flag或代价检查）
            all_trajectories_collide = fail_flag || checkAllTrajectoriesCollide();
            
            if (best_trajectory_collide || all_trajectories_collide) {
                if (!fallback(fail_flag)) {
                    // fallback返回false表示重试次数已用完，抛出异常
                    throw std::runtime_error("Optimizer failed to compute path: all trajectories collide after maximum retries");
                }
                // 重置后重新准备优化
                prepare(robot_pose, robot_speed, path);
                prev_min_cost = std::numeric_limits<float>::max();
                fail_flag = false;  // 重置失败标志
            }
        }

        applyControlSequenceConstraints();
        applyAccelerationConstraints();

        // ====== 阻塞检测：如果检测到原地打转，输出零速度 ======
        if (isBlocked()) {
            is_blocked_ = true;
            // 彻底归零控制序列，打破正反馈环路
            control_sequence_.reset(settings_.time_steps);
            last_control_ = {0.0f, 0.0f, 0.0f};
            last_speed_ = robot_speed;
            return Twist2D(0.0f, 0.0f, 0.0f);
        }
        is_blocked_ = false;
        // 注意：不再在此处调用 resetSpinningCounter()！
        // 计数器的重置已移入 isBlocked() 内部，仅在确认不打转（wz小/比率低）时重置。
        // 之前的 bug：无论 isBlocked 返回 false 的原因是什么都重置计数器，
        // 导致计数器永远无法累积到 spinning_detect_frames。

        // 应用 Savitzky-Golay 滤波器平滑控制序列（对整个序列滤波，参考Nav2）
        if (settings_.use_sg_filter) {
            savitskyGolayFilter(control_sequence_, control_history_, settings_);
        }
        
        Twist2D control = getControlFromSequence();
        
        last_control_ = control;
        last_speed_ = robot_speed;
        
        // 更新控制历史（用于SG滤波）
        updateControlHistory();

        if (settings_.shift_control_sequence) {
            shiftControlSequence();
        }
        return control;
    }

    /** @return 生成的采样轨迹（用于可视化） */
    Trajectories & getGeneratedTrajectories() { return generated_trajectories_; }

    /**
     * @brief 获取最优轨迹（根据更新后的控制序列重新积分）
     * @return 最优轨迹矩阵 (time_steps x 3) [x, y, yaw]
     */
    xt::xtensor<float, 2> getOptimizedTrajectory()
    {
        std::vector<size_t> shape = {settings_.time_steps, 3};
        xt::xtensor<float, 2> trajectory = xt::zeros<float>(shape);
        float x = state_.pose.x;
        float y = state_.pose.y;
        float yaw = state_.pose.theta;
        trajectory(0, 0) = x; trajectory(0, 1) = y; trajectory(0, 2) = yaw;

        for (size_t j = 1; j < settings_.time_steps; ++j) {
            float cos_yaw = std::cos(yaw);
            float sin_yaw = std::sin(yaw);
            x += control_sequence_.vx(j-1) * cos_yaw * settings_.model_dt;
            y += control_sequence_.vx(j-1) * sin_yaw * settings_.model_dt;
            yaw += control_sequence_.wz(j-1) * settings_.model_dt;
            trajectory(j, 0) = x; trajectory(j, 1) = y; trajectory(j, 2) = yaw;
        }
        return trajectory;
    }

    // 占位函数（未实现）
    void setSpeedLimit(double /*speed_limit*/, bool /*percentage*/) {}
    /** @return 优化器设置 */
    const OptimizerSettings & getSettings() const { return settings_; }
    /** @brief 设置动态障碍物列表 */
    void setDynamicObstacles(const std::vector<ObstaclePrediction> & obstacles) { dynamic_obstacles_ = obstacles; }

    /** @return 当前是否被阻塞（上一次 evalControl 是否检测到阻塞） */
    bool isCurrentlyBlocked() const { return is_blocked_; }

private:
    /**
     * @brief 准备本次优化：更新状态中的位姿和速度，存储路径
     */
    void prepare(const Pose2D & robot_pose, const Twist2D & robot_speed, const Path & path)
    {
        state_.pose = robot_pose;
        state_.speed = robot_speed;
        path_ = path;
        settings_.constraints = settings_.base_constraints;
        if (shouldDisableReverseAtStartup(robot_pose, robot_speed, path_)) {
            settings_.constraints.vx_min = std::max(settings_.constraints.vx_min, 0.0f);
        }
        costs_ = xt::zeros<float>({settings_.batch_size});
    }

    bool shouldDisableReverseAtStartup(const Pose2D & robot_pose,
                                       const Twist2D & robot_speed,
                                       const Path & path) const
    {
        if (path.empty()) return false;

        const float linear_speed = std::abs(robot_speed.vx);
        const float angular_speed = std::abs(robot_speed.wz);
        if (linear_speed > 0.05f || angular_speed > 0.20f) {
            return false;
        }

        const float dist_to_start = std::hypot(robot_pose.x - path.x(0), robot_pose.y - path.y(0));
        return dist_to_start < 0.35f;
    }

    /** @brief 单次优化迭代：生成轨迹、评分、更新控制序列
     * @param fail_flag 输出参数，表示是否所有轨迹都失败（碰撞）
     */
    void optimize(bool& fail_flag)
    {
        generateNoisedTrajectories();
        std::optional<size_t> furthest = findFurthestReachedPathPoint();

        CriticData data{state_, generated_trajectories_, path_, costs_,
                        settings_.model_dt, false, motion_model_, furthest, &dynamic_obstacles_};
        critic_manager_->evalTrajectoriesScores(data);
        
        // 获取fail_flag状态
        fail_flag = data.fail_flag;

        updateControlSequence();
    }

    /**
     * @brief 查找当前机器人位姿在路径上的最近点索引
     * @return 最近点索引（若路径为空则返回 std::nullopt）
     */
    std::optional<size_t> findFurthestReachedPathPoint()
    {
        if (path_.empty()) return std::nullopt;

        size_t closest_idx = 0;
        float min_dist = std::numeric_limits<float>::max();
        for (size_t k = 0; k < path_.size(); ++k) {
            float dist = std::hypot(state_.pose.x - path_.x(k), state_.pose.y - path_.y(k));
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = k;
            }
        }
        return closest_idx;
    }

    /**
     * @brief 检查是否所有轨迹都发生碰撞
     * @return true 如果所有轨迹的代价都超过碰撞阈值
     */
    bool checkAllTrajectoriesCollide() const
    {
        const float collision_threshold = settings_.constraints.collision_cost_threshold;
        for (size_t i = 0; i < costs_.shape(0); ++i) {
            if (costs_(i) < collision_threshold) {
                return false;  // 至少有一条轨迹无碰撞
            }
        }
        return true;  // 所有轨迹都碰撞
    }

    /**
     * @brief 故障恢复机制：若失败次数过多则重置优化器
     * @param fail 是否失败
     * @return true表示可以继续重试，false表示重试次数已用完
     */
    bool fallback(bool fail)
    {
        if (!fail) { 
            fallback_counter_ = 0; 
            return true; 
        }
        
        fallback_counter_++;
        if (fallback_counter_ > settings_.retry_attempt_limit) {
            fallback_counter_ = 0;
            reset();
            return false;
        }
        
        reset();
        return true;
    }

    /** @brief 生成带噪声的轨迹（采样） */
    void generateNoisedTrajectories()
    {
        noise_generator_.setNoisedControls(state_, control_sequence_);
        applyControlConstraintsBatch(state_);
        motion_model_->applyConstraints(state_.cvx, state_.cvy, state_.cwz);
        updateStateVelocities(state_);
        integrateStateVelocities(generated_trajectories_, state_);
        noise_generator_.generateNextNoises();  // 触发异步预生成下一组噪声
    }

    /**
     * @brief 对一批含噪声控制量施加边界约束
     * @param state 状态容器（cvx/cvy/cwz 将被裁剪）
     */
    void applyControlConstraintsBatch(State & state)
    {
        auto & s = settings_;
        state.cvx = xt::clip(state.cvx, s.constraints.vx_min, s.constraints.vx_max);
        state.cwz = xt::clip(state.cwz, -s.constraints.wz_max, s.constraints.wz_max);
        if (isHolonomic()) {
            state.cvy = xt::clip(state.cvy, -s.constraints.vy_max, s.constraints.vy_max);
        }
    }

    /**
     * @brief 更新状态中的速度序列（由运动模型预测，包含加速度约束）
     * @param state 状态容器
     */
    void updateStateVelocities(State & state) const
    {
        // 运动模型的 predict() 已包含：
        // 1. 第一时刻使用当前速度
        // 2. 后续时刻应用加速度约束的速度传播
        motion_model_->predict(state);
    }

    /**
     * @brief 根据速度序列积分得到轨迹（多线程并行）
     * @param trajectories 输出的轨迹容器
     * @param state 状态容器（含速度序列）
     * 优化：使用局部变量存储中间结果，避免线程间数据竞争
     */
    void integrateStateVelocities(Trajectories & trajectories, const State & state) const
    {
        // 提前复制共享状态到局部变量，避免多线程竞争
        const float initial_yaw = state.pose.theta;
        const float pose_x = state.pose.x;
        const float pose_y = state.pose.y;
        const float model_dt = settings_.model_dt;
        const size_t time_steps = state.vx.shape(1);
        
        std::vector<std::thread> threads;
        unsigned int num_threads = settings_.thread_count;
        unsigned int batch_per_thread = (settings_.batch_size + num_threads - 1) / num_threads;

        auto worker = [&](unsigned int start, unsigned int end) {
            // 使用局部变量存储中间结果，最后一次性写入
            std::vector<float> local_x(time_steps);
            std::vector<float> local_y(time_steps);
            std::vector<float> local_yaws(time_steps);
            
            for (size_t i = start; i < end; ++i) {
                float yaw = initial_yaw;
                local_x[0] = pose_x;
                local_y[0] = pose_y;
                local_yaws[0] = yaw;
                
                // 使用局部变量进行计算，避免直接操作共享内存
                for (size_t j = 1; j < time_steps; ++j) {
                    yaw += state.wz(i, j-1) * model_dt;
                    float cos_yaw = std::cos(yaw);
                    float sin_yaw = std::sin(yaw);
                    
                    local_x[j] = local_x[j-1] + state.vx(i, j-1) * cos_yaw * model_dt;
                    local_y[j] = local_y[j-1] + state.vx(i, j-1) * sin_yaw * model_dt;
                    local_yaws[j] = yaw;
                }
                
                // 一次性写入结果到共享内存（每个线程写不同的行，避免竞争）
                trajectories.x(i, 0) = local_x[0];
                trajectories.y(i, 0) = local_y[0];
                trajectories.yaws(i, 0) = local_yaws[0];
                trajectories.times(i, 0) = 0.0f;
                
                for (size_t j = 1; j < time_steps; ++j) {
                    trajectories.x(i, j) = local_x[j];
                    trajectories.y(i, j) = local_y[j];
                    trajectories.yaws(i, j) = local_yaws[j];
                    trajectories.times(i, j) = j * model_dt;
                }
            }
        };

        for (unsigned int t = 0; t < num_threads; ++t) {
            unsigned int start = t * batch_per_thread;
            unsigned int end = std::min(start + batch_per_thread, settings_.batch_size);
            if (start < end)
                threads.emplace_back(worker, start, end);
        }
        for (auto & t : threads) t.join();
    }

    /** @brief 根据代价加权更新控制序列（支持自适应温度和均值归一化）
     *  参考Nav2 MPPI实现，添加控制成本项以惩罚偏离当前控制序列的噪声
     */
    void updateControlSequence()
    {
        // 1. 添加控制成本项：惩罚偏离当前控制序列的噪声
        // 这是MPPI算法的核心部分，缺少会导致优化器过度探索
        auto bounded_noises_vx = state_.cvx - xt::view(control_sequence_.vx, xt::newaxis(), xt::all());
        auto bounded_noises_wz = state_.cwz - xt::view(control_sequence_.wz, xt::newaxis(), xt::all());
        
        // 控制成本 = gamma / std^2 * sum(control_sequence * noise)
        costs_ += settings_.gamma / (settings_.sampling_std.vx * settings_.sampling_std.vx) * 
                  xt::sum(xt::view(control_sequence_.vx, xt::newaxis(), xt::all()) * bounded_noises_vx, 1);
        costs_ += settings_.gamma / (settings_.sampling_std.wz * settings_.sampling_std.wz) * 
                  xt::sum(xt::view(control_sequence_.wz, xt::newaxis(), xt::all()) * bounded_noises_wz, 1);
        
        if (isHolonomic()) {
            auto bounded_noises_vy = state_.cvy - xt::view(control_sequence_.vy, xt::newaxis(), xt::all());
            costs_ += settings_.gamma / (settings_.sampling_std.vy * settings_.sampling_std.vy) * 
                      xt::sum(xt::view(control_sequence_.vy, xt::newaxis(), xt::all()) * bounded_noises_vy, 1);
        }
        
        // 2. 计算softmax权重
        float min_cost = xt::amin(costs_)(0);
        float max_cost = xt::amax(costs_)(0);
        float T = settings_.temperature;
        
        // 自适应温度：根据代价分布动态调整温度
        if (settings_.adaptive_temperature) {
            float cost_range = max_cost - min_cost;
            if (cost_range > EPSILON) {
                // 代价范围越大，温度越高，允许更多探索
                float adaptive_factor = std::log(cost_range + 1.0f) / 5.0f;
                T = clamp(settings_.temperature * (1.0f + adaptive_factor),
                         settings_.adaptive_temperature_min,
                         settings_.adaptive_temperature_max);
            }
        }
        T = std::max(T, 1e-3f);
        
        xt::xtensor<float, 1> weights;
        
        if (settings_.use_mean_normalization) {
            // 均值归一化：使用均值作为参考点
            float mean_cost = xt::mean(costs_)(0);
            weights = xt::exp(-(costs_ - mean_cost) / T);
        } else {
            // min归一化（Nav2标准做法）：最优轨迹权重=1
            weights = xt::exp(-(costs_ - min_cost) / T);
        }
        
        float sum_weights = xt::sum(weights)(0);

        if (sum_weights < EPSILON) sum_weights = 1.0f;
        weights /= sum_weights;

        for (size_t j = 0; j < settings_.time_steps; ++j) {
            float sum_vx = 0.0f, sum_wz = 0.0f, sum_vy = 0.0f;
            for (size_t i = 0; i < settings_.batch_size; ++i) {
                sum_vx += weights(i) * state_.cvx(i, j);
                sum_wz += weights(i) * state_.cwz(i, j);
                if (isHolonomic()) sum_vy += weights(i) * state_.cvy(i, j);
            }
            control_sequence_.vx(j) = sum_vx;
            control_sequence_.wz(j) = sum_wz;
            if (isHolonomic()) control_sequence_.vy(j) = sum_vy;
        }
    }

    /** @brief 对控制序列施加边界约束 */
    void applyControlSequenceConstraints()
    {
        auto & s = settings_;
        for (size_t j = 0; j < settings_.time_steps; ++j) {
            control_sequence_.vx(j) = clamp(control_sequence_.vx(j), s.constraints.vx_min, s.constraints.vx_max);
            control_sequence_.wz(j) = clamp(control_sequence_.wz(j), -s.constraints.wz_max, s.constraints.wz_max);
            if (isHolonomic()) control_sequence_.vy(j) = clamp(control_sequence_.vy(j), -s.constraints.vy_max, s.constraints.vy_max);
        }
    }

    /** @brief 对控制序列施加加速度约束 */
    void applyAccelerationConstraints()
    {
        auto & s = settings_;
        float prev_vx = last_speed_.vx;
        float prev_vy = last_speed_.vy;
        float prev_wz = last_speed_.wz;

        for (size_t j = 0; j < settings_.time_steps; ++j) {
            float dt = s.model_dt;
            float max_dvx = s.constraints.ax_max * dt;
            float max_dvy = s.constraints.ay_max * dt;
            float max_dwz = s.constraints.az_max * dt;

            float desired_vx = control_sequence_.vx(j);
            float dvx = desired_vx - prev_vx;
            dvx = clamp(dvx, -max_dvx, max_dvx);
            control_sequence_.vx(j) = prev_vx + dvx;
            prev_vx = control_sequence_.vx(j);

            if (isHolonomic()) {
                float desired_vy = control_sequence_.vy(j);
                float dvy = desired_vy - prev_vy;
                dvy = clamp(dvy, -max_dvy, max_dvy);
                control_sequence_.vy(j) = prev_vy + dvy;
                prev_vy = control_sequence_.vy(j);
            }

            float desired_wz = control_sequence_.wz(j);
            float dwz = desired_wz - prev_wz;
            dwz = clamp(dwz, -max_dwz, max_dwz);
            control_sequence_.wz(j) = prev_wz + dwz;
            prev_wz = control_sequence_.wz(j);
        }
    }

    /** @brief 将控制序列左移一位（用于时序平滑） */
    void shiftControlSequence()
    {
        for (size_t j = 0; j < settings_.time_steps - 1; ++j) {
            control_sequence_.vx(j) = control_sequence_.vx(j+1);
            control_sequence_.wz(j) = control_sequence_.wz(j+1);
            if (isHolonomic()) control_sequence_.vy(j) = control_sequence_.vy(j+1);
        }
        control_sequence_.vx(settings_.time_steps - 1) = 0.0f;
        control_sequence_.wz(settings_.time_steps - 1) = 0.0f;
        if (isHolonomic()) control_sequence_.vy(settings_.time_steps - 1) = 0.0f;
    }

    /** @return 当前控制序列的第一个控制量 */
    Twist2D getControlFromSequence()
    {
        return Twist2D(control_sequence_.vx(0),
                       isHolonomic() ? control_sequence_.vy(0) : 0.0f,
                       control_sequence_.wz(0));
    }

    /** @return 是否为全向模型 */
    bool isHolonomic() const { return motion_model_->isHolonomic(); }

    /**
     * @brief 阻塞检测：判断当前是否因障碍物导致原地打转
     * 检查逻辑：
     *   1. 优化后的控制序列是否表现为"低前进 + 高旋转"（打转特征）
     *   2. 代价分布是否显示大部分轨迹均被阻挡
     * @return true 表示被阻塞，应输出零速度
     */
    bool isBlocked()
    {
        // 计算优化后控制序列的平均速度
        float avg_vx = 0.0f;
        float avg_abs_wz = 0.0f;
        for (size_t j = 0; j < settings_.time_steps; ++j) {
            avg_vx += control_sequence_.vx(j);
            avg_abs_wz += std::abs(control_sequence_.wz(j));
        }
        avg_vx /= settings_.time_steps;
        avg_abs_wz /= settings_.time_steps;

        // 条件1：如果角速度很小，不是打转（正常行进或停止）
        if (avg_abs_wz < 0.3f) {
            spinning_counter_ = 0; // 不打转时重置计数
            return false;
        }

        // 条件2：使用 wz/vx 比率检测打转（比绝对阈值更可靠）
        // 当 |wz|/vx >> 1 时，说明旋转占主导
        float effective_vx = std::abs(avg_vx);
        bool spinning_by_ratio = (effective_vx < EPSILON) ||
            (avg_abs_wz / effective_vx > settings_.spinning_ratio_threshold);

        if (!spinning_by_ratio) {
            spinning_counter_ = 0; // 不打转时重置计数
            return false;
        }

        // 到此说明：旋转占主导特征确立
        // 进一步确认：连续多帧检测，避免正常转弯被误判
        spinning_counter_++;
        if (spinning_counter_ < settings_.spinning_detect_frames) return false;

        // 已连续多帧检测到打转 → 确认阻塞
        return true;
    }

    /**
     * @brief 更新控制历史（用于SG滤波）
     */
    void updateControlHistory()
    {
        // 将历史控制前移，添加最新控制到队首
        control_history_[3] = control_history_[2];
        control_history_[2] = control_history_[1];
        control_history_[1] = control_history_[0];
        control_history_[0] = {control_sequence_.vx(0), 
                               isHolonomic() ? control_sequence_.vy(0) : 0.0f, 
                               control_sequence_.wz(0)};
    }

    /**
     * @brief Savitzky-Golay滤波器（对整个控制序列滤波，参考Nav2实现）
     * @param control_sequence 控制序列
     * @param control_history 控制历史
     * @param settings 优化器设置
     */
    void savitskyGolayFilter(ControlSequence & control_sequence,
                             const std::array<Control, 4> & control_history,
                             const OptimizerSettings & settings)
    {
        // SG滤波器系数（窗口大小5，多项式阶数3）
        // 这些系数用于平滑控制序列，减少抖动
        const float c0 = 0.2f;  // 当前点权重
        const float c1 = 0.2f;  // 相邻点权重
        const float c2 = 0.2f;  // 次相邻点权重
        
        // 对控制序列的每个维度应用SG滤波
        for (size_t i = 0; i < settings.time_steps; ++i) {
            // vx滤波：结合历史控制和当前控制序列
            float vx_filtered = c0 * control_sequence.vx(i);
            if (i > 0) vx_filtered += c1 * control_sequence.vx(i - 1);
            if (i > 1) vx_filtered += c2 * control_sequence.vx(i - 2);
            if (i < settings.time_steps - 1) vx_filtered += c1 * control_sequence.vx(i + 1);
            if (i < settings.time_steps - 2) vx_filtered += c2 * control_sequence.vx(i + 2);
            
            // 结合控制历史进行平滑（开头几个点）
            if (i == 0) {
                vx_filtered = 0.5f * vx_filtered + 0.5f * control_history[0].vx;
            } else if (i == 1) {
                vx_filtered = 0.6f * vx_filtered + 0.4f * control_history[1].vx;
            }
            
            control_sequence.vx(i) = vx_filtered;
            
            // wz滤波
            float wz_filtered = c0 * control_sequence.wz(i);
            if (i > 0) wz_filtered += c1 * control_sequence.wz(i - 1);
            if (i > 1) wz_filtered += c2 * control_sequence.wz(i - 2);
            if (i < settings.time_steps - 1) wz_filtered += c1 * control_sequence.wz(i + 1);
            if (i < settings.time_steps - 2) wz_filtered += c2 * control_sequence.wz(i + 2);
            
            if (i == 0) {
                wz_filtered = 0.5f * wz_filtered + 0.5f * control_history[0].wz;
            } else if (i == 1) {
                wz_filtered = 0.6f * wz_filtered + 0.4f * control_history[1].wz;
            }
            
            control_sequence.wz(i) = wz_filtered;
            
            // vy滤波（全向模型）
            if (isHolonomic()) {
                float vy_filtered = c0 * control_sequence.vy(i);
                if (i > 0) vy_filtered += c1 * control_sequence.vy(i - 1);
                if (i > 1) vy_filtered += c2 * control_sequence.vy(i - 2);
                if (i < settings.time_steps - 1) vy_filtered += c1 * control_sequence.vy(i + 1);
                if (i < settings.time_steps - 2) vy_filtered += c2 * control_sequence.vy(i + 2);
                
                if (i == 0) {
                    vy_filtered = 0.5f * vy_filtered + 0.5f * control_history[0].vy;
                } else if (i == 1) {
                    vy_filtered = 0.6f * vy_filtered + 0.4f * control_history[1].vy;
                }
                
                control_sequence.vy(i) = vy_filtered;
            }
        }
    }

private:
    OptimizerSettings settings_;
    std::shared_ptr<MotionModel> motion_model_;
    CriticManager * critic_manager_ = nullptr;
    State state_;
    ControlSequence control_sequence_;
    Trajectories generated_trajectories_;
    Path path_;
    xt::xtensor<float, 1> costs_;
    NoiseGenerator noise_generator_;
    Twist2D last_control_;
    Twist2D last_speed_;
    std::vector<ObstaclePrediction> dynamic_obstacles_;
    bool is_blocked_ = false;  ///< 上一次评估是否检测到阻塞
    int spinning_counter_ = 0; ///< 连续检测到打转的帧数
    size_t fallback_counter_ = 0; ///< fallback重试计数器（线程安全，成员变量替代static）
    
    // 控制历史（用于SG滤波，参考Nav2）
    std::array<Control, 4> control_history_;
};

// ==================== 对外接口：MPPIController ====================

/**
 * @brief MPPI 控制器对外接口类，封装优化器和代价函数管理器
 */
class MPPIController
{
public:
    /**
     * @brief 初始化控制器
     * @param settings 优化器设置
     * @param motion_model_type 运动模型类型："DiffDrive", "Omni", "Ackermann"
     * @param ackermann_min_turning_radius 阿克曼模型的最小转弯半径（仅当类型为 Ackermann 时有效）
     */
    void initialize(const OptimizerSettings & settings,
                    const std::string & motion_model_type = "DiffDrive",
                    float ackermann_min_turning_radius = 0.2f)
    {
        settings_ = settings;
        if (motion_model_type == "DiffDrive") motion_model_ = std::make_shared<DiffDriveMotionModel>();
        else if (motion_model_type == "Omni") motion_model_ = std::make_shared<OmniMotionModel>();
        else if (motion_model_type == "Ackermann") motion_model_ = std::make_shared<AckermannMotionModel>(ackermann_min_turning_radius);
        else throw std::runtime_error("Unknown motion model");

        critic_manager_ = std::make_unique<CriticManager>();

        // 创建所有 critic（但参数稍后由外部通过 setCriticParams 设置）
        auto obstacles_critic = std::make_unique<ObstaclesCritic>();
        obstacles_critic->setName("ObstaclesCritic");
        obstacles_critic_ = obstacles_critic.get();
        critic_manager_->addCritic(std::move(obstacles_critic));

        auto goal_critic = std::make_unique<GoalCritic>();
        goal_critic->setName("GoalCritic");
        goal_critic_ = goal_critic.get();
        critic_manager_->addCritic(std::move(goal_critic));

        auto goal_angle_critic = std::make_unique<GoalAngleCritic>();
        goal_angle_critic->setName("GoalAngleCritic");
        goal_angle_critic_ = goal_angle_critic.get();
        critic_manager_->addCritic(std::move(goal_angle_critic));

        auto path_align_critic = std::make_unique<PathAlignCritic>();
        path_align_critic->setName("PathAlignCritic");
        path_align_critic_ = path_align_critic.get();
        critic_manager_->addCritic(std::move(path_align_critic));

        auto path_angle_critic = std::make_unique<PathAngleCritic>();
        path_angle_critic->setName("PathAngleCritic");
        path_angle_critic_ = path_angle_critic.get();
        critic_manager_->addCritic(std::move(path_angle_critic));

        auto path_follow_critic = std::make_unique<PathFollowCritic>();
        path_follow_critic->setName("PathFollowCritic");
        path_follow_critic_ = path_follow_critic.get();
        critic_manager_->addCritic(std::move(path_follow_critic));

        auto prefer_forward_critic = std::make_unique<PreferForwardCritic>();
        prefer_forward_critic->setName("PreferForwardCritic");
        prefer_forward_critic_ = prefer_forward_critic.get();
        critic_manager_->addCritic(std::move(prefer_forward_critic));

        auto constraint_critic = std::make_unique<ConstraintCritic>();
        constraint_critic->setName("ConstraintCritic");
        constraint_critic_ = constraint_critic.get();
        critic_manager_->addCritic(std::move(constraint_critic));

        auto twirling_critic = std::make_unique<TwirlingCritic>();
        twirling_critic->setName("TwirlingCritic");
        twirling_critic_ = twirling_critic.get();
        // 默认使用settings中的权重和vx_max初始化
        twirling_critic_->setParams(settings_.twirling_weight, settings_.base_constraints.vx_max);
        critic_manager_->addCritic(std::move(twirling_critic));
        
        auto velocity_deadband_critic = std::make_unique<VelocityDeadbandCritic>();
        velocity_deadband_critic->setName("VelocityDeadbandCritic");
        velocity_deadband_critic_ = velocity_deadband_critic.get();
        critic_manager_->addCritic(std::move(velocity_deadband_critic));

        critic_manager_->initializeCritics();

        optimizer_ = std::make_unique<Optimizer>();
        optimizer_->initialize(settings_, motion_model_, critic_manager_.get());
    }

    /**
     * @brief 从 YAML 解析后设置各个 critic 的参数
     * @param name 代价函数名称
     * @param params 参数列表（顺序和数量与对应的 setParams 一致）
     */
    void setCriticParams(const std::string & name, const std::vector<float> & params)
    {
        auto * critic = critic_manager_->getCritic(name);
        if (!critic) return;

        if (name == "ObstaclesCritic") {
            auto * obs = dynamic_cast<ObstaclesCritic*>(critic);
            if (obs && params.size() >= 11) {
                obs->setParams(params[0], params[1], params[2], params[3],
                               params[4], params[5], params[6], params[7],
                               params[8], static_cast<int>(params[9]), static_cast<int>(params[10]));
                robot_radius_ = params[7];
            }
        } else if (name == "GoalCritic") {
            auto * g = dynamic_cast<GoalCritic*>(critic);
            if (g && params.size() >= 2) g->setParams(params[0], params[1]);
        } else if (name == "GoalAngleCritic") {
            auto * ga = dynamic_cast<GoalAngleCritic*>(critic);
            if (ga && params.size() >= 2) ga->setParams(params[0], params[1]);
        } else if (name == "PathAlignCritic") {
            auto * pa = dynamic_cast<PathAlignCritic*>(critic);
            if (pa && params.size() >= 6) pa->setParams(params[0], static_cast<int>(params[1]), params[2], static_cast<int>(params[3]), params[4], params[5] > 0.5f);
        } else if (name == "PathAngleCritic") {
            auto * pang = dynamic_cast<PathAngleCritic*>(critic);
            if (pang && params.size() >= 5) pang->setParams(params[0], static_cast<int>(params[1]), params[2], params[3], static_cast<int>(params[4]));
        } else if (name == "PathFollowCritic") {
            auto * pf = dynamic_cast<PathFollowCritic*>(critic);
            if (pf && params.size() >= 3) pf->setParams(params[0], static_cast<int>(params[1]), params[2]);
        } else if (name == "PreferForwardCritic") {
            auto * pfwd = dynamic_cast<PreferForwardCritic*>(critic);
            if (pfwd && params.size() >= 2) pfwd->setParams(params[0], params[1]);
        } else if (name == "ConstraintCritic") {
            auto * cc = dynamic_cast<ConstraintCritic*>(critic);
            if (cc && params.size() >= 7) cc->setParams(params[0], params[1], params[2], params[3], params[4], params[5], static_cast<int>(params[6]));
        } else if (name == "VelocityDeadbandCritic") {
            auto * vdc = dynamic_cast<VelocityDeadbandCritic*>(critic);
            if (vdc && params.size() >= 4) vdc->setParams(params[0], params[1], params[2], params[3]);
        }
    }

    /**
     * @brief 设置全局路径（通过 Pose2D 列表）
     * @param path 路径点列表
     */
    void setPath(const std::vector<Pose2D> & path) {
        path_.reset(path.size());
        for (size_t i = 0; i < path.size(); ++i) {
            path_.x(i) = path[i].x;
            path_.y(i) = path[i].y;
            path_.yaws(i) = path[i].theta;
        }
    }

    /** @brief 设置全局路径（直接使用 Path 结构） */
    void setPath(const Path & path) { path_ = path; }

    /** @brief 设置动态障碍物列表 */
    void setDynamicObstacles(const std::vector<ObstaclePrediction> & obstacles) {
        optimizer_->setDynamicObstacles(obstacles);
    }

    /**
     * @brief 更新静态障碍物点云，并传入当前机器人位姿用于栅格重建
     * @param points 静态障碍物点集（世界坐标系）
     * @param robot_pose 当前机器人位姿（用于确定栅格中心）
     */
    void updateStaticObstacles(const std::vector<Point2D> & points, const Pose2D & robot_pose) {
        if (obstacles_critic_)
            obstacles_critic_->setLaserPoints(points, robot_pose);
        if (path_align_critic_)
            path_align_critic_->setObstacles(&points, robot_radius_);
    }

    /**
     * @brief 计算速度指令（主循环调用）
     * @param robot_pose 当前机器人位姿
     * @param robot_speed 当前机器人速度
     * @return 计算出的控制量（Twist2D）
     */
    Twist2D computeVelocityCommands(const Pose2D & robot_pose, const Twist2D & robot_speed)
    {
        if (path_.empty()) return Twist2D();
        return optimizer_->evalControl(robot_pose, robot_speed, path_);
    }

    // Getters
    ObstaclesCritic* getObstaclesCritic() const { return obstacles_critic_; }
    GoalCritic* getGoalCritic() const { return goal_critic_; }
    GoalAngleCritic* getGoalAngleCritic() const { return goal_angle_critic_; }
    PathAlignCritic* getPathAlignCritic() const { return path_align_critic_; }
    PathAngleCritic* getPathAngleCritic() const { return path_angle_critic_; }
    PathFollowCritic* getPathFollowCritic() const { return path_follow_critic_; }
    PreferForwardCritic* getPreferForwardCritic() const { return prefer_forward_critic_; }
    ConstraintCritic* getConstraintCritic() const { return constraint_critic_; }
    TwirlingCritic* getTwirlingCritic() const { return twirling_critic_; }
    VelocityDeadbandCritic* getVelocityDeadbandCritic() const { return velocity_deadband_critic_; }

    /** @return 生成的采样轨迹（用于可视化） */
    Trajectories & getGeneratedTrajectories() { return optimizer_->getGeneratedTrajectories(); }
    /** @return 最优轨迹（用于可视化） */
    xt::xtensor<float, 2> getOptimizedTrajectory() { return optimizer_->getOptimizedTrajectory(); }
    /** @brief 重置优化器内部状态 */
    void reset() { optimizer_->reset(); }
    /** @return 是否为全向模型 */
    bool isHolonomic() const { return motion_model_->isHolonomic(); }
    /** @return 当前路径引用 */
    Path& getPath() { return path_; }
    /** @return 当前是否被阻塞 */
    bool isCurrentlyBlocked() const { return optimizer_->isCurrentlyBlocked(); }

private:
    OptimizerSettings settings_;
    std::shared_ptr<MotionModel> motion_model_;
    std::unique_ptr<CriticManager> critic_manager_;
    std::unique_ptr<Optimizer> optimizer_;
    Path path_;

    // 存储各代价函数的原始指针，方便外部访问
    ObstaclesCritic* obstacles_critic_ = nullptr;
    GoalCritic* goal_critic_ = nullptr;
    GoalAngleCritic* goal_angle_critic_ = nullptr;
    PathAlignCritic* path_align_critic_ = nullptr;
    PathAngleCritic* path_angle_critic_ = nullptr;
    PathFollowCritic* path_follow_critic_ = nullptr;
    PreferForwardCritic* prefer_forward_critic_ = nullptr;
    ConstraintCritic* constraint_critic_ = nullptr;
    TwirlingCritic* twirling_critic_ = nullptr;
    VelocityDeadbandCritic* velocity_deadband_critic_ = nullptr;
    float robot_radius_ = 0.3f;
};

} // namespace mppi

#endif // MPPI_CONTROLLER_HPP
