// ============================================================================
// 文件：mppi_controller.hpp
// 功能：纯C++版MPPI控制器 (工业级完善版) – 独立障碍物处理机制（修复版）
// 版本：适配 xtensor 0.21.0
// 特性：加速度限制、独立障碍物判断机制、绕障路径生成、多线程优化
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

inline float normalizeAngle(float angle)
{
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

inline float shortestAngularDistance(float from, float to)
{
    return normalizeAngle(to - from);
}

inline float clamp(float val, float min_val, float max_val)
{
    return std::max(min_val, std::min(val, max_val));
}

// ==================== 数据结构定义 ====================

struct Point2D
{
    float x = 0.0f;
    float y = 0.0f;
    Point2D() = default;
    Point2D(float x_, float y_) : x(x_), y(y_) {}
};

struct Pose2D
{
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
    Pose2D() = default;
    Pose2D(float x_, float y_, float theta_) : x(x_), y(y_), theta(theta_) {}
};

struct Twist2D
{
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    Twist2D() = default;
    Twist2D(float vx_, float vy_, float wz_) : vx(vx_), vy(vy_), wz(wz_) {}
};

struct ControlConstraints
{
    float vx_max = 0.5f;
    float vx_min = -0.35f;
    float vy_max = 0.5f;
    float wz_max = 1.9f;
    float ax_max = 2.0f;
    float ay_max = 2.0f;
    float az_max = 3.0f;
    float collision_cost_threshold = 50000.0f;
};

struct SamplingStd
{
    float vx = 0.2f;
    float vy = 0.2f;
    float wz = 0.4f;
};

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
    unsigned int thread_count = 4;
};

struct Control
{
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
};

struct ControlSequence
{
    xt::xtensor<float, 1> vx;
    xt::xtensor<float, 1> vy;
    xt::xtensor<float, 1> wz;

    void reset(unsigned int time_steps)
    {
        vx = xt::zeros<float>({time_steps});
        vy = xt::zeros<float>({time_steps});
        wz = xt::zeros<float>({time_steps});
    }
};

struct State
{
    xt::xtensor<float, 2> vx;
    xt::xtensor<float, 2> vy;
    xt::xtensor<float, 2> wz;
    xt::xtensor<float, 2> cvx;
    xt::xtensor<float, 2> cvy;
    xt::xtensor<float, 2> cwz;
    Pose2D pose;
    Twist2D speed;

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

struct Trajectories
{
    xt::xtensor<float, 2> x;
    xt::xtensor<float, 2> y;
    xt::xtensor<float, 2> yaws;
    xt::xtensor<float, 2> times;

    void reset(unsigned int batch_size, unsigned int time_steps)
    {
        x = xt::zeros<float>({batch_size, time_steps});
        y = xt::zeros<float>({batch_size, time_steps});
        yaws = xt::zeros<float>({batch_size, time_steps});
        times = xt::zeros<float>({batch_size, time_steps});
    }
};

struct Path
{
    xt::xtensor<float, 1> x;
    xt::xtensor<float, 1> y;
    xt::xtensor<float, 1> yaws;

    void reset(unsigned int size)
    {
        x = xt::zeros<float>({size});
        y = xt::zeros<float>({size});
        yaws = xt::zeros<float>({size});
    }

    size_t size() const { return x.shape(0); }
    bool empty() const { return x.shape(0) == 0; }

    Pose2D getGoal() const
    {
        if (empty()) return Pose2D();
        size_t last = size() - 1;
        return Pose2D(x(last), y(last), yaws(last));
    }
};

struct DynamicObstacle
{
    float x, y;
    float vx, vy;
    float radius;
    bool is_moving;

    Point2D positionAt(float t) const {
        return Point2D(x + vx * t, y + vy * t);
    }
};

// ==================== 避障配置参数 ====================

struct AvoidanceConfig
{
    bool enable_avoidance = true;
    float max_lateral_avoidance_distance = 0.8f;  // 增加最大横向避障距离
    float safety_margin_lateral = 0.15f;          // 减小横向安全距离
    float safety_margin_longitudinal = 0.3f;      // 减小纵向安全距离
    float robot_radius = 0.3f;
    float emergency_brake_decel = 2.0f;
    float ttc_emergency_threshold = 0.8f;         // 降低TTC紧急阈值
    float follow_distance = 0.8f;                 // 减小跟随距离
    float grid_resolution = 0.05f;
    int grid_width = 100;
    int grid_height = 100;
    float avoidance_lookahead = 4.0f;             // 增加避障前瞻距离
    float avoidance_smooth_distance = 0.8f;       // 增加平滑距离
    float avoid_speed_ratio = 0.6f;               // 降低避障时的速度比例
    float min_avoid_speed = 0.05f;                // 降低最小避障速度
    float prediction_dt = 0.05f;
    float early_avoidance_distance = 0.2f;        // 大幅减小提前避障距离
    float obstacle_inflation = 0.05f;
};

// ==================== 障碍物处理结果 ====================

enum class ObstacleAction
{
    NONE,
    EMERGENCY_BRAKE,
    NORMAL_STOP,
    FOLLOW,
    LATERAL_AVOID
};

struct ObstacleResult
{
    ObstacleAction action = ObstacleAction::NONE;
    float target_speed = 0.0f;
    float lateral_offset = 0.0f;
    float min_distance = std::numeric_limits<float>::max();
    bool path_blocked = false;
    bool collision_imminent = false;
    Point2D closest_obstacle_pos;
    size_t obstacle_path_idx = 0;
    size_t avoid_start_idx = 0;
    size_t avoid_end_idx = 0;
};

// ==================== 独立障碍物处理器（修复版）====================

class ObstacleHandler
{
public:
    void initialize(const AvoidanceConfig & config)
    {
        config_ = config;
        grid_distance_field_.resize(config_.grid_width * config_.grid_height, 
                                     std::numeric_limits<float>::max());
    }

    void setConfig(const AvoidanceConfig & config)
    {
        config_ = config;
        grid_distance_field_.resize(config_.grid_width * config_.grid_height,
                                     std::numeric_limits<float>::max());
    }

    void updateStaticObstacles(const std::vector<Point2D> & points, const Pose2D & robot_pose)
    {
        static_obstacles_ = points;
        robot_pose_ = robot_pose;
        rebuildGrid(robot_pose);
    }

    void updateDynamicObstacles(const std::vector<DynamicObstacle> & obstacles)
    {
        dynamic_obstacles_ = obstacles;
    }

    ObstacleResult evaluate(const Path & path, const Twist2D & current_speed)
    {
        ObstacleResult result;
        
        if (static_obstacles_.empty() && dynamic_obstacles_.empty()) {
            return result;
        }

        // 分别评估静态和动态，取优先级最高的结果
        ObstacleResult static_result = evaluateStaticObstacles(path, current_speed);
        ObstacleResult dynamic_result = evaluateDynamicObstacles(path, current_speed);
        
        // 合并结果，优先级：EMERGENCY_BRAKE > NORMAL_STOP > LATERAL_AVOID > FOLLOW > NONE
        auto priority = [](ObstacleAction a) {
            switch(a) {
                case ObstacleAction::EMERGENCY_BRAKE: return 5;
                case ObstacleAction::NORMAL_STOP: return 4;
                case ObstacleAction::LATERAL_AVOID: return 3;
                case ObstacleAction::FOLLOW: return 2;
                default: return 1;
            }
        };

        if (priority(static_result.action) > priority(dynamic_result.action)) {
            result = static_result;
        } else if (priority(dynamic_result.action) > priority(static_result.action)) {
            result = dynamic_result;
        } else {
            // 相同优先级，取更小的距离
            result = (static_result.min_distance < dynamic_result.min_distance) ? static_result : dynamic_result;
        }

        last_result_ = result;
        return result;
    }

    Path generateAvoidancePath(const Path & original_path, const ObstacleResult & result)
    {
        Path avoidance_path;
        if (original_path.empty() || result.action != ObstacleAction::LATERAL_AVOID) {
            return original_path;
        }

        avoidance_path.reset(original_path.size());
        
        for (size_t i = 0; i < original_path.size(); ++i) {
            avoidance_path.x(i) = original_path.x(i);
            avoidance_path.y(i) = original_path.y(i);
            avoidance_path.yaws(i) = original_path.yaws(i);
        }

        if (result.lateral_offset == 0.0f) {
            return avoidance_path;
        }

        size_t start_idx = result.avoid_start_idx;
        size_t end_idx = std::min(result.avoid_end_idx, original_path.size() - 1);
        
        if (start_idx >= end_idx || start_idx >= original_path.size() - 1) {
            return avoidance_path;
        }

        // 生成平滑横向偏移路径
        for (size_t i = start_idx; i <= end_idx && i < original_path.size() - 1; ++i) {
            float path_dx = original_path.x(i + 1) - original_path.x(i);
            float path_dy = original_path.y(i + 1) - original_path.y(i);
            float path_len = std::hypot(path_dx, path_dy);
            
            if (path_len < EPSILON) continue;
            
            float path_nx = -path_dy / path_len;
            float path_ny = path_dx / path_len;
            
            float t = static_cast<float>(i - start_idx) / static_cast<float>(end_idx - start_idx + 1);
            float smooth_factor = std::sin(t * PI); // 0->0, 中间最大, 末尾->0
            
            float offset = result.lateral_offset * smooth_factor;
            
            avoidance_path.x(i) = original_path.x(i) + offset * path_nx;
            avoidance_path.y(i) = original_path.y(i) + offset * path_ny;
        }
        
        if (end_idx < original_path.size() - 1) {
            for (size_t i = end_idx + 1; i < original_path.size(); ++i) {
                avoidance_path.x(i) = original_path.x(i);
                avoidance_path.y(i) = original_path.y(i);
            }
        }

        // 重新计算航向角
        for (size_t i = 1; i < avoidance_path.size(); ++i) {
            float dx = avoidance_path.x(i) - avoidance_path.x(i-1);
            float dy = avoidance_path.y(i) - avoidance_path.y(i-1);
            avoidance_path.yaws(i) = std::atan2(dy, dx);
        }

        // 碰撞检测：如果生成的避障路径与任何障碍物碰撞，则回退到停车
        if (isPathColliding(avoidance_path)) {
            ObstacleResult fallback;
            fallback.action = ObstacleAction::NORMAL_STOP;
            last_result_ = fallback;
            return original_path; // 返回原路径，但后续控制器会停车
        }

        return avoidance_path;
    }

    bool isPathBlocked() const { return last_result_.path_blocked; }
    const ObstacleResult & getLastResult() const { return last_result_; }

    float getGridDistance(float x, float y) const
    {
        int ix = static_cast<int>((x - grid_origin_x_) / config_.grid_resolution);
        int iy = static_cast<int>((y - grid_origin_y_) / config_.grid_resolution);
        if (ix < 0 || ix >= config_.grid_width || iy < 0 || iy >= config_.grid_height)
            return std::numeric_limits<float>::max();
        return grid_distance_field_[iy * config_.grid_width + ix];
    }

private:
    ObstacleResult evaluateStaticObstacles(const Path & path, const Twist2D & current_speed)
    {
        ObstacleResult result;
        
        if (path.empty() || static_obstacles_.empty()) return result;

        size_t closest_path_idx = findClosestPathIndex(path);
        
        std::vector<size_t> path_indices = getPathSegment(path, closest_path_idx, config_.avoidance_lookahead);
        
        // 存储最佳的避障选项
        ObstacleResult best_avoid_result;
        bool found_avoid_option = false;
        float min_obstacle_distance = std::numeric_limits<float>::max();
        Point2D closest_obstacle_pos;
        size_t closest_obstacle_idx = 0;

        for (size_t idx : path_indices) {
            float px = path.x(idx);
            float py = path.y(idx);
            
            float obs_dist = getObstacleDistance(px, py);
            float min_dist = obs_dist - config_.robot_radius;
            
            // 记录最近的障碍物信息
            if (min_dist < min_obstacle_distance) {
                min_obstacle_distance = min_dist;
                closest_obstacle_pos = findClosestObstacle(px, py);
                closest_obstacle_idx = idx;
            }
            
            // 如果距离非常近，需要紧急处理
            if (min_dist < config_.safety_margin_lateral) {
                if (min_dist < config_.early_avoidance_distance) {
                    result.action = ObstacleAction::EMERGENCY_BRAKE;
                    result.collision_imminent = true;
                    result.min_distance = min_dist;
                    result.closest_obstacle_pos = closest_obstacle_pos;
                    result.obstacle_path_idx = closest_obstacle_idx;
                    result.path_blocked = true;
                    return result;
                }
            }
            
            // 如果距离在可避障范围内，尝试计算避障方案
            if (min_dist < config_.safety_margin_longitudinal && config_.enable_avoidance) {
                float lateral_offset = calculateLateralOffset(
                    closest_obstacle_pos.x, 
                    closest_obstacle_pos.y, 
                    path, idx);
                
                // 如果所需横向偏移在允许范围内
                if (std::abs(lateral_offset) <= config_.max_lateral_avoidance_distance) {
                    float avoid_offset = (lateral_offset >= 0) ? 
                        config_.max_lateral_avoidance_distance : -config_.max_lateral_avoidance_distance;
                    
                    size_t avoid_start_idx = getIndexByDistance(path, idx, config_.avoidance_smooth_distance, false);
                    size_t avoid_end_idx = getIndexByDistance(path, idx, config_.avoidance_smooth_distance * 2.0f, true);
                    
                    // 检查整段避障路径是否安全（仅静态障碍物）
                    if (canAvoidLaterallyOnSegmentStatic(path, avoid_start_idx, avoid_end_idx, avoid_offset)) {
                        // 找到一个可行的避障方案，记录但不立即返回
                        ObstacleResult avoid_result;
                        avoid_result.action = ObstacleAction::LATERAL_AVOID;
                        avoid_result.lateral_offset = avoid_offset;
                        avoid_result.target_speed = std::max(config_.min_avoid_speed,
                                                           current_speed.vx * config_.avoid_speed_ratio);
                        avoid_result.avoid_start_idx = avoid_start_idx;
                        avoid_result.avoid_end_idx = avoid_end_idx;
                        avoid_result.min_distance = min_dist;
                        avoid_result.closest_obstacle_pos = closest_obstacle_pos;
                        avoid_result.obstacle_path_idx = idx;
                        
                        // 选择距离最近的可行避障方案
                        if (!found_avoid_option || min_dist < best_avoid_result.min_distance) {
                            best_avoid_result = avoid_result;
                            found_avoid_option = true;
                        }
                    }
                }
            }
        }
        
        // 如果找到了可行的避障方案，使用它
        if (found_avoid_option) {
            return best_avoid_result;
        }
        
        // 如果没有找到避障方案，但有障碍物在安全距离内，才停车
        if (min_obstacle_distance < config_.safety_margin_longitudinal) {
            result.action = ObstacleAction::NORMAL_STOP;
            result.min_distance = min_obstacle_distance;
            result.closest_obstacle_pos = closest_obstacle_pos;
            result.obstacle_path_idx = closest_obstacle_idx;
            result.path_blocked = true;
            return result;
        }
        
        // 没有障碍物威胁，返回NONE
        return result;
    }

    ObstacleResult evaluateDynamicObstacles(const Path & path, const Twist2D & current_speed)
    {
        ObstacleResult result;
        
        if (dynamic_obstacles_.empty()) return result;

        size_t closest_path_idx = findClosestPathIndex(path);
        
        // 预先计算路径累计距离，用于时间匹配
        std::vector<float> cumulative_dist(path.size(), 0.0f);
        for (size_t i = closest_path_idx + 1; i < path.size(); ++i) {
            float dx = path.x(i) - path.x(i-1);
            float dy = path.y(i) - path.y(i-1);
            cumulative_dist[i] = cumulative_dist[i-1] + std::hypot(dx, dy);
        }

        // 存储最佳的避障选项和最危险的障碍物
        ObstacleResult best_avoid_result;
        bool found_avoid_option = false;
        float min_obstacle_distance = std::numeric_limits<float>::max();
        Point2D closest_obstacle_pos;
        size_t closest_obstacle_idx = 0;
        DynamicObstacle closest_dynamic_obs;
        float closest_time_to_point = 0.0f;

        for (const auto & obs : dynamic_obstacles_) {
            for (size_t idx = closest_path_idx; idx < path.size(); ++idx) {
                // 根据累计距离估计到达该路径点的时间（假设以当前速度行驶）
                float dist_to_point = cumulative_dist[idx] - cumulative_dist[closest_path_idx];
                float time_to_point = (current_speed.vx > EPSILON) ? dist_to_point / current_speed.vx : 0.0f;
                
                Point2D pred_pos = obs.positionAt(time_to_point);
                
                float px = path.x(idx);
                float py = path.y(idx);
                float dist = std::hypot(pred_pos.x - px, pred_pos.y - py) - obs.radius - config_.robot_radius;
                
                // 记录最近的障碍物信息
                if (dist < min_obstacle_distance) {
                    min_obstacle_distance = dist;
                    closest_obstacle_pos = pred_pos;
                    closest_obstacle_idx = idx;
                    closest_dynamic_obs = obs;
                    closest_time_to_point = time_to_point;
                }
                
                // 紧急情况处理
                if (dist < config_.safety_margin_lateral) {
                    float ttc = calculateTTC(obs, current_speed);
                    if (ttc < config_.ttc_emergency_threshold) {
                        result.action = ObstacleAction::EMERGENCY_BRAKE;
                        result.collision_imminent = true;
                        result.min_distance = dist;
                        result.closest_obstacle_pos = pred_pos;
                        result.obstacle_path_idx = idx;
                        result.path_blocked = true;
                        return result;
                    }
                }
                
                // 尝试横向避障
                if (dist < config_.safety_margin_longitudinal && config_.enable_avoidance) {
                    float lateral_offset = calculateLateralOffset(pred_pos.x, pred_pos.y, path, idx);
                    
                    if (std::abs(lateral_offset) <= config_.max_lateral_avoidance_distance) {
                        float avoid_offset = (lateral_offset >= 0) ? 
                            config_.max_lateral_avoidance_distance : -config_.max_lateral_avoidance_distance;
                        
                        size_t avoid_start_idx = getIndexByDistance(path, idx, config_.avoidance_smooth_distance, false);
                        size_t avoid_end_idx = getIndexByDistance(path, idx, config_.avoidance_smooth_distance * 2.0f, true);
                        
                        // 检查整段避障路径是否同时满足静态和动态障碍物安全
                        if (canAvoidLaterallyOnSegmentDynamic(path, avoid_start_idx, avoid_end_idx, avoid_offset, obs, time_to_point)) {
                            // 找到一个可行的避障方案
                            ObstacleResult avoid_result;
                            avoid_result.action = ObstacleAction::LATERAL_AVOID;
                            avoid_result.lateral_offset = avoid_offset;
                            avoid_result.target_speed = std::max(config_.min_avoid_speed,
                                                               current_speed.vx * config_.avoid_speed_ratio);
                            avoid_result.avoid_start_idx = avoid_start_idx;
                            avoid_result.avoid_end_idx = avoid_end_idx;
                            avoid_result.min_distance = dist;
                            avoid_result.closest_obstacle_pos = pred_pos;
                            avoid_result.obstacle_path_idx = idx;
                            
                            // 选择距离最近的可行避障方案
                            if (!found_avoid_option || dist < best_avoid_result.min_distance) {
                                best_avoid_result = avoid_result;
                                found_avoid_option = true;
                            }
                        }
                    }
                }
            }
        }
        
        // 如果找到了可行的避障方案，使用它
        if (found_avoid_option) {
            return best_avoid_result;
        }
        
        // 如果没有找到避障方案，但有障碍物在安全距离内
        if (min_obstacle_distance < config_.safety_margin_longitudinal) {
            // 根据是否在路径上决定跟随或停车
            if (isObstacleOnPath(closest_dynamic_obs, path, closest_path_idx)) {
                result.action = ObstacleAction::FOLLOW;
                result.target_speed = calculateFollowSpeed(closest_dynamic_obs, current_speed);
            } else {
                result.action = ObstacleAction::NORMAL_STOP;
                result.path_blocked = true;
            }
            result.min_distance = min_obstacle_distance;
            result.closest_obstacle_pos = closest_obstacle_pos;
            result.obstacle_path_idx = closest_obstacle_idx;
            return result;
        }
        
        return result;
    }
                        result.target_speed = calculateFollowSpeed(obs, current_speed);
                    } else {
                        result.action = ObstacleAction::NORMAL_STOP;
                        result.path_blocked = true;
                    }
                    return result;
                }
            }
        }
        
        return result;
    }

    float calculateTTC(const DynamicObstacle & obs, const Twist2D & current_speed)
    {
        float dx = obs.x - robot_pose_.x;
        float dy = obs.y - robot_pose_.y;
        float dist = std::hypot(dx, dy);
        
        float rel_vx = obs.vx - current_speed.vx * std::cos(robot_pose_.theta);
        float rel_vy = obs.vy - current_speed.vx * std::sin(robot_pose_.theta);
        
        float closing_speed = -(dx * rel_vx + dy * rel_vy) / dist;
        
        if (closing_speed <= 0) return std::numeric_limits<float>::max();
        
        return (dist - obs.radius - config_.robot_radius) / closing_speed;
    }

    float calculateFollowSpeed(const DynamicObstacle & obs, const Twist2D & current_speed)
    {
        float dist = std::hypot(obs.x - robot_pose_.x, obs.y - robot_pose_.y);
        float safe_dist = config_.follow_distance + obs.radius + config_.robot_radius;
        
        if (dist <= safe_dist) {
            return std::hypot(obs.vx, obs.vy);
        }
        
        float speed_diff = current_speed.vx - std::hypot(obs.vx, obs.vy);
        float decel_needed = speed_diff * speed_diff / (2.0f * (dist - safe_dist));
        
        return std::max(0.0f, current_speed.vx - decel_needed * config_.prediction_dt);
    }

    bool isObstacleOnPath(const DynamicObstacle & obs, const Path & path, size_t start_idx)
    {
        for (size_t idx = start_idx; idx < path.size(); ++idx) {
            float dist = std::hypot(obs.x - path.x(idx), obs.y - path.y(idx));
            if (dist < obs.radius + config_.robot_radius + config_.safety_margin_lateral) {
                return true;
            }
        }
        return false;
    }

    size_t findClosestPathIndex(const Path & path)
    {
        size_t closest_idx = 0;
        float min_dist = std::numeric_limits<float>::max();
        
        for (size_t i = 0; i < path.size(); ++i) {
            float dist = std::hypot(path.x(i) - robot_pose_.x, path.y(i) - robot_pose_.y);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        return closest_idx;
    }

    std::vector<size_t> getPathSegment(const Path & path, size_t start_idx, float distance)
    {
        std::vector<size_t> indices;
        float accumulated_dist = 0.0f;
        
        for (size_t i = start_idx; i < path.size() - 1; ++i) {
            indices.push_back(i);
            float segment_dist = std::hypot(path.x(i+1) - path.x(i), path.y(i+1) - path.y(i));
            accumulated_dist += segment_dist;
            if (accumulated_dist > distance) break;
        }
        
        return indices;
    }

    size_t getIndexByDistance(const Path & path, size_t start_idx, float target_distance, bool forward = true)
    {
        if (path.empty()) return start_idx;
        
        float accumulated_dist = 0.0f;
        
        if (forward) {
            for (size_t i = start_idx; i < path.size() - 1; ++i) {
                float segment_dist = std::hypot(path.x(i+1) - path.x(i), path.y(i+1) - path.y(i));
                accumulated_dist += segment_dist;
                if (accumulated_dist >= target_distance) {
                    return i + 1;
                }
            }
            return path.size() - 1;
        } else {
            for (size_t i = start_idx; i > 0; --i) {
                float segment_dist = std::hypot(path.x(i) - path.x(i-1), path.y(i) - path.y(i-1));
                accumulated_dist += segment_dist;
                if (accumulated_dist >= target_distance) {
                    return i - 1;
                }
            }
            return 0;
        }
    }

    float calculateLateralOffset(float obs_x, float obs_y, const Path & path, size_t path_idx)
    {
        if (path_idx >= path.size() - 1) return 0.0f;
        
        float path_dx = path.x(path_idx + 1) - path.x(path_idx);
        float path_dy = path.y(path_idx + 1) - path.y(path_idx);
        float path_len = std::hypot(path_dx, path_dy);
        
        if (path_len < EPSILON) return 0.0f;
        
        float path_nx = -path_dy / path_len;
        float path_ny = path_dx / path_len;
        
        float dx = obs_x - path.x(path_idx);
        float dy = obs_y - path.y(path_idx);
        
        return dx * path_nx + dy * path_ny;
    }

    // 检查静态障碍物下横向避障是否可行
    bool canAvoidLaterallyOnSegmentStatic(const Path & path, size_t start_idx, size_t end_idx, float lateral_offset)
    {
        if (start_idx >= path.size() || end_idx >= path.size() || start_idx >= end_idx) return false;
        
        for (size_t i = start_idx; i <= end_idx && i < path.size() - 1; ++i) {
            float path_dx = path.x(i + 1) - path.x(i);
            float path_dy = path.y(i + 1) - path.y(i);
            float path_len = std::hypot(path_dx, path_dy);
            
            if (path_len < EPSILON) continue;
            
            float path_nx = -path_dy / path_len;
            float path_ny = path_dx / path_len;
            
            float check_x = path.x(i) + lateral_offset * path_nx;
            float check_y = path.y(i) + lateral_offset * path_ny;
            
            float grid_dist = getGridDistance(check_x, check_y);
            float clearance = grid_dist - config_.robot_radius - config_.safety_margin_lateral;
            
            if (clearance <= 0) return false;
        }
        return true;
    }

    // 检查动态障碍物下横向避障是否可行（考虑障碍物预测）
    bool canAvoidLaterallyOnSegmentDynamic(const Path & path, size_t start_idx, size_t end_idx, float lateral_offset,
                                           const DynamicObstacle & obs, float base_time)
    {
        if (start_idx >= path.size() || end_idx >= path.size() || start_idx >= end_idx) return false;

        // 先检查静态障碍物
        if (!canAvoidLaterallyOnSegmentStatic(path, start_idx, end_idx, lateral_offset))
            return false;

        // 再检查动态障碍物
        for (size_t i = start_idx; i <= end_idx && i < path.size() - 1; ++i) {
            // 估计到达该路径点的时间（粗略，假设速度恒定）
            float time_to_point = base_time + (i - start_idx) * config_.prediction_dt; // 简化，可根据路径距离细化
            Point2D pred_obs_pos = obs.positionAt(time_to_point);

            float path_dx = path.x(i + 1) - path.x(i);
            float path_dy = path.y(i + 1) - path.y(i);
            float path_len = std::hypot(path_dx, path_dy);
            if (path_len < EPSILON) continue;

            float path_nx = -path_dy / path_len;
            float path_ny = path_dx / path_len;

            float check_x = path.x(i) + lateral_offset * path_nx;
            float check_y = path.y(i) + lateral_offset * path_ny;

            float dist_to_obs = std::hypot(check_x - pred_obs_pos.x, check_y - pred_obs_pos.y);
            float clearance = dist_to_obs - obs.radius - config_.robot_radius - config_.safety_margin_lateral;

            if (clearance <= 0) return false;
        }
        return true;
    }

    bool isPathColliding(const Path & path)
    {
        // 检查路径点是否与静态或动态障碍物碰撞
        for (size_t i = 0; i < path.size(); ++i) {
            float px = path.x(i);
            float py = path.y(i);

            // 静态障碍物
            float static_dist = getObstacleDistance(px, py);
            if (static_dist < config_.robot_radius + config_.safety_margin_lateral)
                return true;

            // 动态障碍物（简单检查当前时刻，实际应预测）
            for (const auto & obs : dynamic_obstacles_) {
                float dist = std::hypot(px - obs.x, py - obs.y);
                if (dist < obs.radius + config_.robot_radius + config_.safety_margin_lateral)
                    return true;
            }
        }
        return false;
    }

    Point2D findClosestObstacle(float px, float py) const
    {
        Point2D closest_obs(px, py);
        float min_dist = std::numeric_limits<float>::max();
        
        for (const auto & obs : static_obstacles_) {
            float dist = std::hypot(obs.x - px, obs.y - py);
            if (dist < min_dist) {
                min_dist = dist;
                closest_obs = obs;
            }
        }
        return closest_obs;
    }

    float getObstacleDistance(float px, float py) const
    {
        float min_dist = std::numeric_limits<float>::max();
        for (const auto & obs : static_obstacles_) {
            float dist = std::hypot(obs.x - px, obs.y - py);
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        return min_dist;
    }

    void rebuildGrid(const Pose2D & robot_pose)
    {
        grid_origin_x_ = robot_pose.x - config_.grid_width * config_.grid_resolution * 0.5f;
        grid_origin_y_ = robot_pose.y - config_.grid_height * config_.grid_resolution * 0.5f;

        std::fill(grid_distance_field_.begin(), grid_distance_field_.end(), 
                  std::numeric_limits<float>::max());

        for (const auto & p : static_obstacles_)
        {
            int ix = static_cast<int>((p.x - grid_origin_x_) / config_.grid_resolution);
            int iy = static_cast<int>((p.y - grid_origin_y_) / config_.grid_resolution);
            if (ix >= 0 && ix < config_.grid_width && iy >= 0 && iy < config_.grid_height)
                grid_distance_field_[iy * config_.grid_width + ix] = 0.0f;
        }

        std::queue<std::pair<int,int>> q;
        for (int i = 0; i < config_.grid_height; ++i)
            for (int j = 0; j < config_.grid_width; ++j)
                if (grid_distance_field_[i * config_.grid_width + j] == 0.0f)
                    q.push({j, i});

        const int dx[4] = {1, 0, -1, 0};
        const int dy[4] = {0, 1, 0, -1};
        while (!q.empty())
        {
            auto [x, y] = q.front(); q.pop();
            float cur_dist = grid_distance_field_[y * config_.grid_width + x];
            for (int k = 0; k < 4; ++k)
            {
                int nx = x + dx[k];
                int ny = y + dy[k];
                if (nx < 0 || nx >= config_.grid_width || ny < 0 || ny >= config_.grid_height)
                    continue;
                float nd = cur_dist + config_.grid_resolution;
                if (nd < grid_distance_field_[ny * config_.grid_width + nx])
                {
                    grid_distance_field_[ny * config_.grid_width + nx] = nd;
                    q.push({nx, ny});
                }
            }
        }
    }

    AvoidanceConfig config_;
    std::vector<Point2D> static_obstacles_;
    std::vector<DynamicObstacle> dynamic_obstacles_;
    Pose2D robot_pose_;
    ObstacleResult last_result_;
    
    std::vector<float> grid_distance_field_;
    float grid_origin_x_ = 0.0f;
    float grid_origin_y_ = 0.0f;
};

// ==================== 运动模型基类及派生类 ====================

class MotionModel
{
public:
    MotionModel() = default;
    virtual ~MotionModel() = default;

    virtual void predict(State & state)
    {
        using namespace xt::placeholders;
        xt::noalias(xt::view(state.vx, xt::all(), xt::range(1, _))) =
            xt::view(state.cvx, xt::all(), xt::range(0, -1));
        xt::noalias(xt::view(state.wz, xt::all(), xt::range(1, _))) =
            xt::view(state.cwz, xt::all(), xt::range(0, -1));
        if (isHolonomic()) {
            xt::noalias(xt::view(state.vy, xt::all(), xt::range(1, _))) =
                xt::view(state.cvy, xt::all(), xt::range(0, -1));
        }
    }

    virtual bool isHolonomic() const = 0;

    virtual void applyConstraints(xt::xtensor<float, 2> & /*cvx*/,
                                   xt::xtensor<float, 2> & /*cvy*/,
                                   xt::xtensor<float, 2> & /*cwz*/) {}
};

class DiffDriveMotionModel : public MotionModel
{
public:
    DiffDriveMotionModel() = default;
    bool isHolonomic() const override { return false; }
};

class OmniMotionModel : public MotionModel
{
public:
    OmniMotionModel() = default;
    bool isHolonomic() const override { return true; }
};

class AckermannMotionModel : public MotionModel
{
public:
    explicit AckermannMotionModel(float min_turning_r = 0.2f)
        : min_turning_r_(min_turning_r) {}

    bool isHolonomic() const override { return false; }

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

struct CriticData
{
    const State & state;
    const Trajectories & trajectories;
    const Path & path;
    xt::xtensor<float, 1> & costs;
    float & model_dt;
    bool fail_flag = false;
    std::shared_ptr<MotionModel> motion_model;
    std::optional<size_t> furthest_reached_path_point;
};

class CriticFunction
{
public:
    CriticFunction() = default;
    virtual ~CriticFunction() = default;

    virtual void initialize() = 0;
    virtual void score(CriticData & data) = 0;

    virtual void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }
    void setName(const std::string & name) { name_ = name; }
    const std::string & getName() const { return name_; }

protected:
    bool enabled_ = true;
    std::string name_;
};

// ==================== 路径跟踪代价函数 ====================

class PathFollowCritic : public CriticFunction
{
public:
    void initialize() override {}
    
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

class PathAlignCritic : public CriticFunction
{
public:
    void initialize() override {}
    
    void setParams(float weight, int offset, float threshold, int step)
    {
        weight_ = weight;
        offset_from_furthest_ = offset;
        threshold_to_consider_ = threshold;
        traj_point_step_ = step;
    }

    void score(CriticData & data) override
    {
        if (!enabled_ || data.path.size() < 2) return;
        Pose2D goal = data.path.getGoal();
        float dist_to_goal = std::hypot(data.state.pose.x - goal.x, data.state.pose.y - goal.y);
        if (dist_to_goal < threshold_to_consider_) return;

        const size_t batch_size = data.trajectories.x.shape(0);
        const size_t traj_len = data.trajectories.x.shape(1);

        size_t path_start_idx = (data.furthest_reached_path_point) ? *data.furthest_reached_path_point : 0;
        size_t path_end_idx = std::min(path_start_idx + offset_from_furthest_, data.path.size() - 1);
        
        std::vector<std::pair<float, float>> path_segment;
        path_segment.reserve(path_end_idx - path_start_idx + 1);
        for (size_t k = path_start_idx; k <= path_end_idx; ++k) {
            path_segment.emplace_back(data.path.x(k), data.path.y(k));
        }
        
        for (size_t i = 0; i < batch_size; ++i) {
            float cost = 0.0f;
            int count = 0;
            for (size_t j = 0; j < traj_len; j += traj_point_step_) {
                float px = data.trajectories.x(i, j);
                float py = data.trajectories.y(i, j);
                float min_dist = std::numeric_limits<float>::max();
                for (const auto& pt : path_segment) {
                    float dist = std::hypot(px - pt.first, py - pt.second);
                    if (dist < min_dist) min_dist = dist;
                }
                cost += min_dist;
                count++;
            }
            if (count > 0) data.costs(i) += weight_ * (cost / count);
        }
    }
private:
    float weight_ = 14.0f;
    float threshold_to_consider_ = 0.4f;
    int offset_from_furthest_ = 20;
    int traj_point_step_ = 4;
};

class PathAngleCritic : public CriticFunction
{
public:
    void initialize() override {}
    
    void setParams(float weight, int offset, float threshold, float max_angle)
    {
        weight_ = weight;
        offset_from_furthest_ = offset;
        threshold_to_consider_ = threshold;
        max_angle_ = max_angle;
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
            float angle_diff = std::abs(shortestAngularDistance(last_yaw, target_yaw));
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
};

class GoalCritic : public CriticFunction
{
public:
    void initialize() override {}
    
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

class GoalAngleCritic : public CriticFunction
{
public:
    void initialize() override {}
    
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

class PreferForwardCritic : public CriticFunction
{
public:
    void initialize() override {}
    
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

class ConstraintCritic : public CriticFunction
{
public:
    void initialize() override {}
    
    void setParams(float weight) { weight_ = weight; }

    void score(CriticData & data) override
    {
        if (!enabled_) return;
        const size_t batch_size = data.state.cvx.shape(0);
        const size_t traj_len = data.state.cvx.shape(1);
        for (size_t i = 0; i < batch_size; ++i) {
            float cost = 0.0f;
            for (size_t j = 0; j < traj_len; ++j) {
                float wz = data.state.cwz(i, j);
                cost += wz * wz;
            }
            data.costs(i) += weight_ * cost / traj_len;
        }
    }
private:
    float weight_ = 4.0f;
};

// ==================== 代价函数管理器 ====================

class CriticManager
{
public:
    CriticManager() = default;
    
    void addCritic(std::unique_ptr<CriticFunction> critic) { critics_.push_back(std::move(critic)); }
    
    void initializeCritics() { for (auto & c : critics_) c->initialize(); }

    void evalTrajectoriesScores(CriticData & data) const
    {
        for (size_t i = 0; i < data.costs.shape(0); ++i) data.costs(i) = 0.0f;
        data.fail_flag = false;

        for (const auto & critic : critics_) {
            if (critic->isEnabled()) critic->score(data);
        }
    }

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

class NoiseGenerator
{
public:
    void initialize(OptimizerSettings & settings, bool is_holonomic)
    {
        settings_ = settings;
        is_holonomic_ = is_holonomic;
        noises_vx_ = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
        noises_vy_ = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
        noises_wz_ = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
        generateNoisedControls();
    }

    void generateNextNoises() { generateNoisedControls(); }

    void getNoisedControls(State & state, const ControlSequence & control_sequence)
    {
        for (size_t i = 0; i < settings_.batch_size; ++i) {
            for (size_t j = 0; j < settings_.time_steps; ++j) {
                state.cvx(i, j) = control_sequence.vx(j) + noises_vx_(i, j);
                state.cwz(i, j) = control_sequence.wz(j) + noises_wz_(i, j);
                if (is_holonomic_) {
                    state.cvy(i, j) = control_sequence.vy(j) + noises_vy_(i, j);
                }
            }
        }
    }

    void reset(OptimizerSettings & settings, bool is_holonomic) { initialize(settings, is_holonomic); }

private:
    void generateNoisedControls()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<float> dist_vx(0.0f, settings_.sampling_std.vx);
        std::normal_distribution<float> dist_wz(0.0f, settings_.sampling_std.wz);
        std::normal_distribution<float> dist_vy(0.0f, settings_.sampling_std.vy);

        for (size_t i = 0; i < settings_.batch_size; ++i) {
            for (size_t j = 0; j < settings_.time_steps; ++j) {
                noises_vx_(i, j) = dist_vx(gen);
                noises_wz_(i, j) = dist_wz(gen);
                if (is_holonomic_) noises_vy_(i, j) = dist_vy(gen);
            }
        }
    }

    xt::xtensor<float, 2> noises_vx_, noises_vy_, noises_wz_;
    OptimizerSettings settings_;
    bool is_holonomic_ = false;
};

// ==================== 优化器核心 ====================

class Optimizer
{
public:
    void initialize(const OptimizerSettings & settings,
                    std::shared_ptr<MotionModel> motion_model,
                    CriticManager * critic_manager)
    {
        settings_ = settings;
        motion_model_ = motion_model;
        critic_manager_ = critic_manager;
        noise_generator_.initialize(settings_, isHolonomic());
        reset();
    }

    void reset()
    {
        state_.reset(settings_.batch_size, settings_.time_steps);
        control_sequence_.reset(settings_.time_steps);
        settings_.constraints = settings_.base_constraints;
        costs_ = xt::zeros<float>({settings_.batch_size});
        generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);
        noise_generator_.reset(settings_, isHolonomic());
        last_control_ = {0.0f, 0.0f, 0.0f};
        last_speed_ = {0.0f, 0.0f, 0.0f};
    }

    Twist2D evalControl(const Pose2D & robot_pose, const Twist2D & robot_speed, const Path & path)
    {
        prepare(robot_pose, robot_speed, path);

        float prev_min_cost = std::numeric_limits<float>::max();

        for (size_t iter = 0; iter < settings_.iteration_count; ++iter) {
            optimize();

            float current_min_cost = xt::amin(costs_)(0);
            if (std::abs(prev_min_cost - current_min_cost) < 0.01f && current_min_cost < 100.0f) {
                break;
            }
            prev_min_cost = current_min_cost;

            size_t best_idx = xt::argmin(costs_)(0);
            if (costs_(best_idx) >= settings_.constraints.collision_cost_threshold) {
                if (fallback(true)) {
                    prepare(robot_pose, robot_speed, path);
                    prev_min_cost = std::numeric_limits<float>::max();
                }
            }
        }

        applyControlSequenceConstraints();
        applyAccelerationConstraints();

        Twist2D control = getControlFromSequence();
        last_control_ = control;
        last_speed_ = robot_speed;

        if (settings_.shift_control_sequence) {
            shiftControlSequence();
        }
        return control;
    }

    Trajectories & getGeneratedTrajectories() { return generated_trajectories_; }

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

    void setSpeedLimit(double /*speed_limit*/, bool /*percentage*/) {}
    const OptimizerSettings & getSettings() const { return settings_; }

private:
    void prepare(const Pose2D & robot_pose, const Twist2D & robot_speed, const Path & path)
    {
        state_.pose = robot_pose;
        state_.speed = robot_speed;
        path_ = path;
        costs_ = xt::zeros<float>({settings_.batch_size});
    }

    void optimize()
    {
        generateNoisedTrajectories();
        std::optional<size_t> furthest = findFurthestReachedPathPoint();

        CriticData data{state_, generated_trajectories_, path_, costs_,
                        settings_.model_dt, false, motion_model_, furthest};
        critic_manager_->evalTrajectoriesScores(data);

        updateControlSequence();
    }

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

    bool fallback(bool fail)
    {
        static size_t counter = 0;
        if (!fail) { counter = 0; return false; }
        reset();
        if (++counter > settings_.retry_attempt_limit) { counter = 0; return false; }
        return true;
    }

    void generateNoisedTrajectories()
    {
        noise_generator_.getNoisedControls(state_, control_sequence_);
        applyControlConstraintsBatch(state_);
        motion_model_->applyConstraints(state_.cvx, state_.cvy, state_.cwz);
        updateStateVelocities(state_);
        integrateStateVelocities(generated_trajectories_, state_);
        noise_generator_.generateNextNoises();
    }

    void applyControlConstraintsBatch(State & state)
    {
        auto & s = settings_;
        state.cvx = xt::clip(state.cvx, s.constraints.vx_min, s.constraints.vx_max);
        state.cwz = xt::clip(state.cwz, -s.constraints.wz_max, s.constraints.wz_max);
        if (isHolonomic()) {
            state.cvy = xt::clip(state.cvy, -s.constraints.vy_max, s.constraints.vy_max);
        }
    }

    void updateStateVelocities(State & state) const
    {
        for (size_t i = 0; i < settings_.batch_size; ++i) {
            state.vx(i, 0) = state.speed.vx;
            state.wz(i, 0) = state.speed.wz;
            if (isHolonomic()) state.vy(i, 0) = state.speed.vy;
        }
        motion_model_->predict(state);
    }

    void integrateStateVelocities(Trajectories & trajectories, const State & state) const
    {
        const float initial_yaw = state.pose.theta;
        std::vector<std::thread> threads;
        unsigned int num_threads = settings_.thread_count;
        unsigned int batch_per_thread = (settings_.batch_size + num_threads - 1) / num_threads;

        auto worker = [&](unsigned int start, unsigned int end) {
            for (size_t i = start; i < end; ++i) {
                float yaw = initial_yaw;
                trajectories.yaws(i, 0) = yaw;
                trajectories.x(i, 0) = state.pose.x;
                trajectories.y(i, 0) = state.pose.y;
                trajectories.times(i, 0) = 0.0f;

                for (size_t j = 1; j < state.vx.shape(1); ++j) {
                    yaw += state.wz(i, j-1) * settings_.model_dt;
                    float cos_yaw = std::cos(yaw);
                    float sin_yaw = std::sin(yaw);

                    trajectories.x(i, j) = trajectories.x(i, j-1) + state.vx(i, j-1) * cos_yaw * settings_.model_dt;
                    trajectories.y(i, j) = trajectories.y(i, j-1) + state.vx(i, j-1) * sin_yaw * settings_.model_dt;
                    trajectories.yaws(i, j) = yaw;
                    trajectories.times(i, j) = j * settings_.model_dt;
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

    void updateControlSequence()
    {
        float max_cost = xt::amax(costs_)(0);
        xt::xtensor<float, 1> weights = xt::exp(-(costs_ - max_cost) / std::max(settings_.temperature, 1e-3f));
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

    void applyControlSequenceConstraints()
    {
        auto & s = settings_;
        for (size_t j = 0; j < settings_.time_steps; ++j) {
            control_sequence_.vx(j) = clamp(control_sequence_.vx(j), s.constraints.vx_min, s.constraints.vx_max);
            control_sequence_.wz(j) = clamp(control_sequence_.wz(j), -s.constraints.wz_max, s.constraints.wz_max);
            if (isHolonomic()) control_sequence_.vy(j) = clamp(control_sequence_.vy(j), -s.constraints.vy_max, s.constraints.vy_max);
        }
    }

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

    Twist2D getControlFromSequence()
    {
        return Twist2D(control_sequence_.vx(0),
                       isHolonomic() ? control_sequence_.vy(0) : 0.0f,
                       control_sequence_.wz(0));
    }

    bool isHolonomic() const { return motion_model_->isHolonomic(); }

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
};

// ==================== 对外接口：MPPIController ====================

class MPPIController
{
public:
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

        critic_manager_->initializeCritics();

        optimizer_ = std::make_unique<Optimizer>();
        optimizer_->initialize(settings_, motion_model_, critic_manager_.get());

        obstacle_handler_ = std::make_unique<ObstacleHandler>();
        obstacle_handler_->initialize(avoidance_config_);
    }

    void setCriticParams(const std::string & name, const std::vector<float> & params)
    {
        auto * critic = critic_manager_->getCritic(name);
        if (!critic) return;

        if (name == "GoalCritic") {
            auto * g = dynamic_cast<GoalCritic*>(critic);
            if (g && params.size() >= 2) g->setParams(params[0], params[1]);
        } else if (name == "GoalAngleCritic") {
            auto * ga = dynamic_cast<GoalAngleCritic*>(critic);
            if (ga && params.size() >= 2) ga->setParams(params[0], params[1]);
        } else if (name == "PathAlignCritic") {
            auto * pa = dynamic_cast<PathAlignCritic*>(critic);
            if (pa && params.size() >= 4) pa->setParams(params[0], static_cast<int>(params[1]), params[2], static_cast<int>(params[3]));
        } else if (name == "PathAngleCritic") {
            auto * pang = dynamic_cast<PathAngleCritic*>(critic);
            if (pang && params.size() >= 4) pang->setParams(params[0], static_cast<int>(params[1]), params[2], params[3]);
        } else if (name == "PathFollowCritic") {
            auto * pf = dynamic_cast<PathFollowCritic*>(critic);
            if (pf && params.size() >= 3) pf->setParams(params[0], static_cast<int>(params[1]), params[2]);
        } else if (name == "PreferForwardCritic") {
            auto * pfwd = dynamic_cast<PreferForwardCritic*>(critic);
            if (pfwd && params.size() >= 2) pfwd->setParams(params[0], params[1]);
        } else if (name == "ConstraintCritic") {
            auto * cc = dynamic_cast<ConstraintCritic*>(critic);
            if (cc && params.size() >= 1) cc->setParams(params[0]);
        }
    }

    void setAvoidanceConfig(const AvoidanceConfig & config)
    {
        avoidance_config_ = config;
        if (obstacle_handler_) {
            obstacle_handler_->setConfig(config);
        }
    }

    const AvoidanceConfig & getAvoidanceConfig() const { return avoidance_config_; }

    void setPath(const std::vector<Pose2D> & path) {
        path_.reset(path.size());
        for (size_t i = 0; i < path.size(); ++i) {
            path_.x(i) = path[i].x;
            path_.y(i) = path[i].y;
            path_.yaws(i) = path[i].theta;
        }
        original_path_ = path_;
    }

    void setPath(const Path & path) { 
        path_ = path;
        original_path_ = path;
    }

    void updateStaticObstacles(const std::vector<Point2D> & points, const Pose2D & robot_pose) {
        if (obstacle_handler_) {
            obstacle_handler_->updateStaticObstacles(points, robot_pose);
        }
    }

    void updateDynamicObstacles(const std::vector<DynamicObstacle> & obstacles) {
        if (obstacle_handler_) {
            obstacle_handler_->updateDynamicObstacles(obstacles);
        }
    }

    Twist2D computeVelocityCommands(const Pose2D & robot_pose, const Twist2D & robot_speed)
    {
        if (path_.empty()) return Twist2D();

        ObstacleResult obstacle_result = obstacle_handler_->evaluate(original_path_, robot_speed);

        if (obstacle_result.action == ObstacleAction::EMERGENCY_BRAKE) {
            Twist2D emergency_stop;
            emergency_stop.vx = 0.0f;
            emergency_stop.wz = 0.0f;
            return emergency_stop;
        }

        if (obstacle_result.action == ObstacleAction::NORMAL_STOP) {
            Twist2D stop;
            stop.vx = 0.0f;
            stop.wz = 0.0f;
            return stop;
        }

        Path active_path = original_path_;
        
        if (obstacle_result.action == ObstacleAction::LATERAL_AVOID) {
            active_path = obstacle_handler_->generateAvoidancePath(original_path_, obstacle_result);
            path_ = active_path;
        } else {
            path_ = original_path_;
        }

        Twist2D cmd = optimizer_->evalControl(robot_pose, robot_speed, active_path);

        if (obstacle_result.action == ObstacleAction::FOLLOW) {
            cmd.vx = std::min(cmd.vx, obstacle_result.target_speed);
        } else if (obstacle_result.action == ObstacleAction::LATERAL_AVOID) {
            cmd.vx = std::min(cmd.vx, obstacle_result.target_speed);
        }

        return cmd;
    }

    GoalCritic* getGoalCritic() const { return goal_critic_; }
    GoalAngleCritic* getGoalAngleCritic() const { return goal_angle_critic_; }
    PathAlignCritic* getPathAlignCritic() const { return path_align_critic_; }
    PathAngleCritic* getPathAngleCritic() const { return path_angle_critic_; }
    PathFollowCritic* getPathFollowCritic() const { return path_follow_critic_; }
    PreferForwardCritic* getPreferForwardCritic() const { return prefer_forward_critic_; }
    ConstraintCritic* getConstraintCritic() const { return constraint_critic_; }
    ObstacleHandler* getObstacleHandler() const { return obstacle_handler_.get(); }

    Trajectories & getGeneratedTrajectories() { return optimizer_->getGeneratedTrajectories(); }
    xt::xtensor<float, 2> getOptimizedTrajectory() { return optimizer_->getOptimizedTrajectory(); }
    void reset() { optimizer_->reset(); }
    bool isHolonomic() const { return motion_model_->isHolonomic(); }
    Path& getPath() { return path_; }
    const Path& getOriginalPath() const { return original_path_; }

private:
    OptimizerSettings settings_;
    std::shared_ptr<MotionModel> motion_model_;
    std::unique_ptr<CriticManager> critic_manager_;
    std::unique_ptr<Optimizer> optimizer_;
    Path path_;
    Path original_path_;
    AvoidanceConfig avoidance_config_;
    std::unique_ptr<ObstacleHandler> obstacle_handler_;

    GoalCritic* goal_critic_ = nullptr;
    GoalAngleCritic* goal_angle_critic_ = nullptr;
    PathAlignCritic* path_align_critic_ = nullptr;
    PathAngleCritic* path_angle_critic_ = nullptr;
    PathFollowCritic* path_follow_critic_ = nullptr;
    PreferForwardCritic* prefer_forward_critic_ = nullptr;
    ConstraintCritic* constraint_critic_ = nullptr;
};

} // namespace mppi

#endif // MPPI_CONTROLLER_HPP_