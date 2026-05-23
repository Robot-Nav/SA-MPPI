// ============================================================================
// 文件：optimizer.hpp
// 功能：MPPI 优化器核心，执行采样、评分、加权更新
// ============================================================================
#ifndef MPPI_OPTIMIZER_HPP_
#define MPPI_OPTIMIZER_HPP_

#include <cmath>
#include <vector>
#include <thread>
#include <stdexcept>
#include <algorithm>
#include <optional>
#include <array>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xnoalias.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xsort.hpp>

#include "models/types.hpp"
#include "models/constraints.hpp"
#include "models/control_sequence.hpp"
#include "models/state.hpp"
#include "models/trajectories.hpp"
#include "models/path.hpp"
#include "models/optimizer_settings.hpp"
#include "motion_models.hpp"
#include "critics/critic_data.hpp"
#include "critics/critic_manager.hpp"
#include "tools/noise_generator.hpp"
#include "tools/math_utils.hpp"

namespace mppi
{

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
        last_control_ = {0.0f, 0.0f, 0.0f};
        control_history_[0] = {0.0f, 0.0f, 0.0f};
        control_history_[1] = {0.0f, 0.0f, 0.0f};
        control_history_[2] = {0.0f, 0.0f, 0.0f};
        control_history_[3] = {0.0f, 0.0f, 0.0f};
    }

    /**
     * @brief 安全重置（用于运动模型切换时）
     * 只重置控制序列和状态，不触及线程相关资源
     */
    void softReset()
    {
        control_sequence_.reset(settings_.time_steps);
        last_control_ = {0.0f, 0.0f, 0.0f};
        control_history_[0] = {0.0f, 0.0f, 0.0f};
        control_history_[1] = {0.0f, 0.0f, 0.0f};
        control_history_[2] = {0.0f, 0.0f, 0.0f};
        control_history_[3] = {0.0f, 0.0f, 0.0f};
    }

    /**
     * @brief 计算最优控制指令（主入口）
     * @param robot_pose 当前机器人位姿
     * @param robot_speed 当前机器人速度
     * @param path 全局路径
     * @param path_pts_valid 路径点有效性标志（可选）
     * @return 最优控制量（Twist2D）
     */
    Twist2D evalControl(const Pose2D & robot_pose, const Twist2D & robot_speed, const Path & path,
                        const std::vector<bool> * path_pts_valid = nullptr)
    {
        prepare(robot_pose, robot_speed, path, path_pts_valid);

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

            size_t best_idx = xt::argmin(costs_)(0);
            bool best_trajectory_collide = (costs_(best_idx) >= settings_.constraints.collision_cost_threshold);

            all_trajectories_collide = fail_flag || checkAllTrajectoriesCollide();

            if (best_trajectory_collide || all_trajectories_collide) {
                if (!fallback(fail_flag)) {
                    throw std::runtime_error("Optimizer failed to compute path: all trajectories collide after maximum retries");
                }
                prepare(robot_pose, robot_speed, path, path_pts_valid_);
                prev_min_cost = std::numeric_limits<float>::max();
                fail_flag = false;
            }
        }

        applyControlSequenceConstraints();

        // 阻塞检测：如果检测到原地打转，输出零速度
        if (isBlocked()) {
            is_blocked_ = true;
            control_sequence_.reset(settings_.time_steps);
            last_control_ = {0.0f, 0.0f, 0.0f};
            last_speed_ = robot_speed;
            return Twist2D(0.0f, 0.0f, 0.0f);
        }
        is_blocked_ = false;

        // 应用 Savitzky-Golay 滤波器平滑控制序列
        if (settings_.use_sg_filter) {
            savitskyGolayFilter(control_sequence_, control_history_, settings_);
        }

        Twist2D control = getControlFromSequence();

        last_control_ = control;
        last_speed_ = robot_speed;

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

            float vx = control_sequence_.vx(j-1);
            float vy = isHolonomic() ? control_sequence_.vy(j-1) : 0.0f;

            x += (vx * cos_yaw - vy * sin_yaw) * settings_.model_dt;
            y += (vx * sin_yaw + vy * cos_yaw) * settings_.model_dt;
            yaw += control_sequence_.wz(j-1) * settings_.model_dt;
            trajectory(j, 0) = x; trajectory(j, 1) = y; trajectory(j, 2) = yaw;
        }
        return trajectory;
    }

    /** @return 优化器设置 */
    const OptimizerSettings & getSettings() const { return settings_; }
    /** @brief 设置动态障碍物列表 */
    void setDynamicObstacles(const std::vector<ObstaclePrediction> & obstacles) { dynamic_obstacles_ = obstacles; }

    /** @return 当前是否被阻塞 */
    bool isCurrentlyBlocked() const { return is_blocked_; }

    /**
     * @brief 设置运动模型（用于动态切换全向/差速模式）
     */
    void setMotionModel(std::shared_ptr<MotionModel> motion_model)
    {
        motion_model_ = motion_model;
        noise_generator_.setHolonomic(isHolonomic());
    }

private:
    /**
     * @brief 准备本次优化：更新状态中的位姿和速度，存储路径（裁剪后）
     */
    void prepare(const Pose2D & robot_pose, const Twist2D & robot_speed, const Path & path,
                 const std::vector<bool> * path_pts_valid = nullptr)
    {
        state_.pose = robot_pose;
        state_.speed = robot_speed;

        // 裁剪路径，使索引0对应机器人当前位置
        path_ = prunePath(path, robot_pose);

        // 同步裁剪path_pts_valid
        if (path_pts_valid && !path_pts_valid->empty()) {
            size_t original_size = path.size();
            size_t pruned_size = path_.size();
            size_t offset = original_size - pruned_size;

            pruned_path_pts_valid_.resize(pruned_size);
            for (size_t i = 0; i < pruned_size; ++i) {
                pruned_path_pts_valid_[i] = (*path_pts_valid)[offset + i];
            }
            path_pts_valid_ = &pruned_path_pts_valid_;
        } else {
            path_pts_valid_ = nullptr;
        }

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

    /**
     * @brief 单次优化迭代：生成轨迹、评分、更新控制序列
     */
    void optimize(bool& fail_flag)
    {
        generateNoisedTrajectories();
        std::optional<size_t> furthest = findFurthestReachedPathPoint();

        CriticData data{state_, generated_trajectories_, path_, costs_,
                        settings_.model_dt, false, motion_model_, furthest, &dynamic_obstacles_};

        data.path_pts_valid = path_pts_valid_;

        critic_manager_->evalTrajectoriesScores(data);

        fail_flag = data.fail_flag;

        updateControlSequence();
    }

    /**
     * @brief 查找路径上最远可达点索引
     */
    std::optional<size_t> findFurthestReachedPathPoint()
    {
        if (path_.empty() || generated_trajectories_.x.shape(0) == 0) {
            return std::nullopt;
        }

        const size_t batch_size = generated_trajectories_.x.shape(0);
        const size_t path_size = path_.size();
        size_t max_reached_idx = 0;

        for (size_t i = 0; i < batch_size; ++i) {
            float traj_end_x = generated_trajectories_.x(i, settings_.time_steps - 1);
            float traj_end_y = generated_trajectories_.y(i, settings_.time_steps - 1);

            size_t closest_idx = 0;
            float min_dist = std::numeric_limits<float>::max();
            for (size_t j = max_reached_idx; j < path_size; ++j) {
                float dist = std::hypot(traj_end_x - path_.x(j), traj_end_y - path_.y(j));
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_idx = j;
                }
            }
            max_reached_idx = std::max(max_reached_idx, closest_idx);
        }

        return max_reached_idx;
    }

    /**
     * @brief 裁剪路径：移除机器人当前位置之前的路径点
     */
    Path prunePath(const Path& path, const Pose2D& robot_pose, float prune_distance = 1.0f)
    {
        if (path.empty()) return path;

        if (prune_distance < 0) {
            prune_distance = settings_.prune_distance;
        }

        // 找到机器人当前位置在路径上的最近点
        size_t closest_idx = 0;
        float min_dist = std::numeric_limits<float>::max();
        for (size_t k = 0; k < path.size(); ++k) {
            float dist = std::hypot(robot_pose.x - path.x(k), robot_pose.y - path.y(k));
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = k;
            }
        }

        // 计算从closest_idx开始的路径累计距离，限制在prune_distance内
        float accumulated_dist = 0.0f;
        size_t max_idx = closest_idx;

        for (size_t i = closest_idx + 1; i < path.size(); ++i) {
            float dx = path.x(i) - path.x(i - 1);
            float dy = path.y(i) - path.y(i - 1);
            accumulated_dist += std::hypot(dx, dy);

            if (accumulated_dist >= prune_distance) {
                max_idx = i;
                break;
            }
            max_idx = i;
        }

        // 创建裁剪后的路径
        Path pruned_path;
        size_t new_size = max_idx - closest_idx + 1;
        pruned_path.reset(new_size);

        for (size_t i = 0; i < new_size; ++i) {
            pruned_path.x(i) = path.x(closest_idx + i);
            pruned_path.y(i) = path.y(closest_idx + i);
            pruned_path.yaws(i) = path.yaws(closest_idx + i);
        }

        return pruned_path;
    }

    /**
     * @brief 检查是否所有轨迹都发生碰撞
     */
    bool checkAllTrajectoriesCollide() const
    {
        const float collision_threshold = settings_.constraints.collision_cost_threshold;
        for (size_t i = 0; i < costs_.shape(0); ++i) {
            if (costs_(i) < collision_threshold) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief 故障恢复机制
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
        noise_generator_.generateNextNoises();
    }

    /**
     * @brief 对一批含噪声控制量施加边界约束
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
     */
    void updateStateVelocities(State & state) const
    {
        motion_model_->predict(state);
    }

    /**
     * @brief 根据速度序列积分得到轨迹（多线程并行）
     */
    void integrateStateVelocities(Trajectories & trajectories, const State & state) const
    {
        const float initial_yaw = state.pose.theta;
        const float pose_x = state.pose.x;
        const float pose_y = state.pose.y;
        const float model_dt = settings_.model_dt;
        const size_t time_steps = state.vx.shape(1);

        std::vector<std::thread> threads;
        unsigned int num_threads = settings_.thread_count;
        unsigned int batch_per_thread = (settings_.batch_size + num_threads - 1) / num_threads;

        auto worker = [&](unsigned int start, unsigned int end) {
            std::vector<float> local_x(time_steps);
            std::vector<float> local_y(time_steps);
            std::vector<float> local_yaws(time_steps);

            for (size_t i = start; i < end; ++i) {
                float yaw = initial_yaw;
                local_x[0] = pose_x;
                local_y[0] = pose_y;
                local_yaws[0] = yaw;

                for (size_t j = 1; j < time_steps; ++j) {
                    yaw += state.wz(i, j-1) * model_dt;
                    float cos_yaw = std::cos(yaw);
                    float sin_yaw = std::sin(yaw);

                    float vx = state.vx(i, j-1);
                    float vy = isHolonomic() ? state.vy(i, j-1) : 0.0f;

                    local_x[j] = local_x[j-1] + (vx * cos_yaw - vy * sin_yaw) * model_dt;
                    local_y[j] = local_y[j-1] + (vx * sin_yaw + vy * cos_yaw) * model_dt;
                    local_yaws[j] = yaw;
                }

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

    /**
     * @brief 根据代价加权更新控制序列（支持自适应温度和均值归一化）
     */
    void updateControlSequence()
    {
        // 添加控制成本项：惩罚偏离当前控制序列的噪声
        auto bounded_noises_vx = state_.cvx - xt::view(control_sequence_.vx, xt::newaxis(), xt::all());
        auto bounded_noises_wz = state_.cwz - xt::view(control_sequence_.wz, xt::newaxis(), xt::all());

        costs_ += settings_.gamma / (settings_.sampling_std.vx * settings_.sampling_std.vx) *
                  xt::sum(xt::view(control_sequence_.vx, xt::newaxis(), xt::all()) * bounded_noises_vx, 1);
        costs_ += settings_.gamma / (settings_.sampling_std.wz * settings_.sampling_std.wz) *
                  xt::sum(xt::view(control_sequence_.wz, xt::newaxis(), xt::all()) * bounded_noises_wz, 1);

        if (isHolonomic()) {
            auto bounded_noises_vy = state_.cvy - xt::view(control_sequence_.vy, xt::newaxis(), xt::all());
            costs_ += settings_.gamma / (settings_.sampling_std.vy * settings_.sampling_std.vy) *
                      xt::sum(xt::view(control_sequence_.vy, xt::newaxis(), xt::all()) * bounded_noises_vy, 1);
        }

        // 计算softmax权重
        float min_cost = xt::amin(costs_)(0);
        float max_cost = xt::amax(costs_)(0);
        float T = settings_.temperature;

        // 自适应温度
        if (settings_.adaptive_temperature) {
            float cost_range = max_cost - min_cost;
            if (cost_range > EPSILON) {
                float adaptive_factor = std::log(cost_range + 1.0f) / 5.0f;
                T = clamp(settings_.temperature * (1.0f + adaptive_factor),
                         settings_.adaptive_temperature_min,
                         settings_.adaptive_temperature_max);
            }
        }
        T = std::max(T, 1e-3f);

        xt::xtensor<float, 1> weights;

        if (settings_.use_mean_normalization) {
            float mean_cost = xt::mean(costs_)(0);
            weights = xt::exp(-(costs_ - mean_cost) / T);
        } else {
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
     */
    bool isBlocked()
    {
        float avg_vx = 0.0f;
        float avg_abs_wz = 0.0f;
        for (size_t j = 0; j < settings_.time_steps; ++j) {
            avg_vx += control_sequence_.vx(j);
            avg_abs_wz += std::abs(control_sequence_.wz(j));
        }
        avg_vx /= settings_.time_steps;
        avg_abs_wz /= settings_.time_steps;

        if (avg_abs_wz < 0.3f) {
            spinning_counter_ = 0;
            return false;
        }

        float effective_vx = std::abs(avg_vx);
        bool spinning_by_ratio = (effective_vx < EPSILON) ||
            (avg_abs_wz / effective_vx > settings_.spinning_ratio_threshold);

        if (!spinning_by_ratio) {
            spinning_counter_ = 0;
            return false;
        }

        spinning_counter_++;
        if (spinning_counter_ < settings_.spinning_detect_frames) return false;

        return true;
    }

    /** @brief 更新控制历史（用于SG滤波） */
    void updateControlHistory()
    {
        control_history_[3] = control_history_[2];
        control_history_[2] = control_history_[1];
        control_history_[1] = control_history_[0];
        control_history_[0] = {control_sequence_.vx(0),
                               isHolonomic() ? control_sequence_.vy(0) : 0.0f,
                               control_sequence_.wz(0)};
    }

    /**
     * @brief Savitzky-Golay滤波器（对整个控制序列滤波）
     */
    void savitskyGolayFilter(ControlSequence & control_sequence,
                             const std::array<Control, 4> & control_history,
                             const OptimizerSettings & settings)
    {
        const std::vector<float> sg_coeffs = {-0.085714f, 0.342857f, 0.485714f, 0.342857f, -0.085714f};
        const int half_window = 2;

        std::vector<float> vx_temp(control_sequence.vx.begin(), control_sequence.vx.end());
        std::vector<float> vy_temp(control_sequence.vy.begin(), control_sequence.vy.end());
        std::vector<float> wz_temp(control_sequence.wz.begin(), control_sequence.wz.end());

        for (size_t i = 0; i < settings.time_steps; ++i) {
            // vx滤波
            float vx_filtered = 0.0f;
            for (int j = -half_window; j <= half_window; ++j) {
                int idx = static_cast<int>(i) + j;
                float value;
                if (idx < 0) {
                    int hist_idx = -idx - 1;
                    value = (hist_idx < 4) ? control_history[hist_idx].vx : vx_temp[0];
                } else if (idx >= static_cast<int>(settings.time_steps)) {
                    value = vx_temp.back();
                } else {
                    value = vx_temp[idx];
                }
                vx_filtered += sg_coeffs[j + half_window] * value;
            }
            control_sequence.vx(i) = vx_filtered;

            // wz滤波
            float wz_filtered = 0.0f;
            for (int j = -half_window; j <= half_window; ++j) {
                int idx = static_cast<int>(i) + j;
                float value;
                if (idx < 0) {
                    int hist_idx = -idx - 1;
                    value = (hist_idx < 4) ? control_history[hist_idx].wz : wz_temp[0];
                } else if (idx >= static_cast<int>(settings.time_steps)) {
                    value = wz_temp.back();
                } else {
                    value = wz_temp[idx];
                }
                wz_filtered += sg_coeffs[j + half_window] * value;
            }
            control_sequence.wz(i) = wz_filtered;

            // vy滤波（全向模型）
            if (isHolonomic()) {
                float vy_filtered = 0.0f;
                for (int j = -half_window; j <= half_window; ++j) {
                    int idx = static_cast<int>(i) + j;
                    float value;
                    if (idx < 0) {
                        int hist_idx = -idx - 1;
                        value = (hist_idx < 4) ? control_history[hist_idx].vy : vy_temp[0];
                    } else if (idx >= static_cast<int>(settings.time_steps)) {
                        value = vy_temp.back();
                    } else {
                        value = vy_temp[idx];
                    }
                    vy_filtered += sg_coeffs[j + half_window] * value;
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
    bool is_blocked_ = false;
    int spinning_counter_ = 0;
    size_t fallback_counter_ = 0;

    std::array<Control, 4> control_history_;

    const std::vector<bool> * path_pts_valid_ = nullptr;
    std::vector<bool> pruned_path_pts_valid_;
};

} // namespace mppi

#endif // MPPI_OPTIMIZER_HPP_
