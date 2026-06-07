// ============================================================================
// 文件：controller.hpp
// 功能：MPPI 控制器对外接口类，封装优化器和代价函数管理器
// ============================================================================
#ifndef MPPI_CONTROLLER_HPP_
#define MPPI_CONTROLLER_HPP_

#include <cmath>
#include <vector>
#include <memory>
#include <string>
#include <iostream>
#include <stdexcept>
#include <limits>

#include "models/types.hpp"
#include "models/constraints.hpp"
#include "models/control_sequence.hpp"
#include "models/state.hpp"
#include "models/trajectories.hpp"
#include "models/path.hpp"
#include "models/optimizer_settings.hpp"
#include "motion_models.hpp"
#include "optimizer.hpp"
#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"
#include "critics/critic_manager.hpp"
#include "critics/obstacles_critic.hpp"
#include "critics/path_follow_critic.hpp"
#include "critics/path_align_critic.hpp"
#include "critics/path_angle_critic.hpp"
#include "critics/goal_critic.hpp"
#include "critics/goal_angle_critic.hpp"
#include "critics/prefer_forward_critic.hpp"
#include "critics/constraint_critic.hpp"
#include "critics/twirling_critic.hpp"
#include "critics/velocity_deadband_critic.hpp"
#include "critics/esdf_critic.hpp"

namespace mppi
{

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
     * @param ackermann_min_turning_radius 阿克曼模型的最小转弯半径
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

        // 创建所有 critic
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
        twirling_critic_->setParams(10.0f, 0.5f);
        critic_manager_->addCritic(std::move(twirling_critic));

        auto velocity_deadband_critic = std::make_unique<VelocityDeadbandCritic>();
        velocity_deadband_critic->setName("VelocityDeadbandCritic");
        velocity_deadband_critic_ = velocity_deadband_critic.get();
        critic_manager_->addCritic(std::move(velocity_deadband_critic));

        auto esdf_critic = std::make_unique<EsdfCritic>();
        esdf_critic->setName("EsdfCritic");
        esdf_critic_ = esdf_critic.get();
        critic_manager_->addCritic(std::move(esdf_critic));

        critic_manager_->initializeCritics();

        optimizer_ = std::make_unique<Optimizer>();
        optimizer_->initialize(settings_, motion_model_, critic_manager_.get());
    }

    /**
     * @brief 从 YAML 解析后设置各个 critic 的参数
     * @param name 代价函数名称
     * @param params 参数列表
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
        } else if (name == "EsdfCritic") {
            auto * ec = dynamic_cast<EsdfCritic*>(critic);
            if (ec && params.size() >= 3) {
                ec->setRobotRadius(params[0]);
                ec->setVoxelSize(params[1]);
                ec->setEnabled(params[2] > 0.5f);
            }
        }
    }

    /**
     * @brief 设置全局路径（通过 Pose2D 列表）
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
     */
    void updateStaticObstacles(const std::vector<Point2D> & points, const Pose2D & robot_pose) {
        if (obstacles_critic_)
            obstacles_critic_->setLaserPoints(points, robot_pose);
        if (path_align_critic_)
            path_align_critic_->setObstacles(&points, path_align_obstacle_check_radius_);
        if (esdf_critic_)
            esdf_critic_->setObstacles(points);

        computePathPointsValidity(points);
    }

    /**
     * @brief 计算路径点有效性（是否被障碍物阻挡）
     */
    void computePathPointsValidity(const std::vector<Point2D> & obstacles)
    {
        if (path_.empty() || obstacles.empty()) {
            path_pts_valid_.clear();
            return;
        }

        path_pts_valid_.resize(path_.size(), true);
        const float check_radius = 0.15f;

        for (size_t i = 0; i < path_.size(); ++i) {
            float px = path_.x(i);
            float py = path_.y(i);
            bool blocked = false;

            for (const auto& obs : obstacles) {
                float dist = std::hypot(px - obs.x, py - obs.y);
                if (dist < check_radius) {
                    blocked = true;
                    break;
                }
            }

            path_pts_valid_[i] = !blocked;
        }
    }

    /** @brief 设置路径对齐障碍物检查半径 */
    void setPathAlignObstacleCheckRadius(float radius) { path_align_obstacle_check_radius_ = radius; }

    /**
     * @brief 计算速度指令（主循环调用）
     */
    Twist2D computeVelocityCommands(const Pose2D & robot_pose, const Twist2D & robot_speed)
    {
        if (path_.empty()) return Twist2D();
        return optimizer_->evalControl(robot_pose, robot_speed, path_,
                                        path_pts_valid_.empty() ? nullptr : &path_pts_valid_);
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
    EsdfCritic* getEsdfCritic() const { return esdf_critic_; }

    Trajectories & getGeneratedTrajectories() { return optimizer_->getGeneratedTrajectories(); }
    xt::xtensor<float, 2> getOptimizedTrajectory() { return optimizer_->getOptimizedTrajectory(); }
    void reset() { optimizer_->reset(); }
    bool isHolonomic() const { return motion_model_->isHolonomic(); }
    Path& getPath() { return path_; }

    void setRobotRadius(float radius) { robot_radius_ = radius; }

private:
    OptimizerSettings settings_;
    std::shared_ptr<MotionModel> motion_model_;
    std::unique_ptr<CriticManager> critic_manager_;
    std::unique_ptr<Optimizer> optimizer_;
    Path path_;

    // 各代价函数的原始指针
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
    EsdfCritic* esdf_critic_ = nullptr;
    float robot_radius_ = 0.3f;
    float path_align_obstacle_check_radius_ = 0.15f;
    std::vector<bool> path_pts_valid_;
};

} // namespace mppi

#endif // MPPI_CONTROLLER_HPP_
