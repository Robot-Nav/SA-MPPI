#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <algorithm>

#include "controller.hpp"

class MPPIRos1Node
{
public:
    MPPIRos1Node(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh)
    {
        initController();

        path_sub_ = nh_.subscribe("plan", 1, &MPPIRos1Node::pathCallback, this);
        odom_sub_ = nh_.subscribe("odom", 10, &MPPIRos1Node::odomCallback, this);
        scan_sub_ = nh_.subscribe("scan", 10, &MPPIRos1Node::scanCallback, this);

        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        debug_traj_pub_ = nh_.advertise<nav_msgs::Path>("debug_optimal_trajectory", 10);

        control_timer_ = nh_.createTimer(ros::Duration(control_period_ms_ / 1000.0),
                                         &MPPIRos1Node::controlLoop, this);

        ROS_INFO("MPPI ROS1 Node initialized.");
    }

private:
    void initController()
    {
        mppi::OptimizerSettings settings;
        
        // MPPI核心参数 - 由于ROS1不支持unsigned int的param，使用中间int变量
        // 默认值与YAML文件保持一致
        int batch_size_tmp, time_steps_tmp, iteration_count_tmp, thread_count_tmp;
        pnh_.param("batch_size", batch_size_tmp, 1300);
        pnh_.param("time_steps", time_steps_tmp, 90);
        pnh_.param("iteration_count", iteration_count_tmp, 1);
        pnh_.param("thread_count", thread_count_tmp, 4);
        settings.batch_size = static_cast<unsigned int>(batch_size_tmp);
        settings.time_steps = static_cast<unsigned int>(time_steps_tmp);
        settings.iteration_count = static_cast<unsigned int>(iteration_count_tmp);
        settings.thread_count = static_cast<unsigned int>(thread_count_tmp);

        pnh_.param("model_dt", settings.model_dt, 0.05f);
        pnh_.param("temperature", settings.temperature, 0.3f);
        pnh_.param("gamma", settings.gamma, 0.015f);

        // 路径裁剪距离参数
        pnh_.param("prune_distance", prune_distance_, 2.0f);
        settings.prune_distance = prune_distance_;
 
         // 控制约束 - float类型（与YAML保持一致）
        pnh_.param("vx_max", settings.base_constraints.vx_max, 1.0f);
        pnh_.param("vx_min", settings.base_constraints.vx_min, 0.0f);
        pnh_.param("vy_max", settings.base_constraints.vy_max, 0.5f);
        pnh_.param("wz_max", settings.base_constraints.wz_max, 2.5f);
        pnh_.param("ax_max", settings.base_constraints.ax_max, 1.6f);
        pnh_.param("ay_max", settings.base_constraints.ay_max, 2.0f);
        pnh_.param("az_max", settings.base_constraints.az_max, 3.2f);
        pnh_.param("collision_cost_threshold", settings.base_constraints.collision_cost_threshold, 5000.0f);

        settings.constraints = settings.base_constraints;

        // 噪声采样标准差 - float类型（与YAML保持一致）
        pnh_.param("vx_std", settings.sampling_std.vx, 0.30f);
        pnh_.param("vy_std", settings.sampling_std.vy, 0.0f);
        pnh_.param("wz_std", settings.sampling_std.wz, 0.30f);

        // SG滤波器参数 - bool类型
        pnh_.param("use_sg_filter", settings.use_sg_filter, false);
        pnh_.param("shift_control_sequence", settings.shift_control_sequence, false);
        pnh_.param("retry_attempt_limit", settings.retry_attempt_limit, 1);
        pnh_.param("use_mean_normalization", settings.use_mean_normalization, false);
        pnh_.param("adaptive_temperature", settings.adaptive_temperature, false);
        pnh_.param("adaptive_temperature_min", settings.adaptive_temperature_min, 0.1f);
        pnh_.param("adaptive_temperature_max", settings.adaptive_temperature_max, 1.0f);

        // 阻塞检测参数（与YAML保持一致）
        pnh_.param("twirling_weight", settings.twirling_weight, 0.5f);
        pnh_.param("spinning_ratio_threshold", settings.spinning_ratio_threshold, 8.0f);
        pnh_.param("spinning_detect_frames", settings.spinning_detect_frames, 4);

        // 全向/差速模式切换参数
        pnh_.param("enable_omni_switching", enable_omni_switching_, false);
        settings.enable_omni_switching = enable_omni_switching_;
        pnh_.param("omni_trigger_obstacle_dist", settings.omni_trigger_obstacle_dist, 0.5f);
        pnh_.param("omni_trigger_path_deviation", settings.omni_trigger_path_deviation, 0.3f);
        pnh_.param("diff_restore_path_threshold", settings.diff_restore_path_threshold, 0.15f);
        pnh_.param("omni_switch_delay_frames", settings.omni_switch_delay_frames, 3);

        // 启动辅助参数
        pnh_.param("startup_assist_enabled", startup_assist_enabled_, true);
        pnh_.param("startup_boost_vx", startup_boost_vx_, 0.06f);
        pnh_.param("startup_boost_wz_gain", startup_boost_wz_gain_, 1.2f);
        pnh_.param("startup_boost_wz_limit", startup_boost_wz_limit_, 0.6f);
        pnh_.param("startup_cmd_vx_eps", startup_cmd_vx_eps_, 0.01f);
        pnh_.param("startup_speed_vx_eps", startup_speed_vx_eps_, 0.02f);
        pnh_.param("startup_goal_distance", startup_goal_distance_, 0.30f);
        // startup_lookahead_ 是 int 类型
        pnh_.param("startup_lookahead", startup_lookahead_, 8);

        // 运动模型参数
        std::string motion_model_str;
        pnh_.param("motion_model", motion_model_str, std::string("DiffDrive"));
        double ackermann_radius;
        pnh_.param("ackermann_min_turning_radius", ackermann_radius, 0.2);

        controller_ = std::make_unique<mppi::MPPIController>();
        controller_->initialize(settings, motion_model_str, ackermann_radius);

        ROS_INFO("vx_min = %f", settings.base_constraints.vx_min);

        // 设置ObstaclesCritic参数
        if (auto* critic = controller_->getObstaclesCritic()) {
            double repulsion_weight, critical_weight, collision_cost, collision_margin;
            double inflation_radius, cost_scaling, near_goal_distance, robot_radius;
            double grid_resolution;
            int grid_width, grid_height;
            bool consider_footprint;
            
            // 障碍物参数（与YAML保持一致）
            pnh_.param("obstacle_repulsion_weight", repulsion_weight, 0.5);
            pnh_.param("obstacle_critical_weight", critical_weight, 20.0);
            pnh_.param("obstacle_collision_cost", collision_cost, 10000.0);
            pnh_.param("obstacle_collision_margin", collision_margin, 0.8);
            pnh_.param("obstacle_inflation_radius", inflation_radius, 2.0);
            pnh_.param("obstacle_cost_scaling", cost_scaling, 5.0);
            pnh_.param("obstacle_near_goal_distance", near_goal_distance, 0.3);
            pnh_.param("robot_radius", robot_radius, 0.25);
            pnh_.param("grid_resolution", grid_resolution, 0.05);
            pnh_.param("grid_width", grid_width, 100);
            pnh_.param("grid_height", grid_height, 100);
            pnh_.param("consider_footprint", consider_footprint, false);
            
            std::vector<double> footprint_vec;
            pnh_.param("footprint", footprint_vec, std::vector<double>());
            
            // 转换footprint为Point2D向量
            std::vector<mppi::Point2D> footprint;
            if (consider_footprint && !footprint_vec.empty()) {
                for (size_t i = 0; i + 1 < footprint_vec.size(); i += 2) {
                    footprint.emplace_back(footprint_vec[i], footprint_vec[i+1]);
                }
            }
            
            critic->setParams(repulsion_weight, collision_cost,
                             collision_margin, inflation_radius, cost_scaling,
                             near_goal_distance, robot_radius, grid_resolution,
                             grid_width, grid_height, consider_footprint, footprint);
            
            // 设置机器人半径
            controller_->setRobotRadius(robot_radius);
        }
        
        // 设置路径对齐障碍物检查半径（与YAML保持一致）
        double path_align_check_radius;
        pnh_.param("path_align_obstacle_check_radius", path_align_check_radius, 0.10);
        controller_->setPathAlignObstacleCheckRadius(path_align_check_radius);

        // Path Align Critic（与YAML保持一致）
        if (auto* critic = controller_->getPathAlignCritic()) {
            double weight, threshold, max_occupancy_ratio;
            int offset, traj_step;
            bool use_orientations;
            pnh_.param("path_align_weight", weight, 6.0);
            pnh_.param("path_align_offset", offset, 16);
            pnh_.param("path_align_threshold", threshold, 0.40);
            pnh_.param("path_align_traj_step", traj_step, 3);
            pnh_.param("path_align_max_occupancy_ratio", max_occupancy_ratio, 0.50);
            pnh_.param("path_align_use_orientations", use_orientations, false);
            critic->setParams(weight, offset, threshold, traj_step, max_occupancy_ratio, use_orientations);
        }

        // Path Angle Critic（与YAML保持一致）
        if (auto* critic = controller_->getPathAngleCritic()) {
            double weight, threshold, angle_max;
            int offset, mode;
            pnh_.param("path_angle_weight", weight, 2.0);
            pnh_.param("path_angle_offset", offset, 4);
            pnh_.param("path_angle_threshold", threshold, 0.40);
            pnh_.param("path_angle_max", angle_max, 0.5);
            pnh_.param("path_angle_mode", mode, 0);
            critic->setParams(weight, offset, threshold, angle_max, mode);
        }

        // Path Follow Critic（与YAML保持一致）
        if (auto* critic = controller_->getPathFollowCritic()) {
            double weight, threshold;
            int offset;
            pnh_.param("path_follow_weight", weight, 4.0);
            pnh_.param("path_follow_offset", offset, 7);
            pnh_.param("path_follow_threshold", threshold, 0.6);
            critic->setParams(weight, offset, threshold);
        }

        // Goal Critic
        if (auto* critic = controller_->getGoalCritic()) {
            double weight, threshold;
            pnh_.param("goal_weight", weight, 5.0);
            pnh_.param("goal_threshold", threshold, 1.0);
            critic->setParams(weight, threshold);
        }

        // Goal Angle Critic
        if (auto* critic = controller_->getGoalAngleCritic()) {
            double weight, threshold;
            pnh_.param("goal_angle_weight", weight, 3.0);
            pnh_.param("goal_angle_threshold", threshold, 0.4);
            critic->setParams(weight, threshold);
        }

        // Prefer Forward Critic
        if (auto* critic = controller_->getPreferForwardCritic()) {
            double weight, threshold;
            pnh_.param("prefer_forward_weight", weight, 5.0);
            pnh_.param("prefer_forward_threshold", threshold, 0.5);
            critic->setParams(weight, threshold);
        }

        // Constraint Critic
        if (auto* critic = controller_->getConstraintCritic()) {
            double weight, vx_max, vx_min, vy_max, wz_max, min_turning_radius;
            int motion_model_type;
            pnh_.param("constraint_weight", weight, 4.0);
            pnh_.param("vx_max", vx_max, 0.5);
            pnh_.param("vx_min", vx_min, 0.0);
            pnh_.param("vy_max", vy_max, 0.5);
            pnh_.param("wz_max", wz_max, 1.9);
            pnh_.param("ackermann_min_turning_radius", min_turning_radius, 0.2);
            pnh_.param("motion_model_type", motion_model_type, 0);
            critic->setParams(weight, vx_max, vx_min, vy_max, wz_max, min_turning_radius, motion_model_type);
        }
        
        // Velocity Deadband Critic
        if (auto* critic = controller_->getVelocityDeadbandCritic()) {
            double weight, vx, vy, wz;
            pnh_.param("velocity_deadband_weight", weight, 1.0);
            pnh_.param("velocity_deadband_vx", vx, 0.05);
            pnh_.param("velocity_deadband_vy", vy, 0.05);
            pnh_.param("velocity_deadband_wz", wz, 0.1);
            critic->setParams(weight, vx, vy, wz);
        }

        // Twirling Critic
        if (auto* critic = controller_->getTwirlingCritic()) {
            double weight, vx_max, threshold;
            pnh_.param("twirling_weight", weight, 10.0);
            pnh_.param("vx_max", vx_max, 0.5);
            pnh_.param("twirling_threshold", threshold, 0.5);  // 接近目标点此距离内关闭TwirlingCritic
            critic->setParams(weight, vx_max, threshold);
        }

        // control_period_ms_ 是 int 类型
        pnh_.param("control_period_ms", control_period_ms_, 50);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (msg->poses.empty()) return;

        std::vector<mppi::Pose2D> path_poses;
        path_poses.reserve(msg->poses.size());
        for (const auto & p : msg->poses) {
            float yaw = std::atan2(2.0f * (p.pose.orientation.z * p.pose.orientation.w +
                                            p.pose.orientation.x * p.pose.orientation.y),
                                    1.0f - 2.0f * (p.pose.orientation.y * p.pose.orientation.y +
                                                   p.pose.orientation.z * p.pose.orientation.z));
            path_poses.emplace_back(p.pose.position.x, p.pose.position.y, yaw);
        }
        controller_->setPath(path_poses);
        path_received_ = true;
        ROS_INFO("Path received with %zu points", msg->poses.size());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        robot_pose_.x = msg->pose.pose.position.x;
        robot_pose_.y = msg->pose.pose.position.y;
        double yaw = std::atan2(2.0 * (msg->pose.pose.orientation.z * msg->pose.pose.orientation.w +
                                        msg->pose.pose.orientation.x * msg->pose.pose.orientation.y),
                                1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                             msg->pose.pose.orientation.z * msg->pose.pose.orientation.z));
        robot_pose_.theta = static_cast<float>(yaw);
        robot_speed_.vx = msg->twist.twist.linear.x;
        robot_speed_.vy = msg->twist.twist.linear.y;
        robot_speed_.wz = msg->twist.twist.angular.z;
        pose_received_ = true;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        laser_points_relative_.clear();
        laser_points_relative_.reserve(msg->ranges.size());

        float angle = msg->angle_min;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
            if (range >= msg->range_min && range <= msg->range_max && range < 4.0f) {
                float x_l = range * std::cos(angle);
                float y_l = range * std::sin(angle);
                laser_points_relative_.emplace_back(x_l, y_l);
            }
            angle += msg->angle_increment;
        }
    }

    void controlLoop(const ros::TimerEvent&)
    {
        if (!pose_received_ || !path_received_) {
            return;
        }

        mppi::Pose2D goal = controller_->getPath().getGoal();
        float dist_to_goal = std::hypot(robot_pose_.x - goal.x, robot_pose_.y - goal.y);
        if (dist_to_goal < 0.20) {
            geometry_msgs::Twist stop_msg;
            cmd_vel_pub_.publish(stop_msg);
            return;
        }

        // 将相对激光点转换到全局坐标系
        std::vector<mppi::Point2D> global_points;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            global_points.reserve(laser_points_relative_.size());
            float cos_theta = std::cos(robot_pose_.theta);
            float sin_theta = std::sin(robot_pose_.theta);
            float robot_x = robot_pose_.x;
            float robot_y = robot_pose_.y;
            for (const auto& pt : laser_points_relative_) {
                float global_x = robot_x + pt.x * cos_theta - pt.y * sin_theta;
                float global_y = robot_y + pt.x * sin_theta + pt.y * cos_theta;
                global_points.emplace_back(global_x, global_y);
            }
        }

        // 更新障碍物
        controller_->updateStaticObstacles(global_points, robot_pose_);

        mppi::Twist2D cmd;
        try {
            cmd = controller_->computeVelocityCommands(robot_pose_, robot_speed_);
        } catch (const std::exception & e) {
            ROS_WARN_THROTTLE(1.0, "MPPI compute failed: %s", e.what());
            // 当所有轨迹都碰撞时，输出零速度，让机器人停在原地
            // 而不是抛出异常导致控制中断
            geometry_msgs::Twist stop_msg;
            stop_msg.linear.x = 0.0;
            stop_msg.linear.y = 0.0;
            stop_msg.angular.z = 0.0;
            cmd_vel_pub_.publish(stop_msg);
            
            // 重置控制器状态，为下一次控制周期做准备
            controller_->reset();
            return;
        }

        if (tryApplyStartupAssist(cmd, global_points.empty(), dist_to_goal)) {
            ROS_WARN_THROTTLE(1.0, "MPPI: startup assist applied (near-zero cmd without obstacles).");
        }

        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = cmd.vx;
        twist_msg.linear.y = cmd.vy;
        twist_msg.angular.z = cmd.wz;
        cmd_vel_pub_.publish(twist_msg);

        // 阻塞检测日志
        if (controller_->isCurrentlyBlocked()) {
            ROS_WARN_THROTTLE(1.0, "MPPI: Blocked by obstacles, stopping robot.");
        }
        
        // 运动模型切换状态日志
        if (enable_omni_switching_) {
            static bool last_omni_state = false;
            bool current_omni_state = controller_->isOmniModeActive();
            if (current_omni_state != last_omni_state) {
                if (current_omni_state) {
                    ROS_WARN("MPPI: Switched to OMNI mode (obstacle_dist=%.2f, path_dev=%.2f)",
                             controller_->getMinObstacleDistance(), controller_->getCurrentPathDeviation());
                } else {
                    ROS_INFO("MPPI: Restored DIFF mode (obstacle_dist=%.2f, path_dev=%.2f)",
                             controller_->getMinObstacleDistance(), controller_->getCurrentPathDeviation());
                }
                last_omni_state = current_omni_state;
            }
        }

        publishDebugTrajectory();
    }

    bool tryApplyStartupAssist(mppi::Twist2D & cmd, bool no_obstacles, float dist_to_goal)
    {
        // 如果启动辅助被禁用，直接返回false
        if (!startup_assist_enabled_) return false;
        
        if (!no_obstacles) return false;
        if (controller_->isCurrentlyBlocked()) return false;
        if (dist_to_goal < startup_goal_distance_) return false;

        if (std::abs(cmd.vx) > startup_cmd_vx_eps_ || std::abs(robot_speed_.vx) > startup_speed_vx_eps_) {
            return false;
        }

        auto & path = controller_->getPath();
        if (path.size() < 2) return false;

        size_t closest_idx = 0;
        float min_dist = std::numeric_limits<float>::max();
        for (size_t i = 0; i < path.size(); ++i) {
            float d = std::hypot(path.x(i) - robot_pose_.x, path.y(i) - robot_pose_.y);
            if (d < min_dist) {
                min_dist = d;
                closest_idx = i;
            }
        }

        size_t lookahead_idx = std::min(closest_idx + static_cast<size_t>(startup_lookahead_), path.size() - 1);
        float target_heading = std::atan2(path.y(lookahead_idx) - robot_pose_.y,
                                          path.x(lookahead_idx) - robot_pose_.x);
        float heading_error = mppi::shortestAngularDistance(robot_pose_.theta, target_heading);

        cmd.vx = startup_boost_vx_;
        cmd.vy = 0.0f;
        cmd.wz = std::clamp(startup_boost_wz_gain_ * heading_error,
                            -startup_boost_wz_limit_, startup_boost_wz_limit_);
        return true;
    }

    void publishDebugTrajectory()
    {
        auto optimal_traj = controller_->getOptimizedTrajectory();
        size_t T = optimal_traj.shape(0);
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";
        path_msg.poses.resize(T);
        for (size_t i = 0; i < T; ++i) {
            path_msg.poses[i].header = path_msg.header;
            path_msg.poses[i].pose.position.x = optimal_traj(i, 0);
            path_msg.poses[i].pose.position.y = optimal_traj(i, 1);
            float yaw = optimal_traj(i, 2);
            path_msg.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
            path_msg.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
        }
        debug_traj_pub_.publish(path_msg);
    }

    // Members
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::unique_ptr<mppi::MPPIController> controller_;
    mppi::Pose2D robot_pose_;
    mppi::Twist2D robot_speed_;
    std::vector<mppi::Point2D> laser_points_relative_;
    std::atomic<bool> pose_received_{false};
    std::atomic<bool> path_received_{false};
    int control_period_ms_;
    float prune_distance_ = 2.0f;  // 路径裁剪距离
    bool startup_assist_enabled_ = true;  // 启动辅助开关
    float startup_boost_vx_ = 0.06f;
    float startup_boost_wz_gain_ = 1.2f;
    float startup_boost_wz_limit_ = 0.6f;
    float startup_cmd_vx_eps_ = 0.01f;
    float startup_speed_vx_eps_ = 0.02f;
    float startup_goal_distance_ = 0.30f;
    int startup_lookahead_ = 8;
    bool enable_omni_switching_ = false;  // 全向/差速模式切换开关

    std::mutex data_mutex_;

    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher debug_traj_pub_;
    ros::Timer control_timer_;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "mppi_ros1_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    MPPIRos1Node node(nh, pnh);
    ros::spin();
    
    return 0;
}