#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <algorithm>

#include "mppi_controller.hpp"  // 请确保路径正确

using namespace std::chrono_literals;

class MPPIRos2Node : public rclcpp::Node
{
public:
    MPPIRos2Node() : Node("mppi_ros2_node")
    {
        declare_parameters();
        initController();

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", rclcpp::QoS(1).transient_local(),
            std::bind(&MPPIRos2Node::pathCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::QoS(10),
            std::bind(&MPPIRos2Node::odomCallback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::QoS(10),
            std::bind(&MPPIRos2Node::scanCallback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        debug_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("debug_optimal_trajectory", 10);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(control_period_ms_),
            std::bind(&MPPIRos2Node::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "MPPI Industrial Node initialized.");
    }

private:
    void declare_parameters()
    {
        // MPPI核心参数
        this->declare_parameter<int>("batch_size", 1200);
        this->declare_parameter<int>("time_steps", 56);
        this->declare_parameter<double>("model_dt", 0.05);
        this->declare_parameter<double>("temperature", 0.3);
        this->declare_parameter<double>("gamma", 0.015);
        this->declare_parameter<int>("iteration_count", 1);
        this->declare_parameter<int>("control_period_ms", 50);
        this->declare_parameter<int>("thread_count", 4);

        // 控制约束
        this->declare_parameter<double>("vx_max", 0.5);
        this->declare_parameter<double>("vx_min", 0);
        this->declare_parameter<double>("vy_max", 0.5);
        this->declare_parameter<double>("wz_max", 1.9);
        this->declare_parameter<double>("ax_max", 2.0);
        this->declare_parameter<double>("ay_max", 2.0);
        this->declare_parameter<double>("az_max", 3.0);

        // 噪声采样标准差
        this->declare_parameter<double>("vx_std", 0.2);
        this->declare_parameter<double>("vy_std", 0.2);
        this->declare_parameter<double>("wz_std", 0.4);

        // 障碍物代价参数
        this->declare_parameter<double>("obstacle_repulsion_weight", 1.5);
        this->declare_parameter<double>("obstacle_critical_weight", 20.0);
        this->declare_parameter<double>("obstacle_collision_cost", 10000.0);
        this->declare_parameter<double>("obstacle_collision_margin", 0.10);
        this->declare_parameter<double>("collision_cost_threshold", 5000.0);
        this->declare_parameter<double>("obstacle_inflation_radius", 0.55);
        this->declare_parameter<double>("obstacle_cost_scaling", 10.0);
        this->declare_parameter<double>("obstacle_near_goal_distance", 0.5);
        this->declare_parameter<double>("robot_radius", 0.3);
        this->declare_parameter<bool>("consider_footprint", false);
        this->declare_parameter<std::vector<double>>("footprint", std::vector<double>{});
        this->declare_parameter<double>("grid_resolution", 0.1);
        this->declare_parameter<int>("grid_width", 100);
        this->declare_parameter<int>("grid_height", 100);

        // 阻塞检测参数
        this->declare_parameter<double>("twirling_weight", 10.0);
        this->declare_parameter<double>("spinning_ratio_threshold", 8.0);
        this->declare_parameter<int>("spinning_detect_frames", 4);

        // 启动辅助参数
        this->declare_parameter<double>("startup_boost_vx", 0.06);
        this->declare_parameter<double>("startup_boost_wz_gain", 1.2);
        this->declare_parameter<double>("startup_boost_wz_limit", 0.6);
        this->declare_parameter<double>("startup_cmd_vx_eps", 0.01);
        this->declare_parameter<double>("startup_speed_vx_eps", 0.02);
        this->declare_parameter<double>("startup_goal_distance", 0.30);
        this->declare_parameter<int>("startup_lookahead", 8);

        // 路径对齐代价参数
        this->declare_parameter<double>("path_align_weight", 14.0);
        this->declare_parameter<int>("path_align_offset", 20);
        this->declare_parameter<double>("path_align_threshold", 0.40);
        this->declare_parameter<int>("path_align_traj_step", 4);
        this->declare_parameter<double>("path_align_max_occupancy_ratio", 0.07);
        this->declare_parameter<bool>("path_align_use_orientations", false);

        // 路径角度代价参数
        this->declare_parameter<double>("path_angle_weight", 2.0);
        this->declare_parameter<int>("path_angle_offset", 4);
        this->declare_parameter<double>("path_angle_threshold", 0.40);
        this->declare_parameter<double>("path_angle_max", 0.8);
        this->declare_parameter<int>("path_angle_mode", 0);

        // 路径跟随代价参数
        this->declare_parameter<double>("path_follow_weight", 5.0);
        this->declare_parameter<int>("path_follow_offset", 5);
        this->declare_parameter<double>("path_follow_threshold", 0.6);

        // 目标点代价参数
        this->declare_parameter<double>("goal_weight", 5.0);
        this->declare_parameter<double>("goal_threshold", 1.0);
        this->declare_parameter<double>("goal_angle_weight", 3.0);
        this->declare_parameter<double>("goal_angle_threshold", 0.4);

        // 偏好前进代价参数
        this->declare_parameter<double>("prefer_forward_weight", 5.0);
        this->declare_parameter<double>("prefer_forward_threshold", 0.5);

        // 约束代价参数
        this->declare_parameter<double>("constraint_weight", 4.0);
        this->declare_parameter<int>("motion_model_type", 0);
        
        // 速度死区代价参数
        this->declare_parameter<double>("velocity_deadband_weight", 1.0);
        this->declare_parameter<double>("velocity_deadband_vx", 0.05);
        this->declare_parameter<double>("velocity_deadband_vy", 0.05);
        this->declare_parameter<double>("velocity_deadband_wz", 0.1);

        // SG滤波器参数
        this->declare_parameter<bool>("use_sg_filter", false);

        // 运动模型参数
        this->declare_parameter<std::string>("motion_model", "DiffDrive");
        this->declare_parameter<double>("ackermann_min_turning_radius", 0.2);
    }

    void initController()
    {
        mppi::OptimizerSettings settings;
        settings.batch_size = get_parameter("batch_size").as_int();
        settings.time_steps = get_parameter("time_steps").as_int();
        settings.model_dt = get_parameter("model_dt").as_double();
        settings.temperature = get_parameter("temperature").as_double();
        settings.gamma = get_parameter("gamma").as_double();
        settings.iteration_count = get_parameter("iteration_count").as_int();
        settings.thread_count = get_parameter("thread_count").as_int();

        settings.base_constraints.vx_max = get_parameter("vx_max").as_double();
        settings.base_constraints.vx_min = get_parameter("vx_min").as_double();
        settings.base_constraints.vy_max = get_parameter("vy_max").as_double();
        settings.base_constraints.wz_max = get_parameter("wz_max").as_double();

        settings.base_constraints.ax_max = get_parameter("ax_max").as_double();
        settings.base_constraints.ay_max = get_parameter("ay_max").as_double();
        settings.base_constraints.az_max = get_parameter("az_max").as_double();
        settings.base_constraints.collision_cost_threshold = get_parameter("collision_cost_threshold").as_double();

        settings.constraints = settings.base_constraints;

        settings.sampling_std.vx = get_parameter("vx_std").as_double();
        settings.sampling_std.vy = get_parameter("vy_std").as_double();
        settings.sampling_std.wz = get_parameter("wz_std").as_double();

        // SG滤波器参数
        settings.use_sg_filter = get_parameter("use_sg_filter").as_bool();

        // Blocking detection / twirling parameters
        settings.twirling_weight = get_parameter("twirling_weight").as_double();
        settings.spinning_ratio_threshold = get_parameter("spinning_ratio_threshold").as_double();
        settings.spinning_detect_frames = get_parameter("spinning_detect_frames").as_int();

        startup_boost_vx_ = static_cast<float>(get_parameter("startup_boost_vx").as_double());
        startup_boost_wz_gain_ = static_cast<float>(get_parameter("startup_boost_wz_gain").as_double());
        startup_boost_wz_limit_ = static_cast<float>(get_parameter("startup_boost_wz_limit").as_double());
        startup_cmd_vx_eps_ = static_cast<float>(get_parameter("startup_cmd_vx_eps").as_double());
        startup_speed_vx_eps_ = static_cast<float>(get_parameter("startup_speed_vx_eps").as_double());
        startup_goal_distance_ = static_cast<float>(get_parameter("startup_goal_distance").as_double());
        startup_lookahead_ = std::max(1, static_cast<int>(get_parameter("startup_lookahead").as_int()));

        std::string motion_model_str = get_parameter("motion_model").as_string();
        double ackermann_radius = get_parameter("ackermann_min_turning_radius").as_double();

        controller_ = std::make_unique<mppi::MPPIController>();
        controller_->initialize(settings, motion_model_str, ackermann_radius);

        // Print vx_min to verify parameter loading
        RCLCPP_INFO(this->get_logger(), "vx_min = %f", get_parameter("vx_min").as_double());

        // Set ObstaclesCritic parameters (grid version)
        if (auto* critic = controller_->getObstaclesCritic()) {
            critic->setParams(
                get_parameter("obstacle_repulsion_weight").as_double(),
                get_parameter("obstacle_critical_weight").as_double(),
                get_parameter("obstacle_collision_cost").as_double(),
                get_parameter("obstacle_collision_margin").as_double(),
                get_parameter("obstacle_inflation_radius").as_double(),
                get_parameter("obstacle_cost_scaling").as_double(),
                get_parameter("obstacle_near_goal_distance").as_double(),
                get_parameter("robot_radius").as_double(),
                get_parameter("grid_resolution").as_double(),
                get_parameter("grid_width").as_int(),
                get_parameter("grid_height").as_int(),
                get_parameter("consider_footprint").as_bool()
            );
            
            // 设置机器人足迹（如果是非圆形机器人）
            if (get_parameter("consider_footprint").as_bool()) {
                auto footprint_vec = get_parameter("footprint").as_double_array();
                std::vector<mppi::Point2D> footprint;
                for (size_t i = 0; i + 1 < footprint_vec.size(); i += 2) {
                    footprint.emplace_back(footprint_vec[i], footprint_vec[i+1]);
                }
                critic->setFootprint(footprint);
            }
        }

        // Path Align Critic
        if (auto* critic = controller_->getPathAlignCritic()) {
            critic->setParams(
                get_parameter("path_align_weight").as_double(),
                get_parameter("path_align_offset").as_int(),
                get_parameter("path_align_threshold").as_double(),
                get_parameter("path_align_traj_step").as_int(),
                get_parameter("path_align_max_occupancy_ratio").as_double(),
                get_parameter("path_align_use_orientations").as_bool()
            );
        }

        // Path Angle Critic
        if (auto* critic = controller_->getPathAngleCritic()) {
            critic->setParams(
                get_parameter("path_angle_weight").as_double(),
                get_parameter("path_angle_offset").as_int(),
                get_parameter("path_angle_threshold").as_double(),
                get_parameter("path_angle_max").as_double(),
                get_parameter("path_angle_mode").as_int()
            );
        }

        // Path Follow Critic
        if (auto* critic = controller_->getPathFollowCritic()) {
            critic->setParams(
                get_parameter("path_follow_weight").as_double(),
                get_parameter("path_follow_offset").as_int(),
                get_parameter("path_follow_threshold").as_double()
            );
        }

        // Goal Critic
        if (auto* critic = controller_->getGoalCritic()) {
            critic->setParams(
                get_parameter("goal_weight").as_double(),
                get_parameter("goal_threshold").as_double()
            );
        }

        // Goal Angle Critic
        if (auto* critic = controller_->getGoalAngleCritic()) {
            critic->setParams(
                get_parameter("goal_angle_weight").as_double(),
                get_parameter("goal_angle_threshold").as_double()
            );
        }

        // Prefer Forward Critic
        if (auto* critic = controller_->getPreferForwardCritic()) {
            critic->setParams(
                get_parameter("prefer_forward_weight").as_double(),
                get_parameter("prefer_forward_threshold").as_double()
            );
        }

        // Constraint Critic
        if (auto* critic = controller_->getConstraintCritic()) {
            critic->setParams(
                get_parameter("constraint_weight").as_double(),
                get_parameter("vx_max").as_double(),
                get_parameter("vx_min").as_double(),
                get_parameter("vy_max").as_double(),
                get_parameter("wz_max").as_double(),
                get_parameter("ackermann_min_turning_radius").as_double(),
                get_parameter("motion_model_type").as_int()
            );
        }
        
        // Velocity Deadband Critic
        if (auto* critic = controller_->getVelocityDeadbandCritic()) {
            critic->setParams(
                get_parameter("velocity_deadband_weight").as_double(),
                get_parameter("velocity_deadband_vx").as_double(),
                get_parameter("velocity_deadband_vy").as_double(),
                get_parameter("velocity_deadband_wz").as_double()
            );
        }

        // Twirling Critic (anti-spinning)
        if (auto* critic = controller_->getTwirlingCritic()) {
            critic->setParams(
                get_parameter("twirling_weight").as_double(),
                get_parameter("vx_max").as_double()
            );
        }

        control_period_ms_ = get_parameter("control_period_ms").as_int();
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
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
        RCLCPP_INFO(this->get_logger(), "Path received with %zu points", msg->poses.size());
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

    // 存储激光点的相对坐标（雷达坐标系下）
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
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

    void controlLoop()
    {
        if (!pose_received_ || !path_received_) {
            return;
        }

        mppi::Pose2D goal = controller_->getPath().getGoal();
        float dist_to_goal = std::hypot(robot_pose_.x - goal.x, robot_pose_.y - goal.y);
        if (dist_to_goal < 0.20) {
            geometry_msgs::msg::Twist stop_msg;
            cmd_vel_pub_->publish(stop_msg);
            return;
        }

        // 将相对激光点转换到全局坐标系（使用最新 robot_pose_）
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

        // 更新障碍物（即使点云为空也要刷新，用于清空旧障碍）
        controller_->updateStaticObstacles(global_points, robot_pose_);
        // TODO: 动态障碍物设置（如果实现）

        mppi::Twist2D cmd;
        try {
            cmd = controller_->computeVelocityCommands(robot_pose_, robot_speed_);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "MPPI compute failed: %s", e.what());
            // 发布停止指令，避免机器人保持上一时刻速度
            geometry_msgs::msg::Twist stop_msg;
            stop_msg.linear.x = 0.0;
            stop_msg.linear.y = 0.0;
            stop_msg.angular.z = 0.0;
            cmd_vel_pub_->publish(stop_msg);
            return;
        }

        if (tryApplyStartupAssist(cmd, global_points.empty(), dist_to_goal)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "MPPI: startup assist applied (near-zero cmd without obstacles).");
        }

        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = cmd.vx;
        twist_msg.linear.y = cmd.vy;
        twist_msg.angular.z = cmd.wz;
        cmd_vel_pub_->publish(twist_msg);

        // 阻塞检测日志
        if (controller_->isCurrentlyBlocked()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "MPPI: Blocked by obstacles, stopping robot.");
        }

        publishDebugTrajectory();
    }

    bool tryApplyStartupAssist(mppi::Twist2D & cmd, bool no_obstacles, float dist_to_goal)
    {
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
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
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
        debug_traj_pub_->publish(path_msg);
    }

    // Members
    std::unique_ptr<mppi::MPPIController> controller_;
    mppi::Pose2D robot_pose_;
    mppi::Twist2D robot_speed_;
    std::vector<mppi::Point2D> laser_points_relative_;  // 存储雷达坐标系下的点
    std::atomic<bool> pose_received_{false};
    std::atomic<bool> path_received_{false};
    int control_period_ms_;
    float startup_boost_vx_ = 0.06f;
    float startup_boost_wz_gain_ = 1.2f;
    float startup_boost_wz_limit_ = 0.6f;
    float startup_cmd_vx_eps_ = 0.01f;
    float startup_speed_vx_eps_ = 0.02f;
    float startup_goal_distance_ = 0.30f;
    int startup_lookahead_ = 8;

    std::mutex data_mutex_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_traj_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPPIRos2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}