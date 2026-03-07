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

#include "mppi_controller.hpp"

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

        RCLCPP_INFO(this->get_logger(), "MPPI Industrial Node initialized with Independent Obstacle Handler.");
    }

private:
    void declare_parameters()
    {
        // Controller Core
        this->declare_parameter<int>("batch_size", 1200);
        this->declare_parameter<int>("time_steps", 56);
        this->declare_parameter<double>("model_dt", 0.05);
        this->declare_parameter<double>("temperature", 0.3);
        this->declare_parameter<double>("gamma", 0.015);
        this->declare_parameter<int>("iteration_count", 1);
        this->declare_parameter<bool>("shift_control_sequence", false);
        this->declare_parameter<int>("retry_attempt_limit", 1);
        this->declare_parameter<std::string>("motion_model", "DiffDrive");
        this->declare_parameter<double>("ackermann_min_turning_radius", 0.2);
        this->declare_parameter<int>("control_period_ms", 50);
        this->declare_parameter<int>("thread_count", 4);

        // Constraints
        this->declare_parameter<double>("vx_max", 0.5);
        this->declare_parameter<double>("vx_min", 0);
        this->declare_parameter<double>("vy_max", 0.5);
        this->declare_parameter<double>("wz_max", 1.9);

        // Acceleration Constraints
        this->declare_parameter<double>("ax_max", 2.0);
        this->declare_parameter<double>("ay_max", 2.0);
        this->declare_parameter<double>("az_max", 3.0);

        // Sampling
        this->declare_parameter<double>("vx_std", 0.2);
        this->declare_parameter<double>("vy_std", 0.2);
        this->declare_parameter<double>("wz_std", 0.4);

        // ==================== 独立避障参数 ====================
        this->declare_parameter<bool>("enable_avoidance", true);
        this->declare_parameter<double>("max_lateral_avoidance_distance", 0.5);
        this->declare_parameter<double>("safety_margin_lateral", 0.2);
        this->declare_parameter<double>("safety_margin_longitudinal", 0.5);
        this->declare_parameter<double>("robot_radius", 0.3);
        this->declare_parameter<double>("emergency_brake_decel", 2.0);
        this->declare_parameter<double>("ttc_emergency_threshold", 1.0);
        this->declare_parameter<double>("follow_distance", 1.0);
        this->declare_parameter<double>("grid_resolution", 0.05);
        this->declare_parameter<int>("grid_width", 100);
        this->declare_parameter<int>("grid_height", 100);
        this->declare_parameter<double>("avoidance_lookahead", 3.0);
        this->declare_parameter<double>("avoidance_smooth_distance", 0.5);
        this->declare_parameter<double>("avoid_speed_ratio", 0.7);
        this->declare_parameter<double>("min_avoid_speed", 0.08);

        // Path Align Critic
        this->declare_parameter<double>("path_align_weight", 14.0);
        this->declare_parameter<int>("path_align_offset", 20);
        this->declare_parameter<double>("path_align_threshold", 0.40);
        this->declare_parameter<int>("path_align_traj_step", 4);

        // Path Angle Critic
        this->declare_parameter<double>("path_angle_weight", 2.0);
        this->declare_parameter<int>("path_angle_offset", 4);
        this->declare_parameter<double>("path_angle_threshold", 0.40);
        this->declare_parameter<double>("path_angle_max", 0.8);

        // Path Follow Critic
        this->declare_parameter<double>("path_follow_weight", 5.0);
        this->declare_parameter<int>("path_follow_offset", 5);
        this->declare_parameter<double>("path_follow_threshold", 0.6);

        // Goal Critic
        this->declare_parameter<double>("goal_weight", 5.0);
        this->declare_parameter<double>("goal_threshold", 1.0);

        // Goal Angle Critic
        this->declare_parameter<double>("goal_angle_weight", 3.0);
        this->declare_parameter<double>("goal_angle_threshold", 0.4);

        // Prefer Forward Critic
        this->declare_parameter<double>("prefer_forward_weight", 5.0);
        this->declare_parameter<double>("prefer_forward_threshold", 0.4);

        // Constraint Critic
        this->declare_parameter<double>("constraint_weight", 4.0);
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
        settings.shift_control_sequence = get_parameter("shift_control_sequence").as_bool();
        settings.retry_attempt_limit = get_parameter("retry_attempt_limit").as_int();
        settings.thread_count = get_parameter("thread_count").as_int();

        settings.base_constraints.vx_max = get_parameter("vx_max").as_double();
        settings.base_constraints.vx_min = get_parameter("vx_min").as_double();
        settings.base_constraints.vy_max = get_parameter("vy_max").as_double();
        settings.base_constraints.wz_max = get_parameter("wz_max").as_double();

        settings.base_constraints.ax_max = get_parameter("ax_max").as_double();
        settings.base_constraints.ay_max = get_parameter("ay_max").as_double();
        settings.base_constraints.az_max = get_parameter("az_max").as_double();

        settings.constraints = settings.base_constraints;

        settings.sampling_std.vx = get_parameter("vx_std").as_double();
        settings.sampling_std.vy = get_parameter("vy_std").as_double();
        settings.sampling_std.wz = get_parameter("wz_std").as_double();

        std::string motion_model_str = get_parameter("motion_model").as_string();
        double ackermann_radius = get_parameter("ackermann_min_turning_radius").as_double();

        controller_ = std::make_unique<mppi::MPPIController>();
        controller_->initialize(settings, motion_model_str, ackermann_radius);

        RCLCPP_INFO(this->get_logger(), "vx_min = %f", get_parameter("vx_min").as_double());

        // ==================== 设置独立避障参数 ====================
        mppi::AvoidanceConfig avoidance_config;
        avoidance_config.enable_avoidance = get_parameter("enable_avoidance").as_bool();
        avoidance_config.max_lateral_avoidance_distance = get_parameter("max_lateral_avoidance_distance").as_double();
        avoidance_config.safety_margin_lateral = get_parameter("safety_margin_lateral").as_double();
        avoidance_config.safety_margin_longitudinal = get_parameter("safety_margin_longitudinal").as_double();
        avoidance_config.robot_radius = get_parameter("robot_radius").as_double();
        avoidance_config.emergency_brake_decel = get_parameter("emergency_brake_decel").as_double();
        avoidance_config.ttc_emergency_threshold = get_parameter("ttc_emergency_threshold").as_double();
        avoidance_config.follow_distance = get_parameter("follow_distance").as_double();
        avoidance_config.grid_resolution = get_parameter("grid_resolution").as_double();
        avoidance_config.grid_width = get_parameter("grid_width").as_int();
        avoidance_config.grid_height = get_parameter("grid_height").as_int();
        avoidance_config.avoidance_lookahead = get_parameter("avoidance_lookahead").as_double();
        avoidance_config.avoidance_smooth_distance = get_parameter("avoidance_smooth_distance").as_double();
        avoidance_config.avoid_speed_ratio = get_parameter("avoid_speed_ratio").as_double();
        avoidance_config.min_avoid_speed = get_parameter("min_avoid_speed").as_double();
        avoidance_config.prediction_dt = static_cast<float>(settings.model_dt);
        
        controller_->setAvoidanceConfig(avoidance_config);
        
        RCLCPP_INFO(this->get_logger(), "Avoidance Config: enable=%s, max_lateral=%.2f, safety_lateral=%.2f, safety_longitudinal=%.2f, lookahead=%.2f, smooth_dist=%.2f",
                    avoidance_config.enable_avoidance ? "true" : "false",
                    avoidance_config.max_lateral_avoidance_distance,
                    avoidance_config.safety_margin_lateral,
                    avoidance_config.safety_margin_longitudinal,
                    avoidance_config.avoidance_lookahead,
                    avoidance_config.avoidance_smooth_distance);

        // Path Align Critic
        if (auto* critic = controller_.get()->getPathAlignCritic()) {
            critic->setParams(
                get_parameter("path_align_weight").as_double(),
                get_parameter("path_align_offset").as_int(),
                get_parameter("path_align_threshold").as_double(),
                get_parameter("path_align_traj_step").as_int()
            );
        }

        // Path Angle Critic
        if (auto* critic = controller_.get()->getPathAngleCritic()) {
            critic->setParams(
                get_parameter("path_angle_weight").as_double(),
                get_parameter("path_angle_offset").as_int(),
                get_parameter("path_angle_threshold").as_double(),
                get_parameter("path_angle_max").as_double()
            );
        }

        // Path Follow Critic
        if (auto* critic = controller_.get()->getPathFollowCritic()) {
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
            critic->setParams(get_parameter("constraint_weight").as_double());
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
        
        if (dist_to_goal < 0.30) {
            geometry_msgs::msg::Twist stop_msg;
            cmd_vel_pub_->publish(stop_msg);
            return;
        }

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

        controller_->updateStaticObstacles(global_points, robot_pose_);

        mppi::Twist2D cmd;
        try {
            cmd = controller_->computeVelocityCommands(robot_pose_, robot_speed_);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "MPPI compute failed: %s", e.what());
            return;
        }

        bool is_stopped = (std::abs(cmd.vx) < 0.001f && std::abs(cmd.wz) < 0.001f);
        
        auto* obstacle_handler = controller_->getObstacleHandler();
        if (obstacle_handler) {
            const auto& obstacle_result = obstacle_handler->getLastResult();
            
            if (is_stopped && prev_was_moving_) {
                switch (obstacle_result.action) {
                    case mppi::ObstacleAction::EMERGENCY_BRAKE:
                        RCLCPP_WARN(this->get_logger(), "紧急制动：碰撞即将发生，TTC < %.2f 秒", 
                                    controller_->getAvoidanceConfig().ttc_emergency_threshold);
                        break;
                    case mppi::ObstacleAction::NORMAL_STOP:
                        if (obstacle_result.path_blocked) {
                            RCLCPP_WARN(this->get_logger(), "路径阻塞：障碍物完全阻断路径，刹停等待");
                        }
                        break;
                    default:
                        RCLCPP_WARN(this->get_logger(), "障碍物阻塞检测：机器人刹停");
                        break;
                }
                prev_was_moving_ = false;
            } else if (!is_stopped && !prev_was_moving_) {
                RCLCPP_INFO(this->get_logger(), "障碍物已移开：恢复路径跟踪");
                prev_was_moving_ = true;
            }
            
            if (obstacle_result.action == mppi::ObstacleAction::FOLLOW) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "跟随动态障碍物，目标速度: %.2f m/s", obstacle_result.target_speed);
            }
        }

        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = cmd.vx;
        twist_msg.linear.y = cmd.vy;
        twist_msg.angular.z = cmd.wz;
        cmd_vel_pub_->publish(twist_msg);

        publishDebugTrajectory();
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
    std::vector<mppi::Point2D> laser_points_relative_;
    std::atomic<bool> pose_received_{false};
    std::atomic<bool> path_received_{false};
    int control_period_ms_;
    bool prev_was_moving_ = true;

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