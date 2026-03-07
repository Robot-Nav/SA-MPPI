# OptiIO-MPPI (Optimized Independent Obstacle-MPPI)

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%2FIron%2FJazzy-green.svg)](https://docs.ros.org/)
[![C++17](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)

**OptiIO-MPPI** is a high-performance ROS2 MPPI (Model Predictive Path Integral) controller implementation designed for mobile robot path tracking and real-time obstacle avoidance. The core innovation of this project lies in the **independent obstacle handling mechanism**, which decouples the obstacle avoidance logic from the MPPI optimization process, providing more reliable and predictable avoidance behavior.

**Acronym Meaning**:
- **Opti** = Optimized
- **IO** = Independent Obstacle
- **MPPI** = Model Predictive Path Integral

---

## 📋 Table of Contents

- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [MPPI Algorithm Principles and Formulas](#mppi-algorithm-principles-and-formulas)
- [Independent Obstacle Handling Mechanism](#independent-obstacle-handling-mechanism)
- [Comparison with Nav2](#comparison-with-nav2)
- [Installation and Dependencies](#installation-and-dependencies)
- [Quick Start](#quick-start)
- [Parameter Configuration](#parameter-configuration)
- [API Documentation](#api-documentation)
- [License](#license)

---

## 🚀 Key Features

### 1. Independent Obstacle Handling Mechanism
- **Decoupled Design**: Obstacle avoidance logic is independent of the MPPI sampling optimization process
- **Multi-layer Decision Making**: Emergency Brake → Normal Stop → Follow → Lateral Avoidance
- **Static/Dynamic Obstacle Separation**: Different strategies for different obstacle types
- **Distance Field Grid**: Efficient obstacle distance computation

### 2. High-Performance MPPI Core
- **Parallel Trajectory Integration**: Multi-threaded acceleration (configurable thread count)
- **xtensor Acceleration**: Efficient matrix operations using the xtensor library
- **Softmax Weight Update**: Temperature-regulated trajectory weighted averaging
- **Multiple Motion Model Support**: Differential drive, omnidirectional, Ackermann steering

### 3. Comprehensive Critic Function System
- **PathAlignCritic**: Trajectory-path alignment
- **PathFollowCritic**: Trajectory endpoint tracking
- **PathAngleCritic**: Heading consistency
- **GoalCritic / GoalAngleCritic**: Target point and orientation
- **PreferForwardCritic**: Forward preference (penalizes backward motion)
- **ConstraintCritic**: Smoothness constraints

### 4. Industrial-Grade Features
- **Acceleration Constraints**: Linear and angular acceleration limits
- **Control Sequence Smoothing**: Control rate change limits
- **Fault Recovery Mechanism**: Automatic reset on optimization failure
- **Real-Time Performance**: 50ms control cycle support

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                   OptiIO-MPPI Architecture                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐      │
│  │ Sensor Input │    │  Path Input  │    │ Odometry In  │      │
│  │  LaserScan   │    │     Path     │    │   Odometry   │      │
│  └──────┬───────┘    └──────┬───────┘    └──────┬───────┘      │
│         │                   │                   │              │
│         ▼                   ▼                   ▼              │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              MPPIRos2Node (ROS2 Interface)              │   │
│  │  ┌─────────────────────────────────────────────────────┐│   │
│  │  │        Independent Obstacle Handler                 ││   │
│  │  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ││   │
│  │  │  │Static Eval  │  │Dynamic Eval │  │Avoid Path   │  ││   │
│  │  │  │   (Grid)    │  │   (TTC)     │  │  (Smooth)   │  ││   │
│  │  │  └─────────────┘  └─────────────┘  └─────────────┘  ││   │
│  │  └─────────────────────────────────────────────────────┘│   │
│  │                         │                               │   │
│  │                         ▼                               │   │
│  │  ┌─────────────────────────────────────────────────────┐│   │
│  │  │           MPPIController (Core Controller)          ││   │
│  │  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ││   │
│  │  │  │   Optimizer │  │CriticManager│  │ MotionModel │  ││   │
│  │  │  │  (Sampling) │  │ (Cost Calc) │  │(Kinematics) │  ││   │
│  │  │  └─────────────┘  └─────────────┘  └─────────────┘  ││   │
│  │  └─────────────────────────────────────────────────────┘│   │
│  └─────────────────────────────────────────────────────────┘   │
│                              │                                  │
│                              ▼                                  │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              Output: cmd_vel (Twist Message)            │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📐 MPPI Algorithm Principles and Formulas

### 1. Algorithm Overview

MPPI (Model Predictive Path Integral) is a sampling-based stochastic optimal control algorithm. It samples a large number of candidate trajectories, evaluates each trajectory using a cost function, and then uses a softmax mechanism to compute a weighted average to obtain the optimal control sequence.

### 2. Core Mathematical Formulas

#### 2.1 Trajectory Sampling

For each time step $t$ and control dimension, sample noise from a normal distribution:

$$
\delta u_t^{(k)} \sim \mathcal{N}(0, \Sigma)
$$

Where $\Sigma = \text{diag}(\sigma_{vx}^2, \sigma_{vy}^2, \sigma_{\omega}^2)$ is the covariance matrix of control variables.

Noisy control sequence:

$$
\nu_t^{(k)} = u_t + \delta u_t^{(k)}
$$

#### 2.2 Kinematic Model (Differential Drive)

State vector $\mathbf{x} = [x, y, \theta]^T$, control input $\mathbf{u} = [v_x, \omega]^T$

$$
\begin{cases}
\dot{x} = v_x \cos(\theta) \\
\dot{y} = v_x \sin(\theta) \\
\dot{\theta} = \omega
\end{cases}
$$

Discretized form (Euler integration):

$$
\begin{cases}
x_{t+1} = x_t + v_x \cos(\theta_t) \cdot \Delta t \\
y_{t+1} = y_t + v_x \sin(\theta_t) \cdot \Delta t \\
\theta_{t+1} = \theta_t + \omega \cdot \Delta t
\end{cases}
$$

#### 2.3 Trajectory Cost Calculation

Total cost for each sampled trajectory $k$:

$$
S(\tau^{(k)}) = \sum_{t=0}^{T-1} \left( s(\mathbf{x}_t^{(k)}, \mathbf{u}_t^{(k)}) + \frac{1}{2} \delta \mathbf{u}_t^{(k)T} \mathbf{R} \delta \mathbf{u}_t^{(k)} \right) + \phi(\mathbf{x}_T^{(k)})
$$

Where:
- $s(\cdot)$: Running cost (path tracking error, control cost, etc.)
- $\phi(\cdot)$: Terminal cost
- $\mathbf{R}$: Control regularization weight matrix

#### 2.4 Softmax Weight Calculation

Calculate the weight for each trajectory using temperature parameter $\lambda$:

$$
w^{(k)} = \frac{\exp\left(-\frac{1}{\lambda} S(\tau^{(k)})\right)}{\sum_{j=1}^{K} \exp\left(-\frac{1}{\lambda} S(\tau^{(j)})\right)}
$$

Where $K$ is the number of sampled trajectories.

**Role of Temperature Parameter $\lambda$**:
- $\lambda \to 0$: Greedy selection, only choose the lowest cost trajectory
- $\lambda \to \infty$: Uniform averaging, all trajectories have equal weight

#### 2.5 Control Sequence Update

Update control sequence using weighted averaging:

$$
u_t^{new} = \sum_{k=1}^{K} w^{(k)} \cdot \nu_t^{(k)}
$$

### 3. Critic Functions in Detail

#### 3.1 PathAlignCritic (Path Alignment Cost)

Measures the deviation between the trajectory and the reference path:

$$
J_{align} = \frac{w_{align}}{N} \sum_{i=0}^{N-1} \min_{p \in \mathcal{P}} \|\mathbf{x}_i - \mathbf{p}\|
$$

Where $\mathcal{P}$ is the reference path point set, $N$ is the number of sample points.

#### 3.2 PathFollowCritic (Path Following Cost)

Penalizes the distance between the trajectory endpoint and the target path point:

$$
J_{follow} = w_{follow} \cdot \|\mathbf{x}_T - \mathbf{x}_{target}\|
$$

#### 3.3 PathAngleCritic (Heading Angle Cost)

Penalizes the deviation between the trajectory endpoint heading and the target heading:

$$
J_{angle} = w_{angle} \cdot |\text{normalize}(\theta_T - \theta_{target})|
$$

Angle normalization function:

$$
\text{normalize}(\Delta\theta) = \text{atan2}(\sin(\Delta\theta), \cos(\Delta\theta))
$$

#### 3.4 GoalCritic (Goal Point Cost)

Activates when approaching the goal, penalizing distance to the goal:

$$
J_{goal} = \begin{cases}
w_{goal} \cdot \|\mathbf{x}_T - \mathbf{x}_{goal}\|^2 & \text{if } \|\mathbf{x} - \mathbf{x}_{goal}\| < d_{threshold} \\
0 & \text{otherwise}
\end{cases}
$$

#### 3.5 PreferForwardCritic (Forward Preference)

Penalizes backward motion:

$$
J_{forward} = w_{forward} \cdot \sum_{t=0}^{T-1} \max(0, -v_{x,t})
$$

#### 3.6 ConstraintCritic (Smoothness Constraint)

Penalizes the square of angular velocity to encourage smooth steering:

$$
J_{constraint} = \frac{w_{constraint}}{T} \sum_{t=0}^{T-1} \omega_t^2
$$

### 4. Acceleration Constraints

Apply acceleration limits to the control sequence:

$$
\Delta v_{max} = a_{max} \cdot \Delta t
$$

$$
v_t = \text{clamp}(v_t^{desired}, v_{t-1} - \Delta v_{max}, v_{t-1} + \Delta v_{max})
$$

---

## 🛡️ Independent Obstacle Handling Mechanism

### 1. Design Philosophy

Traditional MPPI integrates obstacle costs into the sampling optimization process, which has the following issues:
- Requires a large number of samples to find feasible trajectories
- Obstacle cost is coupled with tracking cost, making parameter tuning difficult
- Avoidance behavior is unpredictable

**OptiIO-MPPI's Solution**:
- Independently evaluate obstacles before MPPI optimization
- Determine control strategy based on evaluation results
- Generate detour paths for MPPI tracking when necessary

### 2. Decision Flow

```
                    ┌─────────────────┐
                    │ Obstacle Eval   │
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
        ┌─────────┐    ┌─────────┐    ┌─────────┐
        │ Static  │    │ Dynamic │    │  None   │
        └────┬────┘    └────┬────┘    └────┬────┘
             │              │              │
             ▼              ▼              ▼
    ┌─────────────────────────────────────────────┐
    │         Decision Priority (High to Low)     │
    │                                             │
    │  1. EMERGENCY_BRAKE                         │
    │     Condition: min_dist < early_avoid_dist  │
    │                                             │
    │  2. NORMAL_STOP                             │
    │     Condition: Cannot avoid, obstacle in    │
    │              safety distance                │
    │                                             │
    │  3. LATERAL_AVOID                           │
    │     Condition: Feasible detour path exists  │
    │                                             │
    │  4. FOLLOW                                  │
    │     Condition: Dynamic obstacle on path     │
    │                                             │
    │  5. NONE (Normal tracking)                  │
    │     Condition: No obstacle threat           │
    └─────────────────────────────────────────────┘
```

### 3. Static Obstacle Handling

#### 3.1 Distance Field Grid

Build a local distance field grid for fast distance queries:

```cpp
// Grid parameters
grid_resolution = 0.05m  // 5cm per cell
grid_width = 100         // Grid width
grid_height = 100        // Grid height

// Coverage: 5m x 5m local area
```

Use BFS (Breadth-First Search) to compute the distance field:

$$
D(i, j) = \min_{(i', j') \in \mathcal{O}} \sqrt{(i-i')^2 + (j-j')^2} \cdot r_{resolution}
$$

Where $\mathcal{O}$ is the set of obstacle grid cells.

#### 3.2 Lateral Avoidance Detection

Calculate the lateral offset of the obstacle relative to the path:

$$
\mathbf{n} = \frac{[-\Delta y, \Delta x]^T}{\|\mathbf{p}_{i+1} - \mathbf{p}_i\|}
$$

$$
d_{lateral} = (\mathbf{x}_{obs} - \mathbf{p}_i) \cdot \mathbf{n}
$$

Avoidance feasibility check:

```cpp
// Check if the entire avoidance path segment is safe
for (i = start_idx; i <= end_idx; i++) {
    check_x = path.x(i) + lateral_offset * normal_x
    check_y = path.y(i) + lateral_offset * normal_y
    
    if (grid_distance(check_x, check_y) < safety_margin)
        return false;  // Not feasible
}
return true;  // Feasible
```

#### 3.3 Smooth Avoidance Path Generation

Use sine function to generate smooth lateral offset:

$$
\text{offset}(t) = d_{avoid} \cdot \sin\left(\pi \cdot \frac{t - t_{start}}{t_{end} - t_{start}}\right)
$$

Where $t \in [t_{start}, t_{end}]$.

New path point calculation:

$$
\mathbf{p}'_i = \mathbf{p}_i + \text{offset}(t) \cdot \mathbf{n}_i
$$

### 4. Dynamic Obstacle Handling

#### 4.1 Time to Collision (TTC) Calculation

$$
\mathbf{v}_{rel} = \mathbf{v}_{obs} - \mathbf{v}_{robot}
$$

$$
v_{closing} = -\frac{\mathbf{d} \cdot \mathbf{v}_{rel}}{\|\mathbf{d}\|}
$$

$$
TTC = \frac{\|\mathbf{d}\| - r_{obs} - r_{robot}}{v_{closing}}
$$

Where $\mathbf{d} = \mathbf{x}_{obs} - \mathbf{x}_{robot}$ is the relative position vector.

#### 4.2 Dynamic Obstacle Prediction

Predict obstacle position at future time:

$$
\mathbf{x}_{obs}(t) = \mathbf{x}_{obs}(0) + \mathbf{v}_{obs} \cdot t
$$

Estimated time for robot to reach path point:

$$
t_{arrival} = \frac{\sum_{i=start}^{idx} \|\mathbf{p}_{i+1} - \mathbf{p}_i\|}{v_{current}}
$$

#### 4.3 Following Speed Calculation

When following a dynamic obstacle:

$$
v_{follow} = \begin{cases}
v_{obs} & \text{if } d \leq d_{safe} \\
v_{current} - \frac{(v_{current} - v_{obs})^2}{2(d - d_{safe})} \cdot \Delta t & \text{otherwise}
\end{cases}
$$

### 5. Comparison with Nav2 Obstacle Handling

| Feature | Nav2 MPPI | OptiIO-MPPI (This Project) |
|---------|-----------|-------------------------|
| **Architecture** | Obstacle cost integrated in MPPI sampling | Independent obstacle handler |
| **Avoidance Decision** | Implicit (through cost weights) | Explicit (state machine) |
| **Detour Path** | Relies on sampling chance | Actively generates detour path |
| **Static/Dynamic Separation** | Unified handling | Separate handling strategies |
| **Predictability** | Low (depends on random sampling) | High (deterministic decisions) |
| **Parameter Tuning** | Difficult (multiple costs coupled) | Simple (independent modules) |
| **Computational Efficiency** | Requires more samples | Fixed computational overhead |
| **Emergency Brake** | Depends on cost threshold | Explicit TTC judgment |

---

## 📦 Installation and Dependencies

### System Requirements

- Ubuntu 22.04+ (recommended)
- ROS2 Humble / Iron / Jazzy
- C++17 compiler

### Dependency Installation

```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-tf2-ros

# Install xtensor (version 0.21.0+)
sudo apt install -y xtensor-dev xtl-dev

# Or install from source
# git clone https://github.com/xtensor-stack/xtensor.git
# cd xtensor && mkdir build && cd build
# cmake .. && make -j$(nproc) && sudo make install

# Install math libraries
sudo apt install -y libopenblas-dev liblapack-dev
```

### Build

```bash
# Create workspace
mkdir -p ~/optiiomppi_ws/src
cd ~/optiiomppi_ws/src

# Clone repository
git clone https://github.com/yourusername/OptiIO-MPPI.git

# Build
cd ~/optiiomppi_ws
colcon build --packages-select optiio_mppi

# Source environment
source install/setup.bash
```

---

## 🚀 Quick Start

### 1. Launch Simulation Environment

```bash
# Terminal 1: Launch Gazebo
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### 2. Launch Static TF

```bash
# Terminal 2: Publish static transform from map to odom
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

### 3. Launch OptiIO-MPPI Controller

```bash
# Terminal 3: Launch MPPI node
cd ~/optiiomppi_ws
source install/setup.bash
ros2 run optiio_mppi mppi_ros2_node --ros-args --params-file ~/optiiomppi_ws/src/OptiIO-MPPI/config/mppi_params.yaml
```

### 4. Launch RViz2

```bash
# Terminal 4: Visualization
rviz2 -d ~/optiiomppi_ws/src/OptiIO-MPPI/config/mppi_test.rviz
```

### 5. Publish Path

```bash
# Terminal 5: Publish test path
cd ~/optiiomppi_ws
source install/setup.bash
ros2 run optiio_mppi mppi_path_publisher.py --ros-args -p path_type:=three_side_rectangle

# Optional path types: straight, three_side_rectangle
```

---

## ⚙️ Parameter Configuration

### Core MPPI Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `batch_size` | int | 1300 | Number of sampled trajectories |
| `time_steps` | int | 90 | Prediction horizon steps |
| `model_dt` | float | 0.05 | Time step (seconds) |
| `temperature` | float | 0.3 | Softmax temperature parameter |
| `iteration_count` | int | 1 | Optimization iteration count |
| `thread_count` | int | 4 | Number of parallel threads |

### Velocity Constraint Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `vx_max` | float | 0.3 | Maximum linear velocity (m/s) |
| `vx_min` | float | 0.0 | Minimum linear velocity (m/s) |
| `wz_max` | float | 3.0 | Maximum angular velocity (rad/s) |

### Acceleration Constraint Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ax_max` | float | 0.15 | Maximum linear acceleration (m/s²) |
| `ay_max` | float | 2.0 | Maximum lateral acceleration (m/s²) |
| `az_max` | float | 6.0 | Maximum angular acceleration (rad/s²) |

### Independent Obstacle Avoidance Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_avoidance` | bool | true | Enable obstacle avoidance |
| `max_lateral_avoidance_distance` | float | 0.5 | Maximum lateral avoidance distance (m) |
| `safety_margin_lateral` | float | 0.2 | Lateral safety margin (m) |
| `safety_margin_longitudinal` | float | 0.5 | Longitudinal safety distance (m) |
| `robot_radius` | float | 0.08 | Robot radius (m) |
| `ttc_emergency_threshold` | float | 0.2 | TTC emergency threshold (seconds) |
| `follow_distance` | float | 1.0 | Following distance (m) |
| `avoidance_lookahead` | float | 3.0 | Avoidance lookahead distance (m) |
| `avoidance_smooth_distance` | float | 0.5 | Avoidance smoothing distance (m) |
| `avoid_speed_ratio` | float | 0.7 | Avoidance speed ratio |

### Critic Function Weight Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `path_align_weight` | float | 10.0 | Path alignment weight |
| `path_follow_weight` | float | 6.0 | Path following weight |
| `path_angle_weight` | float | 2.0 | Heading angle weight |
| `goal_weight` | float | 5.0 | Goal point weight |
| `goal_angle_weight` | float | 3.0 | Goal orientation weight |
| `prefer_forward_weight` | float | 5.0 | Forward preference weight |
| `constraint_weight` | float | 4.0 | Smoothness constraint weight |

---

## 📚 API Documentation

### MPPIController Class

```cpp
namespace mppi {

class MPPIController {
public:
    // Initialization
    void initialize(const OptimizerSettings& settings,
                    const std::string& motion_model_type = "DiffDrive",
                    float ackermann_min_turning_radius = 0.2f);
    
    // Set path
    void setPath(const std::vector<Pose2D>& path);
    
    // Update obstacles
    void updateStaticObstacles(const std::vector<Point2D>& points, 
                               const Pose2D& robot_pose);
    void updateDynamicObstacles(const std::vector<DynamicObstacle>& obstacles);
    
    // Compute velocity commands
    Twist2D computeVelocityCommands(const Pose2D& robot_pose, 
                                    const Twist2D& robot_speed);
    
    // Set avoidance configuration
    void setAvoidanceConfig(const AvoidanceConfig& config);
    
    // Get generated trajectory
    xt::xtensor<float, 2> getOptimizedTrajectory();
};

} // namespace mppi
```

### Data Structures

```cpp
// 2D Pose
struct Pose2D {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
};

// 2D Twist
struct Twist2D {
    float vx = 0.0f;  // Linear velocity x
    float vy = 0.0f;  // Linear velocity y (omni)
    float wz = 0.0f;  // Angular velocity z
};

// Dynamic obstacle
struct DynamicObstacle {
    float x, y;       // Position
    float vx, vy;     // Velocity
    float radius;     // Radius
    bool is_moving;   // Is moving
};

// Avoidance configuration
struct AvoidanceConfig {
    bool enable_avoidance = true;
    float max_lateral_avoidance_distance = 0.5f;
    float safety_margin_lateral = 0.2f;
    float safety_margin_longitudinal = 0.5f;
    float robot_radius = 0.3f;
    float ttc_emergency_threshold = 1.0f;
    float follow_distance = 1.0f;
    float avoidance_lookahead = 3.0f;
    float avoidance_smooth_distance = 0.5f;
    float avoid_speed_ratio = 0.7f;
    float min_avoid_speed = 0.08f;
};
```

---

## 📄 License

This project is licensed under the **GNU General Public License v3.0 (GPL-3.0)**.

```
OptiIO-MPPI - Optimized Independent Obstacle Model Predictive Path Integral Controller
Copyright (C) 2024  [Your Name]

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
```

---

## 🙏 Acknowledgments

- [xtensor](https://github.com/xtensor-stack/xtensor) - C++ multidimensional array library
- [Nav2](https://github.com/ros-planning/navigation2) - ROS2 navigation framework reference
- [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3) - Robot simulation platform

---

## 📧 Contact

For questions or suggestions, please submit an Issue or Pull Request.

---

**OptiIO-MPPI** - Making mobile robot navigation smarter and more reliable!
