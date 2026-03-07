# OptiMPPI

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%2FIron%2FJazzy-green.svg)](https://docs.ros.org/)
[![C++17](https://img.shields.io/badge/C++-17-blue.svg)](https://en.cppreference.com/)
[![Platform](https://img.shields.io/badge/Platform-Cross--Platform-orange.svg)]()

A **pure C++**, high-performance, industrial-grade **Model Predictive Path Integral (MPPI)** controller for mobile robots with laser-based obstacle avoidance. Designed for **cross-platform portability** and **direct industrial deployment**.

> **Key Advantage**: The core MPPI controller (`mppi_controller.hpp`) is implemented in **pure C++17** with **zero ROS2 dependencies**, making it instantly portable to any embedded platform, real-time OS, or robotics framework (ROS1, ROS2, custom middleware).

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Mathematical Foundation](#mathematical-foundation)
- [Architecture](#architecture)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Topics and Interfaces](#topics-and-interfaces)
- [Algorithm Details](#algorithm-details)
- [Performance Optimization](#performance-optimization)
- [License](#license)

---

## Overview

OptiMPPI is a **pure C++ implementation** of the MPPI control algorithm optimized for mobile robot navigation. The core controller is **completely decoupled from ROS2**, requiring only standard C++17 and the header-only `xtensor` library for numerical computations. This design enables:

- **Universal Portability**: Deploy on any platform with a C++17 compiler (ARM, x86, embedded Linux, real-time OS)
- **Industrial Integration**: Direct integration into existing C++ codebases without ROS dependencies
- **Real-time Performance**: Zero overhead from middleware - pure optimized C++ execution
- **Framework Agnostic**: Use with ROS1, ROS2, or any custom robotics framework

The provided ROS2 node (`mppi_ros2_node.cpp`) serves as a **reference implementation** and **usage example**, demonstrating how to wrap the core controller for ROS2 deployment. It uses laser scan data for real-time obstacle detection and avoidance while following a global path. The controller is designed for differential drive robots but supports omnidirectional and Ackermann steering models.

test as follows:

### Key Capabilities

- **Real-time Trajectory Optimization**: Optimizes control sequences at 20Hz (50ms control period)
- **Laser-based Obstacle Avoidance**: Uses grid-based distance field for efficient collision detection
- **Multi-threaded Sampling**: Parallel trajectory generation using configurable thread pools
- **Dynamic Obstacle Support**: Predictive collision avoidance for moving obstacles
- **Robust Blocking Detection**: Prevents spinning behavior when blocked by obstacles
- **Multiple Motion Models**: Supports DiffDrive, Omni, and Ackermann kinematics

---

## Features

### Core Features

| Feature | Description |
|---------|-------------|
| **Batch Sampling** | 1000+ trajectory samples per control cycle |
| **Grid Distance Field** | BFS-based local distance field for fast obstacle queries |
| **Multi-threading** | Configurable thread count for trajectory integration |
| **Adaptive Temperature** | Dynamic temperature adjustment based on cost distribution |
| **SG Filtering** | Savitzky-Golay filter for smooth control sequences |
| **Footprint Support** | Non-circular robot collision detection |
| **Pure C++ Core** | Zero ROS dependencies - portable to any platform |
| **Header-Only Library** | Single header (`mppi_controller.hpp`) for easy integration |
| **Industrial Ready** | Direct deployment on embedded systems and real-time OS |

### Cross-Platform Portability

The core MPPI controller is designed for **maximum portability**:

| Platform | Support | Notes |
|----------|---------|-------|
| **Linux (x86/ARM)** | ✅ Full | Primary development platform |
| **ROS2** | ✅ Full | Reference implementation provided |
| **ROS1** | ✅ Compatible | Wrap with ROS1 message types |
| **Embedded Linux** | ✅ Full | Tested on ARM Cortex-A series |
| **Real-time OS** | ✅ Compatible | No dynamic memory allocation in hot path |
| **Bare Metal** | ⚠️ Possible | Requires xtensor porting |
| **Windows** | ✅ Compatible | C++17 + xtensor supported |

**Dependencies (Core Controller Only)**:
- C++17 compiler
- [xtensor](https://github.com/xtensor-stack/xtensor) (header-only numerical library)
- Standard C++ library only

**NO ROS, NO Boost, NO external middleware required for the core!**

### Critic Functions (Cost Components)

| Critic | Purpose |
|--------|---------|
| `ObstaclesCritic` | Collision avoidance with exponential repulsion cost |
| `PathFollowCritic` | Endpoint tracking along the global path |
| `PathAlignCritic` | Trajectory alignment with path geometry |
| `PathAngleCritic` | Heading alignment with path direction |
| `GoalCritic` | Final position convergence |
| `GoalAngleCritic` | Final heading alignment |
| `PreferForwardCritic` | Penalize backward motion |
| `ConstraintCritic` | Kinematic constraint enforcement |
| `TwirlingCritic` | Anti-spinning behavior suppression |
| `VelocityDeadbandCritic` | Low-speed stability improvement |

---

## Mathematical Foundation

### 1. MPPI Core Algorithm

The MPPI algorithm solves the stochastic optimal control problem by sampling trajectories and computing a weighted average of control sequences.

#### State Dynamics (Differential Drive)

$$
\begin{aligned}
\dot{x} &= v_x \cos(\theta) \\
\dot{y} &= v_x \sin(\theta) \\
\dot{\theta} &= \omega_z
\end{aligned}
$$

Where:
- $(x, y)$: Robot position in world frame
- $\theta$: Robot heading
- $v_x$: Linear velocity
- $\omega_z$: Angular velocity

#### Trajectory Sampling

For each batch $i$ and time step $t$:

$$
u_t^{(i)} = \bar{\nu}_t + \epsilon_t^{(i)}, \quad \epsilon_t^{(i)} \sim \mathcal{N}(0, \Sigma)
$$

Where:
- $\bar{\nu}_t$: Mean control from previous iteration
- $\epsilon_t^{(i)}$: Gaussian noise with standard deviation $\sigma$

#### Cost Function

The total cost for trajectory $i$:

$$
S(\tau^{(i)}) = \sum_{t=0}^{T-1} \left[ s(x_t^{(i)}, \nu_t^{(i)}) + \frac{\gamma}{2} \bar{\nu}_t^T \Sigma^{-1} \nu_t^{(i)} \right]
$$

Where:
- $s(x, \nu)$: State-dependent running cost
- $\gamma$: Control cost weight parameter
- $T$: Time horizon (number of steps)

#### Softmax Weighting

The optimal control update uses softmax weights:

$$
w^{(i)} = \frac{\exp\left(-\frac{1}{\lambda}(S(\tau^{(i)}) - S_{\min})\right)}{\sum_{j=1}^{K} \exp\left(-\frac{1}{\lambda}(S(\tau^{(j)}) - S_{\min})\right)}
$$

Where:
- $\lambda$: Temperature parameter
- $S_{\min}$: Minimum cost across all trajectories
- $K$: Batch size (number of samples)

#### Control Update

The updated control sequence:

$$
\bar{\nu}_t^{\text{new}} = \sum_{i=1}^{K} w^{(i)} \nu_t^{(i)}
$$

### 2. Obstacle Cost Function

The obstacle critic uses an exponential repulsion cost:

$$
C_{\text{obs}}(d) = \begin{cases}
C_{\text{collision}} & \text{if } d < d_{\text{margin}} \\
w_{\text{repulsion}} \cdot \exp(-\alpha \cdot (d - d_{\text{margin}})) & \text{if } d_{\text{margin}} \leq d < r_{\text{inflation}} \\
0 & \text{otherwise}
\end{cases}
$$

Where:
- $d$: Distance to nearest obstacle
- $d_{\text{margin}}$: Collision margin distance
- $r_{\text{inflation}}$: Inflation radius
- $\alpha$: Cost scaling factor
- $w_{\text{repulsion}}$: Repulsion weight

### 3. Grid Distance Field

The local distance field is computed using BFS (Breadth-First Search):

```
For each grid cell (i, j):
    D[i,j] = min_{obstacle cells} ||(i,j) - (i_obs, j_obs)||
```

Grid resolution and size are configurable (default: 0.05m resolution, 100×100 grid).

### 4. Acceleration Constraints

Velocity updates respect acceleration limits:

$$
v_{t+1} = v_t + \text{clamp}(v_{\text{desired}} - v_t, -a_{\max} \Delta t, a_{\max} \Delta t)
$$

Where:
- $a_{\max}$: Maximum acceleration
- $\Delta t$: Time step duration

### 5. Blocking Detection

The controller detects blocking situations using the spinning ratio:

$$
\text{ratio} = \frac{|\omega_z|}{|v_x| + \epsilon}
$$

If $\text{ratio} > \theta_{\text{threshold}}$ for $N$ consecutive frames, the robot is considered blocked and zero velocity is commanded.

### 6. Savitzky-Golay Filtering

Control sequence smoothing uses SG filter coefficients:

$$
\nu_t^{\text{filtered}} = c_0 \nu_t + c_1 (\nu_{t-1} + \nu_{t+1}) + c_2 (\nu_{t-2} + \nu_{t+2})
$$

Default coefficients: $c_0 = 0.2$, $c_1 = 0.2$, $c_2 = 0.2$

---

## Architecture

### Package Structure

```
opti_mppi/                          # ROS2 package directory (create with ros2 pkg create)
├── include/
│   └── mppi_controller.hpp          # Core MPPI controller (PURE C++, ROS-agnostic)
├── src/
│   └── mppi_ros2_node.cpp           # ROS2 wrapper (reference implementation)
├── scripts/
│   ├── mppi_path_publisher.py       # Path publisher with multiple path types
│   └── mppi_path_publisher_short.py # Short path variant
├── config/
│   ├── mppi_params.yaml             # Parameter configuration
│   └── mppi_test.rviz               # RViz configuration
├── CMakeLists.txt                   # Build configuration
└── package.xml                      # ROS2 package manifest
```

### Architecture Design

OptiMPPI follows a **layered architecture** that separates the core algorithm from framework-specific implementations:

```
┌─────────────────────────────────────────────────────────────┐
│                    APPLICATION LAYER                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │   ROS2 Node  │  │   ROS1 Node  │  │  Custom Middleware│  │
│  │ (reference)  │  │  (possible)  │  │    (possible)     │  │
│  └──────┬───────┘  └──────┬───────┘  └────────┬─────────┘  │
└─────────┼─────────────────┼───────────────────┼────────────┘
          │                 │                   │
          └─────────────────┴───────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────┐
│                 CORE CONTROLLER LAYER                       │
│                    (PURE C++17)                             │
│                                                             │
│   ┌─────────────────────────────────────────────────────┐  │
│   │              MPPIController (Header-Only)           │  │
│   │  ┌──────────────┐  ┌─────────────────────────────┐  │  │
│   │  │   Optimizer  │  │      CriticManager          │  │  │
│   │  │  ┌────────┐  │  │  ┌─────────────────────┐    │  │  │
│   │  │  │ Noise  │  │  │  │  ObstaclesCritic    │    │  │  │
│   │  │  │Generator│  │  │  │  PathFollowCritic   │    │  │  │
│   │  │  └────────┘  │  │  │  PathAlignCritic    │    │  │  │
│   │  │  ┌────────┐  │  │  │  PathAngleCritic    │    │  │  │
│   │  │  │ Motion │  │  │  │  GoalCritic         │    │  │  │
│   │  │  │ Models │  │  │  │  GoalAngleCritic    │    │  │  │
│   │  │  │(Diff/  │  │  │  │  PreferForwardCritic│    │  │  │
│   │  │  │ Omni/  │  │  │  │  ConstraintCritic   │    │  │  │
│   │  │  │Ackerm.)│  │  │  │  TwirlingCritic     │    │  │  │
│   │  │  └────────┘  │  │  │  VelocityDeadband   │    │  │  │
│   │  └──────────────┘  │  └─────────────────────┘    │  │  │
│   └─────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────────┐
│              EXTERNAL DEPENDENCIES (Minimal)                │
│     xtensor (header-only)  │  C++17 Standard Library       │
└─────────────────────────────────────────────────────────────┘
```

### Key Design Principles

1. **Zero ROS Dependencies in Core**: The entire `mppi_controller.hpp` uses only standard C++17 and xtensor
2. **Header-Only Library**: Single-file inclusion for easy integration into existing projects
3. **Template-Based Design**: Easy to customize for different state/control types
4. **Callback-Based Interfaces**: Sensor data input through callbacks, not ROS messages
5. **Deterministic Execution**: No non-deterministic operations in the control loop

---

## Installation

### Prerequisites

- ROS2 foxy/Humble/Iron/Jazzy
- C++17 compatible compiler
- xtensor library (≥0.21.0)
- xtl library
- BLAS/LAPACK libraries

### Dependencies Installation

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y \
    libopenblas-dev \
    liblapack-dev \
    libxtensor-dev \
    libxtl-dev
```

### Build from Source

Since this repository contains only the package files (not the entire ROS2 workspace folder), follow these steps to set up the package:

```bash
# 1. Navigate to your ROS2 workspace src directory
cd ~/ros2_ws/src

# 2. Create a new ROS2 package
ros2 pkg create --build-type ament_cmake --license GPL-3.0 --description "MPPI controller for ROS2 mobile robots with laser-based obstacle avoidance" opti_mppi

# 3. Navigate into the package directory
cd opti_mppi

# 4. Clone the repository contents (files will be in current directory)
# Download/clone files from GitHub into this directory
# The files should include: CMakeLists.txt, package.xml, include/, src/, scripts/, config/

# 5. Make Python scripts executable
chmod +x scripts/*.py

# 6. Build the package
cd ~/ros2_ws
colcon build --packages-select opti_mppi --symlink-install

# 7. Source the workspace
source install/setup.bash
```

### File Structure After Setup

Your package structure should look like this:

```
~/ros2_ws/src/opti_mppi/
├── include/
│   └── mppi_controller.hpp          # PURE C++ core - portable to any platform
├── src/
│   └── mppi_ros2_node.cpp           # ROS2 wrapper example
├── scripts/
│   ├── mppi_path_publisher.py
│   └── mppi_path_publisher_short.py
├── config/
│   ├── mppi_params.yaml
│   └── mppi_test.rviz
├── CMakeLists.txt
└── package.xml
```

---

## Usage

### Launch the Controller

```bash
# Terminal 1: Launch the MPPI controller
ros2 run opti_mppi mppi_ros2_node --ros-args --params-file src/opti_mppi/config/mppi_params.yaml

# Terminal 2: Publish a global path
ros2 run opti_mppi mppi_path_publisher.py

# Terminal 3: Visualize in RViz
ros2 run rviz2 rviz2 -d src/opti_mppi/config/mppi_test.rviz
```

### Path Publisher Commands

When running `mppi_path_publisher.py`, use these keyboard commands:
- `s` - Switch to straight path (25m)
- `t` - Switch to three-side rectangle path (70m total)
- `u` - Switch to custom S-shaped path (50m)
- `p` - Print path information

---

## Configuration

### Key Parameters

#### MPPI Core Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `batch_size` | 1300 | Number of trajectory samples |
| `time_steps` | 90 | Prediction horizon length |
| `model_dt` | 0.05 | Time step duration (seconds) |
| `temperature` | 0.3 | Softmax temperature |
| `gamma` | 0.015 | Control cost weight |
| `iteration_count` | 1 | Optimization iterations per cycle |
| `thread_count` | 4 | Parallel trajectory threads |

#### Control Constraints

| Parameter | Default | Description |
|-----------|---------|-------------|
| `vx_max` | 0.3 | Maximum linear velocity (m/s) |
| `vx_min` | 0.0 | Minimum linear velocity (m/s) |
| `wz_max` | 2.2 | Maximum angular velocity (rad/s) |
| `ax_max` | 1.6 | Maximum linear acceleration (m/s²) |
| `az_max` | 3.2 | Maximum angular acceleration (rad/s²) |

#### Obstacle Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `obstacle_repulsion_weight` | 0.2 | Obstacle repulsion cost weight |
| `obstacle_collision_cost` | 10000.0 | Cost for collision trajectories |
| `obstacle_collision_margin` | 0.1 | Minimum safe distance (m) |
| `obstacle_inflation_radius` | 0.2 | Obstacle inflation radius (m) |
| `robot_radius` | 0.08 | Robot radius (m) |
| `grid_resolution` | 0.05 | Distance field resolution (m) |

See `config/mppi_params.yaml` for complete parameter list.

---

## Topics and Interfaces

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/plan` | `nav_msgs/Path` | Global path to follow |
| `/odom` | `nav_msgs/Odometry` | Robot pose and velocity |
| `/scan` | `sensor_msgs/LaserScan` | Laser scan for obstacle detection |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Output velocity command |
| `/debug_optimal_trajectory` | `nav_msgs/Path` | Visualized optimal trajectory |

---

## Algorithm Details

### Trajectory Generation Pipeline

1. **Noise Generation**: Asynchronous generation of Gaussian noise matrices
2. **Control Sampling**: Add noise to current control sequence
3. **Constraint Application**: Apply velocity and acceleration constraints
4. **Motion Prediction**: Integrate velocities using kinematic model
5. **Cost Evaluation**: Score all trajectories using critic functions
6. **Weight Computation**: Calculate softmax weights based on costs
7. **Control Update**: Compute weighted average of sampled controls
8. **Smoothing**: Apply SG filter for control sequence smoothing

### Blocking Detection Algorithm

```
if (avg_abs_wz < 0.3):
    not spinning (normal motion)
else if (|wz|/|vx| > threshold):
    spinning_counter++
    if (spinning_counter >= detect_frames):
        BLOCKED → output zero velocity
else:
    reset spinning counter
```

### Grid Distance Field Construction

1. Initialize grid with infinite distance values
2. Mark obstacle cells with distance 0
3. BFS propagation to fill distance values
4. Query distance using bilinear interpolation

---

## Performance Optimization

### Multi-threading

The controller uses parallel trajectory integration:
- Trajectories are divided among worker threads
- Each thread integrates a subset of trajectories
- Thread count is configurable (default: 4)

### Memory Management

- Pre-allocated tensors for state and trajectory storage
- `xt::noalias` for optimized memory operations
- Local variables in worker threads to avoid cache contention

### Computational Complexity

| Operation | Complexity |
|-----------|------------|
| Trajectory sampling | $O(K \cdot T)$ |
| Cost evaluation | $O(K \cdot T \cdot C)$ |
| Control update | $O(K \cdot T)$ |
| Grid distance field | $O(W \cdot H)$ |

Where:
- $K$: Batch size
- $T$: Time steps
- $C$: Number of critics
- $W, H$: Grid width and height

---

## License

This project is licensed under the **GNU General Public License v3.0 (GPL-3.0)**.

```
OptiMPPI - Cross-Platform MPPI Controller for Mobile Robots
Copyright (C) 2024

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

## Acknowledgments

- Inspired by the [Nav2 MPPI Controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller)
- Uses [xtensor](https://github.com/xtensor-stack/xtensor) for high-performance array operations
- Built for ROS2 navigation stack

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Contact

For questions or issues, please open an issue on the GitHub repository.
