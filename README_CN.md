# OptiIO-MPPI (Optimized Independent Obstacle-MPPI)

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%2FIron%2FJazzy-green.svg)](https://docs.ros.org/)
[![C++17](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)

**OptiIO-MPPI** 是一个高性能的ROS2 MPPI（Model Predictive Path Integral）控制器实现，专为移动机器人路径跟踪和实时避障设计。本项目的核心创新在于**独立的障碍物处理机制（Independent Obstacle Handling）**，将避障逻辑与MPPI优化过程解耦，提供更可靠、更可预测的避障行为。

**缩写含义**：
- **Opti** = Optimized（优化的）
- **IO** = Independent Obstacle（独立障碍物处理）
- **MPPI** = Model Predictive Path Integral（模型预测路径积分）

---

## 📋 目录

- [核心特性](#核心特性)
- [系统架构](#系统架构)
- [MPPI算法原理与公式](#mppi算法原理与公式)
- [独立障碍物处理机制](#独立障碍物处理机制)
- [与Nav2的对比](#与nav2的对比)
- [安装与依赖](#安装与依赖)
- [快速开始](#快速开始)
- [参数配置](#参数配置)
- [API文档](#api文档)
- [许可证](#许可证)

---

## 🚀 核心特性

### 1. 独立障碍物处理机制
- **分离式设计**：避障逻辑独立于MPPI采样优化过程
- **多层决策**：紧急制动 → 停车 → 跟随 → 横向绕行
- **静态/动态障碍物分离处理**：针对不同类型障碍物采用不同策略
- **距离场栅格**：高效的障碍物距离计算

### 2. 高性能MPPI核心
- **并行轨迹积分**：多线程加速（可配置线程数）
- **xtensor加速**：使用xtensor库进行高效的矩阵运算
- **软最大权重更新**：温度调节的轨迹加权平均
- **多种运动模型支持**：差速驱动、全向移动、阿克曼转向

### 3. 完善的代价函数系统
- **PathAlignCritic**：轨迹与路径对齐度
- **PathFollowCritic**：轨迹终点跟踪
- **PathAngleCritic**：航向角一致性
- **GoalCritic / GoalAngleCritic**：目标点及朝向
- **PreferForwardCritic**：偏好前进（惩罚倒车）
- **ConstraintCritic**：平滑性约束

### 4. 工业级特性
- **加速度约束**：线加速度、角加速度限制
- **控制序列平滑**：控制量变化率限制
- **故障恢复机制**：优化失败自动重置
- **实时性能**：50ms控制周期支持

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                      OptiIO-MPPI 架构图                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐      │
│  │   传感器输入  │    │    路径输入   │    │   里程计输入  │      │
│  │  LaserScan   │    │     Path     │    │   Odometry   │      │
│  └──────┬───────┘    └──────┬───────┘    └──────┬───────┘      │
│         │                   │                   │              │
│         ▼                   ▼                   ▼              │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              MPPIRos2Node (ROS2接口层)                   │   │
│  │  ┌─────────────────────────────────────────────────────┐│   │
│  │  │           独立障碍物处理器 (ObstacleHandler)         ││   │
│  │  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ││   │
│  │  │  │ 静态障碍物评估 │  │ 动态障碍物评估 │  │ 避障路径生成 │  ││   │
│  │  │  │   (Grid)    │  │   (TTC)     │  │  (Smooth)   │  ││   │
│  │  │  └─────────────┘  └─────────────┘  └─────────────┘  ││   │
│  │  └─────────────────────────────────────────────────────┘│   │
│  │                         │                               │   │
│  │                         ▼                               │   │
│  │  ┌─────────────────────────────────────────────────────┐│   │
│  │  │              MPPIController (核心控制器)            ││   │
│  │  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ││   │
│  │  │  │   Optimizer │  │CriticManager│  │ MotionModel │  ││   │
│  │  │  │  (采样优化)  │  │  (代价计算)  │  │  (运动模型)  │  ││   │
│  │  │  └─────────────┘  └─────────────┘  └─────────────┘  ││   │
│  │  └─────────────────────────────────────────────────────┘│   │
│  └─────────────────────────────────────────────────────────┘   │
│                              │                                  │
│                              ▼                                  │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              输出: cmd_vel (Twist消息)                  │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📐 MPPI算法原理与公式

### 1. 算法概述

MPPI（Model Predictive Path Integral）是一种基于采样的随机最优控制算法。它通过采样大量候选轨迹，根据代价函数评估每条轨迹，然后使用软最大（softmax）机制加权平均得到最优控制序列。

### 2. 核心数学公式

#### 2.1 轨迹采样

对于每个时间步 $t$ 和控制维度，从正态分布采样噪声：

$$
\delta u_t^{(k)} \sim \mathcal{N}(0, \Sigma)
$$

其中 $\Sigma = \text{diag}(\sigma_{vx}^2, \sigma_{vy}^2, \sigma_{\omega}^2)$ 是控制量的协方差矩阵。

带噪声的控制序列：

$$
u_t^{(k)} = u_t + \delta u_t^{(k)}
$$

#### 2.2 运动学模型（差速驱动）

状态向量 $\mathbf{x} = [x, y, \theta]^T$，控制量 $\mathbf{u} = [v_x, \omega]^T$

$$
\begin{cases}
\dot{x} = v_x \cos(\theta) \\
\dot{y} = v_x \sin(\theta) \\
\dot{\theta} = \omega
\end{cases}
$$

离散化形式（欧拉积分）：

$$
\begin{cases}
x_{t+1} = x_t + v_x \cos(\theta_t) \cdot \Delta t \\
y_{t+1} = y_t + v_x \sin(\theta_t) \cdot \Delta t \\
\theta_{t+1} = \theta_t + \omega \cdot \Delta t
\end{cases}
$$

#### 2.3 轨迹代价计算

每条采样轨迹 $k$ 的总代价：

$$
S(\tau^{(k)}) = \sum_{t=0}^{T-1} \left( s(\mathbf{x}_t^{(k)}, \mathbf{u}_t^{(k)}) + \frac{1}{2} \delta \mathbf{u}_t^{(k)T} \mathbf{R} \delta \mathbf{u}_t^{(k)} \right) + \phi(\mathbf{x}_T^{(k)})
$$

其中：
- $s(\cdot)$：运行代价（路径跟踪误差、控制代价等）
- $\phi(\cdot)$：终端代价
- $\mathbf{R}$：控制正则化权重矩阵

#### 2.4 软最大权重计算

使用温度参数 $\lambda$ 计算每条轨迹的权重：

$$
w^{(k)} = \frac{\exp\left(-\frac{1}{\lambda} S(\tau^{(k)})\right)}{\sum_{j=1}^{K} \exp\left(-\frac{1}{\lambda} S(\tau^{(j)})\right)}
$$

其中 $K$ 是采样轨迹数量。

**温度参数 $\lambda$ 的作用**：
- $\lambda \to 0$：贪婪选择，只选代价最小的轨迹
- $\lambda \to \infty$：均匀平均，所有轨迹权重相同

#### 2.5 控制序列更新

加权平均更新控制序列：

$$
u_t^{new} = \sum_{k=1}^{K} w^{(k)} \cdot \nu_t^{(k)}
$$

### 3. 代价函数详解

#### 3.1 PathAlignCritic（路径对齐代价）

衡量轨迹与参考路径的偏离程度：

$$
J_{align} = \frac{w_{align}}{N} \sum_{i=0}^{N-1} \min_{p \in \mathcal{P}} \|\mathbf{x}_i - \mathbf{p}\|
$$

其中 $\mathcal{P}$ 是参考路径点集，$N$ 是采样点数。

#### 3.2 PathFollowCritic（路径跟踪代价）

惩罚轨迹终点与目标路径点的距离：

$$
J_{follow} = w_{follow} \cdot \|\mathbf{x}_T - \mathbf{x}_{target}\|
$$

#### 3.3 PathAngleCritic（航向角代价）

惩罚轨迹终点航向与目标航向的偏差：

$$
J_{angle} = w_{angle} \cdot |\text{normalize}(\theta_T - \theta_{target})|
$$

角度归一化函数：

$$
\text{normalize}(\Delta\theta) = \text{atan2}(\sin(\Delta\theta), \cos(\Delta\theta))
$$

#### 3.4 GoalCritic（目标点代价）

当接近目标时激活，惩罚与目标的距离：

$$
J_{goal} = \begin{cases}
w_{goal} \cdot \|\mathbf{x}_T - \mathbf{x}_{goal}\|^2 & \text{if } \|\mathbf{x} - \mathbf{x}_{goal}\| < d_{threshold} \\
0 & \text{otherwise}
\end{cases}
$$

#### 3.5 PreferForwardCritic（前进偏好）

惩罚倒车行为：

$$
J_{forward} = w_{forward} \cdot \sum_{t=0}^{T-1} \max(0, -v_{x,t})
$$

#### 3.6 ConstraintCritic（平滑约束）

惩罚角速度的平方，鼓励平滑转向：

$$
J_{constraint} = \frac{w_{constraint}}{T} \sum_{t=0}^{T-1} \omega_t^2
$$

### 4. 加速度约束

对控制序列施加加速度限制：

$$
\Delta v_{max} = a_{max} \cdot \Delta t
$$

$$
v_t = \text{clamp}(v_t^{desired}, v_{t-1} - \Delta v_{max}, v_{t-1} + \Delta v_{max})
$$

---

## 🛡️ 独立障碍物处理机制

### 1. 设计哲学

传统MPPI将障碍物代价集成在采样优化过程中，存在以下问题：
- 需要大量采样才能找到可行轨迹
- 障碍物代价与跟踪代价耦合，参数调优困难
- 避障行为不可预测

**OptiIO-MPPI的解决方案**：
- 在MPPI优化之前独立评估障碍物
- 根据评估结果决定控制策略
- 必要时生成绕行路径供MPPI跟踪

### 2. 决策流程

```
                    ┌─────────────────┐
                    │   障碍物评估    │
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
        ┌─────────┐    ┌─────────┐    ┌─────────┐
        │ 静态障碍 │    │ 动态障碍 │    │ 无障碍  │
        └────┬────┘    └────┬────┘    └────┬────┘
             │              │              │
             ▼              ▼              ▼
    ┌─────────────────────────────────────────────┐
    │              决策优先级（从高到低）           │
    │                                             │
    │  1. EMERGENCY_BRAKE (紧急制动)              │
    │     条件: min_dist < early_avoidance_dist   │
    │                                             │
    │  2. NORMAL_STOP (正常停车)                  │
    │     条件: 无法绕行且障碍物在安全距离内       │
    │                                             │
    │  3. LATERAL_AVOID (横向绕行)                │
    │     条件: 存在可行绕行路径                   │
    │                                             │
    │  4. FOLLOW (跟随)                           │
    │     条件: 动态障碍物在路径上                 │
    │                                             │
    │  5. NONE (正常跟踪)                         │
    │     条件: 无障碍物威胁                       │
    └─────────────────────────────────────────────┘
```

### 3. 静态障碍物处理

#### 3.1 距离场栅格

构建局部距离场栅格用于快速距离查询：

```cpp
// 栅格参数
grid_resolution = 0.05m  // 每个栅格5cm
grid_width = 100         // 栅格宽度
grid_height = 100        // 栅格高度

// 覆盖范围: 5m x 5m 的局部区域
```

使用BFS（广度优先搜索）计算距离场：

$$
D(i, j) = \min_{(i', j') \in \mathcal{O}} \sqrt{(i-i')^2 + (j-j')^2} \cdot r_{resolution}
$$

其中 $\mathcal{O}$ 是障碍物栅格集合。

#### 3.2 横向绕行检测

计算障碍物相对于路径的横向偏移：

$$
\mathbf{n} = \frac{[-\Delta y, \Delta x]^T}{\|\mathbf{p}_{i+1} - \mathbf{p}_i\|}
$$

$$
d_{lateral} = (\mathbf{x}_{obs} - \mathbf{p}_i) \cdot \mathbf{n}
$$

绕行可行性检查：

```cpp
// 检查整段避障路径是否安全
for (i = start_idx; i <= end_idx; i++) {
    check_x = path.x(i) + lateral_offset * normal_x
    check_y = path.y(i) + lateral_offset * normal_y
    
    if (grid_distance(check_x, check_y) < safety_margin)
        return false;  // 不可行
}
return true;  // 可行
```

#### 3.3 平滑绕行路径生成

使用正弦函数生成平滑的横向偏移：

$$
\text{offset}(t) = d_{avoid} \cdot \sin\left(\pi \cdot \frac{t - t_{start}}{t_{end} - t_{start}}\right)
$$

其中 $t \in [t_{start}, t_{end}]$。

新路径点计算：

$$
\mathbf{p}'_i = \mathbf{p}_i + \text{offset}(t) \cdot \mathbf{n}_i
$$

### 4. 动态障碍物处理

#### 4.1 碰撞时间（TTC）计算

$$
\mathbf{v}_{rel} = \mathbf{v}_{obs} - \mathbf{v}_{robot}
$$

$$
v_{closing} = -\frac{\mathbf{d} \cdot \mathbf{v}_{rel}}{\|\mathbf{d}\|}
$$

$$
TTC = \frac{\|\mathbf{d}\| - r_{obs} - r_{robot}}{v_{closing}}
$$

其中 $\mathbf{d} = \mathbf{x}_{obs} - \mathbf{x}_{robot}$ 是相对位置向量。

#### 4.2 动态障碍物预测

预测障碍物在未来时刻的位置：

$$
\mathbf{x}_{obs}(t) = \mathbf{x}_{obs}(0) + \mathbf{v}_{obs} \cdot t
$$

机器人到达路径点的时间估计：

$$
t_{arrival} = \frac{\sum_{i=start}^{idx} \|\mathbf{p}_{i+1} - \mathbf{p}_i\|}{v_{current}}
$$

#### 4.3 跟随速度计算

当需要跟随动态障碍物时：

$$
v_{follow} = \begin{cases}
v_{obs} & \text{if } d \leq d_{safe} \\
v_{current} - \frac{(v_{current} - v_{obs})^2}{2(d - d_{safe})} \cdot \Delta t & \text{otherwise}
\end{cases}
$$

### 5. 与Nav2障碍物处理的对比

| 特性 | Nav2 MPPI | OptiIO-MPPI (本项目) |
|------|-----------|-------------------|
| **架构** | 障碍物代价集成在MPPI采样中 | 独立障碍物处理器 |
| **避障决策** | 隐式（通过代价权重） | 显式（状态机决策） |
| **绕行路径** | 依赖采样偶然发现 | 主动生成绕行路径 |
| **静态/动态区分** | 统一处理 | 分离处理策略 |
| **可预测性** | 低（依赖随机采样） | 高（确定性决策） |
| **参数调优** | 困难（多代价耦合） | 简单（独立模块） |
| **计算效率** | 需要更多采样 | 固定计算开销 |
| **紧急制动** | 依赖代价阈值 | 显式TTC判断 |

---

## 📦 安装与依赖

### 系统要求

- Ubuntu 22.04+ (推荐)
- ROS2 Humble / Iron / Jazzy
- C++17 编译器

### 依赖安装

```bash
# 安装ROS2依赖
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-tf2-ros

# 安装xtensor (版本 0.21.0+)
sudo apt install -y xtensor-dev xtl-dev

# 或从源码安装
# git clone https://github.com/xtensor-stack/xtensor.git
# cd xtensor && mkdir build && cd build
# cmake .. && make -j$(nproc) && sudo make install

# 安装数学库
sudo apt install -y libopenblas-dev liblapack-dev
```

### 编译

```bash
# 创建工作空间
mkdir -p ~/optiiomppi_ws/src
cd ~/optiiomppi_ws/src

# 克隆仓库
git clone https://github.com/yourusername/OptiIO-MPPI.git

# 编译
cd ~/optiiomppi_ws
colcon build --packages-select optiio_mppi

#  source环境
source install/setup.bash
```

---

## 🚀 快速开始

### 1. 启动仿真环境

```bash
# 终端1: 启动Gazebo
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### 2. 启动静态TF

```bash
# 终端2: 发布map到odom的静态变换
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

### 3. 启动OptiIO-MPPI控制器

```bash
# 终端3: 启动MPPI节点
cd ~/optiiomppi_ws
source install/setup.bash
ros2 run optiio_mppi mppi_ros2_node --ros-args --params-file ~/optiiomppi_ws/src/OptiIO-MPPI/config/mppi_params.yaml
```

### 4. 启动RViz2

```bash
# 终端4: 可视化
rviz2 -d ~/optiiomppi_ws/src/OptiIO-MPPI/config/mppi_test.rviz
```

### 5. 发布路径

```bash
# 终端5: 发布测试路径
cd ~/optiiomppi_ws
source install/setup.bash
ros2 run optiio_mppi mppi_path_publisher.py --ros-args -p path_type:=three_side_rectangle

# 可选路径类型: straight, three_side_rectangle
```

---

## ⚙️ 参数配置

### 核心MPPI参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `batch_size` | int | 1300 | 采样轨迹数量 |
| `time_steps` | int | 90 | 预测时域步数 |
| `model_dt` | float | 0.05 | 时间步长(秒) |
| `temperature` | float | 0.3 | 软最大温度参数 |
| `iteration_count` | int | 1 | 优化迭代次数 |
| `thread_count` | int | 4 | 并行线程数 |

### 速度约束参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `vx_max` | float | 0.3 | 最大线速度(m/s) |
| `vx_min` | float | 0.0 | 最小线速度(m/s) |
| `wz_max` | float | 3.0 | 最大角速度(rad/s) |

### 加速度约束参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `ax_max` | float | 0.15 | 最大线加速度(m/s²) |
| `ay_max` | float | 2.0 | 最大横向加速度(m/s²) |
| `az_max` | float | 6.0 | 最大角加速度(rad/s²) |

### 独立避障参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `enable_avoidance` | bool | true | 是否启用避障 |
| `max_lateral_avoidance_distance` | float | 0.5 | 最大横向绕行距离(m) |
| `safety_margin_lateral` | float | 0.2 | 横向安全间隙(m) |
| `safety_margin_longitudinal` | float | 0.5 | 纵向安全距离(m) |
| `robot_radius` | float | 0.08 | 机器人半径(m) |
| `ttc_emergency_threshold` | float | 0.2 | TTC紧急阈值(秒) |
| `follow_distance` | float | 1.0 | 跟随距离(m) |
| `avoidance_lookahead` | float | 3.0 | 避障前瞻距离(m) |
| `avoidance_smooth_distance` | float | 0.5 | 绕行平滑距离(m) |
| `avoid_speed_ratio` | float | 0.7 | 避障速度比例 |

### 代价函数权重参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `path_align_weight` | float | 10.0 | 路径对齐权重 |
| `path_follow_weight` | float | 6.0 | 路径跟踪权重 |
| `path_angle_weight` | float | 2.0 | 航向角权重 |
| `goal_weight` | float | 5.0 | 目标点权重 |
| `goal_angle_weight` | float | 3.0 | 目标朝向权重 |
| `prefer_forward_weight` | float | 5.0 | 前进偏好权重 |
| `constraint_weight` | float | 4.0 | 平滑约束权重 |

---

## 📚 API文档

### MPPIController 类

```cpp
namespace mppi {

class MPPIController {
public:
    // 初始化
    void initialize(const OptimizerSettings& settings,
                    const std::string& motion_model_type = "DiffDrive",
                    float ackermann_min_turning_radius = 0.2f);
    
    // 设置路径
    void setPath(const std::vector<Pose2D>& path);
    
    // 更新障碍物
    void updateStaticObstacles(const std::vector<Point2D>& points, 
                               const Pose2D& robot_pose);
    void updateDynamicObstacles(const std::vector<DynamicObstacle>& obstacles);
    
    // 计算速度指令
    Twist2D computeVelocityCommands(const Pose2D& robot_pose, 
                                    const Twist2D& robot_speed);
    
    // 设置避障配置
    void setAvoidanceConfig(const AvoidanceConfig& config);
    
    // 获取生成的轨迹
    xt::xtensor<float, 2> getOptimizedTrajectory();
};

} // namespace mppi
```

### 数据结构

```cpp
// 2D位姿
struct Pose2D {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
};

// 2D速度
struct Twist2D {
    float vx = 0.0f;  // 线速度x
    float vy = 0.0f;  // 线速度y（全向）
    float wz = 0.0f;  // 角速度z
};

// 动态障碍物
struct DynamicObstacle {
    float x, y;       // 位置
    float vx, vy;     // 速度
    float radius;     // 半径
    bool is_moving;   // 是否移动
};

// 避障配置
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

## 📄 许可证

本项目采用 **GNU General Public License v3.0 (GPL-3.0)** 开源协议。

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

## 🙏 致谢

- [xtensor](https://github.com/xtensor-stack/xtensor) - C++多维数组库
- [Nav2](https://github.com/ros-planning/navigation2) - ROS2导航框架参考
- [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3) - 机器人仿真平台

---

## 📧 联系方式

如有问题或建议，欢迎提交Issue或Pull Request。

---

**OptiIO-MPPI** - 让移动机器人导航更智能、更可靠！
