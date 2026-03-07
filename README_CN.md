# OptiMPPI

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%2FIron%2FJazzy-green.svg)](https://docs.ros.org/)
[![C++17](https://img.shields.io/badge/C++-17-blue.svg)](https://en.cppreference.com/)
[![Platform](https://img.shields.io/badge/Platform-Cross--Platform-orange.svg)]()

**纯C++**实现的高性能、工业级**模型预测路径积分（MPPI）**控制器，专为移动机器人路径跟踪/激光避障设计。支持**跨平台移植**和**直接工业部署**。

> **核心优势**：核心MPPI控制器（`mppi_controller.hpp`）采用**纯C++17**实现，**零ROS2依赖**，可即时移植到任何嵌入式平台、实时操作系统或机器人框架（ROS1、ROS2、自定义中间件）。

实验如下：

## 目录

- [项目概述](#项目概述)
- [功能特性](#功能特性)
- [数学原理](#数学原理)
- [架构设计](#架构设计)
- [安装说明](#安装说明)
- [使用方法](#使用方法)
- [参数配置](#参数配置)
- [话题与接口](#话题与接口)
- [算法细节](#算法细节)
- [性能优化](#性能优化)
- [开源协议](#开源协议)

---

## 项目概述

OptiMPPI 是一个**纯C++实现**的MPPI控制算法，专为移动机器人导航优化。核心控制器与**ROS2完全解耦**，仅需标准C++17和头文件-only的`xtensor`库进行数值计算。这种设计带来以下优势：

- **通用可移植性**：可在任何支持C++17的平台上部署（ARM、x86、嵌入式Linux、实时操作系统）
- **工业级集成**：直接集成到现有C++代码库，无需ROS依赖
- **实时性能**：零中间件开销 - 纯优化C++执行
- **框架无关**：可与ROS1、ROS2或任何自定义机器人框架配合使用

提供的ROS2节点（`mppi_ros2_node.cpp`）作为**参考实现**和**使用示例**，演示如何为核心控制器包装ROS2接口。

### 核心能力

- **实时轨迹优化**：以20Hz频率优化控制序列（50ms控制周期）
- **激光避障**：使用基于栅格的距离场进行高效碰撞检测
- **多线程采样**：使用可配置线程池进行并行轨迹生成
- **动态障碍物支持**：对移动障碍物的预测性碰撞避免
- **鲁棒阻塞检测**：当被障碍物阻挡时防止原地旋转行为
- **多种运动模型**：支持差速驱动、全向移动和阿克曼转向模型

---

## 功能特性

### 核心功能

| 功能 | 描述 |
|---------|-------------|
| **批量采样** | 每个控制周期1000+条轨迹样本 |
| **栅格距离场** | 基于BFS的局部距离场，实现快速障碍物查询 |
| **多线程** | 可配置线程数进行轨迹积分 |
| **自适应温度** | 基于代价分布的动态温度调节 |
| **SG滤波** | Savitzky-Golay滤波器实现平滑控制序列 |
| **足迹支持** | 非圆形机器人碰撞检测 |
| **纯C++核心** | 零ROS依赖 - 可移植到任何平台 |
| **头文件库** | 单头文件（`mppi_controller.hpp`）便于集成 |
| **工业就绪** | 直接部署在嵌入式系统和实时操作系统上 |

### 跨平台可移植性

核心MPPI控制器专为**最大可移植性**而设计：

| 平台 | 支持 | 说明 |
|----------|---------|-------|
| **Linux (x86/ARM)** | ✅ 完全支持 | 主要开发平台 |
| **ROS2** | ✅ 完全支持 | 提供参考实现 |
| **ROS1** | ✅ 兼容 | 使用ROS1消息类型包装 |
| **嵌入式Linux** | ✅ 完全支持 | 在ARM Cortex-A系列上测试 |
| **实时操作系统** | ✅ 兼容 | 热路径无动态内存分配 |
| **裸机** | ⚠️ 可能 | 需要移植xtensor |
| **Windows** | ✅ 兼容 | 支持C++17 + xtensor |

**依赖项（仅核心控制器）**：
- C++17编译器
- [xtensor](https://github.com/xtensor-stack/xtensor)（头文件-only数值库）
- 仅标准C++库

**核心无需ROS、无需Boost、无需外部中间件！**

### 代价函数组件

| 代价函数 | 用途 |
|--------|---------|
| `ObstaclesCritic` | 基于指数排斥代价的碰撞避免 |
| `PathFollowCritic` | 沿全局路径的终点跟踪 |
| `PathAlignCritic` | 轨迹与路径几何对齐 |
| `PathAngleCritic` | 航向与路径方向对齐 |
| `GoalCritic` | 最终位置收敛 |
| `GoalAngleCritic` | 最终航向对齐 |
| `PreferForwardCritic` | 惩罚后退运动 |
| `ConstraintCritic` | 运动学约束执行 |
| `TwirlingCritic` | 抑制原地旋转行为 |
| `VelocityDeadbandCritic` | 低速稳定性改进 |

---

## 数学原理

### 1. MPPI核心算法

MPPI算法通过采样轨迹并计算控制序列的加权平均来解决随机最优控制问题。

#### 状态动力学（差速驱动）

$$
\begin{aligned}
\dot{x} &= v_x \cos(\theta) \\
\dot{y} &= v_x \sin(\theta) \\
\dot{\theta} &= \omega_z
\end{aligned}
$$

其中：
- $(x, y)$：机器人在世界坐标系中的位置
- $\theta$：机器人航向
- $v_x$：线速度
- $\omega_z$：角速度

#### 轨迹采样

对于每个批次$i$和时间步$t$：

$$
u_t^{(i)} = \bar{\nu}_t + \epsilon_t^{(i)}, \quad \epsilon_t^{(i)} \sim \mathcal{N}(0, \Sigma)
$$

其中：
- $\bar{\nu}_t$：前一次迭代的平均控制量
- $\epsilon_t^{(i)}$：标准差为$\sigma$的高斯噪声

#### 代价函数

轨迹$i$的总代价：

$$
S(\tau^{(i)}) = \sum_{t=0}^{T-1} \left[ s(x_t^{(i)}, \nu_t^{(i)}) + \frac{\gamma}{2} \bar{\nu}_t^T \Sigma^{-1} \nu_t^{(i)} \right]
$$

其中：
- $s(x, \nu)$：状态相关的运行代价
- $\gamma$：控制代价权重参数
- $T$：时间范围（步数）

#### Softmax加权

最优控制更新使用softmax权重：

$$
w^{(i)} = \frac{\exp\left(-\frac{1}{\lambda}(S(\tau^{(i)}) - S_{\min})\right)}{\sum_{j=1}^{K} \exp\left(-\frac{1}{\lambda}(S(\tau^{(j)}) - S_{\min})\right)}
$$

其中：
- $\lambda$：温度参数
- $S_{\min}$：所有轨迹中的最小代价
- $K$：批次大小（样本数）

#### 控制更新

更新后的控制序列：

$$
\bar{\nu}_t^{\text{new}} = \sum_{i=1}^{K} w^{(i)} \nu_t^{(i)}
$$

### 2. 障碍物代价函数

障碍物代价函数使用指数排斥代价：

$$
C_{\text{obs}}(d) = \begin{cases}
C_{\text{collision}} & \text{if } d < d_{\text{margin}} \\
w_{\text{repulsion}} \cdot \exp(-\alpha \cdot (d - d_{\text{margin}})) & \text{if } d_{\text{margin}} \leq d < r_{\text{inflation}} \\
0 & \text{otherwise}
\end{cases}
$$

其中：
- $d$：到最近障碍物的距离
- $d_{\text{margin}}$：碰撞边界距离
- $r_{\text{inflation}}$：膨胀半径
- $\alpha$：代价缩放因子
- $w_{\text{repulsion}}$：排斥权重

### 3. 栅格距离场

局部距离场使用BFS（广度优先搜索）计算：

```
对于每个栅格单元 (i, j)：
    D[i,j] = min_{障碍物单元} ||(i,j) - (i_obs, j_obs)||
```

栅格分辨率和大小可配置（默认：0.05m分辨率，100×100栅格）。

### 4. 加速度约束

速度更新遵守加速度限制：

$$
v_{t+1} = v_t + \text{clamp}(v_{\text{desired}} - v_t, -a_{\max} \Delta t, a_{\max} \Delta t)
$$

其中：
- $a_{\max}$：最大加速度
- $\Delta t$：时间步长

### 5. 阻塞检测

控制器使用旋转比率检测阻塞情况：

$$
\text{ratio} = \frac{|\omega_z|}{|v_x| + \epsilon}
$$

如果$\text{ratio} > \theta_{\text{threshold}}$持续$N$帧，则认为机器人被阻塞并命令零速度。

### 6. Savitzky-Golay滤波

控制序列平滑使用SG滤波系数：

$$
\nu_t^{\text{filtered}} = c_0 \nu_t + c_1 (\nu_{t-1} + \nu_{t+1}) + c_2 (\nu_{t-2} + \nu_{t+2})
$$

默认系数：$c_0 = 0.2$, $c_1 = 0.2$, $c_2 = 0.2$

---

## 架构设计

OptiMPPI采用**分层架构**，将核心算法与框架特定实现分离：

```
┌─────────────────────────────────────────────────────────────┐
│                    应用层                                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │   ROS2 节点   │  │   ROS1 节点   │  │   自定义中间件     │  │
│  │   (参考实现)   │  │   (可行)      │  │     (可行)        │  │
│  └──────┬───────┘  └──────┬───────┘  └────────┬─────────┘  │
└─────────┼─────────────────┼───────────────────┼────────────┘
          │                 │                   │
          └─────────────────┴───────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────┐
│                 核心控制器层                                  │
│                    (纯C++17)                                  │
│                                                             │
│   ┌─────────────────────────────────────────────────────┐  │
│   │              MPPIController (仅头文件)               │  │
│   │  ┌──────────────┐  ┌─────────────────────────────┐  │  │
│   │  │   优化器      │  │        代价管理器            │  │  │
│   │  │  ┌────────┐  │  │  ┌─────────────────────┐    │  │  │
│   │  │  │ 噪声   │  │  │  │    障碍物代价        │    │  │  │
│   │  │  │生成器   │  │  │  │    路径跟随代价      │    │  │  │
│   │  │  └────────┘  │  │  │    路径对齐代价      │    │  │  │
│   │  │  ┌────────┐  │  │  │    路径角度代价      │    │  │  │
│   │  │  │ 运动   │  │  │  │    目标点代价        │    │  │  │
│   │  │  │ 模型   │  │  │  │    目标角度代价      │    │  │  │
│   │  │  │(差速/  │  │  │  │    偏好前进代价      │    │  │  │
│   │  │  │全向/  │  │  │  │    约束代价          │    │  │  │
│   │  │  │阿克曼)│  │  │  │    旋转惩罚代价      │    │  │  │
│   │  │  └────────┘  │  │  │    速度死区代价      │    │  │  │
│   │  └──────────────┘  │  └─────────────────────┘    │  │  │
│   └─────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────────┐
│              外部依赖 (最小化)                                │
│     xtensor (头文件-only)  │  C++17标准库                    │
└─────────────────────────────────────────────────────────────┘
```

### 关键设计原则

1. **核心零ROS依赖**：整个`mppi_controller.hpp`仅使用标准C++17和xtensor
2. **头文件库**：单文件包含，便于集成到现有项目
3. **基于模板的设计**：易于为不同状态/控制类型定制
4. **基于回调的接口**：通过回调输入传感器数据，而非ROS消息
5. **确定性执行**：控制循环无非确定性操作

### 包结构

```
opti_mppi/                          # ROS2包目录（使用ros2 pkg create创建）
├── include/
│   └── mppi_controller.hpp          # 核心MPPI控制器（纯C++，ROS无关）
├── src/
│   └── mppi_ros2_node.cpp           # ROS2包装器（参考实现）
├── scripts/
│   ├── mppi_path_publisher.py       # 路径发布器（多种路径类型）
│   └── mppi_path_publisher_short.py # 短路径变体
├── config/
│   ├── mppi_params.yaml             # 参数配置
│   └── mppi_test.rviz               # RViz配置
├── CMakeLists.txt                   # 构建配置
└── package.xml                      # ROS2包清单
```

---

## 安装说明

### 前提条件

- ROS2 Humble/Iron/Jazzy
- C++17兼容编译器
- xtensor库（≥0.21.0）
- xtl库
- BLAS/LAPACK库

### 依赖安装

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y \
    libopenblas-dev \
    liblapack-dev \
    libxtensor-dev \
    libxtl-dev
```

### 源码构建

由于本仓库仅包含包文件（不含整个ROS2工作区文件夹），请按以下步骤设置包：

```bash
# 1. 导航到ROS2工作区src目录
cd ~/ros2_ws/src

# 2. 创建新的ROS2包
ros2 pkg create --build-type ament_cmake --license GPL-3.0 --description "用于ROS2移动机器人激光避障的MPPI控制器" opti_mppi

# 3. 进入包目录
cd opti_mppi

# 4. 克隆仓库内容（文件将在当前目录中）
# 从GitHub下载/克隆文件到此目录
# 文件应包括：CMakeLists.txt、package.xml、include/、src/、scripts/、config/

# 5. 使Python脚本可执行
chmod +x scripts/*.py

# 6. 构建包
cd ~/ros2_ws
colcon build --packages-select opti_mppi --symlink-install

# 7.  source工作区
source install/setup.bash
```

### 设置后的文件结构

你的包结构应如下所示：

```
~/ros2_ws/src/opti_mppi/
├── include/
│   └── mppi_controller.hpp          # 纯C++核心 - 可移植到任何平台
├── src/
│   └── mppi_ros2_node.cpp           # ROS2包装器示例
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

## 使用方法

### 启动控制器

```bash
# 终端1：启动MPPI控制器
ros2 run opti_mppi mppi_ros2_node --ros-args --params-file src/opti_mppi/config/mppi_params.yaml

# 终端2：发布全局路径
ros2 run opti_mppi mppi_path_publisher.py

# 终端3：在RViz中可视化
ros2 run rviz2 rviz2 -d src/opti_mppi/config/mppi_test.rviz
```

### 路径发布器命令

运行`mppi_path_publisher.py`时，使用以下键盘命令：
- `s` - 切换到直线路径（25m）
- `t` - 切换到三边矩形路径（总长70m）
- `u` - 切换到自定义S形路径（50m）
- `p` - 打印路径信息

---

## 参数配置

### 关键参数

#### MPPI核心参数

| 参数 | 默认值 | 描述 |
|-----------|---------|-------------|
| `batch_size` | 1300 | 轨迹样本数 |
| `time_steps` | 90 | 预测范围长度 |
| `model_dt` | 0.05 | 时间步长（秒） |
| `temperature` | 0.3 | Softmax温度 |
| `gamma` | 0.015 | 控制代价权重 |
| `iteration_count` | 1 | 每周期优化迭代次数 |
| `thread_count` | 4 | 并行轨迹线程数 |

#### 控制约束

| 参数 | 默认值 | 描述 |
|-----------|---------|-------------|
| `vx_max` | 0.3 | 最大线速度（m/s） |
| `vx_min` | 0.0 | 最小线速度（m/s） |
| `wz_max` | 2.2 | 最大角速度（rad/s） |
| `ax_max` | 1.6 | 最大线加速度（m/s²） |
| `az_max` | 3.2 | 最大角加速度（rad/s²） |

#### 障碍物参数

| 参数 | 默认值 | 描述 |
|-----------|---------|-------------|
| `obstacle_repulsion_weight` | 0.2 | 障碍物排斥代价权重 |
| `obstacle_collision_cost` | 10000.0 | 碰撞轨迹代价 |
| `obstacle_collision_margin` | 0.1 | 最小安全距离（m） |
| `obstacle_inflation_radius` | 0.2 | 障碍物膨胀半径（m） |
| `robot_radius` | 0.08 | 机器人半径（m） |
| `grid_resolution` | 0.05 | 距离场分辨率（m） |

完整参数列表请参见`config/mppi_params.yaml`。

---

## 话题与接口

### 订阅的话题

| 话题 | 类型 | 描述 |
|-------|------|-------------|
| `/plan` | `nav_msgs/Path` | 要跟随的全局路径 |
| `/odom` | `nav_msgs/Odometry` | 机器人位姿和速度 |
| `/scan` | `sensor_msgs/LaserScan` | 用于障碍物检测的激光扫描 |

### 发布的话题

| 话题 | 类型 | 描述 |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | 输出速度命令 |
| `/debug_optimal_trajectory` | `nav_msgs/Path` | 可视化的最优轨迹 |

---

## 算法细节

### 轨迹生成流程

1. **噪声生成**：异步生成高斯噪声矩阵
2. **控制采样**：向当前控制序列添加噪声
3. **约束应用**：应用速度和加速度约束
4. **运动预测**：使用运动学模型积分速度
5. **代价评估**：使用代价函数对所有轨迹评分
6. **权重计算**：基于代价计算softmax权重
7. **控制更新**：计算采样控制的加权平均
8. **平滑处理**：应用SG滤波器平滑控制序列

### 阻塞检测算法

```
if (avg_abs_wz < 0.3):
    未旋转（正常运动）
else if (|wz|/|vx| > threshold):
    spinning_counter++
    if (spinning_counter >= detect_frames):
        阻塞 → 输出零速度
else:
    重置旋转计数器
```

### 栅格距离场构建

1. 用无限距离值初始化栅格
2. 将障碍物单元标记为距离0
3. BFS传播填充距离值
4. 使用双线性插值查询距离

---

## 性能优化

### 多线程

控制器使用并行轨迹积分：
- 轨迹在工作者线程间分配
- 每个线程积分一部分轨迹
- 线程数可配置（默认：4）

### 内存管理

- 为状态和轨迹存储预分配张量
- 使用`xt::noalias`进行优化内存操作
- 工作者线程使用局部变量避免缓存竞争

### 计算复杂度

| 操作 | 复杂度 |
|-----------|------------|
| 轨迹采样 | $O(K \cdot T)$ |
| 代价评估 | $O(K \cdot T \cdot C)$ |
| 控制更新 | $O(K \cdot T)$ |
| 栅格距离场 | $O(W \cdot H)$ |

其中：
- $K$：批次大小
- $T$：时间步数
- $C$：代价函数数量
- $W, H$：栅格宽度和高度

---

## 开源协议

本项目采用 **GNU通用公共许可证v3.0 (GPL-3.0)** 授权。

```
OptiMPPI - 移动机器人跨平台MPPI控制器
版权所有 (C) 2024

本程序是自由软件：您可以在自由软件基金会发布的GNU通用公共许可证
（第3版或您选择的任何后续版本）的条款下重新分发和/或修改它。

分发本程序是希望它有用，但没有任何担保；甚至没有对适销性或
特定用途适用性的暗示担保。有关更多详细信息，请参阅GNU通用公共许可证。

您应该已经收到了一份GNU通用公共许可证的副本。如果没有，请参阅
<https://www.gnu.org/licenses/>。
```

---

## 致谢

- 灵感来源于 [Nav2 MPPI控制器](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller)
- 使用 [xtensor](https://github.com/xtensor-stack/xtensor) 进行高性能数组操作
- 为ROS2导航栈构建

## 贡献

欢迎贡献！请随时提交Pull Request。

## 联系方式

如有问题或建议，请在GitHub仓库上提交issue。
