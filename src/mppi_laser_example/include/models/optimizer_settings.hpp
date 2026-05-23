// ============================================================================
// 文件：optimizer_settings.hpp
// 功能：优化器配置参数定义
// ============================================================================
#ifndef MPPI_MODELS_OPTIMIZER_SETTINGS_HPP_
#define MPPI_MODELS_OPTIMIZER_SETTINGS_HPP_

#include "models/constraints.hpp"

namespace mppi
{

/**
 * @brief 优化器配置参数
 */
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
    int retry_attempt_limit = 1;
    unsigned int thread_count = 4; // 并行线程数

    // 阻塞检测参数（当所有轨迹都被障碍物阻挡时，输出零速度而非原地旋转）
    float spinning_ratio_threshold = 5.0f;  // wz/vx比率阈值：超此值视为打转
    int spinning_detect_frames = 2;         // 连续检测帧数：连续N帧打转才触发阻塞
    float twirling_weight = 20.0f;          // TwirlingCritic权重

    // 自适应温度参数
    bool adaptive_temperature = false;       // 是否启用自适应温度
    float adaptive_temperature_min = 0.1f;   // 自适应温度下限
    float adaptive_temperature_max = 1.0f;   // 自适应温度上限

    // 归一化模式
    bool use_mean_normalization = false;     // 是否使用均值归一化（默认使用min归一化）

    // Savitzky-Golay 滤波器参数
    bool use_sg_filter = false;              // 是否启用 Savitzky-Golay 滤波

    // 路径裁剪参数
    float prune_distance = 2.0f;             // 路径裁剪距离（米）

    // 全向模式自动切换参数
    bool enable_omni_switching = false;      // 是否启用全向/差速自动切换
    float omni_trigger_obstacle_dist = 0.5f; // 障碍物距离阈值，小于此值触发全向模式
    float omni_trigger_path_deviation = 0.3f;// 路径偏离阈值，大于此值触发全向模式
    float diff_restore_path_threshold = 0.15f;// 恢复差速模式的路径偏离阈值
    int omni_switch_delay_frames = 3;        // 切换延迟帧数，防止频繁切换
};

} // namespace mppi

#endif // MPPI_MODELS_OPTIMIZER_SETTINGS_HPP_
