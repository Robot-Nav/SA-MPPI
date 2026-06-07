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
};

} // namespace mppi

#endif // MPPI_MODELS_OPTIMIZER_SETTINGS_HPP_
