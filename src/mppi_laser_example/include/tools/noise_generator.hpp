// ============================================================================
// 文件：noise_generator.hpp
// 功能：高斯噪声生成器，异步预生成噪声用于MPPI采样
// ============================================================================
#ifndef MPPI_TOOLS_NOISE_GENERATOR_HPP_
#define MPPI_TOOLS_NOISE_GENERATOR_HPP_

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <xtensor/xtensor.hpp>
#include <xtensor/xnoalias.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xview.hpp>

#include "models/optimizer_settings.hpp"
#include "models/control_sequence.hpp"
#include "models/state.hpp"

namespace mppi
{

/**
 * @brief 噪声生成器，为每条轨迹生成高斯噪声
 * 使用独立线程异步预生成噪声，减少主循环延迟
 */
class NoiseGenerator
{
public:
    NoiseGenerator() = default;

    ~NoiseGenerator() { shutdown(); }

    /**
     * @brief 初始化噪声生成器
     * @param settings 优化器设置（含标准差、batch size、时间步数）
     * @param is_holonomic 是否为全向模型
     */
    void initialize(OptimizerSettings & settings, bool is_holonomic)
    {
        settings_ = settings;
        is_holonomic_ = is_holonomic;
        noises_vx_ = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
        noises_vy_ = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
        noises_wz_ = xt::zeros<float>({settings_.batch_size, settings_.time_steps});

        active_ = true;
        ready_ = false;

        // 启动噪声生成线程，异步预生成噪声
        noise_thread_ = std::thread(&NoiseGenerator::noiseThread, this);

        // 触发第一次噪声生成
        generateNextNoises();
    }

    /**
     * @brief 关闭噪声生成线程
     */
    void shutdown()
    {
        active_ = false;
        ready_ = true;
        noise_cond_.notify_all();
        if (noise_thread_.joinable()) {
            noise_thread_.join();
        }
    }

    /**
     * @brief 触发噪声线程生成下一组噪声（异步预生成）
     */
    void generateNextNoises()
    {
        {
            std::unique_lock<std::mutex> lock(noise_lock_);
            ready_ = true;
        }
        noise_cond_.notify_all();
    }

    /**
     * @brief 将噪声叠加到控制序列上，得到含噪声的控制量，存入 state
     * @param state 状态容器（cvx/cvy/cwz 将被写入）
     * @param control_sequence 原始控制序列
     */
    void setNoisedControls(State & state, const ControlSequence & control_sequence)
    {
        std::unique_lock<std::mutex> lock(noise_lock_);

        // 等待噪声生成完成（如果正在生成）
        noise_cond_.wait(lock, [this]() { return !ready_; });

        // 使用xt::noalias优化内存操作
        xt::noalias(state.cvx) = xt::view(control_sequence.vx, xt::newaxis(), xt::all()) + noises_vx_;
        xt::noalias(state.cwz) = xt::view(control_sequence.wz, xt::newaxis(), xt::all()) + noises_wz_;
        if (is_holonomic_) {
            xt::noalias(state.cvy) = xt::view(control_sequence.vy, xt::newaxis(), xt::all()) + noises_vy_;
        }
    }

    /** @brief 重置生成器（重新初始化） */
    void reset(OptimizerSettings & settings, bool is_holonomic)
    {
        settings_ = settings;
        is_holonomic_ = is_holonomic;

        {
            std::unique_lock<std::mutex> lock(noise_lock_);
            xt::noalias(noises_vx_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
            xt::noalias(noises_vy_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
            xt::noalias(noises_wz_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
            ready_ = true;
        }
        noise_cond_.notify_all();
    }

    /** @brief 设置全向模式状态（用于动态切换） */
    void setHolonomic(bool is_holonomic)
    {
        is_holonomic_ = is_holonomic;
    }

private:
    /**
     * @brief 噪声生成线程（后台异步预生成噪声）
     */
    void noiseThread()
    {
        do {
            std::unique_lock<std::mutex> lock(noise_lock_);
            noise_cond_.wait(lock, [this]() { return ready_; });
            ready_ = false;
            if (!active_) break;
            generateNoisedControls();
        } while (active_);
    }

    /**
     * @brief 生成高斯噪声矩阵
     * 使用xt::random::randn生成高斯噪声
     */
    void generateNoisedControls()
    {
        auto & s = settings_;

        xt::noalias(noises_vx_) = xt::random::randn<float>(
            {s.batch_size, s.time_steps}, 0.0f, s.sampling_std.vx);
        xt::noalias(noises_wz_) = xt::random::randn<float>(
            {s.batch_size, s.time_steps}, 0.0f, s.sampling_std.wz);
        if (is_holonomic_) {
            xt::noalias(noises_vy_) = xt::random::randn<float>(
                {s.batch_size, s.time_steps}, 0.0f, s.sampling_std.vy);
        }
    }

    xt::xtensor<float, 2> noises_vx_, noises_vy_, noises_wz_;
    OptimizerSettings settings_;
    bool is_holonomic_ = false;

    // 线程相关成员
    std::thread noise_thread_;
    std::condition_variable noise_cond_;
    std::mutex noise_lock_;
    bool active_ = false;
    bool ready_ = false;
};

} // namespace mppi

#endif // MPPI_TOOLS_NOISE_GENERATOR_HPP_
