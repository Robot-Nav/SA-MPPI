// ============================================================================
// 文件：motion_models.hpp
// 功能：运动模型基类及派生类（差速、全向、阿克曼），定义运动学预测接口
// ============================================================================
#ifndef MPPI_MOTION_MODELS_HPP_
#define MPPI_MOTION_MODELS_HPP_

#include <cmath>
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "models/state.hpp"
#include "models/constraints.hpp"
#include "tools/math_utils.hpp"

namespace mppi
{

/**
 * @brief 运动模型基类，定义运动学预测接口
 */
class MotionModel
{
public:
    MotionModel() = default;
    virtual ~MotionModel() = default;

    /**
     * @brief 根据控制量预测速度（包含加速度约束）
     * @param state 状态容器，其中 cvx/cvy/cwz 为含噪声控制量，本函数将 vx/vy/wz 后移一位
     */
    virtual void predict(State & state)
    {
        using namespace xt::placeholders;

        // 第一时刻使用当前速度
        for (size_t i = 0; i < state.vx.shape(0); ++i) {
            state.vx(i, 0) = state.speed.vx;
            state.wz(i, 0) = state.speed.wz;
            if (isHolonomic()) {
                state.vy(i, 0) = state.speed.vy;
            }
        }

        // 后续时刻：应用加速度约束的速度传播
        for (size_t i = 0; i < state.vx.shape(0); ++i) {
            for (size_t j = 1; j < state.vx.shape(1); ++j) {
                float prev_vx = state.vx(i, j - 1);
                float prev_vy = isHolonomic() ? state.vy(i, j - 1) : 0.0f;
                float prev_wz = state.wz(i, j - 1);

                float ctrl_vx = state.cvx(i, j - 1);
                float ctrl_vy = isHolonomic() ? state.cvy(i, j - 1) : 0.0f;
                float ctrl_wz = state.cwz(i, j - 1);

                state.vx(i, j) = applyAccelerationConstraint(ctrl_vx, prev_vx, ax_max_, model_dt_);
                state.wz(i, j) = applyAccelerationConstraint(ctrl_wz, prev_wz, az_max_, model_dt_);
                if (isHolonomic()) {
                    state.vy(i, j) = applyAccelerationConstraint(ctrl_vy, prev_vy, ay_max_, model_dt_);
                }
            }
        }
    }

    /** @return 是否为全向移动模型 */
    virtual bool isHolonomic() const = 0;

    /**
     * @brief 对含噪声控制量施加运动学约束（如最小转弯半径）
     */
    virtual void applyConstraints(xt::xtensor<float, 2> & /*cvx*/,
                                   xt::xtensor<float, 2> & /*cvy*/,
                                   xt::xtensor<float, 2> & /*cwz*/) {}

    /**
     * @brief 设置加速度约束参数
     */
    void setAccelerationConstraints(float ax_max, float ay_max, float az_max, float model_dt)
    {
        ax_max_ = ax_max;
        ay_max_ = ay_max;
        az_max_ = az_max;
        model_dt_ = model_dt;
    }

protected:
    /**
     * @brief 应用加速度约束
     * @param desired 期望速度
     * @param current 当前速度
     * @param max_accel 最大加速度
     * @param dt 时间步长
     * @return 约束后的速度
     */
    float applyAccelerationConstraint(float desired, float current, float max_accel, float dt) const
    {
        float max_delta = max_accel * dt;
        float delta = desired - current;
        delta = clamp(delta, -max_delta, max_delta);
        return current + delta;
    }

    float ax_max_ = 2.0f;   // 最大线加速度
    float ay_max_ = 2.0f;   // 最大横向加速度
    float az_max_ = 3.0f;   // 最大角加速度
    float model_dt_ = 0.05f; // 时间步长
};

/**
 * @brief 差速运动模型（非全向）
 */
class DiffDriveMotionModel : public MotionModel
{
public:
    DiffDriveMotionModel() = default;
    bool isHolonomic() const override { return false; }
};

/**
 * @brief 全向运动模型
 */
class OmniMotionModel : public MotionModel
{
public:
    OmniMotionModel() = default;
    bool isHolonomic() const override { return true; }
};

/**
 * @brief 阿克曼运动模型（非全向，带最小转弯半径约束）
 */
class AckermannMotionModel : public MotionModel
{
public:
    /**
     * @param min_turning_r 最小转弯半径
     */
    explicit AckermannMotionModel(float min_turning_r = 0.2f)
        : min_turning_r_(min_turning_r) {}

    bool isHolonomic() const override { return false; }

    /**
     * @brief 施加最小转弯半径约束
     */
    void applyConstraints(xt::xtensor<float, 2> & cvx,
                          xt::xtensor<float, 2> & /*cvy*/,
                          xt::xtensor<float, 2> & cwz) override
    {
        for (size_t i = 0; i < cvx.shape(0); ++i) {
            for (size_t j = 0; j < cvx.shape(1); ++j) {
                float vx_val = cvx(i, j);
                float wz_val = cwz(i, j);
                if (std::abs(vx_val) > EPSILON && std::abs(wz_val) > EPSILON) {
                    float radius = std::abs(vx_val / wz_val);
                    if (radius < min_turning_r_) {
                        cwz(i, j) = (wz_val > 0 ? 1.0f : -1.0f) * std::abs(vx_val) / min_turning_r_;
                    }
                }
            }
        }
    }

private:
    float min_turning_r_{0.2f};
};

} // namespace mppi

#endif // MPPI_MOTION_MODELS_HPP_
