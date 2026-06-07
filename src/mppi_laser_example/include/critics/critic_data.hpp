// ============================================================================
// 文件：critic_data.hpp
// 功能：传递给代价函数的数据集合定义
// ============================================================================
#ifndef MPPI_CRITICS_CRITIC_DATA_HPP_
#define MPPI_CRITICS_CRITIC_DATA_HPP_

#include <vector>
#include <optional>
#include <memory>
#include <xtensor/xtensor.hpp>

#include "models/types.hpp"
#include "models/state.hpp"
#include "models/trajectories.hpp"
#include "models/path.hpp"

namespace mppi
{

// 前向声明
class MotionModel;

/**
 * @brief 传递给代价函数的数据集合
 */
struct CriticData
{
    const State & state;                        ///< 当前状态
    const Trajectories & trajectories;          ///< 采样轨迹
    const Path & path;                          ///< 全局路径
    xt::xtensor<float, 1> & costs;              ///< 代价向量（将被修改）
    float & model_dt;                            ///< 模型时间步长
    bool fail_flag = false;                      ///< 是否失败（可由代价函数设置）
    std::shared_ptr<MotionModel> motion_model; ///< 运动模型
    std::optional<size_t> furthest_reached_path_point; ///< 路径上最远已到达点索引
    const std::vector<ObstaclePrediction> * dynamic_obstacles = nullptr; ///< 动态障碍物列表
    const std::vector<Point2D> * obstacles = nullptr; ///< 静态障碍物点云（用于路径有效性检查）
    const std::vector<bool> * path_pts_valid = nullptr; ///< 路径点有效性标志（true=有效，false=被阻挡）
};

} // namespace mppi

#endif // MPPI_CRITICS_CRITIC_DATA_HPP_
