// ============================================================================
// 文件：critic_manager.hpp
// 功能：代价函数管理器，负责存储所有代价函数并批量执行评分
// ============================================================================
#ifndef MPPI_CRITICS_CRITIC_MANAGER_HPP_
#define MPPI_CRITICS_CRITIC_MANAGER_HPP_

#include <vector>
#include <memory>
#include <string>

#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"

namespace mppi
{

/**
 * @brief 代价函数管理器，负责存储所有代价函数并批量执行评分
 */
class CriticManager
{
public:
    CriticManager() = default;
    /** @brief 添加一个代价函数（接管所有权） */
    void addCritic(std::unique_ptr<CriticFunction> critic) { critics_.push_back(std::move(critic)); }
    /** @brief 初始化所有代价函数 */
    void initializeCritics() { for (auto & c : critics_) c->initialize(); }

    /**
     * @brief 对所有代价函数执行评分，累加代价到 data.costs
     * @param data 评分所需数据
     */
    void evalTrajectoriesScores(CriticData & data) const
    {
        for (size_t i = 0; i < data.costs.shape(0); ++i) data.costs(i) = 0.0f;
        data.fail_flag = false;

        for (const auto & critic : critics_) {
            if (critic->isEnabled()) critic->score(data);
        }
    }

    /**
     * @brief 根据名称获取代价函数指针（用于外部设置参数）
     * @param name 代价函数名称
     * @return 代价函数指针，若未找到返回 nullptr
     */
    CriticFunction* getCritic(const std::string & name) const
    {
        for (const auto & c : critics_)
            if (c->getName() == name) return c.get();
        return nullptr;
    }

private:
    std::vector<std::unique_ptr<CriticFunction>> critics_;
};

} // namespace mppi

#endif // MPPI_CRITICS_CRITIC_MANAGER_HPP_
