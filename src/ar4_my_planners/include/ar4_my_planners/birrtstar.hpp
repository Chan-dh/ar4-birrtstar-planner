/*********************************************************************
 * BiRRTStar —— 双向渐近最优快速扩展随机树
 *
 * 设计文档：docs/birrtstar设计_v2.md（主），docs/stage_1_ompl_3.md（辅）
 *
 * 拼装关系（设计文档 §1.3）：
 *   - 骨架取 RRTConnect：双树 + GrowState 协议 + connect 贪心
 *   - 函数拆分取 BiTRRT：addMotion / extendTree / connectTrees，
 *     tree == tStart_ 推导方向，"节点身份 = 在哪个 NN 容器里"
 *   - 代价机制取 RRTstar：cost-to-come + chooseParent + rewire +
 *     updateChildCosts，两棵树内部各自独立运行
 *
 * 本文件只允许 include OMPL 头，禁止任何 ROS / MoveIt / AR4 依赖（设计文档 §11）。
 * 二维 harness 用 fprintf 做诊断，六维插件经 OMPL 日志/回调，不用 #ifdef。
 *********************************************************************/

#pragma once

#include <limits>
#include <memory>
#include <vector>

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSampler.h>
#include <ompl/datastructures/NearestNeighbors.h>

namespace ar4_my_planners
{
/** \brief 双向渐近最优规划器（文档 §6） */
class BiRRTStar : public ompl::base::Planner
{
public:
    explicit BiRRTStar(const ompl::base::SpaceInformationPtr &si);

    ~BiRRTStar() override;

    void setup() override;

    void clear() override;

    /** \brief 主循环（文档 §4.2 逐步注释） */
    ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

    /** \brief tag 约定：1 = 起点树，2 = 目标树；两树的汇合边也加入（文档 §6.6） */
    void getPlannerData(ompl::base::PlannerData &data) const override;

    /** \brief 树扩展的最大单步长度 */
    void setRange(double distance)
    {
        maxDistance_ = distance;
    }

    /** \brief 树扩展的最大单步长度 */
    double getRange() const
    {
        return maxDistance_;
    }

    /** \brief r_rrg = s × r_rrg*（或 k_rrg = s × k_rrg*）中的缩放系数 s */
    void setRewireFactor(double rewireFactor)
    {
        rewireFactor_ = rewireFactor;
        if (setup_)
            calculateRewiringLowerBounds();
    }

    /** \brief r_rrg = s × r_rrg*（或 k_rrg = s × k_rrg*）中的缩放系数 s */
    double getRewireFactor() const
    {
        return rewireFactor_;
    }

    /** \brief 邻域取 k 近邻（默认）或 r 圆盘 */
    void setKNearest(bool useKNearest)
    {
        useKNearest_ = useKNearest;
    }

    /** \brief 邻域取 k 近邻（默认）或 r 圆盘 */
    bool getKNearest() const
    {
        return useKNearest_;
    }

    /** \brief 首次进入 solve 到最后一次循环迭代的次数 */
    unsigned int numIterations() const
    {
        return iterations_;
    }

    /** \brief 当前已找到的最优路径代价 */
    const ompl::base::Cost &bestCost() const
    {
        return bestCost_;
    }

protected:
    /**
     * \brief 树上的节点（文档 §6.1）
     *
     * 相对 RRTstar::Motion 的改动（文档决策 3）：
     *   - 删除 root 字段 —— 树身份由所在 NN 容器决定，不用枚举比对（不变量 B）
     *   - 删除 inGoal 字段 —— 多根按 parent==nullptr 判断即可
     *   - 删除 stateCost 捷径 —— 统一用 cost-to-come
     */
    class Motion
    {
    public:
        explicit Motion(const ompl::base::SpaceInformationPtr &si) : state(si->allocState())
        {
        }

        ~Motion() = default;

        /** \brief 本节点状态 */
        ompl::base::State *state{nullptr};

        /** \brief 本树内的父节点；两个根节点（起点根/目标根）为 nullptr */
        Motion *parent{nullptr};

        /** \brief 本树内的子节点列表 */
        std::vector<Motion *> children;

        /** \brief 到本树根的 cost-to-come，父节点改变时同步更新 */
        ompl::base::Cost cost;

        /** \brief 从父节点到本节点的增量 */
        ompl::base::Cost incCost;
    };

    /** \brief 两树的一对汇合点（文档 §6.2）。只读记录，不含任何指针改动 */
    struct ConnectionPoint
    {
        Motion *startTreeMotion;  // 必属 tStart_
        Motion *goalTreeMotion;   // 必属 tGoal_；与上者状态同位
    };

    /** \brief 双树共用一套扩展结果协议（取自 RRTConnect） */
    enum GrowState
    {
        TRAPPED,   ///< 无进展（碰撞或插值无进展）
        ADVANCED,  ///< 朝目标前进了一步
        REACHED   ///< 一步直达目标
    };

    using TreeData = std::shared_ptr<ompl::NearestNeighbors<Motion *>>;

    /** \brief 用于按 costs_ 排序邻域下标（取自 RRTstar.h:374-386） */
    struct CostIndexCompare
    {
        CostIndexCompare(const std::vector<ompl::base::Cost> &costs, const ompl::base::OptimizationObjective &opt)
          : costs_(costs), opt_(opt)
        {
        }

        bool operator()(unsigned i, unsigned j)
        {
            return opt_.isCostBetterThan(costs_[i], costs_[j]);
        }

        const std::vector<ompl::base::Cost> &costs_;
        const ompl::base::OptimizationObjective &opt_;
    };

    /** \brief 采样器 */
    ompl::base::StateSamplerPtr sampler_;

    /** \brief 树扩展的最大单步长度 */
    double maxDistance_{0.};

    /** \brief 起点树 / 目标树：两个容器指针永不交换（不变量 B），每轮只翻转 expandStartFirst_ */
    TreeData tStart_;
    TreeData tGoal_;

    /** \brief 本轮先扩哪棵树；这是每轮唯一翻转的东西（决策 1） */
    bool expandStartFirst_{true};

    /** \brief 优化目标 */
    ompl::base::OptimizationObjectivePtr opt_;

    /** \brief 当前最优解代价；bestCost_ 单调不增（文档 §9(5)） */
    ompl::base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

    /** \brief 当前最优汇合点对（分别属两棵树） */
    Motion *bestStartTreeMotion_{nullptr};
    Motion *bestGoalTreeMotion_{nullptr};

    /** \brief 历次汇合点对，Motion* 永不失效（不变量 C：运行期不删节点） */
    std::vector<ConnectionPoint> connectionPoints_;

    /** \brief 邻域取 k 近邻（默认）或 r 圆盘 */
    bool useKNearest_{true};

    /** \brief r_rrg / k_rrg 的缩放系数 */
    double rewireFactor_{1.1};

    /** \brief k_rrt = k_rrg × log(n) */
    double k_rrt_{0.};

    /** \brief r_rrg × (log(n)/n)^(1/d) */
    double r_rrt_{0.};

    /** \brief chooseParent 与 rewire 共享的邻域缓存（文档 §5.3(2)），只增不减 */
    std::vector<ompl::base::Cost> costs_;
    std::vector<ompl::base::Cost> incCosts_;
    std::vector<std::size_t> sortedCostIndices_;

    /** \brief chooseParent 已得结论的复用：1 = 边无碰撞，-1 = 有碰撞，0 = 未知 */
    std::vector<int> valid_;

    /** \brief 两树间的最小 NN 距离，用于近似解评估 */
    double distanceBetweenTrees_{std::numeric_limits<double>::infinity()};

    /** \brief 主循环计数 */
    unsigned int iterations_{0u};

    // ── 函数分解（文档 §5.1）──────────────────────────────────────────

    /** \brief 常规 Bresenham 距离（即 si_->distance） */
    double distanceFunction(const Motion *a, const Motion *b) const;

    /** \brief 释放两棵树全部节点 */
    void freeMemory();

    /** \brief 建新节点并入树：根（parent==nullptr）或代价已算好（chooseParent 之后） */
    Motion *addMotion(const ompl::base::State *state, TreeData &tree, Motion *parent,
                      ompl::base::Cost incCost, ompl::base::Cost cost);

    /** \brief 建新节点并入树：内部按 cost-to-come 计算 */
    Motion *addMotion(const ompl::base::State *state, TreeData &tree, Motion *parent);

    /** \brief 主动树单步 Extend：NN 查询 + steer + 方向相关 checkMotion；产出候选，不入树（决策 7）。
     * nmotion 是入参兜底也是出参（§5.3(1)）。REACHED 时 out_state 与 target 值相等。 */
    GrowState extendTree(TreeData &tree, Motion *target, Motion *&nmotion, ompl::base::State *out_state);

    /** \brief 已知最近邻的单步 Extend（connectTrees 的循环体） */
    GrowState extendTree(TreeData &tree, const ompl::base::State *target, Motion *nmotion,
                         ompl::base::State *out_state);

    /** \brief 被动树朝 target 贪心推进；REACHED 时 out_reached 为汇合节点，返回 true（文档 §4.3） */
    bool connectTrees(TreeData &tree, Motion *target, ompl::base::State *scratch, Motion *&out_reached);

    /** \brief 汇合时刻只读记录 (a,b)；增量更新 bestCost_ 并触发中间解回调（决策 5） */
    void considerConnection(Motion *a, Motion *b);

    /** \brief 新节点的邻域（k 近邻或 r 圆盘） */
    void getNeighbors(TreeData &tree, Motion *motion, std::vector<Motion *> &nbh) const;

    /**
     * \brief 依邻域为候选选父（取自 RRTstar.cpp:319-358 的 delayCC 路径，决策 9 硬编码）。
     * 必须以 extendTree 返回的 nmotion 为兜底父节点（§5.3(1)）。副作用：填 costs_/incCosts_/
     * sortedCostIndices_/valid_，供紧随其后的 rewire 复用。
     */
    Motion *chooseParent(Motion *xmotion, Motion *nmotion, std::vector<Motion *> &nbh,
                         ompl::base::Cost &out_incCost, ompl::base::Cost &out_cost);

    /** \brief 新节点入树后的邻域重连（取自 RRTstar.cpp:415-457）。
     * 必须紧接在同一个 nbh 上的 chooseParent 之后调用（§5.3(2)）。 */
    void rewire(Motion *motion, std::vector<Motion *> &nbh);

    /** \brief 把 m 从其当前父节点的 children 中摘除（取自 RRTstar.cpp:620-627） */
    void removeFromParent(Motion *m);

    /** \brief 父节点变更后递归刷新子树代价（取自 RRTstar.cpp:629-639） */
    void updateChildCosts(Motion *m);

    /** \brief k_rrt / r_rrt 下界（取自 RRTstar.cpp:1147-1159） */
    void calculateRewiringLowerBounds();

    /** \brief 沿两树父链回溯成一条完整路径；输出时去重（决策 5） */
    ompl::base::PathPtr constructSolutionPath(Motion *a, Motion *b) const;

private:
    /** \brief 进度属性 */
    std::string numIterationsProperty() const
    {
        return std::to_string(numIterations());
    }

    /** \brief 进度属性 */
    std::string bestCostProperty() const
    {
        return std::to_string(bestCost_.value());
    }
};
}  // namespace ar4_my_planners
