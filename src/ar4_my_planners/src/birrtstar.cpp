/*********************************************************************
 * BiRRTStar 实现
 *
 * 逐步对照 docs/birrtstar设计_v2.md 的伪代码与行号注释：
 *   - 主循环：§4.2（步骤 ①-⑧）
 *   - connectTrees：§4.3
 *   - 函数职责与耦合：§5.1-§5.4
 *   - considerConnection / getPlannerData：§6.2 / §6.6
 *   - chooseParent / rewire 逐行移植自 OMPL 1.7 RRTstar.cpp（行号见各处注释）
 *   - 双树骨架与方向相关碰撞检查移植自 OMPL 1.7 RRTConnect.cpp
 *********************************************************************/

#include "ar4_my_planners/birrtstar.hpp"

#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <limits>

#include <ompl/base/Goal.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/util/GeometricEquations.h>
#include <ompl/util/String.h>

namespace ar4_my_planners
{
namespace
{
/** \brief solve 主循环内部的失败出口统一走 INVALID_START/INVALID_GOAL，与 RRTConnect 一致 */
ompl::base::PlannerStatus invalid(const char *name, const char *msg, ompl::base::PlannerStatus::StatusType status)
{
    OMPL_ERROR("%s: %s", name, msg);
    return ompl::base::PlannerStatus(status);
}
}  // namespace

BiRRTStar::BiRRTStar(const ompl::base::SpaceInformationPtr &si) : ompl::base::Planner(si, "BiRRTStar")
{
    // v2 §6.4：构造只做规格声明、参数注册与进度属性
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.canReportIntermediateSolutions = true;
    specs_.directed = true;
    specs_.recognizedGoal = ompl::base::GOAL_SAMPLEABLE_REGION;

    Planner::declareParam<double>("range", this, &BiRRTStar::setRange, &BiRRTStar::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("rewire_factor", this, &BiRRTStar::setRewireFactor, &BiRRTStar::getRewireFactor,
                                   "1.0:0.01:2.0");
    Planner::declareParam<bool>("use_k_nearest", this, &BiRRTStar::setKNearest, &BiRRTStar::getKNearest, "0,1");

    addPlannerProgressProperty("iterations INTEGER", [this] { return numIterationsProperty(); });
    addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

BiRRTStar::~BiRRTStar()
{
    freeMemory();
}

void BiRRTStar::setup()
{
    Planner::setup();

    ompl::tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    // v2 §4.1：对双树比单树更重要的对称性检查——目标树的边在输出路径里是逆向遍历的
    if (!si_->getStateSpace()->hasSymmetricDistance() || !si_->getStateSpace()->hasSymmetricInterpolate())
    {
        OMPL_WARN("%s requires a state space with symmetric distance and symmetric interpolation.",
                  getName().c_str());
    }

    if (!tStart_)
        tStart_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
        {
            opt_ = pdef_->getOptimizationObjective();
        }
        else
        {
            OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the "
                        "allowed planning time.",
                        getName().c_str());
            opt_ = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si_);
            pdef_->setOptimizationObjective(opt_);
        }
        if (!opt_->isSymmetric())
        {
            // v2 §4.1：非对称 objective 下目标树优化的是"到 goal 的正向代价的反向"
            OMPL_WARN("%s: The optimization objective is asymmetric. The goal tree optimizes the reverse "
                      "direction; results may be inconsistent.",
                      getName().c_str());
        }
        bestCost_ = opt_->infiniteCost();
    }
    else
    {
        // v2 §4.1：没有 pdef 就推迟 setup 完成（留到最后一次机会）
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    calculateRewiringLowerBounds();
}

void BiRRTStar::clear()
{
    // v2 §6.5：镜像 setup 的初始化；freeMemory 会遍历 NN 容器，须先于容器 clear()
    setup_ = false;
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoints_.clear();
    bestStartTreeMotion_ = nullptr;
    bestGoalTreeMotion_ = nullptr;
    bestCost_ = ompl::base::Cost(std::numeric_limits<double>::quiet_NaN());
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
    expandStartFirst_ = true;
    iterations_ = 0u;
}

void BiRRTStar::freeMemory()
{
    // 不变量 C 的推论：freeMemory / clear / 析构绝不出现在 solve 之外的任何路径上（v2 §6.5）
    std::vector<Motion *> motions;
    if (tStart_)
    {
        tStart_->list(motions);
        for (auto *motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
    if (tGoal_)
    {
        motions.clear();
        tGoal_->list(motions);
        for (auto *motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus BiRRTStar::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    // ── 入口守卫（v2 §4.2）─────────────────────────────────────────
    checkValidity();

    auto *goal = dynamic_cast<ompl::base::GoalSampleableRegion *>(pdef_->getGoal().get());
    if (goal == nullptr)
        return invalid(getName().c_str(), "Unknown type of goal", ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE);

    while (const ompl::base::State *st = pis_.nextStart())
        addMotion(st, tStart_, nullptr);
    if (tStart_->size() == 0)
        return invalid(getName().c_str(), "Motion planning start tree could not be initialized!",
                        ompl::base::PlannerStatus::INVALID_START);

    if (!goal->couldSample())
        return invalid(getName().c_str(), "Insufficient states in sampleable goal region",
                       ompl::base::PlannerStatus::INVALID_GOAL);

    // 目标树还没有根：这里必须阻塞拿一个（v2 §4.2；三份双树参考同款）
    if (tGoal_->size() == 0)
    {
        const ompl::base::State *st = pis_.nextGoal(ptc);
        if (st != nullptr)
            addMotion(st, tGoal_, nullptr);
    }
    if (tGoal_->size() == 0)
        return invalid(getName().c_str(), "Unable to sample any valid states for goal tree",
                       ompl::base::PlannerStatus::INVALID_GOAL);

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(),
                static_cast<unsigned int>(tStart_->size() + tGoal_->size()));

    // rmotion = 采样缓冲，xmotion = steer 落点缓冲；两者都是 scratch，绝不入树（v2 §5.2）
    auto *rmotion = new Motion(si_);
    auto *xmotion = new Motion(si_);

    Motion *approxMotion = nullptr;  // 离 goal 最近的起点树叶节点
    double approxDist = std::numeric_limits<double>::infinity();

    std::vector<Motion *> nbh;  // getNeighbors 输出

    // ── 主循环（v2 §4.2 步骤 ①-⑧）───────────────────────────────
    while (!ptc)
    {
        iterations_++;

        // ① 选主动/被动树。每轮唯一交换的东西是 expandStartFirst_（决策 1）；
        //    tStart_/tGoal_ 容器指针永不交换（不变量 B）
        const bool activeIsStart = expandStartFirst_;
        TreeData &active = activeIsStart ? tStart_ : tGoal_;
        TreeData &passive = activeIsStart ? tGoal_ : tStart_;
        expandStartFirst_ = !expandStartFirst_;

        // ② 目标侧多根持续拉取（决策 8，条件取自 RRTConnect.cpp:251）
        if (pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const ompl::base::State *st = pis_.nextGoal();
            if (st != nullptr)
                addMotion(st, tGoal_, nullptr);
        }

        // ③ 均匀采样
        sampler_->sampleUniform(rmotion->state);

        // ④ 主动树 Extend 一步：结果落在 xmotion->state，尚未入树（决策 7）
        // nmotion = nearest motion in tree
        Motion *nmotion = nullptr;
        const GrowState gs = extendTree(active, rmotion, nmotion, xmotion->state);
        if (gs == TRAPPED)
            continue;

        // ⑤ 主动树上跑 RRT*：选父 → 入树 → 重连（决策 2；仅主动树）
        getNeighbors(active, xmotion, nbh);
        ompl::base::Cost incCost, cost;
        // xmotion的 新父节点
        Motion *parent = chooseParent(xmotion, nmotion, nbh, incCost, cost);
        // xmotion->state 入树,新节点用xNew表示
        Motion *xNew = addMotion(xmotion->state, active, parent, incCost, cost);
        rewire(xNew, nbh);

        // ⑥ 近似解：仅当主动树是起点树且尚无任何精确解（决策 6，源自 RRTConnect.cpp:283-297）
        if (activeIsStart && connectionPoints_.empty())
        {
            double distanceFromGoal = std::numeric_limits<double>::infinity();
            goal->isSatisfied(xNew->state, &distanceFromGoal);
            if (distanceFromGoal < approxDist)
            {
                approxDist = distanceFromGoal;
                approxMotion = xNew;
            }
        }

        // ⑦ 被动树朝 xNew connect（贪心，不走 RRT*）。目标永远是 xNew->state，
        //    不是原始采样点 rmotion->state —— 对应 RRTConnect.cpp:289-290 的 copyState
        // rmotion->state 是 connect贪心朝向
        si_->copyState(rmotion->state, xNew->state);  // rmotion 复用为 connect 的查询缓冲
        Motion *reached = nullptr; // 最终到到的点
        // xmotion->state 扩展产生的新状态缓存
        const bool connected = connectTrees(passive, rmotion, xmotion->state, reached);

        // ⑧ 记录汇合点并增量更新 bestCost_（决策 5）
        if (connected)
        {
            // 不变量 A：parent 指针绝不跨树；这里只做只读记录
            Motion *a = activeIsStart ? xNew : reached;  // 必属 tStart_
            Motion *b = activeIsStart ? reached : xNew;   // 必属 tGoal_，与 a 状态同位
            connectionPoints_.push_back({a, b});
            considerConnection(a, b);
        }
        else if (bestStartTreeMotion_ != nullptr)
        {
            // 本轮 rewire 可能改善了当前最优路径上的节点 —— cost 是活值，直接重估（v2 §6.2）
            considerConnection(bestStartTreeMotion_, bestGoalTreeMotion_);
        }

        // 首解后不 break：继续优化到 ptc 或最优性满足（决策 4）
        if (bestStartTreeMotion_ != nullptr && opt_->isSatisfied(bestCost_))
            break;
    }

    // ── 循环外：只输出一次（决策 4）────────────────────────────────
    // 输出前最后一次冲刷：循环里最后一次 rewire 的改善也要进 bestCost_。
    // 主循环的 else 分支只在"本轮未汇合"时重估旧汇合对（§4.2 伪代码的 elif 语义），
    // 若循环停在汇合轮，bestCost_ 会滞后于当前父链的真实代价，违反 §9(4) 的
    // "输出与 bestCost_ 一致"。这里只在严格变优时更新（considerConnection 自带守卫）
    if (bestStartTreeMotion_ != nullptr)
        considerConnection(bestStartTreeMotion_, bestGoalTreeMotion_);
    if (bestStartTreeMotion_ != nullptr)
    {
        auto path = constructSolutionPath(bestStartTreeMotion_, bestGoalTreeMotion_);
        ompl::base::PlannerSolution psol(path);
        psol.setPlannerName(getName());
        psol.setOptimized(opt_, bestCost_, opt_->isSatisfied(bestCost_));
        pdef_->addSolutionPath(psol);
    }
    else if (approxMotion != nullptr)
    {
        // 近似解回溯：只可能来自起点树（决策 6）
        std::vector<Motion *> mpath;
        for (Motion *m = approxMotion; m != nullptr; m = m->parent)
            mpath.push_back(m);
        auto path(std::make_shared<ompl::geometric::PathGeometric>(si_));
        for (std::size_t i = mpath.size(); i > 0; --i)
            path->append(mpath[i - 1]->state);

        ompl::base::PlannerSolution psol(path);
        psol.setPlannerName(getName());
        psol.setApproximate(approxDist);
        pdef_->addSolutionPath(psol);
    }

    si_->freeState(rmotion->state);
    delete rmotion;
    si_->freeState(xmotion->state);
    delete xmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal), %u connections, final cost %.4f", getName().c_str(),
                static_cast<unsigned int>(tStart_->size() + tGoal_->size()),
                static_cast<unsigned int>(tStart_->size()), static_cast<unsigned int>(tGoal_->size()),
                static_cast<unsigned int>(connectionPoints_.size()), bestCost_.value());

    const bool solved = (bestStartTreeMotion_ != nullptr || approxMotion != nullptr);
    const bool approximate = (bestStartTreeMotion_ == nullptr);
    return {solved, approximate};
}

double BiRRTStar::distanceFunction(const Motion *a, const Motion *b) const
{
    return si_->distance(a->state, b->state);
}

BiRRTStar::Motion *BiRRTStar::addMotion(const ompl::base::State *state, TreeData &tree, Motion *parent,
                                        ompl::base::Cost incCost, ompl::base::Cost cost)
{
    auto *motion = new Motion(si_);
    si_->copyState(motion->state, state);
    motion->parent = parent;
    motion->incCost = incCost;
    motion->cost = cost;
    if (parent != nullptr)
        parent->children.push_back(motion);
    tree->add(motion);
    return motion;
}

BiRRTStar::Motion *BiRRTStar::addMotion(const ompl::base::State *state, TreeData &tree, Motion *parent)
{
    if (parent == nullptr)
        return addMotion(state, tree, parent, opt_->identityCost(), opt_->identityCost());
    const ompl::base::Cost incCost = opt_->motionCost(parent->state, state);
    return addMotion(state, tree, parent, incCost, opt_->combineCosts(parent->cost, incCost));
}

BiRRTStar::GrowState BiRRTStar::extendTree(TreeData &tree, Motion *target, Motion *&nmotion,
                                           ompl::base::State *out_state)
{
    nmotion = tree->nearest(target);
    // 采样点与树上已有节点值重合：本轮无进展（对应 RRTstar.cpp:268 对重合样本的跳过）
    if (si_->equalStates(nmotion->state, target->state))
        return TRAPPED;
    return extendTree(tree, target->state, nmotion, out_state);
}

BiRRTStar::GrowState BiRRTStar::extendTree(TreeData &tree, const ompl::base::State *target, Motion *nmotion,
                                           ompl::base::State *out_state)
{
    // 方向由 tree == tStart_ 推导（不变量 B），绝不依赖任何交换 hack（决策 1）
    const bool treeIsStart = (tree == tStart_);

    // 先假定能直达目标
    bool reach = true;
    const ompl::base::State *dstate = target;

    const double d = si_->distance(nmotion->state, target);
    if (d > maxDistance_)
    {
        // 超过步长则截断插值（steer）
        si_->getStateSpace()->interpolate(nmotion->state, target, maxDistance_ / d, out_state);
        // 插值无进展的保护（RRTConnect.cpp:134 同款；受约束空间可能发生）
        if (si_->equalStates(nmotion->state, out_state))
            return TRAPPED;
        dstate = out_state;
        reach = false;
    }

    // 方向相关的碰撞检查（v2 §1.1，取自 RRTConnect.cpp:138-141）：
    // checkMotion 假定第一个参数已验证；目标树是反向生长的，
    // 必须先验证新点再反向查边
    const bool validMotion = treeIsStart ? si_->checkMotion(nmotion->state, dstate)
                                         : si_->isValid(dstate) && si_->checkMotion(dstate, nmotion->state);
    if (!validMotion)
        return TRAPPED;

    // 决策 7：只把落点写到 out_state，不建节点不入树；REACHED 时落点就是 target
    if (dstate != out_state)
        si_->copyState(out_state, dstate);
    return reach ? REACHED : ADVANCED;
}

bool BiRRTStar::connectTrees(TreeData &tree, Motion *target, ompl::base::State *scratch, Motion *&out_reached)
{
    // 距离target 最近的节点
    Motion *nearest = tree->nearest(target);

    // 复用这次 NN 查询顺手更新两树间距（诊断量，取自 BiTRRT.cpp:393-400）
    const double dist = distanceFunction(nearest, target);
    if (dist < distanceBetweenTrees_)
        distanceBetweenTrees_ = dist;

    // d == 0 短路（v2 §9(4)）：被动树已有节点恰在 target 上，直接视为汇合，
    // 不产生零长重复节点。这是相对三份参考的主动偏离，二维 harness 上验证
    if (si_->equalStates(nearest->state, target->state))
    {
        out_reached = nearest;
        return true;
    }

    while (true)
    {
        const GrowState gs = extendTree(tree, target->state, nearest, scratch);
        if (gs == TRAPPED)
            return false;

        // 贪心入树：没有 chooseParent/rewire，父就是这次的最近邻
        Motion *added = addMotion(scratch, tree, nearest);

        if (gs == REACHED)
        {
            out_reached = added;
            return true;
        }
        // ADVANCED：下一轮从刚加的节点继续，不重查 NN（决策 10，源自 RRTConnect.cpp:315）
        nearest = added;
    }
}

void BiRRTStar::considerConnection(Motion *a, Motion *b)
{
    // REACHED 时两节点同位，桥边是零长运动：combineCosts 与之结合为恒等，
    // 故总代价就是两项相加而非三项（v2 §6.2 注）
    const ompl::base::Cost total = opt_->combineCosts(a->cost, b->cost);
    if (!opt_->isCostBetterThan(total, bestCost_)) // ! total >= bestCost_
        return;

    // 单调性诊断（v2 §9(5)）：bestCost_ 只在 isCostBetterThan 成立时更新，理论上单调不增；
    // 若检出变差说明存在其他写入路径破坏了代价语义（不变量 A 被破坏时会出现）
    if (opt_->isFinite(bestCost_) && opt_->isCostBetterThan(bestCost_, total))
        OMPL_ERROR("%s: bestCost_ increased %.4f -> %.4f (invariant A broken?)", getName().c_str(), bestCost_.value(),
                   total.value());

    bestCost_ = total;
    bestStartTreeMotion_ = a;
    bestGoalTreeMotion_ = b;

    // 中间解回调（v2 §5.5）：同时是首解时间戳与 anytime 事件流的唯一出口
    //（stage_1_ompl_3.md §4.2/§7.3）。spath 约定同 RRTstar：不含 start/goal，顺序 goal→start
    const ompl::base::ReportIntermediateSolutionFn intermediateSolutionCallback =
        pdef_->getIntermediateSolutionCallback();
    if (intermediateSolutionCallback)
    {
        std::vector<const ompl::base::State *> spath;
        // goal 端：b 的父链（不含 b 与 goal 根）
        for (Motion *m = bestGoalTreeMotion_->parent; m != nullptr && m->parent != nullptr; m = m->parent)
            spath.push_back(m->state);
        // start 端：a 的父链（不含 a 与 start 根；a 与 b 同位，只出现一次）
        for (Motion *m = bestStartTreeMotion_->parent; m != nullptr && m->parent != nullptr; m = m->parent)
            spath.push_back(m->state);
        intermediateSolutionCallback(this, spath, bestCost_);
    }
}

void BiRRTStar::getNeighbors(TreeData &tree, Motion *motion, std::vector<Motion *> &nbh) const
{
    // 取自 RRTstar.cpp:603-618，改成按树取容器
    const auto cardDbl = static_cast<double>(tree->size() + 1u);
    if (useKNearest_)
    {
        // k-RRG*：k = k_rrg × log(n)
        const auto k = static_cast<std::size_t>(std::ceil(k_rrt_ * std::log(cardDbl)));
        tree->nearestK(motion, k, nbh);
    }
    else
    {
        // r-RRG*：r = min(range, r_rrg × (log(n)/n)^(1/d))
        const double r = std::min(maxDistance_, r_rrt_ * std::pow(std::log(cardDbl) / cardDbl,
                                                                  1.0 / static_cast<double>(si_->getStateDimension())));
        tree->nearestR(motion, r, nbh);
    }
}

BiRRTStar::Motion *BiRRTStar::chooseParent(Motion *xmotion, Motion *nmotion, std::vector<Motion *> &nbh,
                                           ompl::base::Cost &out_incCost, ompl::base::Cost &out_cost)
{
    // cache 数组只增不减：按需扩容后只重置/使用前 nbh.size() 个（v2 §5.3(3)）
    if (costs_.size() < nbh.size())
    {
        costs_.resize(nbh.size());
        incCosts_.resize(nbh.size());
        sortedCostIndices_.resize(nbh.size());
    }
    if (valid_.size() < nbh.size())
        valid_.resize(nbh.size());
    std::fill(valid_.begin(), valid_.begin() + nbh.size(), 0);

    // nmotion 兜底（v2 §5.3(1)）：这条边在 extendTree 里已经 checkMotion 过；
    // r-disc 模式下新节点到 nmotion 的距离恰等于 maxDistance_，nmotion 可能不在 nbh 里，
    // 若不兜底会导致新节点无父（原 bug：落盘于 RRTstar 早期版本）
    out_incCost = opt_->motionCost(nmotion->state, xmotion->state);
    out_cost = opt_->combineCosts(nmotion->cost, out_incCost);
    Motion *parent = nmotion;

    // delayCC 路径（决策 9 硬编码）：全量算代价 → 按代价排序 → 依序查碰撞，
    // 第一个无碰撞的就用（取自 RRTstar.cpp:319-358）
    for (std::size_t i = 0; i < nbh.size(); ++i)
    {
        incCosts_[i] = opt_->motionCost(nbh[i]->state, xmotion->state);
        costs_[i] = opt_->combineCosts(nbh[i]->cost, incCosts_[i]);
    }
    for (std::size_t i = 0; i < nbh.size(); ++i)
        sortedCostIndices_[i] = i;
    CostIndexCompare compareFn(costs_, *opt_);
    std::sort(sortedCostIndices_.begin(), sortedCostIndices_.begin() + nbh.size(), compareFn);

    for (auto i = sortedCostIndices_.begin(); i != sortedCostIndices_.begin() + nbh.size(); ++i)
    {
        if (nbh[*i] == nmotion ||
            ((!useKNearest_ || si_->distance(nbh[*i]->state, xmotion->state) < maxDistance_) &&
             si_->checkMotion(nbh[*i]->state, xmotion->state)))
        {
            out_incCost = incCosts_[*i];
            out_cost = costs_[*i];
            parent = nbh[*i];
            valid_[*i] = 1;
            break;
        }
        else
        {
            valid_[*i] = -1;
        }
    }

    return parent;
}

void BiRRTStar::rewire(Motion *motion, std::vector<Motion *> &nbh) // motion为新节点，rewire this motion 
{
    // 必须紧接在同一个 nbh 上的 chooseParent 之后调用（v2 §5.3(2)）：
    // 本函数复用 incCosts_ 与 valid_ 的结论，顺序换了结果就是错的
    const bool symCost = opt_->isSymmetric();
    for (std::size_t i = 0; i < nbh.size(); ++i)
    {
        if (nbh[i] != motion->parent)
        {
            ompl::base::Cost nbhIncCost; // 计算 motion 到 nbh[i]的 incCost
            if (symCost)
                nbhIncCost = incCosts_[i];  // 对称 objective 直接复用（v2 §7.4）
            else
                nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
            // 计算 motion 到 nbh[i] 的 cost
            const ompl::base::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
            if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost)) // 对比
            {
                bool motionValid;
                if (valid_[i] == 0) // chooseparent未计算，重新计算
                {
                    // r-disc 模式下仅当边长 < maxDistance_ 才考虑重连
                    motionValid = (!useKNearest_ || si_->distance(nbh[i]->state, motion->state) < maxDistance_) &&
                                  si_->checkMotion(motion->state, nbh[i]->state);
                }
                else
                {
                    // chooseParent 已查过这条边（三态 cache：1=无碰撞）
                    motionValid = (valid_[i] == 1);
                }

                if (motionValid) // valid合法，为nbh[i]设置新父motion，并更新子节点
                {
                    removeFromParent(nbh[i]);
                    nbh[i]->parent = motion;
                    nbh[i]->incCost = nbhIncCost;
                    nbh[i]->cost = nbhNewCost;
                    nbh[i]->parent->children.push_back(nbh[i]);
                    updateChildCosts(nbh[i]);
                }
            }
        }
    }
}

void BiRRTStar::removeFromParent(Motion *m)
{
    // 取自 RRTstar.cpp:620-627
    for (auto it = m->parent->children.begin(); it != m->parent->children.end(); ++it)
    {
        if (*it == m)
        {
            m->parent->children.erase(it);
            break;
        }
    }
}

void BiRRTStar::updateChildCosts(Motion *m)
{
    // 取自 RRTstar.cpp:629-639
    for (std::size_t i = 0; i < m->children.size(); ++i)
    {
        m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
        updateChildCosts(m->children[i]);
    }
}

void BiRRTStar::calculateRewiringLowerBounds()
{
    // 取自 RRTstar.cpp:1147-1159；prunedMeasure_ → si_->getSpaceMeasure()
    const auto dimDbl = static_cast<double>(si_->getStateDimension());
    // k-RRG 下界：k_rrg = s × 2^(d+1) × e × (1 + 1/d)
    k_rrt_ = rewireFactor_ *
             (std::pow(2.0, dimDbl + 1.0) * boost::math::constants::e<double>() * (1.0 + 1.0 / dimDbl));
    // r-RRG 下界：r_rrg = s × (2 × (1+1/d) × μ(X)/μ_unit_ball)^(1/d)
    r_rrt_ = rewireFactor_ *
             std::pow(2.0 * (1.0 + 1.0 / dimDbl) *
                          (si_->getSpaceMeasure() / ompl::unitNBallMeasure(si_->getStateDimension())),
                      1.0 / dimDbl);
}

ompl::base::PathPtr BiRRTStar::constructSolutionPath(Motion *a, Motion *b) const
{
    // REACHED 保证 a->state 与 b->state 值相等。三份双树参考都是"记录时退一步"（保存
    // a->parent），我们不能照抄：a->parent 会被后续 rewire 改掉。原样记录 (a,b)，
    // 输出时沿当前父链回溯 + 去重（v2 §5.4，决策 5 的另一半）
    std::vector<Motion *> mpath1;
    for (Motion *m = a; m != nullptr; m = m->parent)
        mpath1.push_back(m);
    std::vector<Motion *> mpath2;
    for (Motion *m = b->parent; m != nullptr; m = m->parent)  // 跳过 b 本身（与 a 同位）
        mpath2.push_back(m);

    auto path(std::make_shared<ompl::geometric::PathGeometric>(si_));
    path->getStates().reserve(mpath1.size() + mpath2.size());
    for (std::size_t i = mpath1.size(); i > 0; --i)  // start 根 → a
        path->append(mpath1[i - 1]->state);
    for (auto *m : mpath2)  // → goal 根
        path->append(m->state);
    return path;
}

void BiRRTStar::getPlannerData(ompl::base::PlannerData &data) const
{
    // v2 §6.6：tag 1 = 起点树，2 = 目标树；目标树的边反向添加，整图统一 start→goal 方向
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);
    for (auto *motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(ompl::base::PlannerDataVertex(motion->state, 1));
        else
            data.addEdge(ompl::base::PlannerDataVertex(motion->parent->state, 1),
                         ompl::base::PlannerDataVertex(motion->state, 1));
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);
    for (auto *motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(ompl::base::PlannerDataVertex(motion->state, 2));
        else
            // 目标树的边反向添加
            data.addEdge(ompl::base::PlannerDataVertex(motion->state, 2),
                         ompl::base::PlannerDataVertex(motion->parent->state, 2));
    }

    // 两树的汇合边：RRTConnect 末尾是无条件 addEdge，我们有多对，全部加；
    // 先判空（vertexIndex 对不存在的顶点会断言失败）
    for (const auto &cp : connectionPoints_)
    {
        data.addEdge(data.vertexIndex(ompl::base::PlannerDataVertex(cp.startTreeMotion->state, 1)),
                     data.vertexIndex(ompl::base::PlannerDataVertex(cp.goalTreeMotion->state, 2)));
    }

    data.properties["approx goal distance REAL"] = ompl::toString(distanceBetweenTrees_);
}
}  // namespace ar4_my_planners
