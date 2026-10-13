/*********************************************************************
 * 二维 BiRRTStar 验证程序（Phase B）
 *
 * 依据：docs/stage_1_ompl_3.md §5 Phase B、§7.2（树/路径导出）、§7.3（c_best 断言）；
 *      docs/birrtstar设计_v2.md §11（算法层零 ROS 依赖，诊断一律 fprintf）
 *
 * 只依赖 OMPL 与 C 标准库。四个场景：
 *   free     无障碍（验证基本寻路）
 *   obstacle 中央矩形障碍（验证绕行 + 路径不穿障碍）
 *   narrow   窄通道（验证双树连接）
 *   circles  对角线上大小不一的圆阵（验证绕行多个凸障碍）
 *
 * 用法：birrtstar_2d [scenario] [time_limit_s] [out_dir]
 *   scenario ∈ {free, obstacle, narrow, circles}，默认 free
 *   time_limit_s 默认 2.0
 *   out_dir   默认当前目录，写 <scenario>_trees.txt
 *
 * 自检项（stage_1_ompl_3.md §5 Phase B 验收）：
 *   [S1] 返回精确解
 *   [S2] 路径逐航点 isValid、逐段 checkMotion（不穿障碍）
 *   [S3] 路径首点是 start、末点是 goal
 *   [S4] 路径代价 == bestCost_（rewiring 后代价一致性）
 *   [S5] c_best 严格递减断言全程不触发
 * 
 * for s in free obstacle narrow circles; do
 * ./build/ar4_my_planners/birrtstar_2d $s 2.0 /tmp
 * python3 bench/plot_birrtstar_trees.py $s /tmp
 * done

colcon build --packages-select ar4_my_planners
./build/ar4_my_planners/birrtstar_2d circles 2.0 /tmp
python3 bench/plot_birrtstar_trees.py circles /tmp
 *********************************************************************/

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include <ompl/base/PlannerData.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/PathGeometric.h>

#include "ar4_my_planners/birrtstar.hpp"

namespace
{
struct Rect
{
    double x0, y0, x1, y1;
};

struct Circle
{
    double cx, cy, r;
};

struct Scenario
{
    const char *name;
    double start[2];
    double goal[2];
    std::vector<Rect> obstacles;
    std::vector<Circle> circles;
};

// 场景表：bounds 统一 [0,100]²
const std::vector<Scenario> &scenarios()
{
    static const std::vector<Scenario> s = {
        {"free", {5.0, 5.0}, {95.0, 95.0}, {}},
        {"obstacle", {5.0, 5.0}, {95.0, 95.0}, {{30.0, 30.0, 70.0, 70.0}}},
        {"narrow", {5.0, 5.0}, {95.0, 95.0}, {{48.0, 0.0, 52.0, 26.0}, {48.0, 34.0, 52.0, 100.0}}},
        {"circles", {5.0, 5.0}, {95.0, 95.0}, {},
         {{18.0, 18.0, 6.0}, {24.0, 45.0, 16.0}, {50.0, 60.0, 7.0}, {70.0, 65.0, 10.0}, {85.0, 80.0, 6.0}}},
    };
    return s;
}

class RectChecker : public ompl::base::StateValidityChecker
{
public:
    RectChecker(const ompl::base::SpaceInformationPtr &si, std::vector<Rect> obstacles,
                std::vector<Circle> circles)
      : ompl::base::StateValidityChecker(si), obstacles_(std::move(obstacles)), circles_(std::move(circles))
    {
    }

    bool isValid(const ompl::base::State *state) const override
    {
        const auto *s = state->as<ompl::base::RealVectorStateSpace::StateType>();
        const double x = s->values[0];
        const double y = s->values[1];
        for (const auto &r : obstacles_)
            if (x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1)
                return false;
        for (const auto &c : circles_)
            if ((x - c.cx) * (x - c.cx) + (y - c.cy) * (y - c.cy) <= c.r * c.r)
                return false;
        return true;
    }

private:
    std::vector<Rect> obstacles_;
    std::vector<Circle> circles_;
};

int g_checksFailed = 0;

void check(bool ok, const char *label, const char *detail = "")
{
    std::fprintf(stdout, "[S] %-4s %s%s%s\n", ok ? "PASS" : "FAIL", label, *detail ? " — " : "", detail);
    if (!ok)
        ++g_checksFailed;
}
}  // namespace

int main(int argc, char **argv)
{
    const char *scenarioName = argc > 1 ? argv[1] : "free";
    const double timeLimit = argc > 2 ? std::atof(argv[2]) : 2.0;
    const std::string outDir = argc > 3 ? argv[3] : ".";

    const Scenario *sc = nullptr;
    for (const auto &candidate : scenarios())
        if (std::strcmp(candidate.name, scenarioName) == 0)
            sc = &candidate;
    if (sc == nullptr)
    {
        std::fprintf(stderr, "unknown scenario '%s' (use free|obstacle|narrow|circles)\n", scenarioName);
        return 2;
    }

    // ── 空间与碰撞检查（stage_1_ompl_3.md §1.3：只替换这四样即可从二维换到六维）──
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(2);
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0.0);
    bounds.setHigh(100.0);
    space->setBounds(bounds);

    auto si = std::make_shared<ompl::base::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<RectChecker>(si, sc->obstacles, sc->circles));
    si->setStateValidityCheckingResolution(0.01);  // lvsf，对齐基线口径
    si->setup();

    ompl::base::ScopedState<> start(space), goal(space);
    start[0] = sc->start[0];
    start[1] = sc->start[1];
    goal[0] = sc->goal[0];
    goal[1] = sc->goal[1];

    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);

    // ── 首解时间戳 + c_best 严格递减断言（§4.2 / §7.3）──────────────
    // 中间解回调是唯一出口：首解时间、c_best 事件流、anytime 曲线共用它
    const auto t0 = std::chrono::steady_clock::now();
    auto elapsed = [&t0] {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    };

    double tFirstSolution = -1.0;
    double previousBest = std::numeric_limits<double>::infinity();
    int cBestViolations = 0; //c_best 递减规则被违反的次数
    int cBestEvents = 0; // 收到 c_best 更新事件的次数

    pdef->setIntermediateSolutionCallback(
        [&](const ompl::base::Planner *, const std::vector<const ompl::base::State *> & /*states*/,
            const ompl::base::Cost cost) {
            const double now = elapsed();
            if (tFirstSolution < 0.0)
                tFirstSolution = now;  // 首次触发 = 首解
            // 严格递减断言：违反即 bug，没有别的解释（§7.3）
            if (cost.value() >= previousBest)
            {
                ++cBestViolations;
                std::fprintf(stderr, "[c_best] VIOLATION %.4f -> %.4f at t=%.3f\n", previousBest, cost.value(), now);
            }
            previousBest = cost.value();
            ++cBestEvents;
            std::fprintf(stderr, "[c_best] t=%.3f cost=%.4f\n", now, cost.value());
        });

    // ── 规划器：同一份 BiRRTStar，未来六维接入只换 si 的四个部件 ──────
    auto planner = std::make_shared<ar4_my_planners::BiRRTStar>(si);
    planner->setProblemDefinition(pdef);
    planner->setup();

    const ompl::base::PlannerTerminationCondition ptc(
        [&elapsed, timeLimit] { return elapsed() >= timeLimit; });

    const ompl::base::PlannerStatus status = planner->solve(ptc);
    const double planningTime = elapsed();

    // ── 结果汇总 ────────────────────────────────────────────────────
    std::fprintf(stdout, "scenario=%s status=%s t_first=%.4f planning_time=%.4f c_best=%.4f iterations=%u\n",
                 sc->name, status.asString().c_str(), tFirstSolution, planningTime, planner->bestCost().value(),
                 planner->numIterations());

    const bool exact = (status == ompl::base::PlannerStatus::EXACT_SOLUTION);
    check(exact, "exact solution");

    if (exact)
    {
        auto *path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
        // 得到最终路径上的状态点
        const std::vector<ompl::base::State *> &states = path->getStates();

        // S2：逐航点 + 逐段碰撞检查（不穿障碍）
        bool pathValid = true;
        for (const auto *st : states)
        {
            if (!si->isValid(st))
            {
                pathValid = false;
                break;
            }
        }
        for (std::size_t i = 1; pathValid && i < states.size(); ++i)
        {
            if (!si->checkMotion(states[i - 1], states[i]))
                pathValid = false;
        }
        check(pathValid, "path collision-free");

        // S3：首尾端点
        const auto *first = states.front()->as<ompl::base::RealVectorStateSpace::StateType>();
        const auto *last = states.back()->as<ompl::base::RealVectorStateSpace::StateType>();
        // hypot 两个点之间的欧式距离
        const bool endpoints = std::hypot(first->values[0] - sc->start[0], first->values[1] - sc->start[1]) < 1e-6 &&
                                std::hypot(last->values[0] - sc->goal[0], last->values[1] - sc->goal[1]) < 1e-6;
        check(endpoints, "start/goal endpoints");

        // S4：rewiring 后代价一致性——路径实际代价与 bestCost_ 应相等（对称 objective）
        // pathCost 计算路径的实际Cost
        const double pathCost = path->cost(pdef->getOptimizationObjective()).value();
        // BiRRTStar 内部记录的 bestCost。
        const double best = planner->bestCost().value();
        // 存在浮点数，tolerance == 10**-6 MAX（1, best）
        // best = 120, tolerance = 1e-6 * 120 = 0.00012
        const bool costConsistent = std::fabs(pathCost - best) < 1e-6 * std::max(1.0, best);
        check(costConsistent, "path cost == bestCost");
        if (!costConsistent)
            std::fprintf(stdout, "  path cost %.6f vs bestCost %.6f\n", pathCost, best);

        // ── 树/路径导出（§7.2）：V/E 带 tag（1=起点树 2=目标树），P 为最终路径 ──
        // 导出规划树
        // 向 BiRRTStar 要它内部的规划树数据。 Vertex， edge
        ompl::base::PlannerData pdata(si);
        planner->getPlannerData(pdata);
        // 路径 /tmp/scenarioName_trees.txt
        const std::string treesFile = outDir + "/" + sc->name + "_trees.txt";
        if (FILE *tf = std::fopen(treesFile.c_str(), "w"))
        {
            for (unsigned int i = 0; i < pdata.numVertices(); ++i)
            {
                // v 是第 i 个顶点
                const ompl::base::PlannerDataVertex &v = pdata.getVertex(i);
                // state （x，y）
                const auto *s = v.getState()->as<ompl::base::RealVectorStateSpace::StateType>();
                // V  节点ID  x  y  tag
                std::fprintf(tf, "V %u %f %f %d\n", i, s->values[0], s->values[1], v.getTag());
                std::vector<unsigned int> edgeList;
                // 这个节点的 Edge list
                pdata.getEdges(i, edgeList);
                for (unsigned int j : edgeList)
                    // E  节点ID  子节点ID  tag
                    std::fprintf(tf, "E %u %u %d\n", i, j, v.getTag());
            }
            // 
            for (const auto *st : states)
            {
                // P  x  y
                const auto *s = st->as<ompl::base::RealVectorStateSpace::StateType>();
                std::fprintf(tf, "P %f %f\n", s->values[0], s->values[1]);
            }
            std::fclose(tf);
            std::fprintf(stdout, "exported %u vertices to %s\n", pdata.numVertices(), treesFile.c_str());
        }
        else
        {
            std::fprintf(stderr, "cannot open %s for writing\n", treesFile.c_str());
        }
    }

    // S5：c_best 严格递减断言
    check(cBestViolations == 0, "c_best strictly decreasing");

    std::fprintf(stdout, "RESULT scenario=%s checks_failed=%d c_best_events=%d t_first=%.4f\n", sc->name, g_checksFailed,
                 cBestEvents, tFirstSolution);
    return g_checksFailed == 0 ? 0 : 1;
}
