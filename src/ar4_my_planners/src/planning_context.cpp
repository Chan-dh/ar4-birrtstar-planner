#include "ar4_my_planners/planning_context.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.hpp>

#include <rclcpp/logging.hpp>

#include <ompl/base/PlannerStatus.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "ar4_my_planners/birrtstar.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ar4_my_planners
{
namespace
{

// stage_1_ompl_3.md §5.2：唯一必须自己写的适配件。
// 每次 solve() 各自构造一份，RobotState 绝不跨请求共享（RobotState 非线程安全）。
class MyValidityChecker : public ompl::base::StateValidityChecker
{
public:
  MyValidityChecker(const ompl::base::SpaceInformationPtr& si,
                    planning_scene::PlanningSceneConstPtr scene,
                    ompl_interface::ModelBasedStateSpacePtr space,
                    std::string group)
  : ompl::base::StateValidityChecker(si)
  , scene_(std::move(scene))
  , space_(std::move(space))
  , group_(std::move(group))
  , state_(scene_->getCurrentState())
  {
  }

  bool isValid(const ompl::base::State* state) const override
  {
    space_->copyToRobotState(state_, state);
    return scene_->isStateValid(state_, group_);
  }

private:
  planning_scene::PlanningSceneConstPtr scene_;
  ompl_interface::ModelBasedStateSpacePtr space_;
  std::string group_;
  mutable moveit::core::RobotState state_;  // 本 checker 私有，绝不共享
};

}  // namespace

MyPlanningContext::MyPlanningContext(const std::string& name, const std::string& group)
: planning_interface::PlanningContext(name, group)
{
}

void MyPlanningContext::solve(planning_interface::MotionPlanResponse& response)
{
  const auto started = std::chrono::steady_clock::now();
  auto elapsed = [&] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  };

  response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
  response.trajectory.reset();
  response.planner_id = request_.planner_id;

  if (!planning_scene_ || request_.goal_constraints.size() != 1) {
    response.planning_time = elapsed();  // 失败分支也填计时（§5.5）
    return;
  }

  const auto* joint_group = planning_scene_->getRobotModel()->getJointModelGroup(group_);
  if (!joint_group) {
    response.planning_time = elapsed();
    return;
  }

  // ---- §5.1 搭 SimpleSetup：state space + validity checker + lvsf
  const ompl_interface::ModelBasedStateSpaceSpecification spec(
    planning_scene_->getRobotModel(), joint_group);
  auto space = std::make_shared<ompl_interface::JointModelStateSpace>(spec);
  space->setup();  // 关节限位从 jmg 自动来，不硬编码
  // 搭建ompl规划环境
  auto ss = std::make_shared<og::SimpleSetup>(space);
  ss->setStateValidityChecker(std::make_shared<MyValidityChecker>(
    ss->getSpaceInformation(), planning_scene_, space, group_));
  ss->getSpaceInformation()->setStateValidityCheckingResolution(0.005);  // lvsf，对齐基线

  // ---- §3.2 planner 查表：request_.planner_id 分发。
  //      与 MyPlannerManager::getPlanningAlgorithms() 用同一张表（RViz 下拉框读它）
  ompl::base::PlannerPtr planner;
  if (request_.planner_id == "RRTstar")
    planner = std::make_shared<og::RRTstar>(ss->getSpaceInformation());
  else if (request_.planner_id == "BiRRTStar")
    planner = std::make_shared<BiRRTStar>(ss->getSpaceInformation());  // 同一份代码，Phase B 二维已验证
  else  // 空/未知 planner_id → RRTConnect 基线
    planner = std::make_shared<og::RRTConnect>(ss->getSpaceInformation());
  ss->setPlanner(planner);

  // ---- 固定 start 和 joint goal（沿用 Stage 0 的解析口径）
  moveit::core::RobotState start(planning_scene_->getCurrentState());
  moveit::core::robotStateMsgToRobotState(request_.start_state, start);
  start.update();

  const auto& joint_names = joint_group->getVariableNames();
  std::unordered_map<std::string, double> goal_values;
  for (const auto& constraint : request_.goal_constraints.front().joint_constraints)
    goal_values[constraint.joint_name] = constraint.position;

  std::vector<double> goal_values_ordered;
  goal_values_ordered.reserve(joint_names.size());
  for (const auto& name : joint_names) {
    const auto it = goal_values.find(name);
    if (it == goal_values.end()) {
      response.planning_time = elapsed();
      return;  // 目标约束不完整
    }
    goal_values_ordered.push_back(it->second);
  }

  moveit::core::RobotState goal(planning_scene_->getCurrentState());
  goal.setJointGroupPositions(joint_group, goal_values_ordered);
  goal.update();

  ompl::base::ScopedState<> ompl_start(space);
  ompl::base::ScopedState<> ompl_goal(space);
  space->copyToOMPLState(ompl_start.get(), start);
  space->copyToOMPLState(ompl_goal.get(), goal);
  ss->setStartAndGoalStates(ompl_start, ompl_goal);

  // ---- §4.2 首解时间 + 收敛事件流：中间解回调是唯一出口（stage_1_ompl_3.md §4.2/§4.3）。
  //      RRTConnect 找到即停、不触发回调，t_first 保持 -1（口径：≈ planning_time）；
  //      RRTstar 实测会触发（2026-09-27：0.007~0.039s）；BiRRTStar 每次刷新 c_best 必调。
  //      决定（2026-09-27）：t_first 不进 CSV —— 逐事件打 (t, cost) 日志，
  //      分析期用 CSV 的 t_request 墙钟列与日志 ROS 时间戳对齐提取；
  //      事件流本身就是 cost-vs-time 收敛曲线（v2 §10(2)）。
  double t_first_solution = -1.0;
  const std::string planner_tag =
    request_.planner_id.empty() ? std::string("RRTConnect") : request_.planner_id;
  ss->getProblemDefinition()->setIntermediateSolutionCallback(
    [&elapsed, &t_first_solution, &planner_tag](const ob::Planner*,
        const std::vector<const ob::State*>&, ob::Cost cost) {
      const double t = elapsed();
      if (t_first_solution < 0.0)
        t_first_solution = t;  // 首次触发 = 首解
      RCLCPP_INFO(rclcpp::get_logger("ar4_my_planners"),
                  "[improved_solution] planner=%s t=%.6f cost=%.6f",
                  planner_tag.c_str(), t, cost.value());
    });

  // ---- §5.3 终止条件：deadline + terminate() 标志（算法只能问"该停了吗"）
  const auto deadline = started +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(request_.allowed_planning_time));
  ss->setup();
  const ompl::base::PlannerStatus status = ss->solve(
    ompl::base::PlannerTerminationCondition([this, deadline] {
      return terminate_requested_.load(std::memory_order_relaxed) ||
             std::chrono::steady_clock::now() >= deadline;
    }));

  // 首解时间出口（打进日志，分析期按 t_request 时间窗对齐提取）。
  // tfirst=-1 表示未触发回调（RRTConnect 属预期，口径：t_first ≈ planning_time）
  RCLCPP_INFO(rclcpp::get_logger("ar4_my_planners"),
              "[t_first_solution] planner=%s tfirst=%.6f status=%s",
              planner_tag.c_str(), t_first_solution, status.asString().c_str());

  if (status != ompl::base::PlannerStatus::EXACT_SOLUTION) {
    response.planning_time = elapsed();
    return;
  }

  // ---- §5.4 解路径：interpolate() 稠密化——ValidateSolution 只查航点不插值；
  //      绝不调 simplifySolution()，避免后处理改变几何路径（Stage 0 约定）
  auto& path = ss->getSolutionPath();
  path.interpolate();

  auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(
    planning_scene_->getRobotModel(), group_);
  moveit::core::RobotState waypoint(planning_scene_->getCurrentState());
  for (const auto* state : path.getStates()) {
    space->copyToRobotState(waypoint, state);
    waypoint.update();
    trajectory->addSuffixWayPoint(waypoint, 0.0);
  }

  response.trajectory = trajectory;
  response.planning_time = elapsed();
  response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

void MyPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& response)
{
  planning_interface::MotionPlanResponse simple_response;
  solve(simple_response);
  response.trajectory.clear();
  response.description.clear();
  response.processing_time.clear();
  if (simple_response.trajectory) {
    response.trajectory.push_back(simple_response.trajectory);
    response.description.push_back(
      simple_response.planner_id.empty() ? "RRTConnect" : simple_response.planner_id);
    response.processing_time.push_back(simple_response.planning_time);
  }
  response.error_code.val = simple_response.error_code.val;
  response.planner_id = simple_response.planner_id;
}

bool MyPlanningContext::terminate()
{
  terminate_requested_ = true;  // 只置标志，绝不清数据结构（§5.3 硬约束）
  return true;
}

void MyPlanningContext::clear()
{
  terminate_requested_ = false;
}

}  // namespace ar4_my_planners
