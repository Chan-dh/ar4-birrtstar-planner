#include "ar4_my_planners/planner_manager.hpp"

#include "ar4_my_planners/planning_context.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace ar4_my_planners
{

bool MyPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model,
                                  const rclcpp::Node::SharedPtr& node,
                                  const std::string&)
{
  model_ = model;
  node_ = node;
  return static_cast<bool>(model_);
}

std::string MyPlannerManager::getDescription() const
{
  return "AR4 OMPL adapter (RRTConnect / RRTstar / BiRRTStar)";
}

void MyPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algorithms) const
{
  // stage_1_ompl_3.md §3.2：与 MyPlanningContext::solve 的分发用同一张表
  algorithms = {"RRTConnect", "RRTstar", "BiRRTStar"};
}

planning_interface::PlanningContextPtr MyPlannerManager::getPlanningContext(
  const planning_scene::PlanningSceneConstPtr& planning_scene,
  const planning_interface::MotionPlanRequest& request,
  moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  if (!canServiceRequest(request) || !planning_scene) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return nullptr;
  }

  auto context = std::make_shared<MyPlanningContext>(
    request.planner_id.empty() ? "EmptyPlanner" : request.planner_id,
    request.group_name);
  context->setPlanningScene(planning_scene);
  context->setMotionPlanRequest(request);
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return context;
}

bool MyPlannerManager::canServiceRequest(const planning_interface::MotionPlanRequest& request) const
{
  if (request.group_name != "ar_manipulator" || request.goal_constraints.size() != 1) {
    return false;
  }
  const auto& goal = request.goal_constraints.front();
  return !goal.joint_constraints.empty() &&
         goal.position_constraints.empty() &&
         goal.orientation_constraints.empty();
}

}  // namespace ar4_my_planners

PLUGINLIB_EXPORT_CLASS(ar4_my_planners::MyPlannerManager, planning_interface::PlannerManager)
