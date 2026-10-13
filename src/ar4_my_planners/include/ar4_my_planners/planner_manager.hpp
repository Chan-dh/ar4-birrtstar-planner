#pragma once

#include <memory>
#include <string>
#include <vector>

#include <moveit/planning_interface/planning_interface.hpp>

namespace ar4_my_planners
{

class MyPlannerManager : public planning_interface::PlannerManager
{
public:
  bool initialize(const moveit::core::RobotModelConstPtr& model,
                  const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override;

  std::string getDescription() const override;

  void getPlanningAlgorithms(std::vector<std::string>& algorithms) const override;

  planning_interface::PlanningContextPtr getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& request,
    moveit_msgs::msg::MoveItErrorCodes& error_code) const override;

  bool canServiceRequest(const planning_interface::MotionPlanRequest& request) const override;

private:
  moveit::core::RobotModelConstPtr model_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace ar4_my_planners
