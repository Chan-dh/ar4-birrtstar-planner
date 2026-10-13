#pragma once

#include <atomic>

#include <moveit/planning_interface/planning_interface.hpp>

namespace ar4_my_planners
{

class MyPlanningContext : public planning_interface::PlanningContext
{
public:
  MyPlanningContext(const std::string& name, const std::string& group);

  void solve(planning_interface::MotionPlanResponse& response) override;
  void solve(planning_interface::MotionPlanDetailedResponse& response) override;
  bool terminate() override;
  void clear() override;

private:
  std::atomic_bool terminate_requested_{false};
};

}  // namespace ar4_my_planners
