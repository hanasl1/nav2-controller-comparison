#include <algorithm>
#include "nav2_line_of_sight_controller/los_controller.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;

namespace nav2_line_of_sight_controller
{
void LineOfSightController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // Your implementation here
  (void)parent;
  (void)name;
  (void)tf;
  (void)costmap_ros;
}

void LineOfSightController::cleanup()
{
  // Your implementation here
}

void LineOfSightController::activate()
{
  // Your implementation here
}

void LineOfSightController::deactivate()
{
  // Your implementation here
}

void LineOfSightController::setPlan(const nav_msgs::msg::Path & path)
{
  // Your implementation here
  (void)path;
}


geometry_msgs::msg::TwistStamped LineOfSightController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // Your implementation here
  (void)pose;
  (void)velocity;
  (void)goal_checker;
  geometry_msgs::msg::TwistStamped cmd_vel;
  return cmd_vel;
}

void LineOfSightController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  // Your implementation here
  (void)speed_limit;
  (void)percentage;
}
}  // namespace nav2_line_of_sight_controller
