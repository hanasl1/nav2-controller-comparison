#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "nav2_circle_planner/circle_planner.hpp"

namespace nav2_circle_planner
{

void Circle::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  
  initial_position_set_ = false; // Flag to indicate if initial position has been set
  initial_position_ = geometry_msgs::msg::Point(); // Initialize initial position to (0, 0, 0)

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void Circle::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void Circle::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void Circle::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path Circle::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
 

  // Checking if the goal and start state are in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only accept start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only accept goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  // If the initial position has not been set yet, store it
  if (!initial_position_set_) {
    initial_position_ = start.pose.position;
    initial_position_set_ = true;
  }

  // Calculate circle parameters using the stored initial position
  double center_x = (initial_position_.x + goal.pose.position.x) / 2.0;
  double center_y = (initial_position_.y + goal.pose.position.y) / 2.0;
  double radius = std::sqrt(std::pow(goal.pose.position.x - center_x, 2) +
                            std::pow(goal.pose.position.y - center_y, 2));
  double theta_start = std::atan2(initial_position_.y - center_y, initial_position_.x - center_x);

  // Calculate angle between start and goal positions
  double angle_diff = std::atan2(
      goal.pose.position.y - initial_position_.y,
      goal.pose.position.x - initial_position_.x);

  // Ensure the angle difference is positive
  if (angle_diff < 0) {
      angle_diff += 2 * M_PI;
  }

  
  //double theta_end = theta_start + angle_diff;

 
  int circle_points = 200; 
  //double theta_increment = (theta_end - theta_start) / circle_points;

  // Generate circular path
  for (int i = 0; i < circle_points-1; ++i) {
    double theta = theta_start + i * (6.25/100 );
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = center_x + radius * std::cos(theta);
    pose.pose.position.y = center_y + radius * std::sin(theta);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  return global_path;
}

}  // namespace nav2_Circle_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_circle_planner::Circle, nav2_core::GlobalPlanner)
