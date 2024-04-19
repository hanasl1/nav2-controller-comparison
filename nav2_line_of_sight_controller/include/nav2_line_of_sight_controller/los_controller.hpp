/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#ifndef NAV2_LINE_OF_SIGHT_CONTROLLER__LOS_CONTROLLER_HPP_
#define NAV2_LINE_OF_SIGHT_CONTROLLER__LOS_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <algorithm>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"

namespace nav2_line_of_sight_controller
{
class LineOfSightController : public nav2_core::Controller
{
public:
  LineOfSightController() = default;
  virtual ~LineOfSightController() {}

  /**
   * @brief Configures controller parameters and initializes resources
   * @param parent pointer to user's node
   * @param name The name of the controller
   * @param tf Shared pointer to the transform buffer
   * @param costmap_ros Shared pointer to the costmap
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleans up controller resources
   */
  void cleanup() override;

  /**
   * @brief Activates controller
   */
  void activate() override;

  /**
   * @brief Deactivates controller
   */
  void deactivate() override;

  /**
   * @brief Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Calculates the best velocity command
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @param goal_checker Pointer to the current goal checker
   * @return The best command for the robot to drive
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief Sets the speed limit of the robot
   * @param speed_limit The speed limit in absolute value (m/s) or percentage from maximum robot speed
   * @param percentage If true, sets the speed limit in percentage, otherwise sets it in absolute value
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief Resets the state of the controller
   */
    

private:
  /**
   * @brief Transforms the global plan into the robot's frame of reference
   * @param path The global plan
   * @return The transformed global plan
   */
  nav_msgs::msg::Path transformGlobalPlan(const nav_msgs::msg::Path & path);

  /**
   * @brief Transforms a pose from one frame to another
   * @param tf Shared pointer to the transform buffer
   * @param frame The target frame
   * @param in_pose The input pose
   * @param out_pose The output pose
   * @param transform_tolerance The transform tolerance duration
   * @return True if the transformation was successful, false otherwise
   */
  bool transformPose(
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose,
    rclcpp::Duration & transform_tolerance);

  // Member variables
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  rclcpp::Logger logger_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Clock::SharedPtr clock_;
  double desired_linear_vel_;
  double lookahead_dist_;
  double max_angular_vel_;
  rclcpp::Duration transform_tolerance_;
  nav_msgs::msg::Path global_plan_;
};

}  // namespace nav2_line_of_sight_controller

#endif  // NAV2_LINE_OF_SIGHT_CONTROLLER__LOS_CONTROLLER_HPP_
