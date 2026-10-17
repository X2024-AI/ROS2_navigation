#ifndef WR_BRINGUP__STANLEYCONTROLLER_HPP_
#define WR_BRINGUP__STANLEYCONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace wr_bringup
{

// Simplified Stanley controller (Hoffman et al., DARPA Grand Challenge)
// adapted to a differential-drive base:
//
//   delta = heading_error + atan2(k_e * e, |v| + k_soft)
//
//   heading_error : path tangent yaw minus robot yaw (turn toward path)
//   e             : signed cross-track of the FRONT AXLE point vs the path
//                   (turn back toward the path; normalised by speed)
//
// The steering angle delta becomes a yaw-rate command (w = k_w * delta),
// clamped; large heading errors trigger a rotate-to-heading phase; speed
// slows with |delta| and near the end of the path.
class StanleyController : public nav2_core::Controller
{
public:
  StanleyController() = default;
  ~StanleyController() override = default;

  // nav2_core::Controller interface
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_fraction, const bool & percentage) override;

private:
  // nearest-point search results on the (frame-transformed) path
  struct PathProjection
  {
    double tangent_yaw{0.0};   // direction of travel at the nearest segment
    double cross_track{0.0};   // + : front axle is LEFT of the path
    double dist_to_end{0.0};   // remaining path length from the nearest point
  };

  PathProjection projectOnPath(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  nav_msgs::msg::Path path_;        // plan, already in the costmap frame

  // parameters
  double desired_vel_{0.4};        // cruise speed m/s (speed-limit aware)
  double base_vel_{0.4};           // un-limited cruise speed
  double max_steer_{0.61};         // rad, combined steering clamp (~35 deg)
  double k_heading_{1.0};          // heading-error gain
  double k_cross_{1.0};            // cross-track gain (Stanley's k)
  double k_soft_{1.0};             // softening denominator (+v), m/s
  double k_steer_to_omega_{3.0};   // steering angle -> yaw rate
  double max_omega_{1.0};          // rad/s
  double front_axle_offset_{0.2};  // steering reference ahead of base, m
  double rotate_threshold_{0.9};   // rad - rotate to heading first
  double k_rotate_{1.5};           // rotate-to-heading gain
  double slow_dist_{0.6};          // m - decelerate to min_approach_vel_
  double min_approach_vel_{0.05};  // m/s
  double k_slow_curve_{1.0};       // speed falloff with |delta|
  double transform_tolerance_{0.1};

  rclcpp::Logger logger_{rclcpp::get_logger("StanleyController")};
};

}  // namespace wr_bringup

#endif  // WR_BRINGUP__STANLEYCONTROLLER_HPP_
