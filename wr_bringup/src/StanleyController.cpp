#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "wr_bringup/StanleyController.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace wr_bringup
{

void
StanleyController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node_->get_logger();

  auto get = [this](const std::string & n, double def) {
      return node_->declare_parameter<double>(name_ + "." + n, def);
    };

  base_vel_ = get("desired_linear_vel", 0.4);
  desired_vel_ = base_vel_;
  max_steer_ = get("max_steer_angle", 0.61);
  k_heading_ = get("heading_gain", 1.0);
  k_cross_ = get("cross_track_gain", 1.0);
  k_soft_ = get("softening", 1.0);
  k_steer_to_omega_ = get("steer_to_omega_gain", 3.0);
  max_omega_ = get("max_angular_vel", 1.0);
  front_axle_offset_ = get("front_axle_offset", 0.2);
  rotate_threshold_ = get("rotate_to_heading_threshold", 0.9);
  k_rotate_ = get("rotate_to_heading_gain", 1.5);
  slow_dist_ = get("slowdown_distance", 0.6);
  min_approach_vel_ = get("min_approach_velocity", 0.05);
  k_slow_curve_ = get("curve_slowdown_gain", 1.0);
  transform_tolerance_ = get("transform_tolerance", 0.1);

  RCLCPP_INFO(
    logger_,
    "StanleyController configured: v=%.2f k_e=%.2f k_soft=%.2f k_w=%.2f",
    base_vel_, k_cross_, k_soft_, k_steer_to_omega_);
}

void
StanleyController::cleanup()
{
  path_ = nav_msgs::msg::Path();
}

void
StanleyController::activate() {}
void
StanleyController::deactivate() {}

void
StanleyController::setPlan(const nav_msgs::msg::Path & path)
{
  // store the plan transformed ONCE into the costmap frame (plans arrive
  // at ~1 Hz from the BT replanner; a per-plan transform is fresh enough)
  path_ = nav_msgs::msg::Path();
  path_.header.frame_id = costmap_ros_->getGlobalFrameID();
  path_.header.stamp = node_->now();
  try {
    const auto tf = tf_->lookupTransform(
      costmap_ros_->getGlobalFrameID(), path.header.frame_id,
      tf2::TimePointZero);
    for (const auto & p : path.poses) {
      geometry_msgs::msg::PoseStamped q;
      tf2::doTransform(p, q, tf);
      path_.poses.push_back(q);
    }
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(logger_, "setPlan: transform failed (%s)", e.what());
    path_.poses.clear();
  }
}

geometry_msgs::msg::TwistStamped
StanleyController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd.header.stamp = node_->now();

  if (path_.poses.size() < 2) {
    return cmd;   // nothing to follow
  }

  const PathProjection prj = projectOnPath(pose);

  const double robot_yaw = tf2::getYaw(pose.pose.orientation);
  double heading_err = std::atan2(
    std::sin(prj.tangent_yaw - robot_yaw),
    std::cos(prj.tangent_yaw - robot_yaw));

  double v_cmd = desired_vel_;
  double omega = 0.0;

  // terminal alignment: near the final path pose, rotate to its yaw so
  // the goal checker can succeed (DWB/MPPI have RotateToGoal critics; a
  // bare path follower does not)
  const auto & last = path_.poses.back().pose;
  const double d_last = std::hypot(
    last.position.x - pose.pose.position.x,
    last.position.y - pose.pose.position.y);
  if (d_last < 0.20) {
    const double yaw_err = std::atan2(
      std::sin(tf2::getYaw(last.orientation) - robot_yaw),
      std::cos(tf2::getYaw(last.orientation) - robot_yaw));
    omega = std::clamp(k_rotate_ * yaw_err, -max_omega_, max_omega_);
    v_cmd = 0.0;
    RCLCPP_DEBUG(logger_, "stanley: terminal yaw align %+.3f", yaw_err);
  } else if (std::fabs(heading_err) > rotate_threshold_) {
    // too far off the path heading: rotate in place toward it first
    omega = std::clamp(k_rotate_ * heading_err, -max_omega_, max_omega_);
    v_cmd = 0.0;
  } else {
    const double speed = std::max(std::fabs(velocity.linear.x), min_approach_vel_);
    // Stanley law: heading term + speed-normalised cross-track term
    // (front axle LEFT of path => steer right => negative contribution)
    double delta =
      k_heading_ * heading_err -
      std::atan2(k_cross_ * prj.cross_track, speed + k_soft_);
    delta = std::clamp(delta, -max_steer_, max_steer_);

    omega = std::clamp(k_steer_to_omega_ * delta, -max_omega_, max_omega_);

    // slow down in sharp steering and near the path end
    v_cmd = desired_vel_ / (1.0 + k_slow_curve_ * std::fabs(delta));
    if (prj.dist_to_end < slow_dist_) {
      v_cmd = std::min(
        v_cmd, std::max(min_approach_vel_, desired_vel_ * prj.dist_to_end / slow_dist_));
    }
  }

  cmd.twist.linear.x = v_cmd;
  cmd.twist.angular.z = omega;

  RCLCPP_DEBUG(
    logger_,
    "stanley: head %+.3f xtrack %+.3f -> delta v=%.2f w=%.2f",
    heading_err, prj.cross_track, v_cmd, omega);
  return cmd;
}

void
StanleyController::setSpeedLimit(const double & speed_fraction, const bool & percentage)
{
  if (percentage) {
    desired_vel_ = base_vel_ * speed_fraction;
  } else {
    desired_vel_ = speed_fraction;
  }
}

StanleyController::PathProjection
StanleyController::projectOnPath(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  PathProjection prj;

  // front-axle (steering) reference point
  const double yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double fx = robot_pose.pose.position.x + std::cos(yaw) * front_axle_offset_;
  const double fy = robot_pose.pose.position.y + std::sin(yaw) * front_axle_offset_;

  // nearest path point (full scan; paths here are short)
  double best_d2 = std::numeric_limits<double>::max();
  size_t best_i = 0;
  for (size_t i = 0; i + 1 < path_.poses.size(); ++i) {
    const double dx = fx - path_.poses[i].pose.position.x;
    const double dy = fy - path_.poses[i].pose.position.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_i = i;
    }
  }

  // tangent from the segment ahead of the nearest point
  const double tx = path_.poses[best_i + 1].pose.position.x -
    path_.poses[best_i].pose.position.x;
  const double ty = path_.poses[best_i + 1].pose.position.y -
    path_.poses[best_i].pose.position.y;
  prj.tangent_yaw = std::atan2(ty, tx);

  // signed cross-track via the left normal of the tangent
  const double len = std::hypot(tx, ty);
  if (len > 1e-6) {
    const double nx = -ty / len;
    const double ny = tx / len;
    prj.cross_track = (fx - path_.poses[best_i].pose.position.x) * nx +
      (fy - path_.poses[best_i].pose.position.y) * ny;
  }

  // remaining path length from the nearest point (for the slowdown)
  double rem = 0.0;
  for (size_t i = best_i + 1; i < path_.poses.size(); ++i) {
    rem += std::hypot(
      path_.poses[i].pose.position.x - path_.poses[i - 1].pose.position.x,
      path_.poses[i].pose.position.y - path_.poses[i - 1].pose.position.y);
  }
  prj.dist_to_end = rem;
  return prj;
}

}  // namespace wr_bringup

PLUGINLIB_EXPORT_CLASS(wr_bringup::StanleyController, nav2_core::Controller)
