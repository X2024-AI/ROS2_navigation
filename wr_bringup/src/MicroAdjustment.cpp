#include <algorithm>
#include <cmath>
#include <string>

#include "wr_bringup/MicroAdjustment.hpp"

namespace wr_bringup
{

MicroAdjustment::MicroAdjustment(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);
  vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/key_vel", 10);
  // odometry via the tree-wide self-healing hub (a private subscription
  // proved it can stall silently while another on the same node streams)
  hub_ = get_shared_freezer(config(), node);
}

double
MicroAdjustment::hub_yaw() const
{
  const auto & q = hub_->odom_base_.getRotation();
  return 2.0 * std::atan2(q.z(), q.w());
}

void
MicroAdjustment::build_plan()
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  std::string type;
  double magnitude = 0.0, pos_tol = 0.02, yaw_tol = 0.00873;
  getInput("correction", type);
  getInput("magnitude", magnitude);
  getInput("pos_tol", pos_tol);
  getInput("yaw_tol", yaw_tol);

  plan_.clear();
  const double min_pos_step = pos_tol / 2.0;
  const double min_yaw_step = yaw_tol / 2.0;

  if (type == "vector") {
    // ONE rotation pair for the whole position error: face the bearing to
    // the target, drive 3/4 of the distance, rotate back. Axis-split
    // corrections each paid the +-90 deg spin-pair slip-drift tax and
    // see-sawed at ~2x that drift.
    double lon = 0.0, lat = 0.0, yaw_err = 0.0;
    getInput("lon", lon);
    getInput("lat", lat);
    getInput("yaw_err", yaw_err);
    const double r = std::hypot(lon, lat);
    const double step = std::max(0.75 * r, min_pos_step);
    // move = -(lon, lat) in the dock frame; robot heading ~= dock axis
    double beta = std::atan2(-lat, -lon) - yaw_err;
    bool back = false;
    if (std::fabs(beta) > M_PI / 2.0) {
      // target lies behind: face the opposite way and reverse
      beta += (beta > 0.0 ? -M_PI : M_PI);
      back = true;
    }
    plan_.push_back({Step::Type::SPIN, beta});
    plan_.push_back({back ? Step::Type::BACK : Step::Type::DRIVE, step});
    plan_.push_back({Step::Type::SPIN, -beta});
  } else if (type == "lateral") {
    // 3/4-error step: in-place spins slip-drift the base ~2.5 cm per
    // lateral cycle on this floor (measured 2026-10-24), so half-error
    // steps asymptote at ~2x drift and stall ~5 cm out; a larger fraction
    // keeps the net reduction positive against the drift floor
    const double step = std::max(0.75 * std::fabs(magnitude), min_pos_step);
    if (magnitude > 0) {            // robot left of axis -> sidestep right
      plan_.push_back({Step::Type::SPIN, -M_PI / 2});
      plan_.push_back({Step::Type::DRIVE, step});
      plan_.push_back({Step::Type::SPIN, M_PI / 2});
    } else {                        // robot right of axis -> sidestep left
      plan_.push_back({Step::Type::SPIN, M_PI / 2});
      plan_.push_back({Step::Type::DRIVE, step});
      plan_.push_back({Step::Type::SPIN, -M_PI / 2});
    }
  } else if (type == "longitudinal") {
    const double step = std::max(std::fabs(magnitude) / 2.0, min_pos_step);
    if (magnitude > 0) {            // robot ahead of dock -> back up
      plan_.push_back({Step::Type::BACK, step});
    } else {
      plan_.push_back({Step::Type::DRIVE, step});
    }
  } else if (type == "yaw") {
    const double step = std::max(std::fabs(magnitude) / 2.0, min_yaw_step);
    plan_.push_back({Step::Type::SPIN, magnitude > 0 ? -step : step});
  } else {
    RCLCPP_INFO(node->get_logger(), "micro adjustment: nothing to correct");
    return;
  }

  RCLCPP_INFO(
    node->get_logger(), "micro adjustment: %s err %+.4f -> %zu step(s)",
    type.c_str(), magnitude, plan_.size());
}

BT::NodeStatus
MicroAdjustment::start_step()
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);
  double speed = 0.05;
  getInput("speed", speed);
  step_speed_ = std::max(0.01, std::fabs(speed));

  // snapshot the CURRENT pose as the step reference - always fresh, never
  // a value gated on the previous step (a stale reference once completed a
  // step instantly, 6x overshoot)
  ref_yaw_ = hub_yaw();
  const auto & o = hub_->odom_base_.getOrigin();
  ref_x_ = o.x();
  ref_y_ = o.y();
  head_x_ = std::cos(ref_yaw_);
  head_y_ = std::sin(ref_yaw_);
  step_active_ = true;
  // WALL clock for step timeouts: the node clock follows sim time, and a
  // sim pause/step (Gazebo GUI) or clock jump made a 20 s timeout fire
  // after just ~4 s of rotation, aborting a 90-degree spin at half travel
  // and sending the drive off diagonally (2026-10-24: "spin done
  // 0.876/1.571" -> error exploded 0.04 -> 1.0 m)
  step_t0_wall_ = std::chrono::steady_clock::now();
  step_t0_ = node->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
MicroAdjustment::poll_step()
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  geometry_msgs::msg::Twist t;

  if (current_.type == Step::Type::SPIN) {
    double delta = hub_yaw() - ref_yaw_;
    delta = std::atan2(std::sin(delta), std::cos(delta));
    const double elapsed =
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - step_t0_wall_).count();
    if (std::fabs(delta) >= std::fabs(current_.value) - 0.004 ||
      elapsed > 40.0)
    {
      vel_pub_->publish(t);   // zero
      step_active_ = false;
      settle_until_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
      RCLCPP_INFO(
        node->get_logger(), "micro spin done: %.3f/%.3f rad",
        delta, current_.value);
      return BT::NodeStatus::RUNNING;   // next step (or drain)
    }
    // gentle: rotation slip-drift scales with speed (0.3 rad/s drifted the
    // base ~2.5 cm per +-90 deg pair, keeping the loop ~5 cm from target)
    t.angular.z = (current_.value > 0 ? 0.2 : -0.2);
    vel_pub_->publish(t);
    return BT::NodeStatus::RUNNING;
  }

  // DRIVE / BACK: displacement projected on the heading captured at start
  const auto & o = hub_->odom_base_.getOrigin();
  const double along = (o.x() - ref_x_) * head_x_ + (o.y() - ref_y_) * head_y_;
  const double sign = (current_.type == Step::Type::BACK) ? -1.0 : 1.0;
  const double prog = along * sign;

  if (prog >= std::fabs(current_.value) - 0.004) {
    vel_pub_->publish(t);   // zero
    step_active_ = false;
    settle_until_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
    RCLCPP_INFO(
      node->get_logger(), "micro %s done: %.3f/%.3f m",
      current_.type == Step::Type::BACK ? "back" : "drive",
      prog, current_.value);
    return BT::NodeStatus::RUNNING;   // next step (or drain)
  }
  if (std::chrono::duration<double>(
      std::chrono::steady_clock::now() - step_t0_wall_).count() > 40.0)
  {
    vel_pub_->publish(t);
    step_active_ = false;
    RCLCPP_WARN(
      node->get_logger(), "micro %s stalled at %.3f/%.3f m (base not moving?)",
      current_.type == Step::Type::BACK ? "back" : "drive",
      prog, current_.value);
    return BT::NodeStatus::FAILURE;
  }
  t.linear.x = sign * step_speed_;
  vel_pub_->publish(t);
  return BT::NodeStatus::RUNNING;
}

void
MicroAdjustment::halt()
{
  if (step_active_) {
    geometry_msgs::msg::Twist zero;
    vel_pub_->publish(zero);
  }
  step_active_ = false;
  plan_.clear();
  started_ = false;
}

BT::NodeStatus
MicroAdjustment::tick()
{
  if (!started_) {
    // fresh activation: execute ONE correction plan built from the CURRENT
    // input ports. When the plan drains we return SUCCESS so the parent
    // Fallback re-ticks CheckPoseTolerance/CalcPoseError for a FRESH
    // measurement - rebuilding from the ports here would loop forever on a
    // one-shot stale error (the 2026-10-20 limit cycle: "yaw err +0.20"
    // repeating with spins executing but the error never re-measured).
    started_ = true;
    build_plan();
    if (plan_.empty()) {started_ = false; return BT::NodeStatus::SUCCESS;}
  }

  if (!step_active_) {
    // plan drains -> SUCCESS: the NEXT activation (after the parent's
    // re-measurement) rebuilds from fresh ports - a lingering started_
    // made every later activation return SUCCESS instantly (the
    // microsecond retry burst, virtual-robot log 2026-10-22)
    if (plan_.empty()) {started_ = false; return BT::NodeStatus::SUCCESS;}
    // settle pause between steps: let the base stop moving before the next
    // step snapshots its reference pose (a mid-settle snapshot once aimed a
    // drive several degrees off)
    if (std::chrono::steady_clock::now() < settle_until_) {
      geometry_msgs::msg::Twist zero;
      vel_pub_->publish(zero);
      return BT::NodeStatus::RUNNING;
    }
    current_ = plan_.front();
    plan_.pop_front();
    return start_step();
  }

  return poll_step();
}

}  // namespace wr_bringup

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<wr_bringup::MicroAdjustment>("MicroAdjustment");
}
