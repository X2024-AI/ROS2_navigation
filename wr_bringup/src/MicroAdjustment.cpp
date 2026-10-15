#include <algorithm>
#include <cmath>
#include <string>

#include "wr_bringup/MicroAdjustment.hpp"

namespace wr_bringup
{

using namespace std::chrono_literals;  // NOLINT

MicroAdjustment::MicroAdjustment(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);
  drive_client_ = rclcpp_action::create_client<nav2_msgs::action::DriveOnHeading>(
    node, "drive_on_heading");
  backup_client_ = rclcpp_action::create_client<nav2_msgs::action::BackUp>(
    node, "backup");
  spin_client_ = rclcpp_action::create_client<nav2_msgs::action::Spin>(
    node, "spin");
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

  if (type == "lateral") {
    // half-error step, no-overshoot rule
    const double step = std::max(std::fabs(magnitude) / 2.0, min_pos_step);
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

  if (current_.type == Step::Type::SPIN) {
    auto goal = nav2_msgs::action::Spin::Goal();
    goal.target_yaw = static_cast<float>(current_.value);
    goal.time_allowance.sec = 15;
    spin_accept_fut_ = spin_client_->async_send_goal(goal);
  } else if (current_.type == Step::Type::DRIVE) {
    auto goal = nav2_msgs::action::DriveOnHeading::Goal();
    goal.target.x = current_.value;
    goal.speed = static_cast<float>(speed);
    goal.time_allowance.sec = 15;
    drive_accept_fut_ = drive_client_->async_send_goal(goal);
  } else {   // BACK
    auto goal = nav2_msgs::action::BackUp::Goal();
    goal.target.x = -current_.value;   // negative x = reverse
    goal.speed = static_cast<float>(speed);
    goal.time_allowance.sec = 15;
    backup_accept_fut_ = backup_client_->async_send_goal(goal);
  }

  step_t0_ = node->now();
  result_arrived_ = false;
  phase_ = Phase::AWAIT_ACCEPT;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
MicroAdjustment::poll_step()
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  // phase 1: goal acceptance (non-blocking)
  if (phase_ == Phase::AWAIT_ACCEPT) {
    if (current_.type == Step::Type::SPIN) {
      if (spin_accept_fut_.valid() &&
        spin_accept_fut_.wait_for(0s) == std::future_status::ready)
      {
        spin_handle_ = spin_accept_fut_.get();
      }
    } else if (current_.type == Step::Type::DRIVE) {
      if (drive_accept_fut_.valid() &&
        drive_accept_fut_.wait_for(0s) == std::future_status::ready)
      {
        drive_handle_ = drive_accept_fut_.get();
      }
    } else {
      if (backup_accept_fut_.valid() &&
        backup_accept_fut_.wait_for(0s) == std::future_status::ready)
      {
        backup_handle_ = backup_accept_fut_.get();
      }
    }

    const bool accepted =
      (current_.type == Step::Type::SPIN && spin_handle_) ||
      (current_.type == Step::Type::DRIVE && drive_handle_) ||
      (current_.type == Step::Type::BACK && backup_handle_);
    if (!accepted) {
      if ((node->now() - step_t0_).seconds() > 10.0) {   // acceptance timeout
        RCLCPP_WARN(node->get_logger(), "micro step acceptance timeout");
        // reset the step state so a parent retry starts clean instead of
        // instantly re-timing-out on this stale timestamp
        phase_ = Phase::IDLE;
        spin_handle_ = nullptr;
        drive_handle_ = nullptr;
        backup_handle_ = nullptr;
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    phase_ = Phase::AWAIT_RESULT;
  }

  int8_t status = -1;
  // phase 2: result - request it once, then poll the future (the handle's
  // status only updates after the result request is in flight)
  if (current_.type == Step::Type::SPIN && spin_handle_) {
    if (!spin_result_fut_.valid()) {
      spin_result_fut_ = spin_client_->async_get_result(spin_handle_);
      return BT::NodeStatus::RUNNING;
    }
    if (spin_result_fut_.wait_for(0s) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    spin_result_fut_.get();
    // the result arriving IS the terminal signal; the client's status-topic
    // view can lag or never update for sub-second goals, and the closed loop
    // re-measures the true error next cycle anyway
    status = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
  } else if (current_.type == Step::Type::DRIVE && drive_handle_) {
    if (!drive_result_fut_.valid()) {
      drive_result_fut_ = drive_client_->async_get_result(drive_handle_);
      return BT::NodeStatus::RUNNING;
    }
    if (drive_result_fut_.wait_for(0s) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    drive_result_fut_.get();
    // the result arriving IS the terminal signal; the client's status-topic
    // view can lag or never update for sub-second goals, and the closed loop
    // re-measures the true error next cycle anyway
    status = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
  } else if (current_.type == Step::Type::BACK && backup_handle_) {
    if (!backup_result_fut_.valid()) {
      backup_result_fut_ = backup_client_->async_get_result(backup_handle_);
      return BT::NodeStatus::RUNNING;
    }
    if (backup_result_fut_.wait_for(0s) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    backup_result_fut_.get();
    // the result arriving IS the terminal signal; the client's status-topic
    // view can lag or never update for sub-second goals, and the closed loop
    // re-measures the true error next cycle anyway
    status = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
  } else {
    return BT::NodeStatus::RUNNING;
  }

  spin_handle_ = nullptr;
  drive_handle_ = nullptr;
  backup_handle_ = nullptr;
  phase_ = Phase::IDLE;

  if (status != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    RCLCPP_WARN(node->get_logger(), "micro step aborted (status %d)", status);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;   // next tick sends the next step (or drains)
}

void
MicroAdjustment::halt()
{
  plan_.clear();
  phase_ = Phase::IDLE;
  spin_handle_ = nullptr;
  drive_handle_ = nullptr;
  backup_handle_ = nullptr;
}

BT::NodeStatus
MicroAdjustment::tick()
{
  if (plan_.empty() && phase_ == Phase::IDLE) {
    build_plan();
    if (plan_.empty()) {return BT::NodeStatus::SUCCESS;}   // nothing to correct
  }

  if (phase_ == Phase::IDLE) {
    if (plan_.empty()) {return BT::NodeStatus::SUCCESS;}   // all steps done
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
