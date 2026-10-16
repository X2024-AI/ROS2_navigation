#ifndef WR_BRINGUP__MICROADJUSTMENT_HPP_
#define WR_BRINGUP__MICROADJUSTMENT_HPP_

#include <deque>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace wr_bringup
{

// Stateful action (v3: ActionNodeBase + member state): executes the
// correction chosen by CalcPoseError using nav2 behavior_server primitives
// (dead-reckoned, odometry-serviced - and immune to controller speed
// limits). Differential drive has no strafe, so a lateral correction
// becomes spin(+-90) -> drive -> spin(back).
// Step rule: half of the current error, floored at half the tolerance
// (no-overshoot convergence, per design).
// All action-client waits are NON-BLOCKING (acceptance and result are
// polled across ticks) - a blocking .get() inside tick() can deadlock the
// single-threaded BT loop against spin_some().
class MicroAdjustment : public BT::ActionNodeBase
{
public:
  MicroAdjustment(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("correction", "lateral|longitudinal|yaw|none (from CalcPoseError)"),
      BT::InputPort<double>("magnitude", "signed error of the chosen term"),
      BT::InputPort<double>("pos_tol", 0.02, "position tolerance in metres"),
      BT::InputPort<double>("yaw_tol", 0.00873, "yaw tolerance in radians"),
      BT::InputPort<double>("speed", 0.05, "translation speed of a micro move, m/s")
    };
  }

private:
  struct Step
  {
    enum class Type {DRIVE, BACK, SPIN} type;
    double value;   // DRIVE/BACK: metres; SPIN: radians (+ CCW)
  };

  void build_plan();
  BT::NodeStatus start_step();     // send goal, move to AWAIT_ACCEPT
  BT::NodeStatus poll_step();      // acceptance -> result -> next step

  rclcpp_action::Client<nav2_msgs::action::DriveOnHeading>::SharedPtr drive_client_;
  rclcpp_action::Client<nav2_msgs::action::BackUp>::SharedPtr backup_client_;
  rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client_;

  std::deque<Step> plan_;
  Step current_{Step::Type::DRIVE, 0.0};
  enum class Phase {IDLE, AWAIT_ACCEPT, AWAIT_RESULT} phase_{Phase::IDLE};

  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::DriveOnHeading>::SharedPtr>
  drive_accept_fut_;
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::BackUp>::SharedPtr>
  backup_accept_fut_;
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::SharedPtr>
  spin_accept_fut_;

  // result futures: on this stack the goal handle's status only updates
  // once a result request is in flight, so completion is polled here
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::DriveOnHeading>::
  WrappedResult> drive_result_fut_;
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::BackUp>::
  WrappedResult> backup_result_fut_;
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::
  WrappedResult> spin_result_fut_;

  rclcpp_action::ClientGoalHandle<nav2_msgs::action::DriveOnHeading>::SharedPtr drive_handle_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::BackUp>::SharedPtr backup_handle_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::SharedPtr spin_handle_;

  // direct rotation (behavior-server Spin aborts on costmap collision
  // checks near the dock): publish on the twist-mux keyboard input and
  // close the loop on our own odometry sample
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::SubscriptionBase::SharedPtr spin_odom_sub_;
  double spin_start_yaw_{0.0};
  double spin_yaw_{0.0};
  bool spin_active_{false};

  rclcpp::Time step_t0_;
  bool result_arrived_{false};
  bool status_polling_{false};
  rclcpp::Time status_t0_;   // result future done; give the status topic
  // one extra spin cycle before trusting get_status()
};

}  // namespace wr_bringup

#endif  // WR_BRINGUP__MICROADJUSTMENT_HPP_
