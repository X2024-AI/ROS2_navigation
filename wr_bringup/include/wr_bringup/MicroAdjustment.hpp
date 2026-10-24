#ifndef WR_BRINGUP__MICROADJUSTMENT_HPP_
#define WR_BRINGUP__MICROADJUSTMENT_HPP_

#include <deque>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "wr_bringup/CheckPoseTolerance.hpp"

namespace wr_bringup
{

// Stateful action (v3: ActionNodeBase + member state): executes the
// correction chosen by CalcPoseError. Differential drive has no strafe, so
// a lateral correction becomes spin(+-90) -> drive -> spin(back).
// Step rule: half of the current error, floored at half the tolerance
// (no-overshoot convergence, per design).
//
// ALL step types execute DIRECTLY: velocity on the twist-mux keyboard
// input, closed loop on the tree-wide shared odometry hub. The nav2
// behavior_server path (DriveOnHeading/BackUp actions) was abandoned on
// 2026-10-24 - on the real stack those goals proved unreliable exactly
// where the demo needs them: instant "Collision Ahead" aborts from the
// local costmap inflation zone that surrounds the dock, 15 s "exceeded
// time allowance" stalls with a frozen base, goal statuses that never
// arrive, and goal handles the Humble client erases once their result is
// taken (canceling then throws). Direct closed-loop moves at crawl speed
// in the creep zone replace all of it; the yaw steps already worked this
// way with <= 0.002 rad accuracy.
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
    double value;   // DRIVE/BACK: metres (DRIVE ahead, BACK reverse); SPIN: radians (+ CCW)
  };

  void build_plan();
  BT::NodeStatus start_step();     // snapshot the start pose, arm the step
  BT::NodeStatus poll_step();      // per-tick closed-loop progress

  std::deque<Step> plan_;
  Step current_{Step::Type::DRIVE, 0.0};
  bool step_active_{false};
  double step_speed_{0.05};

  // direct velocity publisher (twist-mux keyboard input - publish >= 2 Hz
  // or the mux times the input out) and the shared self-healing odometry
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  std::shared_ptr<MapOdomFreezer> hub_;

  // per-step reference snapshot, odom frame
  double ref_yaw_{0.0};
  double ref_x_{0.0}, ref_y_{0.0};
  double head_x_{1.0}, head_y_{0.0};
  rclcpp::Time step_t0_;
  std::chrono::steady_clock::time_point step_t0_wall_{};   // pause-immune timeouts
  std::chrono::steady_clock::time_point settle_until_{};   // inter-step pause

  double hub_yaw() const;
  bool started_{false};   // one correction plan per activation
};

}  // namespace wr_bringup

#endif  // WR_BRINGUP__MICROADJUSTMENT_HPP_
