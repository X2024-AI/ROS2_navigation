#ifndef WR_BRINGUP__DOCKAPPROACH_HPP_
#define WR_BRINGUP__DOCKAPPROACH_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

#include "wr_bringup/ctrl_support/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

namespace wr_bringup
{

// NavigateToPose to the dock with a distance-triggered slow-approach zone:
// while the action runs, the node watches the AMCL pose; once the robot is
// within `distance_threshold` of the dock it publishes an absolute speed
// limit on /speed_limit, and restores full speed when the action ends.
// (This replaces a fixed staging pose: the "slow stage" is a radius, not a point.)
class DockApproach : public wr_bringup::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  explicit DockApproach(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  void on_wait_for_result() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "dock pose"),
      BT::InputPort<std::string>("bt_xml", "", "Nav2-side BT (precise checker)"),
      BT::InputPort<double>("distance_threshold", 1.5, "slow zone radius in metres"),
      BT::InputPort<double>("slow_speed", 0.15, "absolute speed limit in m/s")
    };
  }

private:
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void limit_speed(double limit, bool percentage);
  double distance_to_goal() const;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_pub_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped last_pose_;
  bool pose_received_{false};
  bool slowed_{false};
};

}  // namespace wr_bringup

#endif  // WR_BRINGUP__DOCKAPPROACH_HPP_
