#ifndef WR_BRINGUP__CHECKPOSETOLERANCE_HPP_
#define WR_BRINGUP__CHECKPOSETOLERANCE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace wr_bringup
{

// Shared live-TF pose-error helper (map -> base_footprint vs the dock pose,
// expressed in the dock frame: longitudinal / lateral / yaw).
struct DockFrameError
{
  double longitudinal{0.0};   // + : robot ahead of the dock along dock heading
  double lateral{0.0};        // + : robot left of the dock axis
  double yaw{0.0};            // + : robot rotated CCW relative to dock heading
  bool valid{false};
};

DockFrameError compute_dock_frame_error(
  const geometry_msgs::msg::PoseStamped & dock,
  tf2_ros::Buffer & tf_buffer);

// Condition: SUCCESS when live-TF pose is within pos_tol (m) AND yaw_tol (rad)
// of the dock. Logs the errors each tick.
class CheckPoseTolerance : public BT::ConditionNode
{
public:
  explicit CheckPoseTolerance(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("dock", "dock pose"),
      BT::InputPort<double>("pos_tol", 0.02, "position tolerance in metres"),
      BT::InputPort<double>("yaw_tol", 0.00873, "yaw tolerance in radians (~0.5 deg)")
    };
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace wr_bringup

#endif  // WR_BRINGUP__CHECKPOSETOLERANCE_HPP_
