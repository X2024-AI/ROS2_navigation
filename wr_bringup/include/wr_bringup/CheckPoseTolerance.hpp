#ifndef WR_BRINGUP__CHECKPOSETOLERANCE_HPP_
#define WR_BRINGUP__CHECKPOSETOLERANCE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace wr_bringup
{

// Shared pose-error helper: map-frame robot pose vs the dock pose, expressed
// in the dock frame (longitudinal / lateral / yaw).
struct DockFrameError
{
  double longitudinal{0.0};   // + : robot ahead of the dock along dock heading
  double lateral{0.0};        // + : robot left of the dock axis
  double yaw{0.0};            // + : robot rotated CCW relative to dock heading
  bool valid{false};
};

DockFrameError compute_dock_frame_error(
  const geometry_msgs::msg::PoseStamped & dock,
  const tf2::Transform & map_base);

// During micro-adjustment the blackboard flag "odom_frozen" is true: measure
// against a CACHED map->odom composed with live odom->base. AMCL diverges
// during fast in-place spins (front lidar), which otherwise exactly cancels
// the correction being measured - an unbounded spin feedback loop.
class MapOdomFreezer
{
public:
  tf2::Transform map_odom_;   // cached map->odom
  bool cached_{false};
};

// Build the current map->base transform for a node: frozen map->odom
// composed with live odom->base when the blackboard says to freeze,
// otherwise a direct (live) lookup.
bool compose_map_base(
  const BT::NodeConfiguration & config,
  tf2_ros::Buffer & tf_buffer,
  MapOdomFreezer & freezer,
  tf2::Transform & map_base_out);

// Condition: SUCCESS when the (freeze-aware) pose is within pos_tol (m) AND
// yaw_tol (rad) of the dock. Logs the errors each tick.
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
  MapOdomFreezer freezer_;
};

}  // namespace wr_bringup

#endif  // WR_BRINGUP__CHECKPOSETOLERANCE_HPP_
