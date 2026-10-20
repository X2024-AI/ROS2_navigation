#ifndef WR_BRINGUP__STABLEDOCKVERIFY_HPP_
#define WR_BRINGUP__STABLEDOCKVERIFY_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "wr_bringup/CheckPoseTolerance.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace wr_bringup
{

// Condition: SUCCESS only when the pose has been within tolerance for
// `hold_frames` CONSECUTIVE ticks - filters AMCL noise / body rock so a
// single lucky frame cannot declare the dock complete.
class StableDockVerify : public BT::ConditionNode
{
public:
  explicit StableDockVerify(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("dock", "dock pose"),
      BT::InputPort<double>("pos_tol", 0.02, "position tolerance in metres"),
      BT::InputPort<double>("yaw_tol", 0.00873, "yaw tolerance in radians"),
      BT::InputPort<int>("hold_frames", 5, "consecutive good frames required")
    };
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<MapOdomFreezer> freezer_;
  int good_frames_{0};
};

}  // namespace wr_bringup

#endif  // WR_BRINGUP__STABLEDOCKVERIFY_HPP_
