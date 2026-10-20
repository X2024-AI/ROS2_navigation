#include <cmath>
#include <string>

#include "wr_bringup/CheckPoseTolerance.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace wr_bringup
{

DockFrameError compute_dock_frame_error(
  const geometry_msgs::msg::PoseStamped & dock,
  const tf2::Transform & map_base)
{
  DockFrameError e;

  const double dx = map_base.getOrigin().x() - dock.pose.position.x;
  const double dy = map_base.getOrigin().y() - dock.pose.position.y;
  const double dock_yaw = tf2::getYaw(dock.pose.orientation);
  const double robot_yaw = tf2::getYaw(map_base.getRotation());

  e.longitudinal = dx * std::cos(dock_yaw) + dy * std::sin(dock_yaw);
  e.lateral = -dx * std::sin(dock_yaw) + dy * std::cos(dock_yaw);
  e.yaw = std::atan2(
    std::sin(robot_yaw - dock_yaw), std::cos(robot_yaw - dock_yaw));
  e.valid = true;
  return e;
}

namespace
{
// (re)create the primary /odom reader - pulled out so the watchdog can
// replace a stalled one with a fresh pairing
void make_odom_sub(const rclcpp::Node::SharedPtr & node, MapOdomFreezer & f)
{
  f.odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(10),
    [&f](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      tf2::Transform ob;
      tf2::fromMsg(msg->pose.pose, ob);
      f.odom_base_ = ob;
      f.odom_received_ = true;
      f.odom_stamp_ = rclcpp::Time(msg->header.stamp);
    });
}
}  // namespace

void attach_odom_source(
  const rclcpp::Node::SharedPtr & node,
  MapOdomFreezer & freezer)
{
  make_odom_sub(node, freezer);
  freezer.odom2_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/ground_truth_odom", rclcpp::QoS(10),
    [&freezer](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      tf2::Transform ob;
      tf2::fromMsg(msg->pose.pose, ob);
      freezer.odom_base2_ = ob;
      freezer.odom2_received_ = true;
      freezer.odom2_stamp_ = rclcpp::Time(msg->header.stamp);
    });
  freezer.amcl_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", rclcpp::QoS(10),
    [&freezer](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
      tf2::Transform mb;
      tf2::fromMsg(msg->pose.pose, mb);
      freezer.map_base_amcl_ = mb;
      freezer.amcl_received_ = true;
    });

  // self-healing watchdog: wheel odometry streams continuously (even at
  // standstill), so a stamp that stops advancing means THIS reader stalled -
  // recreate it (fresh pairing) instead of measuring against a frozen pose
  freezer.node_weak_ = node;
  freezer.last_change_wall_ = std::chrono::steady_clock::now();
  freezer.heal_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(500),
    [&freezer]() {
      if (freezer.odom_stamp_ != freezer.prev_stamp_) {
        freezer.prev_stamp_ = freezer.odom_stamp_;
        freezer.last_change_wall_ = std::chrono::steady_clock::now();
        return;
      }
      if (std::chrono::steady_clock::now() - freezer.last_change_wall_ >
        std::chrono::milliseconds(1500))
      {
        auto n = freezer.node_weak_.lock();
        if (n) {
          RCLCPP_WARN(
            n->get_logger(),
            "odom hub: /odom stamp silent >1.5 s - recreating subscription");
          make_odom_sub(n, freezer);
          freezer.last_change_wall_ = std::chrono::steady_clock::now();
        }
      }
    });
}

std::shared_ptr<MapOdomFreezer> get_shared_freezer(
  const BT::NodeConfiguration & config,
  const rclcpp::Node::SharedPtr & node)
{
  std::shared_ptr<MapOdomFreezer> shared;
  config.blackboard->get("shared_freezer", shared);
  if (!shared) {
    shared = std::make_shared<MapOdomFreezer>();
    attach_odom_source(node, *shared);
  }
  return shared;
}

bool compose_map_base(
  const BT::NodeConfiguration & config,
  tf2_ros::Buffer & tf_buffer,
  MapOdomFreezer & freezer,
  tf2::Transform & map_base_out)
{
  bool frozen = false;
  config.blackboard->get("odom_frozen", frozen);

  if (!frozen || !freezer.cached_) {
    // map->odom primary source: topic samples (amcl_pose x odom^-1).
    // Fallback for standstill starts (no motion -> no /amcl_pose publish):
    // the shared TF buffer's map->odom (AMCL publishes it on /tf).
    if (!freezer.amcl_received_) {
      try {
        auto m = tf_buffer.lookupTransform("map", "odom", tf2::TimePointZero);
        tf2::Transform map_odom;
        tf2::fromMsg(m.transform, map_odom);
        // synthesize the AMCL-side sample from the current odom pose.
        // /odom is the PRIMARY source; the GT topic is a transport-failure
        // fallback ONLY: under single-source wheel odometry the two topics
        // carry DIFFERENT frames, and "fresher stamp wins" would mix frames
        // and freeze the error at a constant offset (limit cycle, 2026-10-20).
        if (freezer.odom_received_) {
          freezer.map_base_amcl_ = map_odom * freezer.odom_base_;
          freezer.amcl_received_ = true;
        } else if (freezer.odom2_received_) {
          freezer.map_base_amcl_ = map_odom * freezer.odom_base2_;
          freezer.amcl_received_ = true;
        }
      } catch (const tf2::TransformException &) {
        // no TF either - the diagnostics below will say so
      }
    }
    if (!freezer.amcl_received_ || !freezer.odom_received_) {
      RCLCPP_WARN(
        config.blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
        "compose: frozen=%d amcl=%d odom=%d",
        frozen, freezer.amcl_received_, freezer.odom_received_);
      return false;
    }
    freezer.map_odom_ = freezer.map_base_amcl_ * freezer.odom_base_.inverse();
    freezer.cached_ = true;
  }
  if (!freezer.odom_received_ && !freezer.odom2_received_) {
    return false;   // no odometry sample yet
  }
  // /odom primary, GT topic strictly a fallback (never "fresher wins":
  // under wheel odometry the GT topic is a DIFFERENT frame - composing it
  // with the wheel-frame cached map->odom yields a constant wrong error
  // and a micro-adjustment limit cycle)
  const tf2::Transform & ob =
    freezer.odom_received_ ? freezer.odom_base_ : freezer.odom_base2_;
  map_base_out = freezer.map_odom_ * ob;
  {  // diagnostics: attribute a frozen error to its source
    auto n = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    auto yaw_of = [](const tf2::Transform & t) {
        return std::atan2(2.0 * (t.getRotation().w() * t.getRotation().z()),
          1.0 - 2.0 * t.getRotation().z() * t.getRotation().z());
      };
    double age1 = freezer.odom_received_
      ? (n->get_clock()->now() - freezer.odom_stamp_).seconds() : -1.0;
    double age2 = freezer.odom2_received_
      ? (n->get_clock()->now() - freezer.odom2_stamp_).seconds() : -1.0;
    RCLCPP_INFO(
      n->get_logger(),
      "compose diag: frozen=%d cached=%d odom_age=%.2f odom2_age=%.2f amcl=%d "
      "map_odom_yaw=%+.3f ob_yaw=%+.3f result_yaw=%+.3f",
      frozen, freezer.cached_, age1, age2, freezer.amcl_received_,
      yaw_of(freezer.map_odom_), yaw_of(ob), yaw_of(map_base_out));
  }
  return true;
}

CheckPoseTolerance::CheckPoseTolerance(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);
  // Prefer the tree-wide shared TF buffer (set by parking_bt_main): it keeps
  // the last map->odom cached even while AMCL is idle at standstill. Fall
  // back to a private buffer when this node runs standalone.
  if (!config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer_) ||
    !tf_buffer_)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node, false);
  }
  freezer_ = get_shared_freezer(config(), node);
}

BT::NodeStatus
CheckPoseTolerance::tick()
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  geometry_msgs::msg::PoseStamped dock;
  getInput("dock", dock);
  double pos_tol = 0.02, yaw_tol = 0.00873;
  getInput("pos_tol", pos_tol);
  getInput("yaw_tol", yaw_tol);

  tf2::Transform map_base;
  if (!compose_map_base(config(), *tf_buffer_, *freezer_, map_base)) {
    RCLCPP_WARN(node->get_logger(), "tolerance check: TF unavailable");
    return BT::NodeStatus::FAILURE;
  }

  const DockFrameError e = compute_dock_frame_error(dock, map_base);
  const double pos_err = std::hypot(e.longitudinal, e.lateral);
  RCLCPP_INFO(
    node->get_logger(),
    "tolerance check: pos %.4f m (lon %+.3f, lat %+.3f, tol %.3f), yaw %+.4f rad (tol %.4f)",
    pos_err, e.longitudinal, e.lateral, pos_tol, e.yaw, yaw_tol);

  const bool ok = pos_err <= pos_tol && std::fabs(e.yaw) <= yaw_tol;
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace wr_bringup

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<wr_bringup::CheckPoseTolerance>("CheckPoseTolerance");
}
