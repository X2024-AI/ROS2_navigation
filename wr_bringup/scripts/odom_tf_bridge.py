#!/usr/bin/env python3
"""Bridge /ground_truth_odom (Gazebo physics odometry) -> TF odom->base_footprint.

This machine's Gazebo sims sometimes bring up the diff-drive controller
without its TF output (odometry topic flows, TF never appears - observed
2026-10-15). Nav2 and the docking BT both need that transform. This bridge
re-publishes the controller's own odometry as the transform, so the robot
keeps working regardless of which interface the flaky sim drops.

Only started by bringup_m6.sh when the transform is genuinely missing; torn
down with everything else (pattern 'odom_tf_bridge').
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomTFBridge(Node):

    def __init__(self):
        super().__init__('odom_tf_bridge', parameter_overrides=[
            Parameter('use_sim_time', value=True)])   # stamp in sim time
        self.br = TransformBroadcaster(self)
        # BEST_EFFORT: on hosts where persistent RELIABLE subscriptions to
        # Gazebo-plugin publishers stall after the first message, best-effort
        # keeps the stream flowing (compatible with reliable publishers).
        self.create_subscription(
            Odometry, '/ground_truth_odom', self.cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info('odom->base_footprint TF bridge up')

    def cb(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        # The gazebo odometry plugin parents its pose to the WORLD frame -
        # republishing that verbatim creates world->base_footprint, which no
        # TF consumer asks for (odom literally "does not exist" downstream).
        # The robot spawns at the world origin, so odom == world numerically:
        # force the conventional frame names.
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTFBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
