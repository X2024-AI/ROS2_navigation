#!/usr/bin/env python3
"""Relay nav2 AMCL particles into a type RViz2 can display.

nav2 Humble's amcl publishes /particle_cloud as nav2_msgs/msg/ParticleCloud
(best-effort QoS), and RViz2 has no display for that type - only
geometry_msgs/PoseArray. This node repacks the cloud so the "AMCL Particles"
display in rviz/monitor.rviz (topic /particle_cloud_pa) can render it.

Run alongside RViz:
  ros2 run wr_bringup particle_relay.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import PoseArray


def main():
    rclpy.init()
    n = Node('particle_relay')
    pub = n.create_publisher(PoseArray, '/particle_cloud_pa', 10)

    def relay(msg):
        pa = PoseArray()
        pa.header = msg.header
        pa.poses = [p.pose for p in msg.particles]
        pub.publish(pa)

    # pub is best-effort -> the subscription must be best-effort too
    n.create_subscription(ParticleCloud, '/particle_cloud', relay,
                          qos_profile_sensor_data)
    rclpy.spin(n)


if __name__ == '__main__':
    main()
