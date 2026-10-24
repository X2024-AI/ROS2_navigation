#!/usr/bin/env python3
"""WarehouseRunner M3: visit preset waypoints in sequence (依次前往多个预设目标点).

Waypoints picked with RViz 'Publish Point' on the SLAM warehouse map.
Each goal's orientation faces the next waypoint, so the robot arrives
pointing the way it intends to continue.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# (x, y) in the map frame, in patrol order — a loop ending near the start.
# 2026-10-24: re-expressed after the map-frame correction (the map image was
# rotated ~26.5 deg + shifted vs the Gazebo world; every surveyed pose was
# transformed by the same measured world<-map transform, so these are the
# SAME physical spots as the benchmark runs used).
WAYPOINTS = [
    (2.041, -0.993),   # 1: upper room, right side (was 1.98, -0.70)
    (-1.434, -5.743),  # 2: below the interior wall (was -3.25, -3.40)
    (-3.297, -11.868),  # 3: far lower-left corner (was -7.65, -8.05)
    (2.214, -10.841),  # 4: along the bottom, left of the table column (was -2.26, -9.59)
    (2.634, -6.463),   # 5: center (was 0.07, -5.86)
    (-0.221, -0.557),  # 6: home, near the start pose (was 0.15, 0.70)
]


def make_pose(nav, x, y, yaw):
    """Build a PoseStamped in the map frame from (x, y, yaw)."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)   # yaw -> quaternion
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator()
    # match the rest of the system: stamp goals with sim time, not wall time
    nav.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])

    nav.waitUntilNav2Active()   # blocks until AMCL + bt_navigator are active
    log = nav.get_logger()
    log.info(f'Nav2 ready — patrolling {len(WAYPOINTS)} waypoints')

    n_ok = 0
    t0 = nav.get_clock().now()
    for i, (x, y) in enumerate(WAYPOINTS):
        nxt = WAYPOINTS[(i + 1) % len(WAYPOINTS)]
        yaw = math.atan2(nxt[1] - y, nxt[0] - x)   # arrive facing the next goal
        nav.goToPose(make_pose(nav, x, y, yaw))

        last_sec = -1
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb and fb.navigation_time.sec != last_sec:   # log at ~1 Hz
                last_sec = fb.navigation_time.sec
                log.info(f'wp {i + 1}/{len(WAYPOINTS)}: '
                         f'{fb.distance_remaining:5.2f} m remaining')

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            n_ok += 1
            log.info(f'waypoint {i + 1} ({x:.2f}, {y:.2f}) REACHED')
        elif result == TaskResult.CANCELED:
            log.warning(f'waypoint {i + 1} canceled — stopping patrol')
            break
        else:
            log.error(f'waypoint {i + 1} FAILED — continuing with next')

    dt = (nav.get_clock().now() - t0).nanoseconds / 1e9
    log.info(f'patrol finished: {n_ok}/{len(WAYPOINTS)} reached in {dt:.0f} s')
    rclpy.shutdown()
    raise SystemExit(0 if n_ok == len(WAYPOINTS) else 1)


if __name__ == '__main__':
    main()
