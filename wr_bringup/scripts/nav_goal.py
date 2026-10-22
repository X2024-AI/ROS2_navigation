#!/usr/bin/env python3
"""WarehouseRunner: single-goal navigation WITHOUT the parking BT (M5 style).

Contrast experiment against the M6 parking tree: sends one NavigateToPose
through the M5-era controller BT (default: MPPI) and reports where the robot
ACTUALLY ended - AMCL estimate vs Gazebo ground truth vs the goal - so
"final position far from the goal" can be attributed: navigation error,
goal-checker tolerance, or localization offset.

Usage:
  ros2 run wr_bringup nav_goal.py 1.98 -0.70
  ros2 run wr_bringup nav_goal.py 1.98 -0.70 --yaw 1.2
  ros2 run wr_bringup nav_goal.py 1.98 -0.70 --bt .../nav_bt_normal.xml
"""
import argparse
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

DEFAULT_BT = os.path.join(
    os.path.expanduser(
        '~/Codes/Ros2/bookros2_ws/src/wr_bringup/config/bt/nav_bt_MPPI.xml'))


def yaw_of(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('x', type=float)
    ap.add_argument('y', type=float)
    ap.add_argument('--yaw', type=float, default=None,
                    help='goal yaw; default: face the goal from the start pose')
    ap.add_argument('--bt', default=DEFAULT_BT,
                    help='behavior tree xml (default: M5 MPPI BT)')
    ap.add_argument('--timeout', type=float, default=240.0)
    args = ap.parse_args()

    rclpy.init()
    n = Node('nav_goal')
    client = ActionClient(n, NavigateToPose, '/navigate_to_pose')
    amcl = {}
    gt = {}
    n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                          lambda m: amcl.update(
                              p=m.pose.pose.position,
                              y=yaw_of(m.pose.pose.orientation.x,
                                       m.pose.pose.orientation.y,
                                       m.pose.pose.orientation.z,
                                       m.pose.pose.orientation.w)), 10)
    n.create_subscription(Odometry, '/ground_truth_odom',
                          lambda m: gt.update(
                              p=m.pose.pose.position,
                              y=yaw_of(m.pose.pose.orientation.x,
                                       m.pose.pose.orientation.y,
                                       m.pose.pose.orientation.z,
                                       m.pose.pose.orientation.w)), 10)

    log = n.get_logger()
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    if not client.wait_for_server(timeout_sec=20.0):
        log.error('navigate_to_pose server not available')
        raise SystemExit(1)

    yaw = args.yaw
    if yaw is None:
        # face from the current ground-truth position toward the goal
        gp = gt.get('p')
        yaw = math.atan2(args.y - gp.y, args.x - gp.x) if gp else 0.0

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = args.x
    goal.pose.pose.position.y = args.y
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
    goal.behavior_tree = args.bt
    log.info(f'goal ({args.x:.2f}, {args.y:.2f}, yaw {yaw:.2f}) via {os.path.basename(args.bt)}')

    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=15.0)
    if not fut.result() or not fut.result().accepted:
        log.error('goal rejected')
        raise SystemExit(1)
    result_fut = fut.result().get_result_async()

    t0 = n.get_clock().now()
    last_sec = -1
    import time
    t0w = time.time()
    while not result_fut.done() and time.time() - t0w < args.timeout:
        rclpy.spin_once(n, timeout_sec=0.5)
        fb = fut.result().feedback if hasattr(fut.result(), 'feedback') else None
        now = (n.get_clock().now() - t0).nanoseconds / 1e9
        if fb and int(now) != last_sec:
            last_sec = int(now)
            log.info(f'{fb.distance_remaining:6.2f} m remaining ({now:.0f} s)')

    status = result_fut.result().status if result_fut.done() else GoalStatus.STATUS_UNKNOWN
    verdict = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
               GoalStatus.STATUS_ABORTED: 'ABORTED',
               GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(status, str(status))
    log.info(f'goal {verdict} in {time.time() - t0w:.0f} s')

    def report(name, d):
        if 'p' in d:
            p = d['p']
            dist = math.hypot(p.x - args.x, p.y - args.y)
            log.info(
                f'{name}: ({p.x:+.3f}, {p.y:+.3f}, yaw {d["y"]:+.3f})'
                f'  |goal| = {dist:.3f} m')
        else:
            log.warn(f'{name}: no sample')
    report('AMCL  ', amcl)
    report('truth ', gt)
    if 'p' in amcl and 'p' in gt:
        dp = math.hypot(amcl['p'].x - gt['p'].x, amcl['p'].y - gt['p'].y)
        log.info(f'AMCL-vs-truth offset: {dp:.3f} m  '
                 f'(this is localization error, not navigation error)')
    rclpy.shutdown()
    raise SystemExit(0 if status == GoalStatus.STATUS_SUCCEEDED else 1)


if __name__ == '__main__':
    main()
