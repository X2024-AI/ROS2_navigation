#!/usr/bin/env bash
# WarehouseRunner M6 one-shot bring-up (post-reboot, all terminals closed).
# Usage:  ./bringup_m6.sh          (sim + nav + parking demo)
#         ./bringup_m6.sh --no-demo (stack only)
# Gates learned the hard way on 2026-10-13:
#   1. /odom must exist (relay from /mobile_base_controller/odom if missing)
#   2. lifecycle activation can take 90+ s under boot contention — poll, don't assume
#   3. AMCL must be seeded after (re)start or every goal silently fails
WS="${WR_WS:-$HOME/Codes/Ros2/bookros2_ws}"   # override with WR_WS if your workspace lives elsewhere
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "== [1/5] simulation =="
# Idempotent teardown: leftover nav/sim processes from a previous run create
# duplicate action servers and DDS chaos (learned 2026-10-15). Kill launch
# PARENTS first so ros2 launch tears down its own children, then sweep.
pkill -f "parking_[b]t"        2>/dev/null
pkill -f "tiago_navigation[.]launch" 2>/dev/null
pkill -f "warehouse_sim[.]launch"    2>/dev/null
sleep 2
pkill -f "nav2_[a-z]"          2>/dev/null
pkill -f "lifecycle_[m]anager" 2>/dev/null
pkill -f "topic_tool[s]/relay" 2>/dev/null
pkill -f "gzserver" 2>/dev/null
pkill -f "gzclient" 2>/dev/null
sleep 3
# Stale FastRTPS shared-memory locks (owners SIGKILLed earlier) block new DDS
# ports with "open_and_lock_file failed". Scoped to the ROS middleware's own
# files only, and safe because every robot process is down at this point.
rm -rf /dev/shm/fastrtps_*
cd "$WS" && nohup ros2 launch wr_bringup warehouse_sim.launch.py > /tmp/sim_launch.log 2>&1 &
echo "   waiting for /scan_raw + controllers ..."
for i in $(seq 1 45); do
  sleep 2
  timeout 3 ros2 topic list 2>/dev/null | grep -q "^/scan_raw$" && break
done
timeout 3 ros2 topic list 2>/dev/null | grep -q "^/scan_raw$" || { echo "FAIL: no /scan_raw"; exit 1; }

echo "== [2/5] odometry gate =="
odom_ok() {
  # true only if /odom has at least one PUBLISHER (topic-list alone also
  # matches subscriber-only topics, and can come from a stale daemon cache)
  timeout 5 ros2 topic info /odom 2>/dev/null | grep -Eq "Publisher count: [1-9]"
}
if ! odom_ok; then
  echo "   /odom has no publisher -> starting relay"
  nohup ros2 run topic_tools relay /mobile_base_controller/odom /odom > /tmp/odom_relay.log 2>&1 &
fi
for i in $(seq 1 15); do
  odom_ok && { echo "   /odom OK"; break; }
  sleep 1
done
odom_ok || { echo "FAIL: /odom never published"; exit 1; }

echo "== [3/5] Nav2 (be patient: up to 3 min under boot contention) =="
cd "$WS" && nohup ros2 launch br2_navigation tiago_navigation.launch.py \
  map:=$WS/src/wr_bringup/maps/warehouse.yaml \
  params_file:=$WS/src/wr_bringup/config/tiago_nav_params.yaml > /tmp/nav_launch.log 2>&1 &
for i in $(seq 1 90); do
  sleep 2
  [ "$(grep -c 'Managed nodes are active' /tmp/nav_launch.log)" -ge 2 ] && break
done
for n in map_server amcl controller_server planner_server bt_navigator; do
  st=$(timeout 5 ros2 lifecycle get "/$n" 2>/dev/null)
  if ! echo "$st" | grep -q "active"; then
    echo "   $n not active ($st) -> manual transitions"
    timeout 90 ros2 lifecycle set "/$n" configure >/dev/null 2>&1
    timeout 90 ros2 lifecycle set "/$n" activate  >/dev/null 2>&1
  fi
done
echo "   lifecycle states:"
for n in map_server amcl controller_server planner_server bt_navigator; do
  echo "     $n: $(timeout 5 ros2 lifecycle get /$n 2>/dev/null)"
done

echo "== [4/5] AMCL seed (robot spawns at map origin) =="
for i in 1 2 3; do
  timeout 20 ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}" \
    >/dev/null 2>&1 && { echo "   seeded (0,0,0)"; break; }
  sleep 2
done

if [ "${1:-}" = "--no-demo" ]; then echo "== stack ready (no demo) =="; exit 0; fi

echo "== [5/5] parking demo =="
# Optional custom parking slot ( metres, metres, radians ):
#   DOCK="x y yaw"  STAGING="x y yaw"  ros2 run wr_bringup bringup_m6.sh
DOCK_ARGS=()
[ -n "${DOCK:-}" ] && DOCK_ARGS+=( -p "dock_pose:=[${DOCK}]" )
[ -n "${STAGING:-}" ] && DOCK_ARGS+=( -p "staging_pose:=[${STAGING}]" )
[ -n "${DOCK:-}" ] && echo "   custom dock: [${DOCK}]  staging: [${STAGING:-default}]"
cd "$WS" && ros2 run wr_bringup parking_bt --ros-args "${DOCK_ARGS[@]}" 2>&1 | \
  tee "$WS/src/wr_bringup/results/parking_m6.log"
