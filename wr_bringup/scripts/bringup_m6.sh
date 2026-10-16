#!/usr/bin/env bash
# WarehouseRunner M6 one-shot bring-up (post-reboot, all terminals closed).
# Usage:  ./bringup_m6.sh          (sim + nav + parking demo)
#         ./bringup_m6.sh --no-demo (stack only)
# Gates learned the hard way on 2026-10-13:
#   1. /odom must exist (relay from /ground_truth_odom if missing)
#   2. lifecycle activation can take 90+ s under boot contention — poll, don't assume
#   3. AMCL must be seeded after (re)start or every goal silently fails
WS="${WR_WS:-$HOME/Codes/Ros2/bookros2_ws}"   # override with WR_WS if your workspace lives elsewhere
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# --- middleware "reboot": bypass the FastRTPS shared-memory transport ------
# On hosts with long uptime / SIGKILL history the SHM layer corrupts: fresh
# participants receive /tf bytes but listener callbacks never fire. Pure-UDP
# transport sidesteps that layer completely (what an OS reboot would restore).
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_udp_only.xml
if [ ! -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
cat > "$FASTRTPS_DEFAULT_PROFILES_FILE" <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_transport</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="udp_only" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_transport</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
XML
fi

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
pkill -f "odom_[r]elay_cpp" 2>/dev/null
pkill -f "odom_tf_[b]ridge" 2>/dev/null
pkill -f "gzserver" 2>/dev/null
pkill -f "gzclient" 2>/dev/null
pkill -f "ros2-daemon" 2>/dev/null
sleep 3
# --- userspace middleware reboot (everything an OS reboot would clear) -----
# 1. FastRTPS SHM port locks: dead owners block new DDS ports
rm -rf /dev/shm/fastrtps_* 2>/dev/null
# 2. FastDDS named semaphores (32-hex names) whose owners are now dead
find /dev/shm -maxdepth 1 -name 'sem.????????????????????????????????' -delete 2>/dev/null
# 3. orphaned SysV semaphore arrays (Gazebo transport): only those whose
#    creating process no longer exists
ipcs -s -p 2>/dev/null | awk 'NR > 2 && NF >= 3 {print $2, $3}' | while read -r id cpid; do
  if [ -n "$cpid" ] && ! ps -p "$cpid" > /dev/null 2>&1; then
    ipcrm -s "$id" 2>/dev/null
  fi
done
wait_for_scan() {
  # the TIAGo sim cold-boot takes 3-8 min under desktop load (gzserver
  # ~175% CPU while loading world/controllers/MoveIt); ~1 in 5 launches is
  # a dud whose laser never appears and needs a relaunch
  local waited=0
  while [ $waited -lt 480 ]; do
    sleep 4; waited=$((waited+4))
    if timeout 3 ros2 topic list 2>/dev/null | grep -q "^/scan_raw$"; then
      echo "   /scan_raw up after ${waited}s"
      return 0
    fi
    [ $((waited % 60)) -eq 0 ] && echo "   still booting... ${waited}s (normal: 3-8 min under load)"
  done
  return 1
}

launch_sim() {
  cd "$WS" && nohup ros2 launch wr_bringup warehouse_sim.launch.py > /tmp/sim_launch.log 2>&1 &
}

launch_sim
if ! wait_for_scan; then
  echo "   no laser after 8 min - dud instance, relaunching the sim once..."
  pkill -f "gzserver" 2>/dev/null; pkill -f "gzclient" 2>/dev/null
  sleep 5
  rm -rf /dev/shm/fastrtps_* 2>/dev/null
  launch_sim
  wait_for_scan || { echo "FAIL: no /scan_raw even after relaunch"; exit 1; }
fi

echo "== [2/5] odometry gate =="
odom_ok() {
  # true only if /odom has at least one PUBLISHER (topic-list alone also
  # matches subscriber-only topics, and can come from a stale daemon cache)
  timeout 5 ros2 topic info /odom 2>/dev/null | grep -Eq "Publisher count: [1-9]"
}
tf_ok() {
  # true if odom->base_footprint answers (the controller's TF output is
  # sometimes missing on this machine's sims - observed 2026-10-15)
  timeout 4 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | grep -q "Translation"
}
if ! odom_ok; then
  echo "   /odom has no publisher -> starting relay"
  nohup ros2 run wr_bringup odom_relay_cpp > /tmp/odom_relay.log 2>&1 &
fi
for i in $(seq 1 15); do
  odom_ok && { echo "   /odom OK"; break; }
  sleep 1
done
odom_ok || { echo "FAIL: /odom never published"; exit 1; }

# The bridge runs UNCONDITIONALLY: the sim's native odom->base TF output
# has been observed dying MID-SESSION (alive at boot, gone minutes later,
# 2026-10-15). Alongside a healthy native TF it is harmless (same values);
# when native dies, it keeps the robot alive.
echo "   starting odom->base TF bridge"
nohup ros2 run wr_bringup odom_tf_bridge_cpp > /tmp/odom_tf_bridge.log 2>&1 &
sleep 3
tf_ok && echo "   odom->base TF OK" || { echo "FAIL: odom->base TF unavailable even via bridge"; exit 1; }

echo "== [3/5] Nav2 (be patient: up to 3 min under boot contention) =="
cd "$WS" && nohup ros2 launch br2_navigation tiago_navigation.launch.py \
  map:=$WS/src/wr_bringup/maps/warehouse.yaml \
  params_file:=$WS/src/wr_bringup/config/tiago_nav_params.yaml > /tmp/nav_launch.log 2>&1 &
for i in $(seq 1 90); do
  sleep 2
  [ "$(grep -c 'Managed nodes are active' /tmp/nav_launch.log)" -ge 2 ] && break
done
# The CLI lifecycle tool silently no-ops when service responses are lost
# under load (and the lifecycle manager itself stalls waiting for the
# planner's configure response). This direct python driver is robust.
python3 - <<'PYEOF'
import rclpy, time, sys
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition

rclpy.init()
n = Node('lc_driver')
def get_state(name):
    c = n.create_client(GetState, f'/{name}/get_state')
    if not c.wait_for_service(timeout_sec=5.0): return None
    fut = c.call_async(GetState.Request()); t0 = time.time()
    while not fut.done() and time.time()-t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.3)
    return fut.result().current_state.id if fut.done() else None
def drive(name, tr, label):
    c = n.create_client(ChangeState, f'/{name}/change_state')
    if not c.wait_for_service(timeout_sec=5.0): return
    req = ChangeState.Request(); req.transition.id = tr
    fut = c.call_async(req); t0 = time.time()
    while not fut.done() and time.time()-t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.5)
for name in ['map_server', 'amcl', 'controller_server', 'planner_server',
             'smoother_server', 'behavior_server', 'waypoint_follower',
             'velocity_smoother', 'bt_navigator']:
    for attempt in range(3):
        st = get_state(name)
        if st == 3: break
        if st == 1: drive(name, Transition.TRANSITION_CONFIGURE, 'configure')
        if st in (1, 2): drive(name, Transition.TRANSITION_ACTIVATE, 'activate')
    print(f"{name}: state={get_state(name)}")
rclpy.shutdown()
PYEOF
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

# Reuse an already-running healthy stack (brought up with --no-demo) and
# only re-seed + rerun the demo: fast iteration inside one sim window.
if [ "${1:-}" = "--demo-only" ]; then
  echo "== demo-only (reusing running stack) =="
  timeout 3 ros2 topic list 2>/dev/null | grep -q "^/scan_raw$" || { echo "FAIL: no running stack (see --no-demo)"; exit 1; }
  POSE=$(python3 - <<'PYEOF'
import rclpy, math, time
from rclpy.node import Node
from nav_msgs.msg import Odometry
rclpy.init()
n = Node('seed_reader'); m = [None]
n.create_subscription(Odometry, '/odom', lambda x: m.__setitem__(0, x), 10)
t0 = time.time()
while m[0] is None and time.time()-t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.5)
rclpy.shutdown()
if m[0]:
    p = m[0].pose.pose.position; o = m[0].pose.pose.orientation
    yaw = 2*math.atan2(o.z, o.w)
    print(f"{p.x:.3f} {p.y:.3f} {math.sin(yaw/2):.4f} {math.cos(yaw/2):.4f}")
PYEOF
)
  set -- $POSE
  if [ -n "$1" ]; then
    timeout 20 ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
      "{header: {frame_id: map}, pose: {pose: {position: {x: $1, y: $2}, orientation: {z: $3, w: $4}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}" \
      >/dev/null 2>&1 && echo "   seeded from odom ($1, $2)"
  fi
  P=$(ss -tlnp 2>/dev/null | grep 2666 | grep -oP 'pid=\K[0-9]+' | head -1)
  [ -n "$P" ] && kill -9 "$P"
  cd "$WS" && ros2 run wr_bringup parking_bt 2>&1 | tee "$WS/src/wr_bringup/results/parking_m6.log"
  exit $?
fi

echo "== [5/5] parking demo =="
# Optional custom parking slot ( metres, metres, radians ):
#   DOCK="x y yaw"  STAGING="x y yaw"  ros2 run wr_bringup bringup_m6.sh
DOCK_ARGS=()
[ -n "${DOCK:-}" ] && DOCK_ARGS+=( -p "dock_pose:=[${DOCK}]" )
[ -n "${STAGING:-}" ] && DOCK_ARGS+=( -p "staging_pose:=[${STAGING}]" )
[ -n "${DOCK:-}" ] && echo "   custom dock: [${DOCK}]  staging: [${STAGING:-default}]"
cd "$WS" && ros2 run wr_bringup parking_bt --ros-args "${DOCK_ARGS[@]}" 2>&1 | \
  tee "$WS/src/wr_bringup/results/parking_m6.log"
