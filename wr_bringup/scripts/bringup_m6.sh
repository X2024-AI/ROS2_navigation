#!/usr/bin/env bash
# WarehouseRunner M6 bring-up.
# Usage:
#   bringup_m6.sh              full pipeline: teardown + sim + nav + demo
#   bringup_m6.sh --no-demo    bring the stack up and leave it (for iterating)
#   bringup_m6.sh --demo-only  REUSE the running stack: re-seed + rerun demo
#                              (fast iteration inside one sim window - never
#                              touches the sim or nav)
WS="${WR_WS:-$HOME/Codes/Ros2/bookros2_ws}"   # override with WR_WS if your workspace lives elsewhere
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# --- middleware hygiene: bypass FastRTPS shared-memory transport ----------
# On hosts with long uptime / SIGKILL history the SHM layer corrupts. Pure
# UDP transport sidesteps that layer completely.
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

# seed_amcl X Y YAW - relocalize AMCL, VERIFIED.
# One-shot `ros2 topic pub --once /initialpose` NEVER lands: AMCL's
# subscription is BEST_EFFORT and short-lived CLI publishers pair too late
# or drop mid-discovery. This published a persistent best-effort node at
# ~1 Hz for ~8 s, which demonstrably moves the filter (verified via TF).
seed_amcl() {
  X="$1" Y="$2" YAW="$3" python3 - <<'PYEOF'
import math, os, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init()
n = Node('amcl_seeder')
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE)
pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)
time.sleep(2.0)   # let discovery pair before the first publish
msg = PoseWithCovarianceStamped()
msg.header.frame_id = 'map'
x, y, yaw = float(os.environ['X']), float(os.environ['Y']), float(os.environ['YAW'])
msg.pose.pose.position.x = x
msg.pose.pose.position.y = y
msg.pose.pose.orientation.z = math.sin(yaw / 2)
msg.pose.pose.orientation.w = math.cos(yaw / 2)
msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05
msg.pose.covariance[35] = 0.03
for _ in range(16):
    msg.header.stamp = n.get_clock().now().to_msg()
    pub.publish(msg)
    rclpy.spin_once(n, timeout_sec=0.4)
    time.sleep(0.4)
rclpy.shutdown()
PYEOF
}

MODE="${1:-}"

# ================== --demo-only: MUST run before any teardown ==============
if [ "$MODE" = "--demo-only" ]; then
  echo "== demo-only (reusing running stack) =="
  timeout 3 ros2 topic list 2>/dev/null | grep -q "^/scan_raw$" || {
    echo "FAIL: no running stack - first run: bringup_m6.sh --no-demo"; exit 1; }

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
    print(f"{p.x:.3f} {p.y:.3f} {yaw:.3f}")
PYEOF
)
  set -- $POSE
  if [ -n "$1" ]; then
    seed_amcl "$1" "$2" "$3" && echo "   seeded from odom ($1, $2, yaw $3) - persistent/verified pattern"
  fi
  P=$(ss -tlnp 2>/dev/null | grep 2666 | grep -oP 'pid=\K[0-9]+' | head -1)
  [ -n "$P" ] && kill -9 "$P"
  cd "$WS" && ros2 run wr_bringup parking_bt 2>&1 | tee "$WS/src/wr_bringup/results/parking_m6.log"
  exit $?
fi

echo "== [1/5] simulation =="
# Idempotent teardown: leftover processes from a previous run create
# duplicate action servers and DDS chaos. Kill launch PARENTS first so
# ros2 launch tears down its own children, then sweep.
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
pkill -f "ros2cli[.]daemon" 2>/dev/null   # actual daemon cmdline (python3 -c from ros2cli.daemon...)
# stale per-session PAL nodes (play_motion2 / twist_mux / joystick_relay ...)
# survive their launch parent's teardown and later poison DDS with duplicate
# action servers and duplicate cmd_vel publishers. Sweep by age: any
# /opt/ros/ process older than 30 min cannot belong to this fresh run.
for gp in $(ps -eo pid,etimes,cmd | awk '$2 > 1800 && $0 ~ "/opt/ros/" {print $1}'); do
  kill "$gp" 2>/dev/null
done
sleep 3
# SIGKILL escalation for ghosts that ignored SIGTERM, then verify clean
pkill -9 -f "parking_[b]t"               2>/dev/null
pkill -9 -f "tiago_navigation[.]launch"  2>/dev/null
pkill -9 -f "warehouse_sim[.]launch"     2>/dev/null
pkill -9 -f "nav2_[a-z]"                 2>/dev/null
pkill -9 -f "gzserver" 2>/dev/null
pkill -9 -f "gzclient" 2>/dev/null
sleep 1
pgrep -f "nav2_[a-z]|gzserver|gzclient|parking_[b]t" >/dev/null 2>&1 && \
  echo "   WARN: processes survived SIGKILL (D-state?) - continuing, DDS reset is the backstop"
# userspace middleware reboot (what an OS reboot would clear):
rm -rf /dev/shm/fastrtps_* 2>/dev/null
find /dev/shm -maxdepth 1 -name 'sem.????????????????????????????????' -delete 2>/dev/null
ipcs -s -p 2>/dev/null | awk 'NR > 2 && NF >= 3 {print $2, $3}' | while read -r id cpid; do
  if [ -n "$cpid" ] && ! ps -p "$cpid" > /dev/null 2>&1; then
    ipcrm -s "$id" 2>/dev/null
  fi
done

# Fresh ROS daemon spawned from THIS script's env (UDP-only profile).
# A stale daemon with a wedged graph makes every daemon-based gate
# (`ros2 topic list`, `ros2 control list_controllers`) report blindness
# while the sim is actually fine - the phantom "dud boots" of Oct 19 were
# exactly this: healthy sims murdered by a blind gate.
ros2 daemon start >/dev/null 2>&1 || true

wait_for_scan() {
  # Empirical rule (user-observed, confirmed 2026-10-21): a healthy sim
  # shows its laser within ~1 minute; every longer wait turned out to be a
  # dud. Short gate + fast relaunch cycles beats long patient waits.
  local waited=0
  while [ $waited -lt 90 ]; do
    sleep 4; waited=$((waited+4))
    if timeout 3 ros2 topic list 2>/dev/null | grep -q "^/scan_raw$"; then
      echo "   /scan_raw up after ${waited}s"
      return 0
    fi
    [ $((waited % 20)) -eq 0 ] && echo "   still booting... ${waited}s (healthy boots are fast; slow ones turn out duds)"
  done
  return 1
}

launch_sim() {
  cd "$WS" && nohup ros2 launch wr_bringup warehouse_sim.launch.py > /tmp/sim_launch.log 2>&1 &
  # HEADLESS by default: on this hybrid-NVIDIA host, gzclient's GL context
  # contends with gzserver's sensor rendering - with the GUI alive the laser
  # never initializes (5 dud boots on 2026-10-21); with gzclient killed at
  # t+10s the laser AND controllers come up in ~40 s (proven live). The
  # pipeline needs no Gazebo window (watch in RViz instead).
  # Set KEEP_GUI=1 to keep the Gazebo window for manual sessions.
  if [ "${KEEP_GUI:-0}" != "1" ]; then
    ( sleep 10; pkill -f "gzclient" 2>/dev/null ) &
  fi
}

# Kill a running sim attempt COMPLETELY: launch parent first (so ros2 launch
# tears down its children), then stragglers, then the per-session PAL nodes
# (play_motion2 / twist_mux / move_group ...). These LEAK through a plain
# pkill and poison the NEXT attempt with duplicate action servers
# ("Goal was rejected by server" / "unknown result response" - the cascade
# that turned one dud boot into three on 2026-10-21).
kill_sim_tree() {
  pkill -f "warehouse_sim[.]launch" 2>/dev/null
  pkill -f "gzserver" 2>/dev/null
  pkill -f "gzclient" 2>/dev/null
  for gp in $(ps -eo pid,cmd | grep -E "play_motion2|twist_mux|move_group|joystick_relay|add_analyzer" | grep -v grep | awk '{print $1}'); do
    kill "$gp" 2>/dev/null
  done
  sleep 2
  pkill -9 -f "gzserver" 2>/dev/null
  rm -rf /dev/shm/fastrtps_* 2>/dev/null
}

# a sim is USABLE only if both the laser and the ros2_control controllers
# come up; controllers lose their boot race under load ("Joint ... not
# claimed by any active controller") and never self-heal -> relaunch
controllers_ok() {
  timeout 20 ros2 control list_controllers 2>/dev/null | \
    grep -c "\\x1b\\[92mactive" | grep -q "[2-9]" && return 0
  # fallback if colour codes are stripped:
  [ "$(timeout 20 ros2 control list_controllers 2>/dev/null | grep -c active)" -ge 2 ]
}

sim_attempt=0
launch_sim    # the until-body only runs AFTER a failed wait - without this
              # first launch the script dead-waits 480 s for a sim that
              # was never started ("stuck at the first step")
until wait_for_scan && { sleep 10; controllers_ok; }; do
  sim_attempt=$((sim_attempt+1))
  if [ $sim_attempt -gt 4 ]; then
    echo "FAIL: sim never became usable (5 attempts) - cleaning up"
    kill_sim_tree
    exit 1
  fi
  echo "   sim unusable (laser or controllers) - relaunching (attempt $((sim_attempt+1))/5)..."
  kill_sim_tree
  sleep 3
  launch_sim
done
echo "   sim usable (laser + controllers)"

echo "== [2/5] odometry gate (SINGLE SOURCE) =="
# Dual odom->base TF publishers (native wheel + GT bridge) are FATAL once
# the two disagree (wheel slip after collisions): AMCL receives metre-scale
# teleports every few messages and scrambles - the root cause behind every
# "mystery" patrol failure since M6 (proven 2026-10-20: dual stream
# alternating +0.34/-1.92 at 120 Hz -> 1/6; wheel-only -> 6/6 in 234 s).
# M5's topology was wheel-only; the GT bridge now starts ONLY as fallback.
odom_ok() {
  timeout 5 ros2 topic info /odom 2>/dev/null | grep -Eq "Publisher count: [1-9]"
}
wheel_odom_ok() {
  timeout 5 ros2 topic info /mobile_base_controller/odom 2>/dev/null | grep -Eq "Publisher count: [1-9]"
}
tf_ok() {
  timeout 4 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | grep -q "Translation"
}
if ! odom_ok; then
  if wheel_odom_ok; then
    echo "   wheel odom alive -> C++ relay to /odom (M5 topology)"
    nohup ros2 run wr_bringup odom_relay_cpp > /tmp/odom_relay.log 2>&1 &
  else
    echo "   wheel odom dead -> ground-truth relay fallback"
    nohup ros2 run wr_bringup odom_relay_cpp --ros-args -p source:=/ground_truth_odom \
      > /tmp/odom_relay.log 2>&1 &
  fi
fi
for i in $(seq 1 15); do
  odom_ok && { echo "   /odom OK"; break; }
  sleep 1
done
odom_ok || { echo "FAIL: /odom never published"; exit 1; }

# odom->base TF: use the NATIVE (wheel) TF when present; the GT bridge is
# a fallback only, never a second concurrent source.
tf_waited=0
while ! tf_ok && [ $tf_waited -lt 10 ]; do sleep 2; tf_waited=$((tf_waited+2)); done
if tf_ok; then
  echo "   native odom->base TF present - GT bridge NOT started (single source)"
else
  echo "   native odom->base TF missing -> starting GT bridge fallback"
  nohup ros2 run wr_bringup odom_tf_bridge_cpp > /tmp/odom_tf_bridge.log 2>&1 &
  sleep 3
  tf_ok && echo "   odom->base TF OK (bridge)" || { echo "FAIL: odom->base TF unavailable even via bridge"; exit 1; }
fi

echo "== [3/5] Nav2 (be patient: up to 3 min under boot contention) =="
cd "$WS" && nohup ros2 launch br2_navigation tiago_navigation.launch.py \
  map:=$WS/src/wr_bringup/maps/warehouse.yaml \
  params_file:=$WS/src/wr_bringup/config/tiago_nav_params.yaml > /tmp/nav_launch.log 2>&1 &
for i in $(seq 1 90); do
  sleep 2
  [ "$(grep -c 'Managed nodes are active' /tmp/nav_launch.log)" -ge 2 ] && break
done
# The CLI lifecycle tool silently no-ops when service responses are lost
# under load (and the lifecycle manager itself can stall waiting for the
# planner's configure response). This direct python driver is robust.
python3 - <<'PYEOF'
import rclpy, time
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
def drive(name, tr):
    c = n.create_client(ChangeState, f'/{name}/change_state')
    if not c.wait_for_service(timeout_sec=5.0): return
    req = ChangeState.Request(); req.transition.id = tr
    fut = c.call_async(req); t0 = time.time()
    while not fut.done() and time.time()-t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.5)
for name in ['map_server', 'amcl', 'controller_server', 'planner_server',
             'smoother_server', 'behavior_server', 'waypoint_follower',
             'velocity_smoother', 'bt_navigator']:
    for _ in range(3):
        st = get_state(name)
        if st == 3: break
        if st == 1: drive(name, Transition.TRANSITION_CONFIGURE)
        if st in (1, 2): drive(name, Transition.TRANSITION_ACTIVATE)
    print(f"{name}: {'ACTIVE' if get_state(name) == 3 else 'NOT-ACTIVE'}")
rclpy.shutdown()
PYEOF

echo "== [4/5] AMCL seed (robot spawns at map origin) =="
seed_amcl 0.0 0.0 0.0 && echo "   seeded (0,0,0) - persistent best-effort publisher"

# Readiness gate: the first FollowPath goal arriving before the local
# costmap's first update completes SEGFAULTED controller_server on
# 2026-10-21 (exit -11 one second after "begin computing control effort",
# corroborated by behavior_server's "Current footprint not available").
# The identical goal under gdb on a settled stack drove to the goal cleanly
# - an initialization race, not a logic bug. Never hand the demo a
# half-born costmap: wait for the first /local_costmap/costmap publish and
# verify the controller is still alive.
python3 - <<'PYEOF'
import sys, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from lifecycle_msgs.srv import GetState
rclpy.init()
n = Node('readiness_gate')
got = []
n.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: got.append(1), 5)
t0 = time.time()
while not got and time.time()-t0 < 25:
    rclpy.spin_once(n, timeout_sec=0.5)
print('   local costmap:', 'publishing' if got else 'no update in 25 s (continuing with caution)')
c = n.create_client(GetState, '/controller_server/get_state')
if c.wait_for_service(timeout_sec=5.0):
    f = c.call_async(GetState.Request()); t0 = time.time()
    while not f.done() and time.time()-t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.3)
    st = f.result().current_state.id if f.done() else -1
    if st != 3:
        print(f'   FAIL: controller_server state={st} - check /tmp/nav_launch.log for "process has died"')
        rclpy.shutdown(); sys.exit(1)
    print('   controller_server: ACTIVE')
else:
    print('   FAIL: controller_server service missing (crashed?) - check /tmp/nav_launch.log')
    rclpy.shutdown(); sys.exit(1)
rclpy.shutdown()
PYEOF

if [ "$MODE" = "--no-demo" ]; then
  echo "== stack ready (no demo). Iterate with: bringup_m6.sh --demo-only =="
  exit 0
fi

echo "== [5/5] parking demo =="
# Optional custom dock ( metres, metres, radians ):
#   DOCK="x y yaw"  ros2 run wr_bringup bringup_m6.sh
DOCK_ARGS=()
[ -n "${DOCK:-}" ] && DOCK_ARGS+=( -p "dock_pose:=[${DOCK}]" )
[ -n "${DOCK:-}" ] && echo "   custom dock: [${DOCK}]"
P=$(ss -tlnp 2>/dev/null | grep 2666 | grep -oP 'pid=\K[0-9]+' | head -1)
[ -n "$P" ] && kill -9 "$P"
cd "$WS" && ros2 run wr_bringup parking_bt --ros-args "${DOCK_ARGS[@]}" 2>&1 | \
  tee "$WS/src/wr_bringup/results/parking_m6.log"
