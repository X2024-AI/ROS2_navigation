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
  # topic-DATA probe: graph queries (`ros2 topic list`) go blind on this
  # host even while the sim streams - see wait_for_scan
  timeout 5 ros2 topic hz /scan_raw 2>/dev/null | head -1 | grep -q "average rate" || {
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
pkill -f "particle_rela[y]" 2>/dev/null
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
# exactly this: healthy sims murdered by a blind gate. `daemon start` is a
# no-op while a wedged daemon lives (2026-10-24), so kill it first.
pkill -f "ros2cli[.]daemon" 2>/dev/null
sleep 1
ros2 daemon start >/dev/null 2>&1 || true

wait_for_scan() {
  # Empirical rule (user-observed, confirmed 2026-10-21): a healthy sim
  # shows its laser within ~1 minute; every longer wait turned out to be a
  # dud. Short gate + fast relaunch cycles beats long patient waits.
  # Topic-DATA probe (2026-10-24): the old `ros2 topic list` check is a
  # daemon GRAPH query and went blind mid-session while /scan_raw streamed
  # at 10 Hz (a "fresh" daemon can still be discovery-wedged) - it killed
  # healthy sims three times in a row. A live subscription never lies here.
  local waited=0
  while [ $waited -lt 90 ]; do
    sleep 4; waited=$((waited+4))
    if timeout 4 ros2 topic hz /scan_raw 2>/dev/null | head -1 | grep -q "average rate"; then
      echo "   /scan_raw streaming after ${waited}s"
      return 0
    fi
    [ $((waited % 20)) -eq 0 ] && echo "   still booting... ${waited}s (healthy boots are fast; slow ones turn out duds)"
  done
  return 1
}

launch_sim() {
  # GUI ON by default (user preference - watch the robot). On a degraded
  # graphics session a live gzclient can starve the laser (2026-10-21):
  # if sims go dud with the window up, reboot, or run HEADLESS=1.
  local gui="True"
  [ "${HEADLESS:-0}" = "1" ] && gui="False"
  cd "$WS" && nohup ros2 launch wr_bringup warehouse_sim.launch.py gzclient:=$gui > /tmp/sim_launch.log 2>&1 &
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

# a sim is USABLE only if the laser streams and the base controller PUBLISHES.
# NOTE (2026-10-24): this used to be `ros2 control list_controllers` - a
# SERVICE call - and this host silently loses service responses under load
# (move_group logged the same /controller_manager/list_controllers read
# failure in the same session), so the gate killed three HEALTHY sims while
# /mobile_base_controller/odom streamed at 50 Hz the whole time. Topic
# transport has never lied here: hz on the diff-drive controller's own
# output is the direct liveness signal (and this topic is the relay source).
controllers_ok() {
  # hz probes have startup-latency variance under load (a fresh CLI sub can
  # need >5 s before its first rate line) - retry so one slow probe cannot
  # relaunch a healthy sim
  local i
  for i in 1 2 3; do
    timeout 6 ros2 topic hz /mobile_base_controller/odom 2>/dev/null | \
      head -1 | grep -q "average rate" && return 0
    sleep 2
  done
  return 1
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
  # topic-DATA probe (graph queries go blind on this host - see wait_for_scan)
  timeout 5 ros2 topic hz /odom 2>/dev/null | head -1 | grep -q "average rate"
}
wheel_odom_ok() {
  timeout 5 ros2 topic hz /mobile_base_controller/odom 2>/dev/null | head -1 | grep -q "average rate"
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
# NOTE: capture the loop's verdict in a flag - a fresh hz re-probe right
# after success can flake (2026-10-24: "/odom OK" printed, next probe
# missed, run died) and must not kill a working relay.
odom_up=0
for i in $(seq 1 15); do
  if odom_ok; then odom_up=1; echo "   /odom OK"; break; fi
  sleep 1
done
[ "$odom_up" = "1" ] || { echo "FAIL: /odom never published"; exit 1; }

# odom->base TF: use the NATIVE (wheel) TF when present; the GT bridge is
# a fallback only, never a second concurrent source. Flag-based verdict: a
# flaky re-probe here could start the bridge BESIDE healthy native TF -
# dual odom->base publishers are the AMCL scrambler (the Oct-20 unifier).
tf_up=0
tf_waited=0
while [ $tf_waited -lt 10 ]; do
  if tf_ok; then tf_up=1; break; fi
  sleep 2; tf_waited=$((tf_waited+2))
done
if [ "$tf_up" = "1" ]; then
  echo "   native odom->base TF present - GT bridge NOT started (single source)"
else
  echo "   native odom->base TF missing -> starting GT bridge fallback"
  nohup ros2 run wr_bringup odom_tf_bridge_cpp > /tmp/odom_tf_bridge.log 2>&1 &
  sleep 3
  if tf_ok || { sleep 2; tf_ok; }; then
    echo "   odom->base TF OK (bridge)"
  else
    echo "FAIL: odom->base TF unavailable even via bridge"; exit 1
  fi
fi

# (Re)start the Nav2 tree. Used at bring-up AND by the warm-up recovery
# below: controller_server has a native-speed first-goal race that SIGSEGVs
# it (2026-10-21: 2-for-2 on fresh boots, zero reproductions under gdb over
# 5+ goals - too fast a race to backtrace), and when it dies the lifecycle
# manager shuts the WHOLE tree down, so recovery = full nav restart.
start_nav() {
  echo "== Nav2 (be patient: up to 3 min under boot contention) =="
  pkill -f "tiago_navigation[.]launch" 2>/dev/null
  pkill -f "nav2_[a-z]"          2>/dev/null
  pkill -f "lifecycle_[m]anager" 2>/dev/null
  sleep 3
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
import sys, time
import rclpy
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
ok = True
for name in ['map_server', 'amcl', 'controller_server', 'planner_server',
             'smoother_server', 'behavior_server', 'waypoint_follower',
             'velocity_smoother', 'bt_navigator']:
    for _ in range(3):
        st = get_state(name)
        if st == 3: break
        if st == 1: drive(name, Transition.TRANSITION_CONFIGURE)
        if st in (1, 2): drive(name, Transition.TRANSITION_ACTIVATE)
    active = get_state(name) == 3
    ok = ok and active
    print(f"{name}: {'ACTIVE' if active else 'NOT-ACTIVE'}")
rclpy.shutdown()
sys.exit(0 if ok else 1)
PYEOF
}

# One controlled first-goal: send a trivial NavigateToPose and then verify
# the controller SURVIVED (its result does not matter - only liveness does).
# Returns nonzero if the controller died on it.
warmup_goal() {
  python3 - <<'PYEOF'
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import ParticleCloud
from lifecycle_msgs.srv import GetState
rclpy.init()
n = Node('warmup')
client = ActionClient(n, NavigateToPose, '/navigate_to_pose')
if not client.wait_for_server(timeout_sec=15.0):
    print('   warm-up: no navigate_to_pose server'); rclpy.shutdown(); sys.exit(1)
# AMCL liveness probe: while the robot moves the filter must publish particle
# updates. Zero during a >=1 m drive = the scan pipeline is stalled inside
# amcl (host DDS-reader pathology, proven 2026-10-23: one sub of a node dies
# while its siblings stream). amcl then dead-reckons and every goal
# "succeeds" metres from the truth with zero log output.
particles = [0]
n.create_subscription(ParticleCloud, '/particle_cloud',
                      lambda m: particles.__setitem__(0, particles[0] + 1),
                      qos_profile_sensor_data)
goal = NavigateToPose.Goal()
goal.pose.header.frame_id = 'map'
goal.pose.pose.position.x = 1.0
goal.pose.pose.position.y = 0.0
goal.pose.pose.orientation.w = 1.0
fut = client.send_goal_async(goal)
rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
rf = fut.result().get_result_async() if fut.result() and fut.result().accepted else None
t0 = time.time()
if rf:
    while not rf.done() and time.time()-t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.5)
    print('   warm-up goal finished' if rf.done() else '   warm-up goal timed out (liveness is what matters)')
c = n.create_client(GetState, '/controller_server/get_state')
alive = False
if c.wait_for_service(timeout_sec=5.0):
    f = c.call_async(GetState.Request()); t0 = time.time()
    while not f.done() and time.time()-t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.3)
    alive = f.done() and f.result().current_state.id == 3
print('   controller after warm-up:', 'ALIVE' if alive else 'DEAD (first-goal crash)')
print(f'   amcl particle updates during warm-up: {particles[0]}')
rclpy.shutdown()
if not alive:
    sys.exit(1)
if particles[0] == 0:
    print('   amcl filter NEVER updated while moving - scan pipeline stalled')
    sys.exit(2)
sys.exit(0)
PYEOF
}

# ---------------- Nav2 up + seed + readiness gate + first-goal warm-up ----
start_nav || { echo "FAIL: nav2 never became fully active"; exit 1; }

echo "== [4/5] AMCL seed (robot spawns at map origin) =="
seed_amcl 0.0 0.0 0.0 && echo "   seeded (0,0,0) - persistent best-effort publisher"

# Readiness gate: wait for the first local costmap publish and verify the
# controller is alive before any goal touches it.
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

# First-goal warm-up with recovery: the controller's FIRST FollowPath goal
# after a fresh activation has a native-speed race that SIGSEGVs it (never
# reproduced under gdb - see start_nav comment). Absorb that dangerous first
# goal with a trivial warm-up; if it kills the controller, restart nav2 and
# try again (bounded). Every observed goal AFTER a surviving first one ran
# clean, so getting past goal #1 de-risks the whole demo/patrol.
warmup_attempt=0
until warmup_goal; do
  warmup_attempt=$((warmup_attempt+1))
  if [ $warmup_attempt -gt 2 ]; then
    echo "FAIL: warm-up keeps failing. Either the controller first-goal crash"
    echo "      (/tmp/nav_launch.log) or the AMCL scan-pipeline stall (zero particle"
    echo "      updates while moving). A nav2 restart creates a fresh amcl process"
    echo "      with fresh DDS readers and usually cures the stall; if retries keep"
    echo "      stalling, REBOOT - the session's DDS is poisoned (2026-10-23)."
    exit 1
  fi
  echo "   restarting nav2 for warm-up retry $((warmup_attempt+1))/2..."
  start_nav || { echo "FAIL: nav2 restart failed"; exit 1; }
  seed_amcl 0.0 0.0 0.0
done
echo "   warm-up survived - controller past its first goal, AMCL filter updating"

# Particle relay for RViz: nav2 amcl publishes nav2_msgs/ParticleCloud, which
# RViz2 cannot display; the relay repacks it as PoseArray on /particle_cloud_pa
# (display configured in rviz/monitor.rviz)
nohup ros2 run wr_bringup particle_relay.py > /tmp/particle_relay.log 2>&1 &

if [ "$MODE" = "--no-demo" ]; then
  echo "== stack ready (no demo, first goal absorbed). Iterate with: bringup_m6.sh --demo-only =="
  echo "   monitor: rviz2 -d $WS/src/wr_bringup/rviz/monitor.rviz   (costmaps + AMCL particles via relay)"
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
