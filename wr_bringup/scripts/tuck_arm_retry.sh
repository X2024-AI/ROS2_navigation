#!/usr/bin/env bash
# Robust arm tuck for TIAGo.
#
# The sim's built-in tuck_arm.py races play_motion2 startup under boot
# contention and can fail (goal rejected -> arm left sticking out).
# This wrapper waits out the boot storm, then retries the tuck until it
# succeeds. Tucking an already-tucked arm is harmless, so it is safe to
# run this in addition to the built-in attempt.
WS="${WR_WS:-$HOME/Codes/Ros2/bookros2_ws}"   # override with WR_WS if your workspace lives elsewhere
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "[tuck_retry] waiting out the boot storm..."
for i in $(seq 1 12); do
  sleep 10
  # tuck_arm.py blocks until the motion finishes; give it 2 min each try
  if timeout 120 ros2 run tiago_gazebo tuck_arm.py >/dev/null 2>&1; then
    echo "[tuck_retry] arm tucked (attempt $i)"
    exit 0
  fi
  echo "[tuck_retry] attempt $i failed, retrying..."
done
echo "[tuck_retry] gave up after 12 attempts"
exit 1
