#!/usr/bin/env bash
set -eo pipefail
# NOTE: we intentionally do NOT enable 'set -u' until after sourcing ROS

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$WS_DIR/src/p2_remote_connection"

API_HOST="0.0.0.0"
API_PORT="8000"
UI_PORT="8081"

# Only force HOME on the real Go2 user if needed
if [ "$(whoami)" = "unitree" ] && [ -d "/home/unitree" ]; then
  export HOME="/home/unitree"
fi

pids=()

cleanup() {
  echo ""
  echo "[run_all] Stopping processes..."
  for pid in "${pids[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  sleep 0.5 || true
  for pid in "${pids[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

ok_or_die() {
  local name="$1"
  local pid="$2"
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "[run_all] Error occurred ❌ ($name failed to start)"
    cleanup
    exit 1
  fi
}

echo "[run_all] Workspace: $WS_DIR"

# --- Source ROS safely ---
set +u

# 0) Change to fast rtps
use_fastrtps

# 1) Source ROS 2 environment (Humble preferred, fallback to Foxy)
if [ -f /opt/ros/humble/setup.bash ]; then
  echo "[env] Sourcing ROS 2 Humble"
  source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash ]; then
  echo "[env] Sourcing ROS 2 Foxy"
  source /opt/ros/foxy/setup.bash
else
  echo "[env] ❌ No ROS 2 setup.bash found in /opt/ros"
  exit 1
fi

# 2) Unitree env: (SIM) intentionally not sourced
# (keep your sim-unique behavior)

# 3) Your overlay (P2RemoteConnection)
if [ -f "$WS_DIR/install/setup.bash" ]; then
  echo "[run_all] Sourcing overlay: $WS_DIR/install/setup.bash"
  source "$WS_DIR/install/setup.bash"
else
  echo "[run_all] WARNING: overlay not found: $WS_DIR/install/setup.bash (did you colcon build?)"
fi

set -u

# --- SIM: avoid CycloneDDS interface pinning issues (KEEP) ---
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ----------------------------
# Load Go2 API token (NEW - same as Go2)
# ----------------------------
if [ -f "$HOME/.go2_token" ]; then
  export GO2_API_TOKEN="$(cat "$HOME/.go2_token")"
else
  echo "[run_all] ❌ ERROR: ~/.go2_token not found"
  exit 1
fi
echo "[run_all] GO2_API_TOKEN loaded"

# 1) Start FastAPI backend
echo "[run_all] Starting FastAPI (uvicorn) on :$API_PORT ..."
cd "$PKG_DIR"
python3 -m uvicorn app.main:app --host "$API_HOST" --port "$API_PORT" &
API_PID=$!
pids+=("$API_PID")

sleep 0.5
ok_or_die "FastAPI" "$API_PID"

# 2) Start static file server for frontend
echo "[run_all] Starting http.server on :$UI_PORT ..."
python3 -m http.server "$UI_PORT" &
UI_PID=$!
pids+=("$UI_PID")

# 3) Wait for Go2 driver before starting bridge (KEEP)
echo "[run_all] Waiting for /go2_driver_node..."
for i in {1..30}; do
  if ros2 node list 2>/dev/null | grep -q "^/go2_driver_node$"; then
    echo "[run_all] go2_driver_node is up."
    break
  fi
  sleep 1
done

# 4) Start SIM bridge node (prefer launching the binary directly)
SIM_BRIDGE_BIN="$WS_DIR/install/p2_remote_connection/lib/p2_remote_connection/web_teleop_bridge_sim"

if [ -x "$SIM_BRIDGE_BIN" ]; then
  echo "[run_all] Starting SIM bridge: $SIM_BRIDGE_BIN"
  "$SIM_BRIDGE_BIN" --ros-args -p robot_index:=0 &
  SIM_BRIDGE_PID=$!
  pids+=("$SIM_BRIDGE_PID")
else
  echo "[run_all] Starting SIM bridge via ros2 run (fallback)"
  cd "$WS_DIR"
  ros2 run p2_remote_connection web_teleop_bridge_sim --ros-args -p robot_index:=0 &
  SIM_BRIDGE_PID=$!
  pids+=("$SIM_BRIDGE_PID")
fi

sleep 0.5
ok_or_die "UI server" "$UI_PID"
ok_or_die "SIM bridge" "$SIM_BRIDGE_PID"

echo ""
echo "[run_all] ✅ All started."
echo "[run_all] UI:  http://<this-machine-ip>:$UI_PORT/app/go2_joystick.html"
echo "[run_all] API: http://<this-machine-ip>:$API_PORT"
echo "[run_all] Press Ctrl+C to stop everything."
echo ""

wait -n
echo "[run_all] A process exited; shutting down..."
exit 0
