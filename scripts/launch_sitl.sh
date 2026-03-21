#!/usr/bin/env bash
# launch_sitl.sh – Start ArduPilot SITL in Gazebo and configure MAVProxy
# for use with this obstacle avoidance system.
#
# Prerequisites:
#   - ArduPilot source checked out (set ARDUPILOT_HOME below)
#   - Gazebo + ardupilot_gazebo plugin installed
#   - MAVProxy installed (pip install MAVProxy)
#
# Usage:
#   ./scripts/launch_sitl.sh            # default copter, gazebo world
#   ./scripts/launch_sitl.sh --no-gcs   # no MAVProxy GCS window
#   ./scripts/launch_sitl.sh --headless # no Gazebo GUI

set -e

# ── Configuration ─────────────────────────────────────────────────────────────
ARDUPILOT_HOME="${ARDUPILOT_HOME:-$HOME/ardupilot}"
GAZEBO_WORLD="${GAZEBO_WORLD:-default}"
VEHICLE="ArduCopter"
FRAME="-f gazebo-iris"
SITL_PORT=5760         # TCP port for SITL
MAVSDK_PORT=14540      # UDP port MAVSDK listens on (udpin://0.0.0.0:14540)
GCS_PORT=14550         # UDP port for GCS (QGroundControl etc.)

HEADLESS=0
NO_GCS=0

# ── Parse arguments ────────────────────────────────────────────────────────────
for arg in "$@"; do
    case $arg in
        --headless) HEADLESS=1 ;;
        --no-gcs)   NO_GCS=1 ;;
    esac
done

echo "======================================================"
echo "  ArduPilot SITL + Gazebo launcher"
echo "  ARDUPILOT_HOME : $ARDUPILOT_HOME"
echo "  Vehicle        : $VEHICLE"
echo "  MAVSDK port    : $MAVSDK_PORT (udpin://0.0.0.0:$MAVSDK_PORT)"
echo "======================================================"

# ── Launch Gazebo ──────────────────────────────────────────────────────────────
if [ $HEADLESS -eq 0 ]; then
    echo "[1/3] Starting Gazebo..."
    # Source Gazebo environment if needed
    if [ -f /usr/share/gazebo/setup.sh ]; then
        source /usr/share/gazebo/setup.sh
    fi

    gazebo --verbose worlds/iris_arducopter_runway.world &
    GAZEBO_PID=$!
    sleep 5
    echo "      Gazebo PID: $GAZEBO_PID"
else
    echo "[1/3] Headless mode – skipping Gazebo GUI"
    export DISPLAY=:0  # needed for some headless configs
fi

# ── Launch ArduPilot SITL ──────────────────────────────────────────────────────
echo "[2/3] Starting ArduPilot SITL..."
cd "$ARDUPILOT_HOME"

# sim_vehicle.py handles both SITL binary and MAVProxy
SITL_ARGS="$FRAME"
SITL_ARGS+=" --console"
SITL_ARGS+=" --out=udp:127.0.0.1:$MAVSDK_PORT"   # MAVSDK companion computer output
SITL_ARGS+=" --out=udp:127.0.0.1:$GCS_PORT"       # GCS output (QGC)
SITL_ARGS+=" --map"

if [ $NO_GCS -eq 1 ]; then
    SITL_ARGS+=" --no-mavproxy"
fi

python3 Tools/autotest/sim_vehicle.py \
    -v "$VEHICLE" \
    $SITL_ARGS \
    &

SITL_PID=$!
echo "      SITL PID: $SITL_PID"

# Wait for SITL to be ready
echo "      Waiting for SITL to become ready..."
sleep 8

# ── Configure ArduPilot parameters for offboard / companion computer ───────────
echo "[3/3] Configuring ArduPilot parameters..."
# These can also be set via MAVProxy or QGC.
# Key params for offboard (GUIDED mode / MAVSDK):
#   SYSID_MYGCS=1       – trust GCS with sysid=1
#   FS_GCS_ENABLE=0     – disable GCS failsafe for testing (re-enable in production!)
#   ARMING_CHECK=0      – disable arming checks for SITL (remove in production!)
mavproxy.py --master=tcp:127.0.0.1:$SITL_PORT --cmd="
    param set SYSID_MYGCS 1;
    param set FS_GCS_ENABLE 0;
    param set ARMING_CHECK 0;
    param set WPNAV_SPEED 500;
    param set WPNAV_ACCEL 200;
    exit" 2>/dev/null || echo "      (mavproxy param set skipped – configure manually)"

echo ""
echo "======================================================"
echo "  SITL is ready."
echo "  Run obstacle_avoidance to connect:"
echo "    ./build/obstacle_avoidance config/params.json"
echo ""
echo "  Or with QGroundControl on port $GCS_PORT"
echo "======================================================"

# Keep script alive until Ctrl-C
wait $SITL_PID
