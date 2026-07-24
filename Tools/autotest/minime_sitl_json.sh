#!/bin/bash
#
# AS-10 MiniMe SITL with JSON Physics Model
#
# This script launches the MiniMe physics model and ArduPilot SITL together.
#
# Usage:
#   ./Tools/autotest/minime_sitl_json.sh
#
# The script will:
#   1. Start the MiniMe Python physics model
#   2. Start ArduPilot SITL with JSON frame type
#   3. Connect both via UDP on port 9002
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

cd "$PROJECT_ROOT"

PHYSICS_SCRIPT="$SCRIPT_DIR/minime_physics.py"
PARAM_FILE="$SCRIPT_DIR/default_params/copter-heli-minime.parm"

if [ ! -f "$PHYSICS_SCRIPT" ]; then
    echo "Error: Physics model not found at $PHYSICS_SCRIPT"
    exit 1
fi

if [ ! -f "$PARAM_FILE" ]; then
    echo "Error: Parameter file not found at $PARAM_FILE"
    exit 1
fi

cleanup() {
    echo ""
    echo "Shutting down..."
    if [ -n "$PHYSICS_PID" ]; then
        kill $PHYSICS_PID 2>/dev/null || true
    fi
    if [ -n "$SITL_PID" ]; then
        kill $SITL_PID 2>/dev/null || true
    fi
    exit 0
}

trap cleanup INT TERM

echo "============================================================"
echo "AS-10 MiniMe SITL with JSON Physics Model"
echo "============================================================"
echo ""
echo "Starting physics model..."

source .venv/bin/activate 2>/dev/null || true

python3 "$PHYSICS_SCRIPT" --fps 400 &
PHYSICS_PID=$!

sleep 2

if ! kill -0 $PHYSICS_PID 2>/dev/null; then
    echo "Error: Physics model failed to start"
    exit 1
fi

echo "Physics model running (PID: $PHYSICS_PID)"
echo ""
echo "Starting ArduPilot SITL..."

python3 Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    --model JSON \
    --add-param-file "$PARAM_FILE" \
    --no-mavproxy \
    -w &
SITL_PID=$!

echo "SITL starting (PID: $SITL_PID)"
echo ""
echo "============================================================"
echo "MiniMe SITL is running!"
echo ""
echo "Connect with:"
echo "  MAVProxy: mavproxy.py --master=tcp:127.0.0.1:5760"
echo "  QGC: TCP connection to 127.0.0.1:5760"
echo ""
echo "Press Ctrl+C to stop"
echo "============================================================"

wait $SITL_PID
cleanup
