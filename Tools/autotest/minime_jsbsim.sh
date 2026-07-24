#!/bin/bash
#
# AS-10 MiniMe SITL with JSBSim Physics Model
#
# This script launches ArduPilot SITL with the MiniMe JSBSim model.
#
# Usage:
#   ./Tools/autotest/minime_jsbsim.sh
#
# The script will:
#   1. Verify the JSBSim model exists
#   2. Start ArduPilot SITL with JSBSim backend
#   3. Load MiniMe parameters
#
# Connect with:
#   MAVProxy: mavproxy.py --master=tcp:127.0.0.1:5760
#   QGC: TCP connection to 127.0.0.1:5760
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

cd "$PROJECT_ROOT"

MODEL_DIR="$SCRIPT_DIR/aircraft/minime"
PARAM_FILE="$SCRIPT_DIR/default_params/copter-heli-minime.parm"

if [ ! -d "$MODEL_DIR" ]; then
    echo "Error: JSBSim model not found at $MODEL_DIR"
    exit 1
fi

if [ ! -f "$MODEL_DIR/minime.xml" ]; then
    echo "Error: minime.xml not found in $MODEL_DIR"
    exit 1
fi

if [ ! -f "$PARAM_FILE" ]; then
    echo "Error: Parameter file not found at $PARAM_FILE"
    exit 1
fi

echo "============================================================"
echo "AS-10 MiniMe SITL with JSBSim Physics Model"
echo "============================================================"
echo ""
echo "Model: $MODEL_DIR/minime.xml"
echo "Parameters: $PARAM_FILE"
echo ""
echo "Starting ArduPilot SITL with JSBSim..."
echo ""

python3 Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f jsbsim:minime \
    --add-param-file "$PARAM_FILE" \
    --console --map \
    -w

echo ""
echo "============================================================"
echo "SITL terminated"
echo "============================================================"
