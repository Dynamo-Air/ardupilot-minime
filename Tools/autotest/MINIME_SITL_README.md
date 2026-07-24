# AS-10 MiniMe SITL Testing

This directory contains tools for Software-In-The-Loop (SITL) testing of the AS-10 MiniMe tandem helicopter.

## Overview

ArduPilot SITL supports multiple physics backends for helicopter simulation:

1. **Internal Physics** (`heli-dual` frame): ArduPilot's built-in helicopter dynamics model (SIM_Helicopter.cpp)

2. **JSON Interface** (`JSON-minime` frame): External physics via UDP JSON protocol, enabling custom physics models

3. **JSBSim**: Industry standard flight dynamics library with rotor modeling capability (FGRotor class). JSBSim is primarily configured for ArduPlane in the standard ArduPilot distribution, but has helicopter rotor support. A JSBSim model for MiniMe is included in `aircraft/minime/`.

For the MiniMe project:
- **Quick testing**: Use `heli-dual` with MiniMe parameters
- **Custom physics**: Use `JSON-minime` with `minime_physics.py`
- **Industry standard FDM**: Use JSBSim model (requires additional configuration)

## Vehicle Parameters

All physics models use these parameters from SPECS.md:

| Parameter | Value (SI) | Value (Imperial) | Notes |
|-----------|------------|------------------|-------|
| Mass | 97.5 kg | 214.94 lbm | MGTOW |
| Ixx | 8.54 kg m^2 | 6.30 slug ft^2 | Very low, responsive roll |
| Iyy | 180.76 kg m^2 | 133.35 slug ft^2 | High pitch inertia |
| Izz | 180.76 kg m^2 | 133.35 slug ft^2 | High yaw inertia |
| Hub distance | 2.5 m | 8.2 ft | Hub to hub span |
| Rotor diameter | 2.0 m | 6.56 ft | Per rotor |
| Blades per rotor | 3 | 3 | 6 total |
| Hover RPM | 1074.2 | 1074.2 | Rotor RPM |
| Frame type | Tandem | Tandem | Counter rotating hingeless |

## Quick Start

### Option A: Internal Physics (Fastest Setup)

Uses ArduPilot's built-in helicopter dynamics with MiniMe parameters:

```bash
cd /path/to/ardupilot-minime
python3 Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f heli-dual \
    --add-param-file Tools/autotest/default_params/copter-heli-minime.parm \
    --console --map
```

### Option B: JSON Physics Model (Custom Dynamics)

Uses the Python physics model for MiniMe specific dynamics:

Terminal 1 (Physics Model):
```bash
cd /path/to/ardupilot-minime
source .venv/bin/activate
python3 Tools/autotest/minime_physics.py --fps 400
```

Terminal 2 (ArduPilot SITL):
```bash
python3 Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f JSON-minime \
    --add-param-file Tools/autotest/default_params/copter-heli-minime.parm \
    --console --map
```

Or use the launcher:
```bash
./Tools/autotest/minime_sitl_json.sh
```

### Option C: JSBSim Model (Industry Standard FDM)

JSBSim provides full aerodynamic modeling with the FGRotor rotor simulation class. A MiniMe JSBSim model is included in `aircraft/minime/`.

Reference: [JSBSim FGRotor Class](https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGRotor.html)

### Connecting

After SITL starts, connect with:

MAVProxy:
```bash
mavproxy.py --master=tcp:127.0.0.1:5760
```

QGroundControl:
1. Open QGC
2. Application Settings > Comm Links
3. Add new TCP link: 127.0.0.1:5760

## Files

| File | Purpose |
|------|---------|
| `aircraft/minime/minime.xml` | JSBSim aircraft definition |
| `aircraft/minime/Engines/` | JSBSim rotor and engine definitions |
| `aircraft/minime/Systems/` | JSBSim tandem control system |
| `minime_physics.py` | Python physics model (JSON interface) |
| `minime_sitl_json.sh` | Python model SITL launcher |
| `minime_hitl.py` | HITL bridge for SimOnHardWare firmware |
| `default_params/copter-heli-minime.parm` | ArduPilot parameters for MiniMe |
| `minime_test_modes_sitl.py` | Test script for H_RSC_MODE and cyclic tests |
| `minime_failsafe_test.py` | Automated failsafe testing |

## Physics Model Details

### Internal Physics (heli-dual)

ArduPilot's SIM_Helicopter.cpp provides:
- Basic rotor dynamics
- Swashplate to blade pitch conversion
- Ground effect
- Dual rotor support for tandem configurations

### Python JSON Model (minime_physics.py)

Custom physics implementing:
- Tandem rotor dynamics with counter-rotating heads
- Tip path plane (TPP) dynamics
- Rotor RPM response with spool-up/down
- Ground contact with collision response
- Correct mass properties from SPECS.md

### JSBSim Model (aircraft/minime/)

Industry standard FDM with:
- Full 6-DOF dynamics
- FGRotor rotor aerodynamics
- Configurable blade parameters
- Ground effect modeling
- Tandem control system (differential collective for pitch, differential cyclic for yaw)

## SimOnHardWare HITL

For Hardware-In-The-Loop testing on real Cube Orange+ with SimOnHardWare firmware:

1. Flash SimOnHardWare firmware via QGC
2. Connect Cube via USB
3. Run the HITL bridge:

```bash
source .venv/bin/activate
python3 Tools/autotest/minime_hitl.py --device /dev/cu.usbmodem*
```

The HITL bridge sends HIL_SENSOR, HIL_GPS, and HIL_STATE messages to the Cube based on the physics model.

Reference: [Simulation on Hardware](https://ardupilot.org/dev/docs/sim-on-hardware.html)

## Control Mapping

Servo channels for heli-dual frame:

| Channel | Function |
|---------|----------|
| 1 to 3 | Forward rotor swashplate |
| 4 to 6 | Aft rotor swashplate |
| 8 | Motor interlock / RSC |

### Tandem Control Philosophy

| Axis | Method | Notes |
|------|--------|-------|
| Roll | Symmetric lateral cyclic | Both rotors tilt same direction |
| Pitch | Differential collective | Fwd/aft thrust difference |
| Yaw | Differential lateral cyclic | eta_yaw = 0.30 authority limit |
| Heave | Symmetric collective | Both rotors same collective |

## Running Tests

### Test Mode Verification

Tests H_RSC_MODE=3 lockout and cyclic saturation:

```bash
source .venv/bin/activate
python3 Tools/autotest/minime_test_modes_sitl.py
```

### Failsafe Testing

Tests RC loss, GCS loss, battery, and geofence failsafes:

```bash
source .venv/bin/activate
python3 Tools/autotest/minime_failsafe_test.py
```

## Troubleshooting

### Python physics model not connecting

1. Ensure physics model starts before SITL
2. Check UDP port 9002 is not in use: `lsof -i :9002`
3. Verify firewall allows localhost UDP

### Vehicle unstable

1. Check gains in copter-heli-minime.parm
2. Start with conservative gains (lower P, higher D)
3. Verify servo limits match expected ranges

### HITL not receiving data

1. Ensure Cube has SimOnHardWare firmware
2. Check USB connection: `ls /dev/cu.usbmodem*`
3. Close QGC before running HITL bridge (serial port conflict)

## References

- [Using SITL with JSBSim](https://ardupilot.org/dev/docs/sitl-with-jsbsim.html)
- [ArduPilot JSON SITL Interface](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/examples/JSON/readme.md)
- [Simulation on Hardware](https://ardupilot.org/dev/docs/sim-on-hardware.html)
- [JSBSim FGRotor Class](https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGRotor.html)
- [Dual Helicopter Documentation](https://ardupilot.org/copter/docs/dual-helicopter.html)
- [Simulate ArduCopter with JSBSim Discussion](https://discuss.ardupilot.org/t/simulate-arducopter-with-jsbsim/82235)
- SPECS.md Sections 6, 25, 29
- STATE.md Phase W and Phase 11
