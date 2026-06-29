# Phase B-3 Verification Procedures

This document provides verification procedures for Task 3.0 Phase B-3 Immediate Priority Items as defined in STATE.md.

## 1. Servo Performance Monitoring

### 1.1 LOG_BITMASK Verification

The baseline parameter file (params/minime_baseline.param) sets LOG_BITMASK = 176126, which enables RCOUT (servo output) logging.

Verification Procedure:
1. Load minime_baseline.param onto Cube Orange+
2. Reboot the flight controller
3. Arm the vehicle (bench test configuration)
4. Run for 60 seconds with servos connected
5. Download the log file from SD card
6. Open log in Mission Planner or MAVExplorer
7. Verify RCOU messages are present with servo command values for SERVO1 through SERVO8

Expected Result: RCOU messages logged at 10 Hz minimum with valid PWM values (typically 1000 to 2000 microseconds).

### 1.2 Servo Saturation Event Detection

Saturation occurs when servo PWM commands reach limits, indicating the servo cannot achieve the commanded position.

Detection Criteria:
- PWM value at or below 1000 microseconds (minimum limit)
- PWM value at or above 2000 microseconds (maximum limit)
- Duration of saturation event tracked per occurrence

Monitoring Procedure:
1. During bench motor testing, log all servo commands
2. Post-process log file to identify saturation events
3. Record frequency (events per minute) and duration (milliseconds per event)
4. Correlate saturation events with flight control commands

### 1.3 Saturation Acceptance Criteria

Reference: SPECS.md Section 10.6

| Saturation Frequency | Saturation Duration | Assessment | Action |
|----------------------|---------------------|------------|--------|
| Less than 1% of operation time | Less than 100ms per event | Acceptable | Document, continue with current servos |
| 1% to 5% of operation time | Less than 500ms per event | Marginal | Document, monitor closely during Phase W |
| Greater than 5% of operation time | Any duration | Unacceptable | Evaluate servo upgrade before flight test |
| Any frequency | Greater than 1 second continuous | Unacceptable | Evaluate servo upgrade before flight test |

### 1.4 Go/No-Go Decision

The servo undersizing constraint (SF = 1.33 vs 3.0x design margin, SF = 3.98 vs worst case) requires monitoring during Phase B-3.

Decision Matrix:
- If saturation frequency is less than 1%: GO with current servos
- If saturation frequency is 1% to 5%: Conditional GO, monitor during Phase W
- If saturation frequency exceeds 5%: NO GO, evaluate upgrade path

Document the go/no-go decision in the Phase B-3 test report before proceeding to Phase W.

## 2. Cyclic Authority Saturation Verification

### 2.1 Bench Test Configuration

Reference: SPECS.md Section 2 (Blade Pitch Limits) and Section 31 (Cyclic Authority Management)

Configure bench test with:
- Full collective range: H_COL_MIN = negative 2.0 degrees to H_COL_MAX = 12.0 degrees
- Cyclic limit: H_CYC_MAX = 7.0 degrees at hover
- Servos connected and powered
- Logging enabled (LOG_BITMASK = 176126)

### 2.2 Maximum Collective Test Procedure

Purpose: Verify ArduPilot native saturation handling correctly limits cyclic at maximum collective.

Procedure:
1. Arm the vehicle in bench test configuration
2. Slowly increase collective to maximum (12.0 degrees)
3. While at maximum collective, command cyclic inputs (roll and pitch)
4. Observe servo response and log data
5. Verify cyclic commands are limited to available budget

Expected Behavior:
- At maximum collective (12.0 degrees), zero cyclic authority remains
- ArduPilot saturation handling prevents blade stall by limiting servo commands
- No servo commands should exceed the total pitch budget (14.0 degrees)

### 2.3 Documentation Requirements

Record in Phase B-3 test report:
- Maximum collective achieved during test
- Cyclic authority available at various collective settings
- Any saturation events observed
- Servo response behavior at maximum collective
- Confirmation that native ArduPilot saturation handling is functional

## 3. Autorotation Firmware Confirmation

### 3.1 Implementation Verification

The autorotation system is implemented in native C++ firmware, not Lua scripts. This satisfies the safety critical sub 100ms response latency requirement.

Confirmed Implementation Locations:
- Primary Entry: heli_update_autorotation() in ArduCopter/heli.cpp (lines 179-200)
- Main Controller: libraries/AC_Autorotation/AC_Autorotation.cpp (518 lines)
- RSC State Machine: libraries/AC_Autorotation/RSC_Autorotation.cpp (159 lines)
- Flight Mode: ArduCopter/mode_autorotate.cpp (136 lines, mode number 26)

### 3.2 Entry Code Paths

Autorotation entry is triggered when:
1. Aircraft is flying (not landed)
2. Motor interlock is OFF (motors.get_interlock() returns false)
3. Either manual throttle mode is active OR in AUTOROTATE flight mode

The entry logic calls motors.set_autorotation_active(true) which transitions the RSC state machine to ACTIVE state.

### 3.3 H_RSC_AROT_ENBL Parameter

Parameter: H_RSC_AROT_ENBL
Location: libraries/AC_Autorotation/RSC_Autorotation.cpp (line 17)
Type: INT8 (0 = Disabled, 1 = Enabled)
Purpose: Enables firmware autonomous autorotation capability

Related Parameters:
- H_RSC_AROT_RAMP: Bailout throttle ramp time (0.1 to 10 seconds)
- H_RSC_AROT_IDLE: Idle throttle during autorotation (0 to 40%)
- H_RSC_AROT_RUNUP: Run up time after bailout (1 to 10 seconds)

Baseline Configuration (params/minime_baseline.param):
- H_RSC_AROT_ENBL = 1
- H_RSC_AROT_IDLE = 0
- H_RSC_AROT_RAMP = 1.5

### 3.4 Lua Script Verification

Confirmed: NO Lua scripts implement autorotation functionality.

Verification: Grep search of all .lua files in APM/scripts/ shows no autorotation logic. All autorotation control is compiled into the ArduCopter firmware.

Response Latency:
- Native firmware: less than 10ms deterministic
- Lua scripting: greater than 100ms typical

The firmware implementation satisfies the safety critical response latency requirement.

### 3.5 Phase W Verification Reference

Phase W whirl stand testing will measure actual autorotation response latency:
1. Instrument throttle command output on oscilloscope
2. Trigger autorotation condition (throttle cut or mode entry)
3. Measure time from trigger event to command change
4. Record histogram of multiple measurements (minimum 10 trials)
5. Acceptance: All measurements must be less than 100ms, mean should be less than 10ms

Reference: SPECS.md Section 4.6 (Autorotation Verification Plan)

## 4. Phase B-3 Acceptance Gates

| Gate | Verification | Status |
|------|--------------|--------|
| PM message verification | PM messages present in 10 minute bench test log at 400 Hz | Pending |
| Frame overrun rate | Less than 1 in 10^6 calculated from PM messages | Pending |
| Servo saturation assessment | Saturation frequency documented, upgrade decision made if needed | Pending |
| HVIL response latency | Less than 100ms from break to contactors open | Pending (AUX5 assigned) |
| Autorotation implementation | Confirmed firmware based (C++), not Lua | COMPLETE |

Complete all acceptance gates before proceeding to Phase W whirl stand testing.
