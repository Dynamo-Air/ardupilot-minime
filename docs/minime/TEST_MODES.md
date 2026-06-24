# AS-10 MiniMe Test Modes Documentation

This document describes the test mode configuration and operation procedures for Phase G ground vibration testing.

---

## 1. H_RSC_MODE Switching Procedure

Test modes require a different rotor speed control mode than baseline flight operations.

### 1.1 Mode Definitions

| H_RSC_MODE | Name | Purpose |
|------------|------|---------|
| 2 | ESC Governor | Baseline flight operations. ArduPilot governor maintains target RPM via throttle adjustment. |
| 3 | Throttle Curve | Required for test modes. Direct throttle control without governor interference. |

### 1.2 Why Test Modes Require H_RSC_MODE = 3

The synchronization control loop in test modes adjusts throttle to match rotor RPMs. If the ArduPilot governor (H_RSC_MODE = 2) is also active, both loops attempt to control throttle simultaneously. The governor interprets synchronization adjustments as disturbances to correct, causing oscillation or instability.

H_RSC_MODE = 3 disables the ArduPilot governor, giving the synchronization loop sole control of throttle during test modes.

### 1.3 Switching Procedure

To switch from baseline to test mode configuration:

1. Connect GCS (Mission Planner or MAVProxy)
2. Navigate to parameter configuration
3. Change H_RSC_MODE from 2 to 3
4. Write parameter to flight controller
5. Reboot flight controller (parameter change requires reboot)
6. Verify H_RSC_MODE = 3 after reboot

To return to baseline configuration:

1. Change H_RSC_MODE from 3 to 2
2. Write parameter to flight controller
3. Reboot flight controller
4. Verify H_RSC_MODE = 2 after reboot
5. Verify normal governor operation before flight

### 1.4 Reboot Requirement

H_RSC_MODE cannot be changed dynamically during flight. The parameter change requires a full reboot to take effect. Attempting to change this parameter without a reboot will not update the active control mode.

---

## 2. Preflight Checklist for Test Modes

Complete all items before enabling test modes (OBJ-MOD-4, OBJ-MOD-5).

### 2.1 Parameter Verification

| Step | Action | Verification |
|------|--------|--------------|
| 1 | Check H_RSC_MODE parameter | Must be 3 (throttle curve), not 2 (ESC governor) |
| 2 | Confirm reboot after change | Controller must have rebooted since parameter was changed |
| 3 | Verify minime_test_modes.lua loaded | Script present on SD card in APM/scripts/ |

### 2.2 Encoder Verification

| Step | Action | Verification |
|------|--------|--------------|
| 4 | Confirm encoder integration | RLS RM44SI must provide position feedback |
| 5 | Check encoder data valid | mm_enc_valid_fwd and mm_enc_valid_aft should be true |

### 2.3 Safety Conditions

| Step | Action | Verification |
|------|--------|--------------|
| 6 | Confirm vehicle on ground | Altitude < 1m AGL |
| 7 | Set flight mode | STABILIZE or ALT_HOLD only |
| 8 | Confirm test area clear | No personnel within rotor arc |

---

## 3. Lockout Enforcement

The test modes script (minime_test_modes.lua) enforces lockout conditions to prevent test mode operation during unsafe conditions.

### 3.1 Lockout Matrix

| Condition | Check Method | Lockout Behavior |
|-----------|--------------|------------------|
| H_RSC_MODE not 3 | param:get("H_RSC_MODE") | Test modes locked |
| Flight mode not STABILIZE or ALT_HOLD | vehicle:get_mode() | Test modes locked |
| Altitude > 1m AGL | ahrs:get_hagl() | Test modes locked |

### 3.2 Lockout Behavior

When any lockout condition is violated:
1. Test mode state machine remains in SYNC_IDLE
2. mm_test_lockout_active global is set to true
3. mm_test_lockout_reason contains human readable explanation
4. STATUSTEXT warning sent to GCS (rate limited to 5 second intervals)

### 3.3 Lockout Recovery

Lockout clears automatically when all conditions are satisfied:
1. H_RSC_MODE = 3 (requires reboot after parameter change)
2. Flight mode is STABILIZE or ALT_HOLD
3. Altitude is < 1m AGL

### 3.4 GCS Override

Manual GCS command can enable test modes on ground only (altitude < 1m). However, the H_RSC_MODE = 3 requirement cannot be overridden. If H_RSC_MODE is not 3, test modes remain locked regardless of GCS override.

---

## 4. Test Mode Global Variables

The test modes script exports the following global variables for monitoring:

| Variable | Type | Description |
|----------|------|-------------|
| mm_test_lockout_active | boolean | True if any lockout condition prevents test mode |
| mm_test_lockout_reason | string | Human readable lockout reason or empty string |
| mm_test_rsc_mode_ok | boolean | True if H_RSC_MODE = 3 |
| mm_test_flight_mode_ok | boolean | True if flight mode allows test modes |
| mm_test_altitude_ok | boolean | True if altitude < 1m AGL |

---

## 5. Related Documentation

Reference documents for test mode operation:

| Document | Section | Content |
|----------|---------|---------|
| SPECS.md | Section 3.1 | Governor mode configuration |
| SPECS.md | Section 29 | Test mode requirements |
| SPECS.md | Section 29.5 | Test mode lockout conditions |
| SPECS.md | Section 29.5A | Lockout matrix details |
| SPECS.md | Section 29.5B | Entry prerequisites checklist |
| STATE.md | Task 5.0a | H_RSC_MODE switching implementation |
| STATE.md | Task 5.1 | Synchronized rotor mode (OBJ-MOD-4) |
| STATE.md | Task 5.2 | Delta RPM sweep mode (OBJ-MOD-5) |

---

End of Document
