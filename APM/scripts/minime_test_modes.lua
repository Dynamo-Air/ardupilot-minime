--[[
   MiniMe Test Modes
   Synchronized rotor mode and delta RPM sweep for Phase G ground vibration testing

   Prerequisites:
     H_RSC_MODE = 3 (throttle curve) required for test modes
     Parameter change requires reboot
     Phase 6A encoder integration must be complete for synchronized mode

   Test Mode Lockout Conditions (per SPECS.md Section 29.5):
     H_RSC_MODE not 3 (throttle curve): locked
     Flight mode not STABILIZE or ALT_HOLD: locked
     Altitude > 1m AGL: locked
     Pilot arm switch active during flight: locked

   Synchronized Mode States (OBJ-MOD-4):
     SYNC_IDLE: Both rotors at matched RPM, waiting for start command
     SYNC_RAMP: Ramping to synchronized 100% RPM (1131.7 RPM)
     SYNC_PHASE_0: Synchronized at 0 degree phase offset
     SYNC_PHASE_60: Synchronized at 60 degree phase offset
     SYNC_FAULT: Synchronization lost, safe state

   Inter-Script Interface:
   Outputs (mm_test_ prefix):
     mm_test_lockout_active     boolean  True if any lockout condition prevents test mode
     mm_test_lockout_reason     string   Human readable lockout reason or empty
     mm_test_rsc_mode_ok        boolean  True if H_RSC_MODE = 3
     mm_test_flight_mode_ok     boolean  True if flight mode allows test modes
     mm_test_altitude_ok        boolean  True if altitude < 1m AGL
     mm_test_arm_flight_ok      boolean  True if not armed or not flying
     mm_test_state              integer  Current sync state (0 to 4)
     mm_test_state_name         string   Human readable state name
     mm_test_active             boolean  True when test mode is controlling RPM
     mm_test_cmd_rpm_fwd        number   Commanded RPM for forward rotor (nil when inactive)
     mm_test_cmd_rpm_aft        number   Commanded RPM for aft rotor (nil when inactive)
     mm_test_phase_target       number   Target phase offset (0 or 60)
     mm_test_phase_error        number   Current phase error in degrees
     mm_test_sync_achieved      boolean  True when RPM and phase targets met
     mm_test_gcs_override       boolean  GCS override active for ground testing

   Inputs (read from other scripts):
     mm_hv_state                HV state for mode lockout checks
     mm_hv_command_enable       HV command enable status
     mm_enc_valid_fwd           Encoder data validity for synchronized mode
     mm_enc_valid_aft           Encoder data validity for synchronized mode
     mm_enc_phase_diff          Cross rotor phase difference
     mm_dti_actual_rpm_fwd      Actual forward rotor RPM
     mm_dti_actual_rpm_aft      Actual aft rotor RPM

   Commands (set externally to control test mode):
     mm_test_cmd_start          Set true to start sync mode from SYNC_IDLE
     mm_test_cmd_stop           Set true to stop and return to SYNC_IDLE
     mm_test_cmd_phase_0        Set true to change to 0 degree offset
     mm_test_cmd_phase_60       Set true to change to 60 degree offset
     mm_test_cmd_reset          Set true to reset from SYNC_FAULT to SYNC_IDLE
]]--

local common = require("minime_common")

local SCRIPT_NAME = "TestModes"
local UPDATE_RATE_MS = 10

local last_lockout_alert_ms = 0
local last_state_change_ms = 0
local initialized = false

local test_mode_state = common.SYNC_IDLE
local target_phase_offset = 0
local ramp_start_time_ms = 0
local ramp_start_rpm = 0
local settle_start_ms = 0
local rpm_fault_start_ms = 0
local phase_fault_start_ms = 0

mm_test_lockout_active = true
mm_test_lockout_reason = ""
mm_test_rsc_mode_ok = false
mm_test_flight_mode_ok = false
mm_test_altitude_ok = false
mm_test_arm_flight_ok = false
mm_test_state = common.SYNC_IDLE
mm_test_state_name = "SYNC_IDLE"
mm_test_active = false
mm_test_cmd_rpm_fwd = nil
mm_test_cmd_rpm_aft = nil
mm_test_phase_target = 0
mm_test_phase_error = 0
mm_test_rpm_fwd = 0
mm_test_rpm_aft = 0
mm_test_sync_achieved = false
mm_test_gcs_override = false

mm_test_cmd_start = false
mm_test_cmd_stop = false
mm_test_cmd_phase_0 = false
mm_test_cmd_phase_60 = false
mm_test_cmd_reset = false

--[[
   Check H_RSC_MODE parameter
   Test modes require H_RSC_MODE = 3 (throttle curve)
   @return true if H_RSC_MODE is correct for test modes
]]--
local function check_rsc_mode()
    local rsc_mode = param:get("H_RSC_MODE")
    if rsc_mode == nil then
        mm_test_rsc_mode_ok = false
        return false, "H_RSC_MODE parameter not found"
    end

    if rsc_mode ~= common.RSC_MODE_THROTTLE_CURVE then
        mm_test_rsc_mode_ok = false
        return false, string.format("H_RSC_MODE must be 3 (current: %d)", rsc_mode)
    end

    mm_test_rsc_mode_ok = true
    return true, nil
end

--[[
   Check flight mode is valid for test modes
   Only STABILIZE and ALT_HOLD are permitted
   @return true if flight mode allows test modes
]]--
local function check_flight_mode()
    local mode = vehicle:get_mode()
    if mode == nil then
        mm_test_flight_mode_ok = false
        return false, "Flight mode unavailable"
    end

    if mode ~= common.MODE_STABILIZE and mode ~= common.MODE_ALT_HOLD then
        mm_test_flight_mode_ok = false
        return false, "Flight mode must be STABILIZE or ALT_HOLD"
    end

    mm_test_flight_mode_ok = true
    return true, nil
end

--[[
   Check altitude is below threshold for test modes
   Test modes locked when airborne (> 1m AGL)
   @return true if altitude permits test modes
]]--
local function check_altitude()
    local altitude = ahrs:get_hagl()
    if altitude == nil then
        altitude = 0
    end

    if altitude > common.TEST_MODE_MAX_ALTITUDE_M then
        mm_test_altitude_ok = false
        return false, string.format("Altitude must be < %.1fm (current: %.1fm)",
            common.TEST_MODE_MAX_ALTITUDE_M, altitude)
    end

    mm_test_altitude_ok = true
    return true, nil
end

--[[
   Check if pilot arm switch is active during flight
   Test modes locked when vehicle is armed and likely flying
   @return true if condition allows test modes (not armed or not flying)
]]--
local function check_arm_during_flight()
    local is_armed = arming:is_armed()
    local likely_flying = vehicle:get_likely_flying()

    if is_armed and likely_flying then
        mm_test_arm_flight_ok = false
        return false, "Test modes locked while armed and flying"
    end

    mm_test_arm_flight_ok = true
    return true, nil
end

--[[
   Check all lockout conditions for test mode entry
   @return true if all conditions pass, false with reason if any fail
]]--
local function check_all_lockouts()
    local ok, reason

    -- H_RSC_MODE check cannot be overridden per TEST_MODES.md Section 3.4
    ok, reason = check_rsc_mode()
    if not ok then
        mm_test_lockout_active = true
        mm_test_lockout_reason = reason
        return false
    end

    -- GCS override bypasses flight mode and arm checks but still requires altitude < 1m
    if mm_test_gcs_override then
        local alt_ok, _ = check_altitude()
        if alt_ok then
            mm_test_lockout_active = false
            mm_test_lockout_reason = ""
            return true
        end
    end

    ok, reason = check_flight_mode()
    if not ok then
        mm_test_lockout_active = true
        mm_test_lockout_reason = reason
        return false
    end

    ok, reason = check_altitude()
    if not ok then
        mm_test_lockout_active = true
        mm_test_lockout_reason = reason
        return false
    end

    ok, reason = check_arm_during_flight()
    if not ok then
        mm_test_lockout_active = true
        mm_test_lockout_reason = reason
        return false
    end

    mm_test_lockout_active = false
    mm_test_lockout_reason = ""
    return true
end

--[[
   Send lockout alert to GCS with rate limiting
]]--
local function send_lockout_alert()
    local now = millis():toint()

    if now - last_lockout_alert_ms < common.TEST_MODE_LOCKOUT_ALERT_MS then
        return
    end

    if mm_test_lockout_active and mm_test_lockout_reason ~= "" then
        gcs:send_text(common.MAV_SEVERITY.WARNING,
            string.format("%s: Test mode locked: %s", SCRIPT_NAME, mm_test_lockout_reason))
        last_lockout_alert_ms = now
    end
end

--[[
   Transition to a new state with logging
]]--
local function transition_to_state(new_state, reason)
    local old_state = test_mode_state
    local old_name = common.SYNC_STATE_NAMES[old_state] or "UNKNOWN"
    local new_name = common.SYNC_STATE_NAMES[new_state] or "UNKNOWN"

    test_mode_state = new_state
    mm_test_state = new_state
    mm_test_state_name = new_name
    last_state_change_ms = millis():toint()

    gcs:send_text(common.MAV_SEVERITY.INFO,
        string.format("%s: %s to %s (%s)", SCRIPT_NAME, old_name, new_name, reason))

    if new_state == common.SYNC_IDLE or new_state == common.SYNC_FAULT then
        mm_test_active = false
        mm_test_cmd_rpm_fwd = nil
        mm_test_cmd_rpm_aft = nil
        mm_test_sync_achieved = false
    end
end

--[[
   Clear all command flags after processing
]]--
local function clear_command_flags()
    mm_test_cmd_start = false
    mm_test_cmd_stop = false
    mm_test_cmd_phase_0 = false
    mm_test_cmd_phase_60 = false
    mm_test_cmd_reset = false
end

--[[
   Check if encoder data is valid for synchronized mode
   @return true if both encoders are providing valid data
]]--
local function check_encoder_validity()
    local fwd_valid = mm_enc_valid_fwd or false
    local aft_valid = mm_enc_valid_aft or false
    return fwd_valid and aft_valid
end

--[[
   Check if HV system is ready for test mode operation
   @return true if HV is armed and commands enabled
]]--
local function check_hv_ready()
    local hv_state = mm_hv_state or common.STATE_DE_ENERGIZED
    local cmd_enabled = mm_hv_command_enable or false
    return hv_state == common.STATE_ARMED and cmd_enabled
end

--[[
   Get current actual RPM from DTI telemetry
   @return fwd_rpm, aft_rpm or nil if data unavailable
]]--
local function get_actual_rpm()
    local fwd = mm_dti_actual_rpm_fwd
    local aft = mm_dti_actual_rpm_aft
    if fwd == nil or aft == nil then
        return nil, nil
    end
    return fwd, aft
end

--[[
   Calculate phase adjustment for aft rotor to achieve target offset
   Uses proportional control on phase error
   @return RPM adjustment for aft rotor
]]--
local function calculate_phase_adjustment()
    if not check_encoder_validity() then
        return 0
    end

    local actual_diff = mm_enc_phase_diff or 0
    local error = target_phase_offset - actual_diff

    if error > 60 then
        error = error - 120
    elseif error < -60 then
        error = error + 120
    end

    mm_test_phase_error = error

    local adjustment = common.SYNC_PHASE_K_P * error

    if adjustment > common.SYNC_PHASE_MAX_RPM_ADJ then
        adjustment = common.SYNC_PHASE_MAX_RPM_ADJ
    elseif adjustment < -common.SYNC_PHASE_MAX_RPM_ADJ then
        adjustment = -common.SYNC_PHASE_MAX_RPM_ADJ
    end

    return adjustment
end

--[[
   Check for synchronization fault conditions
   @return true if fault detected, false otherwise
]]--
local function check_sync_fault()
    local now = millis():toint()
    local fwd_rpm, aft_rpm = get_actual_rpm()

    if fwd_rpm == nil or aft_rpm == nil then
        return false
    end

    local rpm_diff = math.abs(fwd_rpm - aft_rpm)
    local rpm_diff_pct = rpm_diff / common.SYNC_TARGET_RPM

    if rpm_diff_pct > common.SYNC_FAULT_RPM_DIFF_PCT then
        if rpm_fault_start_ms == 0 then
            rpm_fault_start_ms = now
        elseif now - rpm_fault_start_ms > common.SYNC_FAULT_RPM_DIFF_MS then
            gcs:send_text(common.MAV_SEVERITY.CRITICAL,
                string.format("%s: RPM differential fault (%.1f%%)", SCRIPT_NAME, rpm_diff_pct * 100))
            return true
        end
    else
        rpm_fault_start_ms = 0
    end

    if test_mode_state == common.SYNC_PHASE_0 or test_mode_state == common.SYNC_PHASE_60 then
        local phase_err = math.abs(mm_test_phase_error)
        if phase_err > common.SYNC_FAULT_PHASE_ERR_DEG then
            if phase_fault_start_ms == 0 then
                phase_fault_start_ms = now
            elseif now - phase_fault_start_ms > common.SYNC_FAULT_PHASE_ERR_MS then
                gcs:send_text(common.MAV_SEVERITY.CRITICAL,
                    string.format("%s: Phase error fault (%.1f deg)", SCRIPT_NAME, phase_err))
                return true
            end
        else
            phase_fault_start_ms = 0
        end
    end

    if not check_encoder_validity() then
        gcs:send_text(common.MAV_SEVERITY.CRITICAL,
            string.format("%s: Encoder data invalid", SCRIPT_NAME))
        return true
    end

    return false
end

--[[
   Check if synchronization targets are achieved
   @return true if RPM and phase are within tolerance
]]--
local function check_sync_achieved()
    local fwd_rpm, aft_rpm = get_actual_rpm()
    if fwd_rpm == nil or aft_rpm == nil then
        return false
    end

    local rpm_diff = math.abs(fwd_rpm - aft_rpm)
    if rpm_diff > common.SYNC_RPM_TOLERANCE then
        return false
    end

    local fwd_on_target = math.abs(fwd_rpm - common.SYNC_TARGET_RPM) < common.SYNC_RPM_TOLERANCE
    local aft_on_target = math.abs(aft_rpm - common.SYNC_TARGET_RPM) < common.SYNC_RPM_TOLERANCE
    if not (fwd_on_target and aft_on_target) then
        return false
    end

    if test_mode_state == common.SYNC_PHASE_0 or test_mode_state == common.SYNC_PHASE_60 then
        local phase_err = math.abs(mm_test_phase_error)
        if phase_err > common.SYNC_PHASE_TOLERANCE_DEG then
            return false
        end
    end

    return true
end

--[[
   Handle SYNC_IDLE state
   Wait for start command, verify prerequisites
]]--
local function handle_sync_idle()
    mm_test_active = false
    mm_test_cmd_rpm_fwd = nil
    mm_test_cmd_rpm_aft = nil

    if mm_test_cmd_start then
        if not check_hv_ready() then
            gcs:send_text(common.MAV_SEVERITY.WARNING,
                string.format("%s: Cannot start, HV not armed", SCRIPT_NAME))
            return
        end

        if not check_encoder_validity() then
            gcs:send_text(common.MAV_SEVERITY.WARNING,
                string.format("%s: Cannot start, encoder data invalid", SCRIPT_NAME))
            return
        end

        local fwd_rpm, _ = get_actual_rpm()
        ramp_start_rpm = fwd_rpm or 0
        ramp_start_time_ms = millis():toint()
        target_phase_offset = common.PHASE_OFFSET_IN_PHASE
        mm_test_phase_target = 0

        transition_to_state(common.SYNC_RAMP, "start command")
    end
end

--[[
   Handle SYNC_RAMP state
   Ramp both rotors to target RPM
]]--
local function handle_sync_ramp()
    local now = millis():toint()
    local elapsed_s = (now - ramp_start_time_ms) / 1000.0

    local ramp_rpm = ramp_start_rpm + (elapsed_s * common.SYNC_RAMP_RATE_RPM_S)
    if ramp_rpm > common.SYNC_TARGET_RPM then
        ramp_rpm = common.SYNC_TARGET_RPM
    end

    mm_test_active = true
    mm_test_cmd_rpm_fwd = ramp_rpm
    mm_test_cmd_rpm_aft = ramp_rpm

    local fwd_rpm, aft_rpm = get_actual_rpm()
    if fwd_rpm ~= nil then
        mm_test_rpm_fwd = fwd_rpm
    end
    if aft_rpm ~= nil then
        mm_test_rpm_aft = aft_rpm
    end

    local on_target = math.abs(ramp_rpm - common.SYNC_TARGET_RPM) < 0.1
    if on_target then
        if settle_start_ms == 0 then
            settle_start_ms = now
        elseif now - settle_start_ms > common.SYNC_SETTLE_TIME_MS then
            settle_start_ms = 0

            if target_phase_offset == common.PHASE_OFFSET_OUT_PHASE then
                transition_to_state(common.SYNC_PHASE_60, "ramp complete")
            else
                transition_to_state(common.SYNC_PHASE_0, "ramp complete")
            end
        end
    else
        settle_start_ms = 0
    end

    if mm_test_cmd_stop then
        transition_to_state(common.SYNC_IDLE, "stop command")
    end
end

--[[
   Handle SYNC_PHASE_0 state
   Maintain synchronization at 0 degree phase offset
]]--
local function handle_sync_phase_0()
    mm_test_active = true

    local phase_adj = calculate_phase_adjustment()

    mm_test_cmd_rpm_fwd = common.SYNC_TARGET_RPM
    mm_test_cmd_rpm_aft = common.SYNC_TARGET_RPM + phase_adj

    local fwd_rpm, aft_rpm = get_actual_rpm()
    if fwd_rpm ~= nil then
        mm_test_rpm_fwd = fwd_rpm
    end
    if aft_rpm ~= nil then
        mm_test_rpm_aft = aft_rpm
    end

    mm_test_sync_achieved = check_sync_achieved()

    if check_sync_fault() then
        transition_to_state(common.SYNC_FAULT, "sync lost")
        return
    end

    if mm_test_cmd_phase_60 then
        target_phase_offset = common.PHASE_OFFSET_OUT_PHASE
        mm_test_phase_target = 60
        transition_to_state(common.SYNC_PHASE_60, "phase 60 command")
    end

    if mm_test_cmd_stop then
        transition_to_state(common.SYNC_IDLE, "stop command")
    end
end

--[[
   Handle SYNC_PHASE_60 state
   Maintain synchronization at 60 degree phase offset
]]--
local function handle_sync_phase_60()
    mm_test_active = true

    local phase_adj = calculate_phase_adjustment()

    mm_test_cmd_rpm_fwd = common.SYNC_TARGET_RPM
    mm_test_cmd_rpm_aft = common.SYNC_TARGET_RPM + phase_adj

    local fwd_rpm, aft_rpm = get_actual_rpm()
    if fwd_rpm ~= nil then
        mm_test_rpm_fwd = fwd_rpm
    end
    if aft_rpm ~= nil then
        mm_test_rpm_aft = aft_rpm
    end

    mm_test_sync_achieved = check_sync_achieved()

    if check_sync_fault() then
        transition_to_state(common.SYNC_FAULT, "sync lost")
        return
    end

    if mm_test_cmd_phase_0 then
        target_phase_offset = common.PHASE_OFFSET_IN_PHASE
        mm_test_phase_target = 0
        transition_to_state(common.SYNC_PHASE_0, "phase 0 command")
    end

    if mm_test_cmd_stop then
        transition_to_state(common.SYNC_IDLE, "stop command")
    end
end

--[[
   Handle SYNC_FAULT state
   Stop commands and wait for reset
]]--
local function handle_sync_fault()
    mm_test_active = false
    mm_test_cmd_rpm_fwd = nil
    mm_test_cmd_rpm_aft = nil
    mm_test_sync_achieved = false

    rpm_fault_start_ms = 0
    phase_fault_start_ms = 0

    if mm_test_cmd_reset then
        transition_to_state(common.SYNC_IDLE, "reset command")
    end
end

--[[
   Process state machine
]]--
local function process_state_machine()
    if test_mode_state == common.SYNC_IDLE then
        handle_sync_idle()
    elseif test_mode_state == common.SYNC_RAMP then
        handle_sync_ramp()
    elseif test_mode_state == common.SYNC_PHASE_0 then
        handle_sync_phase_0()
    elseif test_mode_state == common.SYNC_PHASE_60 then
        handle_sync_phase_60()
    elseif test_mode_state == common.SYNC_FAULT then
        handle_sync_fault()
    end

    clear_command_flags()
end

--[[
   Check lockout and handle active test interruption
]]--
local function check_lockout_interrupt()
    local was_locked = mm_test_lockout_active

    check_all_lockouts()

    if mm_test_lockout_active and not was_locked then
        if test_mode_state ~= common.SYNC_IDLE and test_mode_state ~= common.SYNC_FAULT then
            transition_to_state(common.SYNC_IDLE, "lockout triggered")
        end
    end

    if mm_test_lockout_active then
        send_lockout_alert()
    end
end

--[[
   Publish test mode state via MAVLink NAMED_VALUE
]]--
local function publish_state()
    gcs:send_named_float("SYNC_ST", test_mode_state)
end

--[[
   Initialize test modes script
]]--
local function init()
    gcs:send_text(common.MAV_SEVERITY.INFO,
        string.format("%s: Initializing synchronized rotor mode", SCRIPT_NAME))

    local ok, reason = check_rsc_mode()
    if ok then
        gcs:send_text(common.MAV_SEVERITY.INFO,
            string.format("%s: H_RSC_MODE = 3 (throttle curve) verified", SCRIPT_NAME))
    else
        gcs:send_text(common.MAV_SEVERITY.WARNING,
            string.format("%s: %s", SCRIPT_NAME, reason))
    end

    mm_test_state = common.SYNC_IDLE
    mm_test_state_name = "SYNC_IDLE"
    last_state_change_ms = millis():toint()

    initialized = true
    return true
end

--[[
   Main update loop
   Checks lockout conditions, processes state machine, publishes telemetry
]]--
local function update()
    if not initialized then
        if not init() then
            return update, 1000
        end
    end

    check_lockout_interrupt()

    if not mm_test_lockout_active or test_mode_state == common.SYNC_FAULT then
        process_state_machine()
    end

    publish_state()

    return update, UPDATE_RATE_MS
end

return update()
