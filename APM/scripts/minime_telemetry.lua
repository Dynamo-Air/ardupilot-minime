--[[
   MiniMe Telemetry Aggregation
   RPM differential calculation, desync monitoring, and gyroscopic coupling
   feedforward for AS-10 tandem helicopter

   Prerequisites:
     minime_dti_driver.lua loaded (provides mm_dti_* globals)
     minime_hv_control.lua loaded (provides mm_hv_* globals)
     minime_common.lua available

   Inter-Script Interface:
   Outputs (mm_tel_ prefix, updated at 50 Hz):
     mm_tel_desync_state       Desync state (0=normal, 1=alert, 2=warning, 3=critical)
     mm_tel_rpm_diff           Current RPM differential (absolute value)
     mm_tel_delta_rpm          Alias for mm_tel_rpm_diff
     mm_tel_delta_pct          RPM differential as percentage of hover RPM
     mm_tel_delta_rpm_avg      Rolling average RPM differential (200 ms window)
     mm_tel_peak_delta_rpm     Peak RPM differential since arm
     mm_tel_peak_delta_pct     Peak percentage since arm
     mm_tel_calc_timestamp     Timestamp of last calculation in ms
     mm_tel_data_valid         True if telemetry data is fresh and valid
     mm_tel_spinup_suppressed  True during 5 second spinup suppression window
     mm_tel_rpm_fwd_filtered   Low pass filtered forward RPM
     mm_tel_rpm_aft_filtered   Low pass filtered aft RPM
     mm_tel_roll_rate_dps      Current roll rate from IMU (deg/s)
     mm_tel_pitch_correction_dps  K_rp feedforward pitch correction (deg/s)

   K_rp Gyroscopic Coupling Feedforward:
     ArduPilot does NOT natively compensate roll to pitch gyroscopic coupling.
     This script calculates the required feedforward: pitch_correction = K_RP * roll_rate
     The calculated value is logged for Phase W validation.

     Implementation Options:
       1. Scripted control mode via vehicle:set_target_throttle_rate_rpy() takes full
          attitude control and requires implementing complete attitude logic in Lua
       2. Firmware modification adds cross-coupling directly in AC_AttitudeControl
       3. Calculate and log for validation, then implement via firmware patch

     This implementation uses option 3: calculate and log for Phase W HIL validation.

   Inputs (read from other scripts):
     mm_dti_actual_rpm_fwd, mm_dti_actual_rpm_aft   Actual RPM from DTI telemetry
     mm_dti_heartbeat_fwd, mm_dti_heartbeat_aft     Timestamps for freshness check
     mm_dti_voltage_fwd, mm_dti_voltage_aft         DC link voltage for precharge display
     mm_dti_fault_fwd, mm_dti_fault_aft             Fault codes for NAMED_VALUE publishing
     mm_hv_state                                    HV state for state publishing
     mm_hv_last_state_change_ms                     State change timestamp for spinup suppression

   Interface for Test Modes (minime_test_modes.lua):
     Test modes reads these globals for synchronized rotor control:
       mm_tel_rpm_fwd_filtered, mm_tel_rpm_aft_filtered (current filtered RPM)
       mm_tel_desync_state (desync alert level for abort conditions)
       mm_dti_actual_rpm_fwd, mm_dti_actual_rpm_aft (raw RPM values)
       mm_hv_state (HV state for test mode lockout checks)
]]--

local common = require("minime_common")

local LPF_ALPHA = 0.3
local ROLLING_BUFFER_SIZE = 10
local EXPECTED_VOLTAGE = 133.2
local PUBLISH_DIVIDER = 5

local lpf_rpm_fwd = 0
local lpf_rpm_aft = 0
local lpf_initialized = false

local rolling_buffer = {}
local rolling_buffer_idx = 1
local rolling_buffer_count = 0

local peak_delta_rpm = 0
local peak_delta_pct = 0
local prev_armed = false

local desync_state = common.DESYNC_NORMAL
local desync_below_threshold_ms = nil
local last_desync_alert_ms = 0
local DESYNC_ALERT_RATE_LIMIT_MS = 5000

local prev_hv_state = nil
local prev_fault_fwd = nil
local prev_fault_aft = nil
local prev_desync_state_pub = nil
local publish_counter = 0

mm_tel_delta_rpm = 0
mm_tel_delta_pct = 0
mm_tel_delta_rpm_avg = 0
mm_tel_peak_delta_rpm = 0
mm_tel_peak_delta_pct = 0
mm_tel_calc_timestamp = 0
mm_tel_data_valid = false
mm_tel_spinup_suppressed = true
mm_tel_rpm_fwd_filtered = 0
mm_tel_rpm_aft_filtered = 0
mm_tel_desync_state = common.DESYNC_NORMAL
mm_tel_rpm_diff = 0
mm_tel_roll_rate_dps = 0
mm_tel_pitch_correction_dps = 0

local function lpf_update(current, new_value)
    return LPF_ALPHA * new_value + (1 - LPF_ALPHA) * current
end

local function rolling_buffer_push(value)
    rolling_buffer[rolling_buffer_idx] = value
    rolling_buffer_idx = (rolling_buffer_idx % ROLLING_BUFFER_SIZE) + 1
    if rolling_buffer_count < ROLLING_BUFFER_SIZE then
        rolling_buffer_count = rolling_buffer_count + 1
    end
end

local function rolling_buffer_average()
    if rolling_buffer_count == 0 then
        return 0
    end
    local sum = 0
    for i = 1, rolling_buffer_count do
        sum = sum + rolling_buffer[i]
    end
    return sum / rolling_buffer_count
end

local function is_spinup_suppressed()
    local hv_state = mm_hv_state or common.STATE_DE_ENERGIZED
    if hv_state ~= common.STATE_ARMED then
        return true
    end
    local now_ms = millis():toint()
    local state_change_ms = mm_hv_last_state_change_ms or 0
    return (now_ms - state_change_ms) < common.SPINUP_SUPPRESS_MS
end

local function reset_on_disarm()
    peak_delta_rpm = 0
    peak_delta_pct = 0
    lpf_initialized = false
    rolling_buffer_count = 0
    rolling_buffer_idx = 1
    for i = 1, ROLLING_BUFFER_SIZE do
        rolling_buffer[i] = 0
    end
    desync_state = common.DESYNC_NORMAL
    desync_below_threshold_ms = nil
    last_desync_alert_ms = 0
    mm_tel_desync_state = common.DESYNC_NORMAL
end

local function get_target_desync_state(delta_pct)
    if delta_pct >= common.DESYNC_HARD_PCT then
        return common.DESYNC_CRITICAL
    elseif delta_pct >= common.DESYNC_WARNING_PCT then
        return common.DESYNC_WARNING
    elseif delta_pct >= common.DESYNC_ALERT_PCT then
        return common.DESYNC_ALERT
    else
        return common.DESYNC_NORMAL
    end
end

local function evaluate_desync_state(delta_pct, now_ms)
    if is_spinup_suppressed() then
        desync_state = common.DESYNC_NORMAL
        desync_below_threshold_ms = nil
        mm_tel_desync_state = desync_state
        return false
    end

    local target_state = get_target_desync_state(delta_pct)

    if target_state > desync_state then
        desync_state = target_state
        desync_below_threshold_ms = nil
        mm_tel_desync_state = desync_state
        return true
    elseif target_state < desync_state then
        if desync_below_threshold_ms == nil then
            desync_below_threshold_ms = now_ms
        elseif now_ms - desync_below_threshold_ms >= common.DESYNC_HYSTERESIS_MS then
            desync_state = target_state
            desync_below_threshold_ms = nil
            mm_tel_desync_state = desync_state
            return true
        end
    else
        desync_below_threshold_ms = nil
    end

    mm_tel_desync_state = desync_state
    return false
end

local function execute_desync_response(new_state, delta_rpm, delta_pct, now_ms)
    if new_state == common.DESYNC_NORMAL then
        return
    end

    if new_state == common.DESYNC_ALERT then
        logger:write("DSYN", "St,dRPM,dPct", "Bff",
            new_state,
            delta_rpm,
            delta_pct)
        return
    end

    local should_alert = now_ms - last_desync_alert_ms >= DESYNC_ALERT_RATE_LIMIT_MS

    if new_state == common.DESYNC_WARNING then
        if should_alert then
            gcs:send_text(common.MAV_SEVERITY.WARNING,
                string.format("TEL: Desync WARNING delta %.1f RPM (%.2f%%)", delta_rpm, delta_pct))
            last_desync_alert_ms = now_ms
        end
    elseif new_state == common.DESYNC_CRITICAL then
        gcs:send_text(common.MAV_SEVERITY.CRITICAL,
            string.format("TEL: Desync CRITICAL delta %.1f RPM (%.2f%%), RTL", delta_rpm, delta_pct))
        last_desync_alert_ms = now_ms

        local current_mode = vehicle:get_mode()
        if current_mode ~= common.MODE_RTL and current_mode ~= common.MODE_LAND then
            vehicle:set_mode(common.MODE_RTL)
        end
    end
end

local function publish_named_values()
    local current_hv_state = mm_hv_state or common.STATE_DE_ENERGIZED
    if prev_hv_state ~= current_hv_state then
        gcs:send_named_float("HV_STATE", current_hv_state)
        prev_hv_state = current_hv_state
    end

    local current_fault_fwd = mm_dti_fault_fwd or 0
    if prev_fault_fwd ~= current_fault_fwd then
        gcs:send_named_float("DTI_F_FLT", current_fault_fwd)
        prev_fault_fwd = current_fault_fwd
    end

    local current_fault_aft = mm_dti_fault_aft or 0
    if prev_fault_aft ~= current_fault_aft then
        gcs:send_named_float("DTI_A_FLT", current_fault_aft)
        prev_fault_aft = current_fault_aft
    end

    if prev_desync_state_pub ~= desync_state then
        gcs:send_named_float("DESYNC_ST", desync_state)
        prev_desync_state_pub = desync_state
    end

    publish_counter = publish_counter + 1
    if publish_counter >= PUBLISH_DIVIDER then
        publish_counter = 0

        gcs:send_named_float("RPM_DIFF", mm_tel_delta_rpm)

        if current_hv_state == common.STATE_PRECHARGING then
            local voltage_fwd = mm_dti_voltage_fwd or 0
            local voltage_aft = mm_dti_voltage_aft or 0
            local min_voltage = math.min(voltage_fwd, voltage_aft)
            local pct = (min_voltage / EXPECTED_VOLTAGE) * 100
            if pct > 100 then
                pct = 100
            end
            gcs:send_named_float("PRECHG_V", pct)
        end
    end
end

local function publish_initial_values()
    local hv_state = mm_hv_state or common.STATE_DE_ENERGIZED
    gcs:send_named_float("HV_STATE", hv_state)
    prev_hv_state = hv_state

    local fault_fwd = mm_dti_fault_fwd or 0
    gcs:send_named_float("DTI_F_FLT", fault_fwd)
    prev_fault_fwd = fault_fwd

    local fault_aft = mm_dti_fault_aft or 0
    gcs:send_named_float("DTI_A_FLT", fault_aft)
    prev_fault_aft = fault_aft

    gcs:send_named_float("DESYNC_ST", common.DESYNC_NORMAL)
    prev_desync_state_pub = common.DESYNC_NORMAL

    gcs:send_named_float("RPM_DIFF", 0)
end

--[[
   K_rp Gyroscopic Coupling Feedforward Calculation
   Roll commands induce pitch moments via gyroscopic precession.
   Feedforward compensates by adding pitch rate proportional to roll rate.

   Formula: pitch_correction = K_RP * roll_rate
   K_RP = 0.4730 per second (derived from 2*H/M_pitch = 2*56.53/239.0)

   Phase W Validation Criteria:
     Roll step 10 deg/s produces pitch transient less than 3 degrees
     Steady roll 30 deg/s produces pitch rate less than 5 deg/s

   This calculates and logs the feedforward correction for Phase W validation.
   Injection into the attitude controller requires either scripted control mode
   (vehicle:set_target_throttle_rate_rpy) or firmware modification.
]]--
local krp_log_counter = 0
local KRP_LOG_DIVIDER = 25             -- Log at 2 Hz (50 Hz / 25)

local function calculate_gyro_coupling_feedforward()
    local gyro = ahrs:get_gyro()
    if not gyro then
        mm_tel_roll_rate_dps = 0
        mm_tel_pitch_correction_dps = 0
        return
    end

    local roll_rate_rad = gyro:x()
    local roll_rate_dps = math.deg(roll_rate_rad)

    local pitch_correction_dps = common.K_RP * roll_rate_dps

    mm_tel_roll_rate_dps = roll_rate_dps
    mm_tel_pitch_correction_dps = pitch_correction_dps

    krp_log_counter = krp_log_counter + 1
    if krp_log_counter >= KRP_LOG_DIVIDER then
        krp_log_counter = 0

        if arming:is_armed() and math.abs(roll_rate_dps) > 1.0 then
            logger:write("GYCF", "RollR,PitchC", "ff",
                roll_rate_dps,
                pitch_correction_dps)
        end
    end
end

local update

function update()
    local now_ms = millis():toint()

    calculate_gyro_coupling_feedforward()

    local raw_rpm_fwd = mm_dti_actual_rpm_fwd
    local raw_rpm_aft = mm_dti_actual_rpm_aft
    local heartbeat_fwd = mm_dti_heartbeat_fwd or 0
    local heartbeat_aft = mm_dti_heartbeat_aft or 0

    local fwd_valid = (raw_rpm_fwd ~= nil) and ((now_ms - heartbeat_fwd) < common.DTI_TIMEOUT_MS)
    local aft_valid = (raw_rpm_aft ~= nil) and ((now_ms - heartbeat_aft) < common.DTI_TIMEOUT_MS)

    if not fwd_valid or not aft_valid then
        mm_tel_data_valid = false
        publish_named_values()
        return update, common.DTI_TELEMETRY_RATE_MS
    end

    mm_tel_data_valid = true

    if not lpf_initialized then
        lpf_rpm_fwd = raw_rpm_fwd
        lpf_rpm_aft = raw_rpm_aft
        lpf_initialized = true
    else
        lpf_rpm_fwd = lpf_update(lpf_rpm_fwd, raw_rpm_fwd)
        lpf_rpm_aft = lpf_update(lpf_rpm_aft, raw_rpm_aft)
    end

    mm_tel_rpm_fwd_filtered = lpf_rpm_fwd
    mm_tel_rpm_aft_filtered = lpf_rpm_aft

    local delta_rpm = math.abs(lpf_rpm_fwd - lpf_rpm_aft)
    local delta_pct = (delta_rpm / common.HOVER_RPM) * 100

    rolling_buffer_push(delta_rpm)
    local delta_rpm_avg = rolling_buffer_average()

    local suppressed = is_spinup_suppressed()
    if not suppressed then
        if delta_rpm > peak_delta_rpm then
            peak_delta_rpm = delta_rpm
            peak_delta_pct = delta_pct
        end
    end

    mm_tel_delta_rpm = delta_rpm
    mm_tel_delta_pct = delta_pct
    mm_tel_delta_rpm_avg = delta_rpm_avg
    mm_tel_rpm_diff = delta_rpm
    mm_tel_peak_delta_rpm = peak_delta_rpm
    mm_tel_peak_delta_pct = peak_delta_pct
    mm_tel_calc_timestamp = now_ms
    mm_tel_spinup_suppressed = suppressed

    local state_changed = evaluate_desync_state(delta_pct, now_ms)
    if state_changed then
        execute_desync_response(desync_state, delta_rpm, delta_pct, now_ms)
    end

    local is_armed = arming:is_armed()
    if prev_armed and not is_armed then
        reset_on_disarm()
    end
    prev_armed = is_armed

    publish_named_values()

    return update, common.DTI_TELEMETRY_RATE_MS
end

local function init()
    for i = 1, ROLLING_BUFFER_SIZE do
        rolling_buffer[i] = 0
    end

    mm_tel_delta_rpm = 0
    mm_tel_delta_pct = 0
    mm_tel_delta_rpm_avg = 0
    mm_tel_peak_delta_rpm = 0
    mm_tel_peak_delta_pct = 0
    mm_tel_calc_timestamp = 0
    mm_tel_data_valid = false
    mm_tel_spinup_suppressed = true
    mm_tel_rpm_fwd_filtered = 0
    mm_tel_rpm_aft_filtered = 0
    mm_tel_desync_state = common.DESYNC_NORMAL
    mm_tel_rpm_diff = 0
    mm_tel_roll_rate_dps = 0
    mm_tel_pitch_correction_dps = 0

    gcs:send_text(common.MAV_SEVERITY.INFO, "TEL: Telemetry with K_rp feedforward initialized")

    publish_initial_values()

    return update, common.DTI_TELEMETRY_RATE_MS
end

return init()
