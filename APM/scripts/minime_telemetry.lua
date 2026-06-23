--[[
   MiniMe Telemetry Aggregation
   RPM differential calculation and desync monitoring for AS-10 tandem helicopter

   Prerequisites:
     minime_dti_driver.lua loaded (provides mm_dti_* globals)
     minime_hv_control.lua loaded (provides mm_hv_* globals)
     minime_common.lua available

   Exposes globals with mm_tel_ prefix for inter-script communication
]]--

local common = require("minime_common")

local LPF_ALPHA = 0.3
local ROLLING_BUFFER_SIZE = 10

local lpf_rpm_fwd = 0
local lpf_rpm_aft = 0
local lpf_initialized = false

local rolling_buffer = {}
local rolling_buffer_idx = 1
local rolling_buffer_count = 0

local peak_delta_rpm = 0
local peak_delta_pct = 0
local prev_armed = false

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
end

local update

function update()
    local now_ms = millis():toint()

    local raw_rpm_fwd = mm_dti_actual_rpm_fwd
    local raw_rpm_aft = mm_dti_actual_rpm_aft
    local heartbeat_fwd = mm_dti_heartbeat_fwd or 0
    local heartbeat_aft = mm_dti_heartbeat_aft or 0

    local fwd_valid = (raw_rpm_fwd ~= nil) and ((now_ms - heartbeat_fwd) < common.DTI_TIMEOUT_MS)
    local aft_valid = (raw_rpm_aft ~= nil) and ((now_ms - heartbeat_aft) < common.DTI_TIMEOUT_MS)

    if not fwd_valid or not aft_valid then
        mm_tel_data_valid = false
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

    local is_armed = arming:is_armed()
    if prev_armed and not is_armed then
        reset_on_disarm()
    end
    prev_armed = is_armed

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

    gcs:send_text(common.MAV_SEVERITY.INFO, "TEL: Telemetry aggregation initialized")

    return update, common.DTI_TELEMETRY_RATE_MS
end

return init()
