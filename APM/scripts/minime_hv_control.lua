--[[
   MiniMe HV Control
   High voltage state machine for AS-10 MiniMe dual rotor helicopter

   Prerequisites:
     SERVO9_FUNCTION = -1  (GPIO 50, FWD Main)
     SERVO10_FUNCTION = -1 (GPIO 51, FWD Precharge)
     SERVO11_FUNCTION = -1 (GPIO 52, AFT Main)
     SERVO12_FUNCTION = -1 (GPIO 53, AFT Precharge)
     Reboot required after SERVO_FUNCTION changes

   HVIL monitoring uses ADC input per SPECS.md Section 9.7
   Voltage thresholds: healthy ~0.9V, broken ~5V, threshold 2.5V

   Inter-Script Interface:
   Outputs (mm_hv_ prefix, updated at 50 Hz):
     mm_hv_state                  Current state (0=de_energized through 4=faulted)
     mm_hv_state_name             Human readable state name
     mm_hv_command_enable         CAN command enable flag (read by DTI driver)
     mm_hv_fault_reason           Last fault reason string or empty
     mm_hv_last_state_change_ms   Timestamp of last state transition
     mm_hv_gpio_initialized       GPIO initialization status
     mm_hv_hvil_healthy           HVIL loop healthy status
     mm_hv_hvil_voltage           HVIL ADC voltage reading
     mm_hv_cmd_energize           Command input: request energize
     mm_hv_cmd_deenergize         Command input: request de-energize
     mm_hv_cmd_reset              Command input: request fault reset
     mm_hv_redline_active         Any redline condition active
     mm_hv_redline_level          Worst redline level (normal/caution/warning/hard)
     mm_hv_redline_param          Parameter causing worst redline
     mm_hv_derate_pct             Power derate percentage (100 = full power)
     mm_hv_saturate_rpm           RPM command saturation active
     mm_hv_single_rotor_fault     Single rotor failure detected
     mm_hv_estop_active           E-stop switch is active
     mm_hv_coolant_motor_temp     Motor coolant inlet temperature or nil
     mm_hv_coolant_motor_sensor_ok Coolant temperature sensor status

   Inputs (read from DTI driver):
     mm_dti_voltage_fwd, mm_dti_voltage_aft       DC link voltage for precharge monitoring
     mm_dti_fault_fwd, mm_dti_fault_aft           DTI fault codes for fault detection
     mm_dti_heartbeat_fwd, mm_dti_heartbeat_aft   Telemetry timestamps for heartbeat check
     mm_dti_temp_motor_fwd, mm_dti_temp_motor_aft Motor temps for redline monitoring
     mm_dti_temp_ctrl_fwd, mm_dti_temp_ctrl_aft   Controller temps for redline monitoring
     mm_dti_current_ac_fwd, mm_dti_current_ac_aft Phase current for redline monitoring
     mm_dti_current_dc_fwd, mm_dti_current_dc_aft DC current for single rotor detection
     mm_dti_actual_rpm_fwd, mm_dti_actual_rpm_aft Actual RPM for overspeed/failure detection
     mm_dti_command_rpm_fwd, mm_dti_command_rpm_aft Commanded RPM for overspeed detection
     mm_dti_temp_sensor_fault_fwd, mm_dti_temp_sensor_fault_aft Sensor fault flags
]]--

local common = require("minime_common")

local MAX_INIT_ATTEMPTS = 5
local GPIO_OUTPUT = 1
local GPIO_LOW = 0
local GPIO_HIGH = 1

local init_attempts = 0
local hv_state = common.STATE_DE_ENERGIZED
local state_entry_ms = 0

local hvil_adc = nil
local hvil_initialized = false

local estop_channel = nil
local estop_initialized = false

local GPIO_PINS = {
    common.GPIO_FWD_MAIN,
    common.GPIO_FWD_PRECHARGE,
    common.GPIO_AFT_MAIN,
    common.GPIO_AFT_PRECHARGE
}

mm_hv_state = common.STATE_DE_ENERGIZED
mm_hv_state_name = "DE_ENERGIZED"
mm_hv_fault_reason = ""
mm_hv_last_state_change_ms = 0
mm_hv_gpio_initialized = false
mm_hv_hvil_healthy = false
mm_hv_hvil_voltage = 0.0
mm_hv_command_enable = false

mm_hv_cmd_energize = false
mm_hv_cmd_deenergize = false
mm_hv_cmd_reset = false

mm_hv_redline_active = false
mm_hv_redline_level = "normal"
mm_hv_redline_param = ""
mm_hv_derate_pct = 100
mm_hv_saturate_rpm = false

mm_hv_single_rotor_fault = false
mm_hv_estop_active = false

mm_hv_coolant_motor_temp = nil
mm_hv_coolant_motor_sensor_ok = false

local precharge_start_ms = 0
local precharge_telem_seen = false
local PRECHARGE_TELEM_TIMEOUT_MS = 2000
local PRECHARGE_FALLBACK_TIME_MS = 3000
local EXPECTED_VOLTAGE = 133.2
local CONTACTOR_SETTLE_MS = 50

local settling_started = false
local settling_start_ms = 0
local settling_reason = ""

local prev_armed = false

local redline_state = {
    motor_winding_fwd = nil,
    motor_winding_aft = nil,
    motor_part_fwd = nil,
    motor_part_aft = nil,
    inverter_temp_fwd = nil,
    inverter_temp_aft = nil,
    phase_current_fwd = nil,
    phase_current_aft = nil,
    dc_link_low_fwd = nil,
    dc_link_low_aft = nil,
    dc_link_high_fwd = nil,
    dc_link_high_aft = nil,
    battery_low = nil,
    battery_high = nil,
    coolant_motor = nil,
    motor_rpm_fwd = nil,
    motor_rpm_aft = nil
}

local redline_last_alert_ms = {}
local overspeed_start_ms_fwd = nil
local overspeed_start_ms_aft = nil
local coolant_sensor_warned = false

local single_rotor_rpm_start_ms_fwd = nil
local single_rotor_rpm_start_ms_aft = nil

local SEVERITY_ORDER = { caution = 1, warning = 2, hard = 3 }

local STATE_NAMES = {
    [common.STATE_DE_ENERGIZED] = "DE_ENERGIZED",
    [common.STATE_PRECHARGING] = "PRECHARGING",
    [common.STATE_ENERGIZED] = "ENERGIZED",
    [common.STATE_ARMED] = "ARMED",
    [common.STATE_FAULTED] = "FAULTED"
}

local function state_name(state)
    return STATE_NAMES[state] or "UNKNOWN"
end

local function set_safe_state()
    for _, pin in ipairs(GPIO_PINS) do
        gpio:write(pin, GPIO_LOW)
    end
end

local function set_precharge_state()
    gpio:write(common.GPIO_FWD_PRECHARGE, GPIO_HIGH)
    gpio:write(common.GPIO_AFT_PRECHARGE, GPIO_HIGH)
    gpio:write(common.GPIO_FWD_MAIN, GPIO_LOW)
    gpio:write(common.GPIO_AFT_MAIN, GPIO_LOW)
end

local function set_energized_state()
    gpio:write(common.GPIO_FWD_MAIN, GPIO_HIGH)
    gpio:write(common.GPIO_AFT_MAIN, GPIO_HIGH)
    gpio:write(common.GPIO_FWD_PRECHARGE, GPIO_LOW)
    gpio:write(common.GPIO_AFT_PRECHARGE, GPIO_LOW)
end

local function close_main_ssrs()
    gpio:write(common.GPIO_FWD_MAIN, GPIO_HIGH)
    gpio:write(common.GPIO_AFT_MAIN, GPIO_HIGH)
end

local function open_precharge_ssrs()
    gpio:write(common.GPIO_FWD_PRECHARGE, GPIO_LOW)
    gpio:write(common.GPIO_AFT_PRECHARGE, GPIO_LOW)
end

local function get_hysteresis_threshold(redline, level, invert)
    local threshold = redline[level]
    if invert then
        return threshold * (1 + common.REDLINE_HYSTERESIS_PCT)
    else
        return threshold * (1 - common.REDLINE_HYSTERESIS_PCT)
    end
end

local function check_redline_hysteresis(value, redline, current_level, invert)
    if value == nil then
        return current_level
    end

    local new_level = common.check_redline(value, redline, invert)

    if new_level == nil and current_level ~= nil then
        local clear_threshold = get_hysteresis_threshold(redline, current_level, invert)
        if invert then
            if value < clear_threshold then
                return current_level
            end
        else
            if value > clear_threshold then
                return current_level
            end
        end
    end

    if new_level ~= nil then
        return new_level
    end
    return nil
end

local function should_alert(param_name, level)
    local now_ms = millis():toint()
    local key = param_name .. "_" .. level
    local last_ms = redline_last_alert_ms[key] or 0
    if now_ms - last_ms >= common.REDLINE_ALERT_RATE_LIMIT_MS then
        redline_last_alert_ms[key] = now_ms
        return true
    end
    return false
end

local function send_redline_alert(param_name, level, value, unit)
    if level == "caution" then
        if should_alert(param_name, level) then
            gcs:send_text(common.MAV_SEVERITY.NOTICE,
                string.format("HV REDLINE: %s caution %.1f%s", param_name, value, unit))
        end
    elseif level == "warning" then
        if should_alert(param_name, level) then
            gcs:send_text(common.MAV_SEVERITY.WARNING,
                string.format("HV REDLINE: %s warning %.1f%s", param_name, value, unit))
        end
    elseif level == "hard" then
        gcs:send_text(common.MAV_SEVERITY.CRITICAL,
            string.format("HV REDLINE: %s HARD LIMIT %.1f%s", param_name, value, unit))
    end
end

local function trigger_derate(percent, reason)
    if mm_hv_derate_pct > percent then
        mm_hv_derate_pct = percent
        gcs:send_text(common.MAV_SEVERITY.WARNING,
            string.format("HV: Power derate to %d%% (%s)", percent, reason))
    end
end

local function trigger_rtl(reason)
    local current_mode = vehicle:get_mode()
    if current_mode ~= common.MODE_RTL and current_mode ~= common.MODE_LAND then
        vehicle:set_mode(common.MODE_RTL)
        gcs:send_text(common.MAV_SEVERITY.CRITICAL,
            string.format("HV: RTL triggered (%s)", reason))
    end
end

local function trigger_land(reason)
    local current_mode = vehicle:get_mode()
    if current_mode ~= common.MODE_LAND then
        vehicle:set_mode(common.MODE_LAND)
        gcs:send_text(common.MAV_SEVERITY.CRITICAL,
            string.format("HV: LAND triggered (%s)", reason))
    end
end

local function saturate_command(reason)
    if not mm_hv_saturate_rpm then
        mm_hv_saturate_rpm = true
        gcs:send_text(common.MAV_SEVERITY.WARNING,
            string.format("HV: Command saturated (%s)", reason))
    end
end

local function read_coolant_motor_temp()
    if not temperature_sensor then
        return nil
    end
    local temp = temperature_sensor:get_temperature(common.COOLANT_MOTOR_SENSOR_INDEX)
    return temp
end

local function check_uncommanded_overspeed()
    local now_ms = millis():toint()
    local cmd_fwd = mm_dti_command_rpm_fwd or 0
    local cmd_aft = mm_dti_command_rpm_aft or 0
    local actual_fwd = mm_dti_actual_rpm_fwd or 0
    local actual_aft = mm_dti_actual_rpm_aft or 0

    if cmd_fwd > 0 then
        local threshold_fwd = cmd_fwd * (1 + common.OVERSPEED_THRESHOLD_PCT)
        if actual_fwd > threshold_fwd then
            if overspeed_start_ms_fwd == nil then
                overspeed_start_ms_fwd = now_ms
            elseif now_ms - overspeed_start_ms_fwd >= common.OVERSPEED_DURATION_MS then
                emergency_shutdown("OVERSPEED_FWD")
                return true
            end
        else
            overspeed_start_ms_fwd = nil
        end
    else
        overspeed_start_ms_fwd = nil
    end

    if cmd_aft > 0 then
        local threshold_aft = cmd_aft * (1 + common.OVERSPEED_THRESHOLD_PCT)
        if actual_aft > threshold_aft then
            if overspeed_start_ms_aft == nil then
                overspeed_start_ms_aft = now_ms
            elseif now_ms - overspeed_start_ms_aft >= common.OVERSPEED_DURATION_MS then
                emergency_shutdown("OVERSPEED_AFT")
                return true
            end
        else
            overspeed_start_ms_aft = nil
        end
    else
        overspeed_start_ms_aft = nil
    end

    return false
end

local function check_single_rotor_failure()
    local now_ms = millis():toint()

    local cmd_fwd = mm_dti_command_rpm_fwd or 0
    local cmd_aft = mm_dti_command_rpm_aft or 0
    local actual_fwd = mm_dti_actual_rpm_fwd or 0
    local actual_aft = mm_dti_actual_rpm_aft or 0
    local current_fwd = mm_dti_current_dc_fwd or 0
    local current_aft = mm_dti_current_dc_aft or 0
    local fault_fwd = mm_dti_fault_fwd or 0
    local fault_aft = mm_dti_fault_aft or 0
    local heartbeat_fwd = mm_dti_heartbeat_fwd or 0
    local heartbeat_aft = mm_dti_heartbeat_aft or 0

    if cmd_fwd <= 0 and cmd_aft <= 0 then
        single_rotor_rpm_start_ms_fwd = nil
        single_rotor_rpm_start_ms_aft = nil
        mm_hv_single_rotor_fault = false
        return nil
    end

    if fault_fwd ~= 0 and fault_aft == 0 then
        mm_hv_single_rotor_fault = true
        return "DTI_FAULT_FWD_ONLY"
    end
    if fault_aft ~= 0 and fault_fwd == 0 then
        mm_hv_single_rotor_fault = true
        return "DTI_FAULT_AFT_ONLY"
    end

    if current_fwd < common.SINGLE_ROTOR_CURRENT_NEAR_ZERO and
       current_aft > common.SINGLE_ROTOR_CURRENT_NORMAL then
        mm_hv_single_rotor_fault = true
        return "CURRENT_ANOMALY_FWD"
    end
    if current_aft < common.SINGLE_ROTOR_CURRENT_NEAR_ZERO and
       current_fwd > common.SINGLE_ROTOR_CURRENT_NORMAL then
        mm_hv_single_rotor_fault = true
        return "CURRENT_ANOMALY_AFT"
    end

    local fwd_heartbeat_age = now_ms - heartbeat_fwd
    local aft_heartbeat_age = now_ms - heartbeat_aft
    if fwd_heartbeat_age > common.SINGLE_ROTOR_HEARTBEAT_TIMEOUT_MS and
       aft_heartbeat_age < common.SINGLE_ROTOR_HEARTBEAT_TIMEOUT_MS then
        mm_hv_single_rotor_fault = true
        return "HEARTBEAT_LOSS_FWD"
    end
    if aft_heartbeat_age > common.SINGLE_ROTOR_HEARTBEAT_TIMEOUT_MS and
       fwd_heartbeat_age < common.SINGLE_ROTOR_HEARTBEAT_TIMEOUT_MS then
        mm_hv_single_rotor_fault = true
        return "HEARTBEAT_LOSS_AFT"
    end

    if cmd_fwd > 0 then
        local drop_threshold = cmd_fwd * (1 - common.SINGLE_ROTOR_RPM_DROP_PCT)
        local maintain_threshold = cmd_aft * (1 - common.SINGLE_ROTOR_RPM_MAINTAIN_PCT)

        if actual_fwd < drop_threshold and actual_aft > maintain_threshold then
            if single_rotor_rpm_start_ms_fwd == nil then
                single_rotor_rpm_start_ms_fwd = now_ms
            elseif now_ms - single_rotor_rpm_start_ms_fwd >= common.SINGLE_ROTOR_DETECT_DURATION_MS then
                mm_hv_single_rotor_fault = true
                return "RPM_LOSS_FWD"
            end
        else
            single_rotor_rpm_start_ms_fwd = nil
        end
    else
        single_rotor_rpm_start_ms_fwd = nil
    end

    if cmd_aft > 0 then
        local drop_threshold = cmd_aft * (1 - common.SINGLE_ROTOR_RPM_DROP_PCT)
        local maintain_threshold = cmd_fwd * (1 - common.SINGLE_ROTOR_RPM_MAINTAIN_PCT)

        if actual_aft < drop_threshold and actual_fwd > maintain_threshold then
            if single_rotor_rpm_start_ms_aft == nil then
                single_rotor_rpm_start_ms_aft = now_ms
            elseif now_ms - single_rotor_rpm_start_ms_aft >= common.SINGLE_ROTOR_DETECT_DURATION_MS then
                mm_hv_single_rotor_fault = true
                return "RPM_LOSS_AFT"
            end
        else
            single_rotor_rpm_start_ms_aft = nil
        end
    else
        single_rotor_rpm_start_ms_aft = nil
    end

    mm_hv_single_rotor_fault = false
    return nil
end

local function check_redlines()
    local worst_level = nil
    local worst_param = ""

    local sensor_fault_fwd = mm_dti_temp_sensor_fault_fwd or false
    local sensor_fault_aft = mm_dti_temp_sensor_fault_aft or false

    if not sensor_fault_fwd then
        local temp = mm_dti_temp_motor_fwd or 0
        local prev = redline_state.motor_winding_fwd
        local level = check_redline_hysteresis(temp, common.REDLINE_MOTOR_WINDING, prev, false)
        redline_state.motor_winding_fwd = level
        if level then
            send_redline_alert("MOTOR_TEMP_FWD", level, temp, "C")
            if level == "hard" then
                emergency_shutdown("MOTOR_TEMP_FWD")
                return
            end
            if SEVERITY_ORDER[level] > (SEVERITY_ORDER[worst_level] or 0) then
                worst_level = level
                worst_param = "MOTOR_TEMP_FWD"
            end
        end
    end

    if not sensor_fault_aft then
        local temp = mm_dti_temp_motor_aft or 0
        local prev = redline_state.motor_winding_aft
        local level = check_redline_hysteresis(temp, common.REDLINE_MOTOR_WINDING, prev, false)
        redline_state.motor_winding_aft = level
        if level then
            send_redline_alert("MOTOR_TEMP_AFT", level, temp, "C")
            if level == "hard" then
                emergency_shutdown("MOTOR_TEMP_AFT")
                return
            end
            if SEVERITY_ORDER[level] > (SEVERITY_ORDER[worst_level] or 0) then
                worst_level = level
                worst_param = "MOTOR_TEMP_AFT"
            end
        end
    end

    if not sensor_fault_fwd then
        local temp = mm_dti_temp_motor_fwd or 0
        local prev = redline_state.motor_part_fwd
        local level = check_redline_hysteresis(temp, common.REDLINE_MOTOR_PART, prev, false)
        redline_state.motor_part_fwd = level
        if level then
            send_redline_alert("MOTOR_PART_FWD", level, temp, "C")
            if level == "hard" then
                emergency_shutdown("MOTOR_PART_FWD")
                return
            end
            if SEVERITY_ORDER[level] > (SEVERITY_ORDER[worst_level] or 0) then
                worst_level = level
                worst_param = "MOTOR_PART_FWD"
            end
        end
    end

    if not sensor_fault_aft then
        local temp = mm_dti_temp_motor_aft or 0
        local prev = redline_state.motor_part_aft
        local level = check_redline_hysteresis(temp, common.REDLINE_MOTOR_PART, prev, false)
        redline_state.motor_part_aft = level
        if level then
            send_redline_alert("MOTOR_PART_AFT", level, temp, "C")
            if level == "hard" then
                emergency_shutdown("MOTOR_PART_AFT")
                return
            end
            if SEVERITY_ORDER[level] > (SEVERITY_ORDER[worst_level] or 0) then
                worst_level = level
                worst_param = "MOTOR_PART_AFT"
            end
        end
    end

    if not sensor_fault_fwd then
        local temp = mm_dti_temp_ctrl_fwd or 0
        local prev = redline_state.inverter_temp_fwd
        local level = check_redline_hysteresis(temp, common.REDLINE_COOLANT_INVERTER, prev, false)
        redline_state.inverter_temp_fwd = level
        if level then
            send_redline_alert("INVERTER_TEMP_FWD", level, temp, "C")
            if level == "hard" then
                trigger_derate(50, "INVERTER_TEMP_FWD")
            elseif level == "warning" then
                trigger_derate(75, "INVERTER_TEMP_FWD")
            end
            if SEVERITY_ORDER[level] > (SEVERITY_ORDER[worst_level] or 0) then
                worst_level = level
                worst_param = "INVERTER_TEMP_FWD"
            end
        end
    end

    if not sensor_fault_aft then
        local temp = mm_dti_temp_ctrl_aft or 0
        local prev = redline_state.inverter_temp_aft
        local level = check_redline_hysteresis(temp, common.REDLINE_COOLANT_INVERTER, prev, false)
        redline_state.inverter_temp_aft = level
        if level then
            send_redline_alert("INVERTER_TEMP_AFT", level, temp, "C")
            if level == "hard" then
                trigger_derate(50, "INVERTER_TEMP_AFT")
            elseif level == "warning" then
                trigger_derate(75, "INVERTER_TEMP_AFT")
            end
            if SEVERITY_ORDER[level] > (SEVERITY_ORDER[worst_level] or 0) then
                worst_level = level
                worst_param = "INVERTER_TEMP_AFT"
            end
        end
    end

    do
        local current_fwd = mm_dti_current_ac_fwd or 0
        local prev = redline_state.phase_current_fwd
        local level = check_redline_hysteresis(current_fwd, common.REDLINE_PHASE_CURRENT, prev, false)
        redline_state.phase_current_fwd = level
        if level then
            send_redline_alert("PHASE_CURR_FWD", level, current_fwd, "A")
            if level == "hard" then
                saturate_command("PHASE_CURR_FWD")
            end
            if SEVERITY_ORDER[level] > (SEVERITY_ORDER[worst_level] or 0) then
                worst_level = level
                worst_param = "PHASE_CURR_FWD"
            end
        end
    end

    do
        local current_aft = mm_dti_current_ac_aft or 0
        local prev = redline_state.phase_current_aft
        local level = check_redline_hysteresis(current_aft, common.REDLINE_PHASE_CURRENT, prev, false)
        redline_state.phase_current_aft = level
        if level then
            send_redline_alert("PHASE_CURR_AFT", level, current_aft, "A")
            if level == "hard" then
                saturate_command("PHASE_CURR_AFT")
            end
            if SEVERITY_ORDER[level] > (SEVERITY_ORDER[worst_level] or 0) then
                worst_level = level
                worst_param = "PHASE_CURR_AFT"
            end
        end
    end

    do
        local voltage_fwd = mm_dti_voltage_fwd or 0
        local prev = redline_state.dc_link_low_fwd
        local level = check_redline_hysteresis(voltage_fwd, common.REDLINE_DC_LINK_LOW, prev, true)
        redline_state.dc_link_low_fwd = level
        if level then
            send_redline_alert("DC_LINK_LOW_FWD", level, voltage_fwd, "V")
            if level == "hard" then
                emergency_shutdown("DC_LINK_LOW_FWD")
                return
            end
            if SEVERITY_ORDER[level] > (SEVERITY_ORDER[worst_level] or 0) then
                worst_level = level
                worst_param = "DC_LINK_LOW_FWD"
            end
        end
    end

    do
        local voltage_aft = mm_dti_voltage_aft or 0
        local prev = redline_state.dc_link_low_aft
        local level = check_redline_hysteresis(voltage_aft, common.REDLINE_DC_LINK_LOW, prev, true)
        redline_state.dc_link_low_aft = level
        if level then
            send_redline_alert("DC_LINK_LOW_AFT", level, voltage_aft, "V")
            if level == "hard" then
                emergency_shutdown("DC_LINK_LOW_AFT")
                return
            end
            if SEVERITY_ORDER[level] > (SEVERITY_ORDER[worst_level] or 0) then
                worst_level = level
                worst_param = "DC_LINK_LOW_AFT"
            end
        end
    end

    do
        local voltage_fwd = mm_dti_voltage_fwd or 0
        local prev = redline_state.dc_link_high_fwd
        local level = check_redline_hysteresis(voltage_fwd, common.REDLINE_DC_LINK_HIGH, prev, false)
        redline_state.dc_link_high_fwd = level
        if level then
            send_redline_alert("DC_LINK_HIGH_FWD", level, voltage_fwd, "V")
            if level == "hard" then
                emergency_shutdown("DC_LINK_HIGH_FWD")
                return
            end
            if SEVERITY_ORDER[level] > (SEVERITY_ORDER[worst_level] or 0) then
                worst_level = level
                worst_param = "DC_LINK_HIGH_FWD"
            end
        end
    end

    do
        local voltage_aft = mm_dti_voltage_aft or 0
        local prev = redline_state.dc_link_high_aft
        local level = check_redline_hysteresis(voltage_aft, common.REDLINE_DC_LINK_HIGH, prev, false)
        redline_state.dc_link_high_aft = level
        if level then
            send_redline_alert("DC_LINK_HIGH_AFT", level, voltage_aft, "V")
            if level == "hard" then
                emergency_shutdown("DC_LINK_HIGH_AFT")
                return
            end
            if SEVERITY_ORDER[level] > (SEVERITY_ORDER[worst_level] or 0) then
                worst_level = level
                worst_param = "DC_LINK_HIGH_AFT"
            end
        end
    end

    do
        local voltage_min = math.min(mm_dti_voltage_fwd or 999, mm_dti_voltage_aft or 999)
        local prev = redline_state.battery_low
        local level = check_redline_hysteresis(voltage_min, common.REDLINE_BATTERY_LOW, prev, true)
        redline_state.battery_low = level
        if level then
            send_redline_alert("BATTERY_LOW", level, voltage_min, "V")
            if level == "hard" then
                trigger_land("BATTERY_LOW")
            end
            if SEVERITY_ORDER[level] > (SEVERITY_ORDER[worst_level] or 0) then
                worst_level = level
                worst_param = "BATTERY_LOW"
            end
        end
    end

    do
        local voltage_max = math.max(mm_dti_voltage_fwd or 0, mm_dti_voltage_aft or 0)
        local prev = redline_state.battery_high
        local level = check_redline_hysteresis(voltage_max, common.REDLINE_BATTERY_HIGH, prev, false)
        redline_state.battery_high = level
        if level then
            send_redline_alert("BATTERY_HIGH", level, voltage_max, "V")
        end
    end

    do
        local coolant_temp = read_coolant_motor_temp()
        if coolant_temp then
            mm_hv_coolant_motor_temp = coolant_temp
            mm_hv_coolant_motor_sensor_ok = true
            local prev = redline_state.coolant_motor
            local level = check_redline_hysteresis(coolant_temp, common.REDLINE_COOLANT_MOTOR, prev, false)
            redline_state.coolant_motor = level
            if level then
                send_redline_alert("COOLANT_MOTOR", level, coolant_temp, "C")
                if level == "hard" then
                    trigger_derate(50, "COOLANT_MOTOR")
                    trigger_rtl("COOLANT_MOTOR")
                elseif level == "warning" then
                    trigger_derate(75, "COOLANT_MOTOR")
                end
                if SEVERITY_ORDER[level] > (SEVERITY_ORDER[worst_level] or 0) then
                    worst_level = level
                    worst_param = "COOLANT_MOTOR"
                end
            end
        else
            mm_hv_coolant_motor_sensor_ok = false
            if not coolant_sensor_warned then
                gcs:send_text(common.MAV_SEVERITY.NOTICE, "HV: Coolant temp sensor not available")
                coolant_sensor_warned = true
            end
        end
    end

    do
        local rpm_fwd = mm_dti_actual_rpm_fwd or 0
        local motor_rpm_fwd = rpm_fwd * common.GEARBOX_RATIO
        local prev = redline_state.motor_rpm_fwd
        local level = check_redline_hysteresis(motor_rpm_fwd, common.REDLINE_MOTOR_RPM, prev, false)
        redline_state.motor_rpm_fwd = level
        if level then
            send_redline_alert("MOTOR_RPM_FWD", level, motor_rpm_fwd, "")
            if level == "hard" then
                saturate_command("MOTOR_RPM_FWD")
            end
            if SEVERITY_ORDER[level] > (SEVERITY_ORDER[worst_level] or 0) then
                worst_level = level
                worst_param = "MOTOR_RPM_FWD"
            end
        end
    end

    do
        local rpm_aft = mm_dti_actual_rpm_aft or 0
        local motor_rpm_aft = rpm_aft * common.GEARBOX_RATIO
        local prev = redline_state.motor_rpm_aft
        local level = check_redline_hysteresis(motor_rpm_aft, common.REDLINE_MOTOR_RPM, prev, false)
        redline_state.motor_rpm_aft = level
        if level then
            send_redline_alert("MOTOR_RPM_AFT", level, motor_rpm_aft, "")
            if level == "hard" then
                saturate_command("MOTOR_RPM_AFT")
            end
            if SEVERITY_ORDER[level] > (SEVERITY_ORDER[worst_level] or 0) then
                worst_level = level
                worst_param = "MOTOR_RPM_AFT"
            end
        end
    end

    if check_uncommanded_overspeed() then
        return
    end

    mm_hv_redline_active = (worst_level ~= nil)
    mm_hv_redline_level = worst_level or "normal"
    mm_hv_redline_param = worst_param
end

local function reset_redline_state()
    for k, _ in pairs(redline_state) do
        redline_state[k] = nil
    end
    redline_last_alert_ms = {}
    overspeed_start_ms_fwd = nil
    overspeed_start_ms_aft = nil
    single_rotor_rpm_start_ms_fwd = nil
    single_rotor_rpm_start_ms_aft = nil
    mm_hv_redline_active = false
    mm_hv_redline_level = "normal"
    mm_hv_redline_param = ""
    mm_hv_derate_pct = 100
    mm_hv_saturate_rpm = false
    mm_hv_single_rotor_fault = false
    mm_hv_coolant_motor_temp = nil
    mm_hv_coolant_motor_sensor_ok = false
    coolant_sensor_warned = false
end

local function init_gpio()
    for _, pin in ipairs(GPIO_PINS) do
        gpio:pinMode(pin, GPIO_OUTPUT)
        gpio:write(pin, GPIO_LOW)
    end
    return true
end

local function init_hvil_adc()
    hvil_adc = analog:channel()
    if hvil_adc then
        hvil_adc:set_pin(common.HVIL_ADC_PIN)
        hvil_initialized = true
        return true
    end
    return false
end

local function read_hvil_voltage()
    if not hvil_adc then
        return nil
    end
    local voltage = hvil_adc:voltage_average()
    mm_hv_hvil_voltage = voltage or 0.0
    return voltage
end

local function is_hvil_healthy()
    local voltage = read_hvil_voltage()
    if voltage == nil then
        mm_hv_hvil_healthy = false
        return false, "HVIL_ADC_UNAVAILABLE"
    end
    if voltage > common.HVIL_THRESHOLD then
        mm_hv_hvil_healthy = false
        return false, "HVIL_BROKEN"
    end
    mm_hv_hvil_healthy = true
    return true, "HVIL_OK"
end

local function find_estop_channel()
    local channel = rc:find_channel_for_option(common.RC_OPTION_MOTOR_ESTOP)
    if channel then
        estop_channel = channel:ch_num()
        estop_initialized = true
        return true
    end
    return false
end

local function check_estop_active()
    if not estop_initialized or estop_channel == nil then
        mm_hv_estop_active = false
        return false
    end

    local pwm = rc:get_pwm(estop_channel)
    if pwm == nil then
        mm_hv_estop_active = false
        return false
    end

    local active = pwm >= common.ESTOP_PWM_THRESHOLD
    mm_hv_estop_active = active
    return active
end

local function emergency_shutdown(reason)
    local shutdown_ms = millis():toint()
    mm_hv_command_enable = false

    set_safe_state()
    reset_redline_state()

    local prev_state = hv_state
    hv_state = common.STATE_FAULTED
    state_entry_ms = shutdown_ms

    mm_hv_state = hv_state
    mm_hv_state_name = state_name(hv_state)
    mm_hv_fault_reason = reason
    mm_hv_last_state_change_ms = state_entry_ms

    gcs:send_text(common.MAV_SEVERITY.CRITICAL,
        string.format("HV: EMERGENCY SHUTDOWN @%dms, %s to FAULTED (%s)",
            shutdown_ms, state_name(prev_state), reason))
end

local function check_hvil_break()
    if hv_state == common.STATE_ENERGIZED or
       hv_state == common.STATE_ARMED then
        local healthy, status = is_hvil_healthy()
        if not healthy then
            emergency_shutdown("HVIL_BREAK")
            return true
        end
    end
    return false
end

local function can_energize()
    local healthy, status = is_hvil_healthy()
    if not healthy then
        gcs:send_text(common.MAV_SEVERITY.ERROR,
            string.format("HV: Cannot energize, %s", status))
        return false
    end
    return true
end

local function transition_state(new_state, reason)
    local prev_state = hv_state
    hv_state = new_state
    state_entry_ms = millis():toint()

    mm_hv_state = hv_state
    mm_hv_state_name = state_name(hv_state)
    mm_hv_last_state_change_ms = state_entry_ms

    gcs:send_text(common.MAV_SEVERITY.INFO,
        string.format("HV: %s to %s (%s)",
            state_name(prev_state), state_name(new_state), reason))
end

local function is_undervoltage_fault(fault_code)
    return fault_code == 0x02
end

local function check_dti_fault_non_precharge()
    local fwd_fault = mm_dti_fault_fwd or 0
    local aft_fault = mm_dti_fault_aft or 0
    if fwd_fault ~= 0 then
        return true, "DTI_FWD_FAULT"
    end
    if aft_fault ~= 0 then
        return true, "DTI_AFT_FAULT"
    end
    return false, nil
end

local function check_dti_fault_precharge()
    local fwd_fault = mm_dti_fault_fwd or 0
    local aft_fault = mm_dti_fault_aft or 0
    if fwd_fault ~= 0 and not is_undervoltage_fault(fwd_fault) then
        return true, "DTI_FWD_FAULT"
    end
    if aft_fault ~= 0 and not is_undervoltage_fault(aft_fault) then
        return true, "DTI_AFT_FAULT"
    end
    return false, nil
end

local function handle_de_energized()
    set_safe_state()
    mm_hv_command_enable = false

    if mm_hv_cmd_energize then
        mm_hv_cmd_energize = false
        if can_energize() then
            precharge_start_ms = millis():toint()
            precharge_telem_seen = false
            settling_started = false
            settling_reason = ""
            transition_state(common.STATE_PRECHARGING, "ENERGIZE_CMD")
        end
    end
end

local function handle_precharging()
    if not settling_started then
        set_precharge_state()
    end
    mm_hv_command_enable = false

    local now_ms = millis():toint()
    local elapsed_ms = now_ms - precharge_start_ms

    local has_fault, fault_reason = check_dti_fault_precharge()
    if has_fault then
        settling_started = false
        emergency_shutdown(fault_reason)
        return
    end

    if mm_hv_cmd_deenergize then
        mm_hv_cmd_deenergize = false
        settling_started = false
        set_safe_state()
        transition_state(common.STATE_DE_ENERGIZED, "DEENERGIZE_CMD")
        return
    end

    if settling_started then
        local settling_elapsed = now_ms - settling_start_ms
        if settling_elapsed >= CONTACTOR_SETTLE_MS then
            open_precharge_ssrs()
            local voltage = math.min(mm_dti_voltage_fwd or 0, mm_dti_voltage_aft or 0)
            gcs:send_text(common.MAV_SEVERITY.INFO,
                string.format("HV: Precharge complete %.0fV %dms", voltage, elapsed_ms))
            settling_started = false
            transition_state(common.STATE_ENERGIZED, settling_reason)
        end
        return
    end

    if elapsed_ms > common.PRECHARGE_TIMEOUT_MS then
        emergency_shutdown("PRECHARGE_TIMEOUT")
        return
    end

    local voltage_fwd = mm_dti_voltage_fwd or 0
    local voltage_aft = mm_dti_voltage_aft or 0
    local heartbeat_fwd = mm_dti_heartbeat_fwd or 0
    local heartbeat_aft = mm_dti_heartbeat_aft or 0

    local has_recent_telem = false
    if heartbeat_fwd > precharge_start_ms or heartbeat_aft > precharge_start_ms then
        has_recent_telem = true
        precharge_telem_seen = true
    end

    local voltage_threshold = EXPECTED_VOLTAGE * common.PRECHARGE_THRESHOLD
    local voltage_ok = false
    if has_recent_telem then
        local min_voltage = math.min(voltage_fwd, voltage_aft)
        if min_voltage >= voltage_threshold then
            voltage_ok = true
        end
    end

    local time_ok = elapsed_ms >= common.PRECHARGE_MIN_TIME_MS

    local complete = false
    local reason = ""

    if has_recent_telem and voltage_ok and time_ok then
        complete = true
        reason = string.format("VOLTAGE_OK_%.0fV", math.min(voltage_fwd, voltage_aft))
    elseif not precharge_telem_seen and elapsed_ms >= PRECHARGE_TELEM_TIMEOUT_MS then
        if elapsed_ms >= PRECHARGE_FALLBACK_TIME_MS then
            complete = true
            reason = "TIME_FALLBACK"
            gcs:send_text(common.MAV_SEVERITY.WARNING,
                "HV: Precharge fallback, DTI telemetry unavailable")
        end
    end

    if complete then
        close_main_ssrs()
        settling_started = true
        settling_start_ms = now_ms
        settling_reason = reason
    end
end

local function handle_energized()
    set_energized_state()
    mm_hv_command_enable = false

    local has_fault, fault_reason = check_dti_fault_non_precharge()
    if has_fault then
        emergency_shutdown(fault_reason)
        return
    end

    local single_rotor_reason = check_single_rotor_failure()
    if single_rotor_reason then
        emergency_shutdown(single_rotor_reason)
        return
    end

    check_redlines()
    if hv_state == common.STATE_FAULTED then
        return
    end

    if mm_hv_cmd_deenergize then
        mm_hv_cmd_deenergize = false
        reset_redline_state()
        set_safe_state()
        transition_state(common.STATE_DE_ENERGIZED, "DEENERGIZE_CMD")
        return
    end

    local is_armed = arming:is_armed()
    if is_armed and not prev_armed then
        mm_hv_command_enable = true
        transition_state(common.STATE_ARMED, "ARDUPILOT_ARM")
    end
    prev_armed = is_armed
end

local function handle_armed()
    set_energized_state()
    mm_hv_command_enable = true

    local has_fault, fault_reason = check_dti_fault_non_precharge()
    if has_fault then
        emergency_shutdown(fault_reason)
        return
    end

    local single_rotor_reason = check_single_rotor_failure()
    if single_rotor_reason then
        emergency_shutdown(single_rotor_reason)
        return
    end

    check_redlines()
    if hv_state == common.STATE_FAULTED then
        return
    end

    if mm_hv_cmd_deenergize then
        mm_hv_cmd_deenergize = false
        mm_hv_command_enable = false
        reset_redline_state()
        set_safe_state()
        transition_state(common.STATE_DE_ENERGIZED, "DEENERGIZE_CMD")
        return
    end

    local is_armed = arming:is_armed()
    if not is_armed and prev_armed then
        mm_hv_command_enable = false
        reset_redline_state()
        transition_state(common.STATE_ENERGIZED, "ARDUPILOT_DISARM")
    end
    prev_armed = is_armed
end

local function handle_faulted()
    set_safe_state()
    mm_hv_command_enable = false

    if mm_hv_cmd_reset then
        mm_hv_cmd_reset = false
        if can_energize() then
            transition_state(common.STATE_DE_ENERGIZED, "FAULT_RESET")
        else
            gcs:send_text(common.MAV_SEVERITY.ERROR,
                "HV: Reset blocked, HVIL not healthy")
        end
    end
end

local update

function update()
    if check_hvil_break() then
        return update, common.HVIL_POLL_RATE_MS
    end

    if hv_state ~= common.STATE_DE_ENERGIZED and
       hv_state ~= common.STATE_FAULTED and
       check_estop_active() then
        emergency_shutdown("ESTOP")
        return update, common.HVIL_POLL_RATE_MS
    end

    if hv_state == common.STATE_DE_ENERGIZED then
        handle_de_energized()
    elseif hv_state == common.STATE_PRECHARGING then
        handle_precharging()
    elseif hv_state == common.STATE_ENERGIZED then
        handle_energized()
    elseif hv_state == common.STATE_ARMED then
        handle_armed()
    elseif hv_state == common.STATE_FAULTED then
        handle_faulted()
    end

    mm_hv_state = hv_state
    mm_hv_state_name = state_name(hv_state)

    return update, common.HVIL_POLL_RATE_MS
end

local function init()
    init_attempts = init_attempts + 1

    local gpio_ok = init_gpio()
    local hvil_ok = init_hvil_adc()
    local estop_ok = find_estop_channel()

    if gpio_ok and hvil_ok then
        hv_state = common.STATE_DE_ENERGIZED
        state_entry_ms = millis():toint()

        mm_hv_state = hv_state
        mm_hv_state_name = state_name(hv_state)
        mm_hv_last_state_change_ms = state_entry_ms
        mm_hv_gpio_initialized = true

        local healthy, status = is_hvil_healthy()
        local hvil_msg = healthy and "HVIL OK" or status

        local estop_msg = ""
        if estop_ok then
            estop_msg = string.format(", E-stop CH%d", estop_channel)
        else
            estop_msg = ", E-stop not configured"
            gcs:send_text(common.MAV_SEVERITY.NOTICE,
                "HV: No RC channel configured with RCx_OPTION=31 for E-stop")
        end

        gcs:send_text(common.MAV_SEVERITY.INFO,
            string.format("HV: Initialized, state DE_ENERGIZED, %s (%.2fV)%s",
                hvil_msg, mm_hv_hvil_voltage, estop_msg))

        return update, common.HVIL_POLL_RATE_MS
    end

    if init_attempts >= MAX_INIT_ATTEMPTS then
        local fail_reason = ""
        if not gpio_ok then
            fail_reason = "GPIO init failed, check SERVO9-12_FUNCTION=-1"
        elseif not hvil_ok then
            fail_reason = "HVIL ADC init failed"
        end

        gcs:send_text(common.MAV_SEVERITY.ERROR,
            string.format("HV: Init failed: %s", fail_reason))
        set_safe_state()
        mm_hv_gpio_initialized = false
        return nil
    end

    gcs:send_text(common.MAV_SEVERITY.WARNING,
        string.format("HV: Init retry %d/%d",
            init_attempts, MAX_INIT_ATTEMPTS))

    return init, 1000
end

return init()
