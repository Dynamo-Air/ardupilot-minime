--[[
   MiniMe DTI Driver
   CAN driver for DTI HV550 motor controllers

   Prerequisites:
     CAN_P1_DRIVER = 1
     CAN_D1_PROTOCOL = 10
     CAN_D1_BITRATE = 500000
     Reboot required after parameter changes

   Inter-Script Interface:
   Outputs (mm_dti_ prefix, updated each cycle):
     mm_dti_command_rpm_fwd, mm_dti_command_rpm_aft     Commanded rotor RPM (100 Hz when armed)
     mm_dti_actual_rpm_fwd, mm_dti_actual_rpm_aft       Actual rotor RPM from telemetry (50 Hz)
     mm_dti_voltage_fwd, mm_dti_voltage_aft             DC link voltage in volts
     mm_dti_fault_fwd, mm_dti_fault_aft                 DTI fault codes (0 = no fault)
     mm_dti_heartbeat_fwd, mm_dti_heartbeat_aft         Last telemetry timestamp in ms
     mm_dti_temp_motor_fwd, mm_dti_temp_motor_aft       Motor temperature in Celsius
     mm_dti_temp_ctrl_fwd, mm_dti_temp_ctrl_aft         Controller temperature in Celsius
     mm_dti_current_dc_fwd, mm_dti_current_dc_aft       DC bus current in Amps
     mm_dti_current_ac_fwd, mm_dti_current_ac_aft       AC phase current RMS in Amps
     mm_dti_temp_sensor_fault_fwd, mm_dti_temp_sensor_fault_aft   Sensor fault flags
     mm_dti_fault_active                                True if any DTI fault is active

   Encoder Outputs (mm_enc_ prefix, updated at 100 Hz):
     mm_enc_position_fwd, mm_enc_position_aft           Rotor position in degrees (0-360)
     mm_enc_phase_fwd, mm_enc_phase_aft                 Rotor phase within blade symmetry (0-120)
     mm_enc_phase_diff                                  Cross-rotor phase difference (-60 to +60)
     mm_enc_heartbeat_fwd, mm_enc_heartbeat_aft         Last encoder timestamp in ms
     mm_enc_valid_fwd, mm_enc_valid_aft                 Encoder data valid and fresh
     mm_enc_sync_error                                  Absolute phase sync error in degrees

   Inputs (read from other scripts):
     mm_hv_command_enable      HV control flag enabling CAN command transmission
]]--

local common = require("minime_common")

local CAN_BUF_LEN = 10

local can_driver = nil

local init_attempts = 0
local MAX_INIT_ATTEMPTS = 5

local CAN_FLAG_EFF = uint32_t(1) << 31
local CMD_ID_FWD = CAN_FLAG_EFF | uint32_t((0x03 << 8) | common.DTI_FWD_NODE)
local CMD_ID_AFT = CAN_FLAG_EFF | uint32_t((0x03 << 8) | common.DTI_AFT_NODE)

local ESC_IDX_FWD = 0
local ESC_IDX_AFT = 1
local ESC_DATA_MASK = 0x0D

local esc_telem_fwd = ESCTelemetryData()
local esc_telem_aft = ESCTelemetryData()

local function parse_i16_le(b0, b1)
    local val = b0 + b1 * 256
    if val >= 32768 then
        val = val - 65536
    end
    return val
end

local function parse_i32_le(b0, b1, b2, b3)
    local val = b0 + b1 * 256 + b2 * 65536 + b3 * 16777216
    if val >= 2147483648 then
        val = val - 4294967296
    end
    return val
end

local function parse_u16_le(b0, b1)
    return b0 + b1 * 256
end

local function parse_u32_le(b0, b1, b2, b3)
    return b0 + b1 * 256 + b2 * 65536 + b3 * 16777216
end

local TEMP_SENSOR_MIN = -40
local TEMP_SENSOR_MAX = 150

local telem_fwd = {
    erpm = 0,
    duty = 0,
    voltage = 0,
    current_ac = 0,
    current_dc = 0,
    temp_controller = 0,
    temp_motor = 0,
    fault_code = 0,
    prev_fault_code = 0,
    id = 0,
    iq = 0,
    last_rx_ms = 0,
    temp_sensor_fault = false,
    enc_position_raw = 0,
    enc_position_deg = 0,
    enc_last_rx_ms = 0,
    enc_valid = false
}

local telem_aft = {
    erpm = 0,
    duty = 0,
    voltage = 0,
    current_ac = 0,
    current_dc = 0,
    temp_controller = 0,
    temp_motor = 0,
    fault_code = 0,
    prev_fault_code = 0,
    id = 0,
    iq = 0,
    last_rx_ms = 0,
    temp_sensor_fault = false,
    enc_position_raw = 0,
    enc_position_deg = 0,
    enc_last_rx_ms = 0,
    enc_valid = false
}

local cmd_enabled = false
local last_cmd_ms = 0

local K_HELIRSC = 31

local function parse_packet_0x00(frame, telem)
    telem.erpm = parse_i32_le(frame:data(0), frame:data(1), frame:data(2), frame:data(3))
    telem.duty = parse_i16_le(frame:data(4), frame:data(5)) / 1000.0
    telem.voltage = parse_u16_le(frame:data(6), frame:data(7))
end

local function parse_packet_0x01(frame, telem)
    telem.current_ac = common.scale_current(parse_i16_le(frame:data(0), frame:data(1)))
    telem.current_dc = common.scale_current(parse_i16_le(frame:data(2), frame:data(3)))
end

local function detect_fault_change(telem, rotor_name)
    local current = telem.fault_code
    local previous = telem.prev_fault_code

    if current ~= previous then
        if previous == 0 and current ~= 0 then
            local fault_name = common.fault_name(current)
            gcs:send_text(common.MAV_SEVERITY.CRITICAL,
                string.format("DTI FAULT: %s %s (code 0x%02X)",
                    rotor_name, fault_name, current))
        elseif previous ~= 0 and current == 0 then
            gcs:send_text(common.MAV_SEVERITY.INFO,
                string.format("DTI %s: Fault cleared", rotor_name))
        end
        telem.prev_fault_code = current
    end
end

local function parse_packet_0x02(frame, telem, rotor_name)
    local temp_ctrl = common.scale_temperature(parse_i16_le(frame:data(0), frame:data(1)))
    local temp_motor = common.scale_temperature(parse_i16_le(frame:data(2), frame:data(3)))
    telem.fault_code = frame:data(4)

    detect_fault_change(telem, rotor_name)

    local sensor_fault = false
    if temp_ctrl < TEMP_SENSOR_MIN or temp_ctrl > TEMP_SENSOR_MAX then
        sensor_fault = true
    else
        telem.temp_controller = temp_ctrl
    end

    if temp_motor < TEMP_SENSOR_MIN or temp_motor > TEMP_SENSOR_MAX then
        sensor_fault = true
    else
        telem.temp_motor = temp_motor
    end

    if sensor_fault and not telem.temp_sensor_fault then
        gcs:send_text(common.MAV_SEVERITY.WARNING,
            string.format("DTI %s: Temp sensor out of range", rotor_name))
    end
    telem.temp_sensor_fault = sensor_fault
end

local function parse_packet_0x03(frame, telem)
    telem.id = parse_i16_le(frame:data(0), frame:data(1)) / 100.0
    telem.iq = parse_i16_le(frame:data(2), frame:data(3)) / 100.0
end

local function parse_packet_enc(frame, telem)
    local raw_position = parse_u32_le(frame:data(0), frame:data(1),
                                       frame:data(2), frame:data(3))
    telem.enc_position_raw = raw_position

    if raw_position < common.ENC_COUNTS_PER_REV then
        telem.enc_position_deg = common.encoder_counts_to_deg(raw_position)
    else
        telem.enc_position_deg = raw_position / 1000.0
    end

    telem.enc_last_rx_ms = millis():toint()
    telem.enc_valid = true
end

local function parse_telemetry_frame(frame)
    local id = frame:id()
    local node_id = id % 256
    local packet_id = math.floor(id / 256) % 256

    local telem = nil
    local rotor_name = nil
    if node_id == common.DTI_FWD_NODE then
        telem = telem_fwd
        rotor_name = "FWD"
    elseif node_id == common.DTI_AFT_NODE then
        telem = telem_aft
        rotor_name = "AFT"
    else
        return
    end

    if packet_id == 0x00 then
        parse_packet_0x00(frame, telem)
    elseif packet_id == 0x01 then
        parse_packet_0x01(frame, telem)
    elseif packet_id == 0x02 then
        parse_packet_0x02(frame, telem, rotor_name)
    elseif packet_id == 0x03 then
        parse_packet_0x03(frame, telem)
    elseif packet_id == common.ENC_PACKET_ID then
        parse_packet_enc(frame, telem)
    end

    telem.last_rx_ms = millis():toint()
end

local function pack_erpm_le(erpm)
    local b0 = erpm % 256
    local b1 = math.floor(erpm / 256) % 256
    local b2 = math.floor(erpm / 65536) % 256
    local b3 = math.floor(erpm / 16777216) % 256
    return b0, b1, b2, b3
end

local function send_erpm_command(can_id, erpm)
    local msg = CANFrame()
    msg:id(can_id)
    local b0, b1, b2, b3 = pack_erpm_le(erpm)
    msg:data(0, b0)
    msg:data(1, b1)
    msg:data(2, b2)
    msg:data(3, b3)
    msg:dlc(4)
    can_driver:write_frame(msg, 10000)
end

local function update_esc_telemetry()
    esc_telem_fwd:voltage(telem_fwd.voltage)
    esc_telem_fwd:current(telem_fwd.current_dc)
    esc_telem_fwd:temperature_cdeg(math.floor(telem_fwd.temp_motor * 100))
    local rpm_fwd = common.rpm_from_erpm(telem_fwd.erpm)
    esc_telem:update_rpm(ESC_IDX_FWD, rpm_fwd, 0)
    esc_telem:update_telem_data(ESC_IDX_FWD, esc_telem_fwd, ESC_DATA_MASK)

    esc_telem_aft:voltage(telem_aft.voltage)
    esc_telem_aft:current(telem_aft.current_dc)
    esc_telem_aft:temperature_cdeg(math.floor(telem_aft.temp_motor * 100))
    local rpm_aft = common.rpm_from_erpm(telem_aft.erpm)
    esc_telem:update_rpm(ESC_IDX_AFT, rpm_aft, 0)
    esc_telem:update_telem_data(ESC_IDX_AFT, esc_telem_aft, ESC_DATA_MASK)
end

mm_dti_command_rpm_fwd = 0
mm_dti_command_rpm_aft = 0
mm_dti_actual_rpm_fwd = 0
mm_dti_actual_rpm_aft = 0
mm_dti_voltage_fwd = 0
mm_dti_voltage_aft = 0
mm_dti_fault_fwd = 0
mm_dti_fault_aft = 0
mm_dti_heartbeat_fwd = 0
mm_dti_heartbeat_aft = 0
mm_dti_temp_motor_fwd = 0
mm_dti_temp_motor_aft = 0
mm_dti_temp_ctrl_fwd = 0
mm_dti_temp_ctrl_aft = 0
mm_dti_current_dc_fwd = 0
mm_dti_current_dc_aft = 0
mm_dti_current_ac_fwd = 0
mm_dti_current_ac_aft = 0
mm_dti_temp_sensor_fault_fwd = false
mm_dti_temp_sensor_fault_aft = false
mm_dti_fault_active = false

mm_enc_position_fwd = 0
mm_enc_position_aft = 0
mm_enc_phase_fwd = 0
mm_enc_phase_aft = 0
mm_enc_phase_diff = 0
mm_enc_heartbeat_fwd = 0
mm_enc_heartbeat_aft = 0
mm_enc_valid_fwd = false
mm_enc_valid_aft = false
mm_enc_sync_error = 0

local update

function update()
    local frame = can_driver:read_frame()
    while frame do
        parse_telemetry_frame(frame)
        frame = can_driver:read_frame()
    end

    mm_dti_actual_rpm_fwd = common.rpm_from_erpm(telem_fwd.erpm)
    mm_dti_actual_rpm_aft = common.rpm_from_erpm(telem_aft.erpm)
    mm_dti_voltage_fwd = telem_fwd.voltage
    mm_dti_voltage_aft = telem_aft.voltage
    mm_dti_fault_fwd = telem_fwd.fault_code
    mm_dti_fault_aft = telem_aft.fault_code
    mm_dti_heartbeat_fwd = telem_fwd.last_rx_ms
    mm_dti_heartbeat_aft = telem_aft.last_rx_ms
    mm_dti_temp_motor_fwd = telem_fwd.temp_motor
    mm_dti_temp_motor_aft = telem_aft.temp_motor
    mm_dti_temp_ctrl_fwd = telem_fwd.temp_controller
    mm_dti_temp_ctrl_aft = telem_aft.temp_controller
    mm_dti_current_dc_fwd = telem_fwd.current_dc
    mm_dti_current_dc_aft = telem_aft.current_dc
    mm_dti_current_ac_fwd = telem_fwd.current_ac
    mm_dti_current_ac_aft = telem_aft.current_ac
    mm_dti_temp_sensor_fault_fwd = telem_fwd.temp_sensor_fault
    mm_dti_temp_sensor_fault_aft = telem_aft.temp_sensor_fault
    mm_dti_fault_active = (telem_fwd.fault_code ~= 0) or (telem_aft.fault_code ~= 0)

    local now_ms = millis():toint()

    local enc_fwd_fresh = (now_ms - telem_fwd.enc_last_rx_ms) < common.ENC_TIMEOUT_MS
    mm_enc_valid_fwd = telem_fwd.enc_valid and enc_fwd_fresh
    if mm_enc_valid_fwd then
        mm_enc_position_fwd = telem_fwd.enc_position_deg
        mm_enc_phase_fwd = common.calculate_blade_phase(telem_fwd.enc_position_deg)
        mm_enc_heartbeat_fwd = telem_fwd.enc_last_rx_ms
    end

    local enc_aft_fresh = (now_ms - telem_aft.enc_last_rx_ms) < common.ENC_TIMEOUT_MS
    mm_enc_valid_aft = telem_aft.enc_valid and enc_aft_fresh
    if mm_enc_valid_aft then
        mm_enc_position_aft = telem_aft.enc_position_deg
        mm_enc_phase_aft = common.calculate_blade_phase(telem_aft.enc_position_deg)
        mm_enc_heartbeat_aft = telem_aft.enc_last_rx_ms
    end

    if mm_enc_valid_fwd and mm_enc_valid_aft then
        mm_enc_phase_diff = common.calculate_phase_diff(mm_enc_phase_fwd, mm_enc_phase_aft)
        mm_enc_sync_error = math.abs(mm_enc_phase_diff)
    end

    update_esc_telemetry()

    local hv_cmd_enabled = mm_hv_command_enable or false
    if arming:is_armed() and hv_cmd_enabled then
        local rsc_output = SRV_Channels:get_output_scaled(K_HELIRSC)
        if rsc_output then
            local erpm = math.floor((rsc_output / 1000.0) * common.ERPM_HOVER + 0.5)
            if erpm < 0 then
                erpm = 0
            end

            send_erpm_command(CMD_ID_FWD, erpm)
            send_erpm_command(CMD_ID_AFT, erpm)

            local rotor_rpm = common.rpm_from_erpm(erpm)
            mm_dti_command_rpm_fwd = rotor_rpm
            mm_dti_command_rpm_aft = rotor_rpm

            last_cmd_ms = millis():toint()
        end
    else
        mm_dti_command_rpm_fwd = 0
        mm_dti_command_rpm_aft = 0
    end

    return update, common.DTI_COMMAND_RATE_MS
end

local function init()
    init_attempts = init_attempts + 1

    can_driver = CAN:get_device(CAN_BUF_LEN)

    if can_driver then
        gcs:send_text(common.MAV_SEVERITY.INFO, "DTI: CAN initialized")
        return update, common.DTI_COMMAND_RATE_MS
    end

    if init_attempts >= MAX_INIT_ATTEMPTS then
        gcs:send_text(common.MAV_SEVERITY.ERROR,
            "DTI: CAN init failed, check CAN_D1_PROTOCOL=10")
        return
    end

    gcs:send_text(common.MAV_SEVERITY.WARNING,
        string.format("DTI: CAN init retry %d/%d", init_attempts, MAX_INIT_ATTEMPTS))
    return init, 1000
end

return init()
