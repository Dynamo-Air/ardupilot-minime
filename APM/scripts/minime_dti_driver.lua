--[[
   MiniMe DTI Driver
   CAN driver for DTI HV550 motor controllers

   Prerequisites:
     CAN_P1_DRIVER = 1
     CAN_D1_PROTOCOL = 10
     CAN_D1_BITRATE = 500000
     Reboot required after parameter changes
]]--

local common = require("minime_common")

local CAN_BUF_LEN = 10

local can_driver = nil

local init_attempts = 0
local MAX_INIT_ATTEMPTS = 5

local CAN_FLAG_EFF = uint32_t(1) << 31
local CMD_ID_FWD = CAN_FLAG_EFF | uint32_t((0x03 << 8) | common.DTI_FWD_NODE)
local CMD_ID_AFT = CAN_FLAG_EFF | uint32_t((0x03 << 8) | common.DTI_AFT_NODE)

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
    id = 0,
    iq = 0,
    last_rx_ms = 0,
    temp_sensor_fault = false
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
    id = 0,
    iq = 0,
    last_rx_ms = 0,
    temp_sensor_fault = false
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

local function parse_packet_0x02(frame, telem, rotor_name)
    local temp_ctrl = common.scale_temperature(parse_i16_le(frame:data(0), frame:data(1)))
    local temp_motor = common.scale_temperature(parse_i16_le(frame:data(2), frame:data(3)))
    telem.fault_code = frame:data(4)

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

    if arming:is_armed() then
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
