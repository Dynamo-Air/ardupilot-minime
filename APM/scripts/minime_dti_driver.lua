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
    last_rx_ms = 0
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
    last_rx_ms = 0
}

local cmd_enabled = false
local last_cmd_ms = 0

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

local update

function update()
    local frame = can_driver:read_frame()
    while frame do
        frame = can_driver:read_frame()
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
