--[[
   MiniMe HV Control
   High voltage state machine for AS-10 MiniMe dual rotor helicopter

   Prerequisites:
     SERVO9_FUNCTION = -1  (GPIO 50, FWD Main)
     SERVO10_FUNCTION = -1 (GPIO 51, FWD Precharge)
     SERVO11_FUNCTION = -1 (GPIO 52, AFT Main)
     SERVO12_FUNCTION = -1 (GPIO 53, AFT Precharge)
     Reboot required after SERVO_FUNCTION changes
]]--

local common = require("minime_common")

local HV_UPDATE_RATE_MS = 100
local MAX_INIT_ATTEMPTS = 5
local GPIO_OUTPUT = 1
local GPIO_LOW = 0
local GPIO_HIGH = 1

local init_attempts = 0
local hv_state = common.STATE_DE_ENERGIZED
local state_entry_ms = 0

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

local function init_gpio()
    for _, pin in ipairs(GPIO_PINS) do
        gpio:pinMode(pin, GPIO_OUTPUT)
        gpio:write(pin, GPIO_LOW)
    end
    return true
end

local update

function update()
    mm_hv_state = hv_state
    mm_hv_state_name = state_name(hv_state)

    return update, HV_UPDATE_RATE_MS
end

local function init()
    init_attempts = init_attempts + 1

    local gpio_ok = init_gpio()

    if gpio_ok then
        hv_state = common.STATE_DE_ENERGIZED
        state_entry_ms = millis():toint()

        mm_hv_state = hv_state
        mm_hv_state_name = state_name(hv_state)
        mm_hv_last_state_change_ms = state_entry_ms
        mm_hv_gpio_initialized = true

        gcs:send_text(common.MAV_SEVERITY.INFO,
            "HV: GPIO initialized, state DE_ENERGIZED")

        return update, HV_UPDATE_RATE_MS
    end

    if init_attempts >= MAX_INIT_ATTEMPTS then
        gcs:send_text(common.MAV_SEVERITY.ERROR,
            "HV: GPIO init failed, check SERVO9-12_FUNCTION=-1")
        set_safe_state()
        mm_hv_gpio_initialized = false
        return nil
    end

    gcs:send_text(common.MAV_SEVERITY.WARNING,
        string.format("HV: GPIO init retry %d/%d",
            init_attempts, MAX_INIT_ATTEMPTS))

    return init, 1000
end

return init()
