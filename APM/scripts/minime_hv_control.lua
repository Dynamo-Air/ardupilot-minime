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
local state_machine_counter = 0
local STATE_MACHINE_DIVISOR = 5

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

local function emergency_shutdown(reason)
    mm_hv_command_enable = false

    set_safe_state()

    local prev_state = hv_state
    hv_state = common.STATE_FAULTED
    state_entry_ms = millis():toint()

    mm_hv_state = hv_state
    mm_hv_state_name = state_name(hv_state)
    mm_hv_fault_reason = reason
    mm_hv_last_state_change_ms = state_entry_ms

    gcs:send_text(common.MAV_SEVERITY.CRITICAL,
        string.format("HV: EMERGENCY SHUTDOWN, %s to FAULTED (%s)",
            state_name(prev_state), reason))
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

local update

function update()
    if check_hvil_break() then
        return update, common.HVIL_POLL_RATE_MS
    end

    local _, hvil_status = is_hvil_healthy()

    state_machine_counter = state_machine_counter + 1
    if state_machine_counter >= STATE_MACHINE_DIVISOR then
        state_machine_counter = 0
    end

    mm_hv_state = hv_state
    mm_hv_state_name = state_name(hv_state)

    return update, common.HVIL_POLL_RATE_MS
end

local function init()
    init_attempts = init_attempts + 1

    local gpio_ok = init_gpio()
    local hvil_ok = init_hvil_adc()

    if gpio_ok and hvil_ok then
        hv_state = common.STATE_DE_ENERGIZED
        state_entry_ms = millis():toint()

        mm_hv_state = hv_state
        mm_hv_state_name = state_name(hv_state)
        mm_hv_last_state_change_ms = state_entry_ms
        mm_hv_gpio_initialized = true

        local healthy, status = is_hvil_healthy()
        local hvil_msg = healthy and "HVIL OK" or status

        gcs:send_text(common.MAV_SEVERITY.INFO,
            string.format("HV: Initialized, state DE_ENERGIZED, %s (%.2fV)",
                hvil_msg, mm_hv_hvil_voltage))

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
