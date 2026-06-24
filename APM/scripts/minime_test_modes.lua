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

   Inter-Script Interface:
   Outputs (mm_test_ prefix):
     mm_test_lockout_active     boolean  True if any lockout condition prevents test mode
     mm_test_lockout_reason     string   Human readable lockout reason or empty
     mm_test_rsc_mode_ok        boolean  True if H_RSC_MODE = 3
     mm_test_flight_mode_ok     boolean  True if flight mode allows test modes
     mm_test_altitude_ok        boolean  True if altitude < 1m AGL

   Inputs (read from other scripts):
     mm_hv_state                HV state for mode lockout checks
     mm_enc_valid_fwd           Encoder data validity for synchronized mode
     mm_enc_valid_aft           Encoder data validity for synchronized mode
]]--

local common = require("minime_common")

local SCRIPT_NAME = "TestModes"
local UPDATE_RATE_MS = 100

local last_lockout_alert_ms = 0
local initialized = false

mm_test_lockout_active = true
mm_test_lockout_reason = ""
mm_test_rsc_mode_ok = false
mm_test_flight_mode_ok = false
mm_test_altitude_ok = false

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
   Check all lockout conditions for test mode entry
   @return true if all conditions pass, false with reason if any fail
]]--
local function check_all_lockouts()
    local ok, reason

    ok, reason = check_rsc_mode()
    if not ok then
        mm_test_lockout_active = true
        mm_test_lockout_reason = reason
        return false
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
   Initialize test modes script
]]--
local function init()
    gcs:send_text(common.MAV_SEVERITY.INFO,
        string.format("%s: Initializing", SCRIPT_NAME))

    local ok, reason = check_rsc_mode()
    if ok then
        gcs:send_text(common.MAV_SEVERITY.INFO,
            string.format("%s: H_RSC_MODE = 3 (throttle curve) verified", SCRIPT_NAME))
    else
        gcs:send_text(common.MAV_SEVERITY.WARNING,
            string.format("%s: %s", SCRIPT_NAME, reason))
    end

    initialized = true
    return true
end

--[[
   Main update loop
   Checks lockout conditions and manages test mode state
]]--
local function update()
    if not initialized then
        if not init() then
            return update, 1000
        end
    end

    check_all_lockouts()

    if mm_test_lockout_active then
        send_lockout_alert()
    end

    return update, UPDATE_RATE_MS
end

return update()
