--[[
   MiniMe Common Module
   Shared constants and conversion functions for AS-10 MiniMe ArduPilot scripts

   This module is required by all other MiniMe scripts:
   local common = require("minime_common")
]]--

local M = {}

-- Vehicle constants
M.HOVER_RPM = 1131.7
M.MOTOR_RPM = 4526.8
M.ERPM_HOVER = 45268
M.GEARBOX_RATIO = 4
M.POLE_PAIRS = 10

-- CAN constants
M.DTI_FWD_NODE = 0x01
M.DTI_AFT_NODE = 0x02
M.DTI_BROADCAST = 0xFF
M.CAN_BITRATE = 500000

-- GPIO constants (AUX pin numbers)
M.GPIO_FWD_MAIN = 50
M.GPIO_FWD_PRECHARGE = 51
M.GPIO_AFT_MAIN = 52
M.GPIO_AFT_PRECHARGE = 53
M.GPIO_FWD_STO = 54
M.GPIO_AFT_STO = 55

-- HVIL ADC configuration
M.HVIL_ADC_PIN = 8               -- Cube Orange+ ADC port input (per hwdef)
M.HVIL_POLL_RATE_MS = 20         -- 50 Hz polling
M.HVIL_VOLTAGE_HEALTHY = 0.9     -- Loop closed voltage (V)
M.HVIL_VOLTAGE_BROKEN = 5.0      -- Loop open voltage (V)
M.HVIL_THRESHOLD = 2.5           -- Detection threshold (V)
M.HVIL_RESPONSE_MS = 100         -- Max response time requirement (ms)

-- HV state machine states
M.STATE_DE_ENERGIZED = 0
M.STATE_PRECHARGING = 1
M.STATE_ENERGIZED = 2
M.STATE_ARMED = 3
M.STATE_FAULTED = 4

-- Timing constants (milliseconds)
M.DTI_COMMAND_RATE_MS = 10
M.DTI_TELEMETRY_RATE_MS = 20
M.DTI_TIMEOUT_MS = 500
M.PRECHARGE_MIN_TIME_MS = 1000
M.PRECHARGE_TIMEOUT_MS = 10000
M.PRECHARGE_THRESHOLD = 0.90

-- Desync alert states
M.DESYNC_NORMAL = 0
M.DESYNC_ALERT = 1
M.DESYNC_WARNING = 2
M.DESYNC_CRITICAL = 3

-- Desync thresholds (percentage of hover RPM)
M.DESYNC_ALERT_PCT = 0.25
M.DESYNC_WARNING_PCT = 0.50
M.DESYNC_HARD_PCT = 1.00

-- Desync thresholds (absolute RPM)
M.DESYNC_ALERT_RPM = 2.8
M.DESYNC_WARNING_RPM = 5.7
M.DESYNC_HARD_RPM = 11.3

-- Spin up suppression time after arming (milliseconds)
M.SPINUP_SUPPRESS_MS = 5000

-- Desync hysteresis for downward state transitions (milliseconds)
M.DESYNC_HYSTERESIS_MS = 2000

-- E-stop RC channel configuration
M.RC_OPTION_MOTOR_ESTOP = 31       -- RC_OPTION value for Motor Emergency Stop
M.ESTOP_PWM_THRESHOLD = 1700       -- PWM value above which E-stop is considered active

-- Flight mode numbers for mode transitions
M.MODE_STABILIZE = 0
M.MODE_ALT_HOLD = 2
M.MODE_RTL = 6
M.MODE_LAND = 9

-- Landing detection constants
M.LANDING_RPM_THRESHOLD = 100           -- Rotors below this RPM considered stopped
M.LANDING_DETECT_DURATION_MS = 2000     -- Duration to confirm landing complete (2 seconds)
M.LANDING_FLYING_CHECK_ENABLED = true   -- Use vehicle:get_likely_flying() for landing detection

-- Coolant temperature sensor indices (DroneCAN via Beyond Robotix node)
M.COOLANT_MOTOR_SENSOR_INDEX = 0
M.COOLANT_INVERTER_SENSOR_INDEX = 1

-- Redline monitoring constants
M.REDLINE_HYSTERESIS_PCT = 0.05
M.OVERSPEED_THRESHOLD_PCT = 0.05
M.OVERSPEED_DURATION_MS = 500
M.REDLINE_ALERT_RATE_LIMIT_MS = 5000

-- Single rotor failure detection constants
M.SINGLE_ROTOR_RPM_DROP_PCT = 0.50          -- 50% RPM drop threshold
M.SINGLE_ROTOR_RPM_MAINTAIN_PCT = 0.10      -- Other rotor must be within 10% of commanded
M.SINGLE_ROTOR_DETECT_DURATION_MS = 100     -- 100ms sustained detection for RPM loss
M.SINGLE_ROTOR_CURRENT_NEAR_ZERO = 5.0      -- Current below 5A considered near zero
M.SINGLE_ROTOR_CURRENT_NORMAL = 20.0        -- Current above 20A considered normal load
M.SINGLE_ROTOR_HEARTBEAT_TIMEOUT_MS = 200   -- Heartbeat loss threshold for one rotor

-- Encoder constants (RLS RM44SI via DTI HV550)
M.ENC_PACKET_ID = 0x04                -- DTI packet ID for encoder data (verify during bench test)
M.ENC_COUNTS_PER_REV = 16384          -- RLS RM44SI 14-bit resolution
M.ENC_DEGREES_PER_COUNT = 360.0 / 16384  -- 0.02197 degrees per count
M.ENC_BLADES_PER_ROTOR = 3            -- 3-blade rotor symmetry
M.ENC_BLADE_SPACING_DEG = 120.0       -- 360 / 3 blades
M.ENC_TIMEOUT_MS = 100                -- Encoder data staleness threshold

-- Phase synchronization constants for test modes
M.PHASE_SYNC_TOLERANCE_DEG = 1.0      -- Phase matching tolerance for synchronized mode
M.PHASE_OFFSET_IN_PHASE = 0.0         -- 0 degree offset (blades aligned)
M.PHASE_OFFSET_OUT_PHASE = 60.0       -- 60 degree offset (maximized out of phase)

-- Redline thresholds: motor winding temperature (Celsius)
M.REDLINE_MOTOR_WINDING = {
    caution = 80,
    warning = 95,
    hard = 100
}

-- Redline thresholds: motor part temperature (Celsius)
M.REDLINE_MOTOR_PART = {
    caution = 95,
    warning = 110,
    hard = 120
}

-- Redline thresholds: coolant inlet motor loop (Celsius)
M.REDLINE_COOLANT_MOTOR = {
    caution = 40,
    warning = 45,
    hard = 50
}

-- Redline thresholds: coolant inlet inverter loop (Celsius)
M.REDLINE_COOLANT_INVERTER = {
    caution = 45,
    warning = 55,
    hard = 60
}

-- Redline thresholds: phase current RMS (Amps)
M.REDLINE_PHASE_CURRENT = {
    caution = 200,
    warning = 300,
    hard = 380
}

-- Redline thresholds: inverter DC link voltage low (Volts)
M.REDLINE_DC_LINK_LOW = {
    caution = 100,
    warning = 50,
    hard = 30
}

-- Redline thresholds: inverter DC link voltage high (Volts)
M.REDLINE_DC_LINK_HIGH = {
    caution = 700,
    warning = 800,
    hard = 830
}

-- Redline thresholds: battery DC bus voltage low (Volts)
M.REDLINE_BATTERY_LOW = {
    caution = 130,
    warning = 128,
    hard = 120
}

-- Redline thresholds: battery DC bus voltage high (Volts)
M.REDLINE_BATTERY_HIGH = {
    caution = 154,
    warning = 156,
    hard = 165
}

-- Redline thresholds: motor RPM (mechanical)
M.REDLINE_MOTOR_RPM = {
    caution = 6000,
    warning = 6300,
    hard = 6500
}

-- Redline thresholds: rotor RPM
M.REDLINE_ROTOR_RPM = {
    caution = 1004,
    warning = 1050,
    hard = 1100
}

-- Redline thresholds: battery pack temperature (Celsius)
M.REDLINE_PACK_TEMP = {
    caution = 50,
    warning = 60,
    hard = 70
}

-- Redline thresholds: state of charge (percent)
M.REDLINE_SOC = {
    caution = 30,
    warning = 25,
    hard = 20
}

-- DTI fault codes
M.DTI_FAULT = {
    [0x00] = "No fault",
    [0x01] = "Over Voltage",
    [0x02] = "Under Voltage",
    [0x03] = "DRV Error",
    [0x04] = "ABS Over Current",
    [0x05] = "Controller Over Temperature",
    [0x06] = "Motor Over Temperature",
    [0x07] = "Sensor Wire Fault",
    [0x08] = "Sensor General Fault"
}

-- MAVLink severity levels
M.MAV_SEVERITY = {
    EMERGENCY = 0,
    ALERT = 1,
    CRITICAL = 2,
    ERROR = 3,
    WARNING = 4,
    NOTICE = 5,
    INFO = 6,
    DEBUG = 7
}

--[[
   Convert electrical RPM to rotor RPM
   @param erpm Electrical RPM from DTI controller
   @return Rotor RPM
]]--
function M.rpm_from_erpm(erpm)
    return erpm / M.POLE_PAIRS / M.GEARBOX_RATIO
end

--[[
   Convert rotor RPM to electrical RPM
   @param rotor_rpm Rotor RPM
   @return Electrical RPM for DTI command (integer)
]]--
function M.erpm_from_rpm(rotor_rpm)
    return math.floor(rotor_rpm * M.GEARBOX_RATIO * M.POLE_PAIRS + 0.5)
end

--[[
   Scale raw temperature value from DTI telemetry
   @param raw Raw temperature value (scaled by 10)
   @return Temperature in Celsius
]]--
function M.scale_temperature(raw)
    return raw / 10.0
end

--[[
   Scale raw current value from DTI telemetry
   @param raw Raw current value (scaled by 10)
   @return Current in Amps
]]--
function M.scale_current(raw)
    return raw / 10.0
end

--[[
   Scale raw voltage value from DTI telemetry
   @param raw Raw voltage value
   @return Voltage in Volts (unchanged)
]]--
function M.scale_voltage(raw)
    return raw
end

--[[
   Get fault name from fault code
   @param code DTI fault code (0x00 to 0x08)
   @return Fault name string
]]--
function M.fault_name(code)
    return M.DTI_FAULT[code] or string.format("Unknown (0x%02X)", code)
end

--[[
   Check if a value exceeds a redline threshold
   @param value Current value
   @param redline Redline table with caution/warning/hard fields
   @param invert If true, trigger when value is below threshold (for low voltage)
   @return "hard", "warning", "caution", or nil
]]--
function M.check_redline(value, redline, invert)
    if invert then
        if value <= redline.hard then
            return "hard"
        elseif value <= redline.warning then
            return "warning"
        elseif value <= redline.caution then
            return "caution"
        end
    else
        if value >= redline.hard then
            return "hard"
        elseif value >= redline.warning then
            return "warning"
        elseif value >= redline.caution then
            return "caution"
        end
    end
    return nil
end

--[[
   Calculate rotor phase from encoder position using 3-blade symmetry
   @param position_deg Encoder position in degrees (0-360)
   @return Phase within blade symmetry (0-120 degrees)
]]--
function M.calculate_blade_phase(position_deg)
    local normalized = position_deg % 360.0
    return normalized % M.ENC_BLADE_SPACING_DEG
end

--[[
   Calculate cross-rotor phase difference
   @param phase_fwd Forward rotor phase (0-120)
   @param phase_aft Aft rotor phase (0-120)
   @return Phase difference in degrees (-60 to +60)
]]--
function M.calculate_phase_diff(phase_fwd, phase_aft)
    local diff = phase_fwd - phase_aft
    if diff > 60.0 then
        diff = diff - 120.0
    elseif diff < -60.0 then
        diff = diff + 120.0
    end
    return diff
end

--[[
   Convert raw encoder counts to degrees
   @param counts Raw encoder counts (0 to ENC_COUNTS_PER_REV-1)
   @return Position in degrees (0-360)
]]--
function M.encoder_counts_to_deg(counts)
    return counts * M.ENC_DEGREES_PER_COUNT
end

return M

--[[
   Inter-Script Communication Contract
   Reference: TODO.md Section 2.1a

   All MiniMe scripts share data via global variables with prefixed namespaces.
   Scripts may load in any order. Dependent scripts should check for nil values
   before accessing shared globals. Data older than 500 ms should be considered stale.

   DTI Driver Outputs (mm_dti_ prefix, updated at 100 Hz command / 50 Hz telemetry):
     mm_dti_command_rpm_fwd    number   Last commanded rotor RPM for forward rotor
     mm_dti_command_rpm_aft    number   Last commanded rotor RPM for aft rotor
     mm_dti_actual_rpm_fwd     number   Actual rotor RPM from DTI telemetry, forward
     mm_dti_actual_rpm_aft     number   Actual rotor RPM from DTI telemetry, aft
     mm_dti_voltage_fwd        number   DC link voltage in volts, forward
     mm_dti_voltage_aft        number   DC link voltage in volts, aft
     mm_dti_fault_fwd          integer  DTI fault code, forward (0 = no fault)
     mm_dti_fault_aft          integer  DTI fault code, aft (0 = no fault)
     mm_dti_heartbeat_fwd      integer  Last telemetry timestamp in ms, forward
     mm_dti_heartbeat_aft      integer  Last telemetry timestamp in ms, aft
     mm_dti_temp_motor_fwd     number   Motor temperature in Celsius, forward
     mm_dti_temp_motor_aft     number   Motor temperature in Celsius, aft
     mm_dti_temp_ctrl_fwd      number   Controller temperature in Celsius, forward
     mm_dti_temp_ctrl_aft      number   Controller temperature in Celsius, aft
     mm_dti_current_dc_fwd     number   DC bus current in Amps, forward
     mm_dti_current_dc_aft     number   DC bus current in Amps, aft
     mm_dti_current_ac_fwd     number   AC phase current RMS in Amps, forward
     mm_dti_current_ac_aft     number   AC phase current RMS in Amps, aft
     mm_dti_temp_sensor_fault_fwd  boolean  Temperature sensor fault flag, forward
     mm_dti_temp_sensor_fault_aft  boolean  Temperature sensor fault flag, aft
     mm_dti_fault_active       boolean  True if any DTI fault is active

   HV Control Outputs (mm_hv_ prefix, updated at 50 Hz):
     mm_hv_state               integer  Current HV state (0=de_energized through 4=faulted)
     mm_hv_state_name          string   Human readable state name
     mm_hv_command_enable      boolean  CAN commands enabled flag (DTI driver reads this)
     mm_hv_fault_reason        string   Last fault reason or empty string
     mm_hv_last_state_change_ms integer Timestamp of last state transition
     mm_hv_gpio_initialized    boolean  GPIO initialization status
     mm_hv_hvil_healthy        boolean  HVIL loop healthy status
     mm_hv_hvil_voltage        number   HVIL ADC voltage reading
     mm_hv_cmd_energize        boolean  Command input: request energize
     mm_hv_cmd_deenergize      boolean  Command input: request de-energize
     mm_hv_cmd_reset           boolean  Command input: request fault reset
     mm_hv_redline_active      boolean  Any redline condition active
     mm_hv_redline_level       string   Worst redline level (normal/caution/warning/hard)
     mm_hv_redline_param       string   Parameter causing worst redline
     mm_hv_derate_pct          integer  Power derate percentage (100 = full power)
     mm_hv_saturate_rpm        boolean  RPM command saturation active
     mm_hv_single_rotor_fault  boolean  Single rotor failure detected
     mm_hv_estop_active        boolean  E-stop switch is active
     mm_hv_coolant_motor_temp  number   Motor coolant inlet temperature or nil
     mm_hv_coolant_motor_sensor_ok boolean Coolant temperature sensor status
     mm_hv_failsafe_active     boolean  Failsafe mode active (RTL/Land from failsafe trigger)
     mm_hv_failsafe_reason     string   Failsafe reason or empty string
     mm_hv_landing_in_progress boolean  Landing descent in progress
     mm_hv_autorotation_active boolean  Autorotation mode active (CAN commands disabled)

   Telemetry Outputs (mm_tel_ prefix, updated at 50 Hz):
     mm_tel_desync_state       integer  Desync state (0=normal, 1=alert, 2=warning, 3=critical)
     mm_tel_rpm_diff           number   Current RPM differential (absolute value)
     mm_tel_delta_rpm          number   Alias for mm_tel_rpm_diff
     mm_tel_delta_pct          number   RPM differential as percentage of hover RPM
     mm_tel_delta_rpm_avg      number   Rolling average RPM differential (200 ms window)
     mm_tel_peak_delta_rpm     number   Peak RPM differential since arm
     mm_tel_peak_delta_pct     number   Peak percentage since arm
     mm_tel_calc_timestamp     integer  Timestamp of last calculation in ms
     mm_tel_data_valid         boolean  True if telemetry data is fresh and valid
     mm_tel_spinup_suppressed  boolean  True during 5 second spinup suppression window
     mm_tel_rpm_fwd_filtered   number   Low pass filtered forward RPM
     mm_tel_rpm_aft_filtered   number   Low pass filtered aft RPM

   Telemetry to Test Modes Interface:
     Test modes (minime_test_modes.lua) reads the following for synchronization:
       mm_tel_rpm_fwd_filtered, mm_tel_rpm_aft_filtered (current filtered RPM)
       mm_tel_desync_state (desync alert level)
       mm_dti_actual_rpm_fwd, mm_dti_actual_rpm_aft (raw RPM values)
       mm_hv_state (current HV state for mode lockout checks)

   Encoder Interface (mm_enc_ prefix, updated at 100 Hz):
     mm_enc_position_fwd    number   Forward rotor position in degrees (0-360)
     mm_enc_position_aft    number   Aft rotor position in degrees (0-360)
     mm_enc_phase_fwd       number   Forward rotor phase (0-120, blade symmetry)
     mm_enc_phase_aft       number   Aft rotor phase (0-120, blade symmetry)
     mm_enc_phase_diff      number   Cross-rotor phase difference (-60 to +60)
     mm_enc_heartbeat_fwd   integer  Forward encoder last update timestamp in ms
     mm_enc_heartbeat_aft   integer  Aft encoder last update timestamp in ms
     mm_enc_valid_fwd       boolean  Forward encoder data valid and fresh
     mm_enc_valid_aft       boolean  Aft encoder data valid and fresh
     mm_enc_sync_error      number   Absolute phase synchronization error in degrees
]]--
