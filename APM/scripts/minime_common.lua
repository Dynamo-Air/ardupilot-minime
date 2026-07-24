--[[
   MiniMe Common Module
   Shared constants and conversion functions for AS-10 MiniMe ArduPilot scripts

   This module is required by all other MiniMe scripts:
   local common = require("minime_common")
]]--

local M = {}

-- Vehicle constants
M.HOVER_RPM = 1074.2
M.MOTOR_RPM = 4296.7
M.ERPM_HOVER = 42967
M.AUTOROT_RPM = 1020.5
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
M.DESYNC_ALERT_RPM = 2.69
M.DESYNC_WARNING_RPM = 5.37
M.DESYNC_HARD_RPM = 10.74

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

-- H_RSC_MODE constants for governor/throttle control
M.RSC_MODE_ESC_GOVERNOR = 2     -- Baseline flight operations
M.RSC_MODE_THROTTLE_CURVE = 3   -- Required for test modes (OBJ-MOD-4, OBJ-MOD-5)

-- Test mode lockout constants
M.TEST_MODE_MAX_ALTITUDE_M = 1.0      -- Test modes locked above 1m AGL
M.TEST_MODE_LOCKOUT_ALERT_MS = 5000   -- Rate limit for lockout alerts

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

-- Synchronized rotor mode states (OBJ-MOD-4)
M.SYNC_IDLE = 0                       -- Both rotors at matched RPM, waiting for command
M.SYNC_RAMP = 1                       -- Ramping to synchronized 100% RPM
M.SYNC_PHASE_0 = 2                    -- Synchronized at 0 degree offset
M.SYNC_PHASE_60 = 3                   -- Synchronized at 60 degree offset
M.SYNC_FAULT = 4                      -- Synchronization lost
M.DELTA_RPM = 5                       -- Delta RPM sweep mode (OBJ-MOD-5)

-- Synchronized mode RPM parameters
M.SYNC_TARGET_RPM = 1074.2            -- Target rotor RPM for synchronized mode
M.SYNC_RPM_TOLERANCE = 1.07           -- 0.1% of hover RPM matching tolerance
M.SYNC_RAMP_RATE_RPM_S = 100.0        -- RPM per second ramp rate
M.SYNC_SETTLE_TIME_MS = 2000          -- Time to stabilize at target RPM

-- Synchronized mode phase control parameters
M.SYNC_PHASE_K_P = 0.3                -- Proportional gain for phase adjustment (RPM per degree)
M.SYNC_PHASE_MAX_RPM_ADJ = 5.0        -- Max RPM adjustment for phase control
M.SYNC_PHASE_TOLERANCE_DEG = 2.0      -- Phase offset tolerance in degrees

-- Synchronized mode fault detection
M.SYNC_FAULT_RPM_DIFF_PCT = 0.02      -- 2% RPM differential triggers fault
M.SYNC_FAULT_RPM_DIFF_MS = 500        -- Duration before fault declared
M.SYNC_FAULT_PHASE_ERR_DEG = 10.0     -- Phase error threshold for fault
M.SYNC_FAULT_PHASE_ERR_MS = 1000      -- Phase error duration before fault

-- Gyroscopic coupling compensation gains
M.K_RP = 0.4730                       -- Roll to pitch coupling (1/s), feedforward required
M.K_PR = 0.0                          -- Pitch to roll (1/s), zero for counter rotating tandem
M.K_CY = 0.0968                       -- Collective to yaw coupling, maps to H_COLYAW = 0.10

-- Yaw authority reduction factor (STRUCTURAL CONSTRAINT, blade root SF protection)
M.ETA_YAW = 0.30                      -- NOT a tuning parameter

-- Cyclic saturation management
M.CYCLIC_ROLL_PRI = 0.70              -- Roll priority in cyclic budget (70%)
M.CYCLIC_SAT_WARN = 0.80              -- Warning threshold at 80% saturation
M.CYCLIC_SAT_LIM = 0.95               -- Limit at 95% saturation
M.YAW_COLLECTIVE_DERATE = true        -- Enable yaw derate at high collective
M.YAW_RATE_MAX_EFFECTIVE = 4.5        -- Effective yaw rate limit with eta_yaw (deg/s)

-- Delta RPM sweep mode parameters (OBJ-MOD-5)
-- Step definitions: percentage of hover RPM and absolute RPM values
M.DELTA_STEPS = {
    { pct = 0.25, rpm = 2.69 },       -- Step 1: +/- 0.25% (+/- 2.69 RPM)
    { pct = 0.50, rpm = 5.37 },       -- Step 2: +/- 0.5% (+/- 5.37 RPM)
    { pct = 1.00, rpm = 10.74 },      -- Step 3: +/- 1.0% (+/- 10.74 RPM)
    { pct = 2.00, rpm = 21.48 }       -- Step 4: +/- 2.0% (+/- 21.48 RPM)
}
M.DELTA_STEP_COUNT = 4                -- Number of delta steps
M.DELTA_DWELL_TIME_S = 30.0           -- Default dwell time per step in seconds
M.DELTA_RPM_LOG_RATE_MS = 20          -- 50 Hz RPM logging rate
M.DELTA_IMU_LOG_RATE_MS = 5           -- 200 Hz IMU logging rate

-- Delta sweep abort thresholds
M.DELTA_ABORT_ACCEL_G = 2.0           -- Abort if vibration exceeds 2g
M.DELTA_ABORT_DURATION_MS = 500       -- Duration before abort triggered
M.DELTA_ABORT_SETTLE_MS = 1000        -- Settle time after step change before monitoring

-- Modal resonance monitoring thresholds
-- Values TBD during Phase G ground vibration test
M.MODAL_ACCEL_CAUTION_G = 0.5         -- Caution threshold (placeholder)
M.MODAL_ACCEL_WARNING_G = 1.0         -- Warning threshold (placeholder)
M.MODAL_HYSTERESIS_MS = 2000          -- Hysteresis for state transitions
M.MODAL_ALERT_RATE_LIMIT_MS = 5000    -- GCS alert rate limiting

-- Modal alert states
M.MODAL_NORMAL = 0
M.MODAL_CAUTION = 1
M.MODAL_WARNING = 2

-- RPM avoidance band constants
M.AVOID_BAND_MAX_COUNT = 4            -- Maximum number of configurable bands
M.AVOID_BAND_CONFIG_PATH = "APM/config/rpm_bands.txt"  -- SD card configuration file
M.AVOID_BAND_CAUTION_PCT = 0.02       -- 2% caution zone from band edge
M.AVOID_BAND_LOG_RATE_MS = 100        -- Rate limit for band interaction logging

-- Avoidance band violation states
M.AVOID_NORMAL = 0                    -- RPM safely outside all bands
M.AVOID_CAUTION = 1                   -- RPM within 2% of band edge
M.AVOID_WARNING = 2                   -- RPM at band edge
M.AVOID_HARD = 3                      -- RPM inside band (requires action)

-- Avoidance band type constants
M.AVOID_TYPE_ABSOLUTE = 1             -- Band defines rotor RPM range to avoid
M.AVOID_TYPE_DELTA = 2                -- Band defines differential RPM range to avoid

-- Drift correction constants
-- When actual RPM drifts into an avoidance band, these control the gradual correction
M.DRIFT_CORRECTION_RATE_RPM_S = 20.0  -- Maximum correction rate (RPM per second)
M.DRIFT_CORRECTION_K_P = 0.5          -- Proportional gain (correction rate per RPM error)
M.DRIFT_CORRECTION_MIN_RPM = 0.5      -- Minimum correction increment threshold (RPM)
M.DRIFT_CORRECTION_MAX_RPM = 5.0      -- Maximum single cycle correction (RPM)
M.DRIFT_CORRECTION_LOG_RATE_MS = 200  -- Rate limit for drift correction logging (ms)
M.DRIFT_CORRECTION_ALERT_RATE_MS = 5000 -- Rate limit for GCS drift alerts (ms)

-- State name lookup for logging
M.SYNC_STATE_NAMES = {
    [0] = "SYNC_IDLE",
    [1] = "SYNC_RAMP",
    [2] = "SYNC_PHASE_0",
    [3] = "SYNC_PHASE_60",
    [4] = "SYNC_FAULT",
    [5] = "DELTA_RPM"
}

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
    caution = 1128,
    warning = 1128,
    hard = 1182
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

--[[
   Calculate beat frequency from RPM differential
   Beat frequency is the rate at which rotor blades pass relative positions
   @param rpm_fwd Forward rotor RPM
   @param rpm_aft Aft rotor RPM
   @return Beat frequency in Hz
]]--
function M.calculate_beat_freq(rpm_fwd, rpm_aft)
    local rpm_diff = math.abs(rpm_fwd - rpm_aft)
    return rpm_diff / 60.0
end

--[[
   Get delta step parameters by index
   @param step Step number (1 to DELTA_STEP_COUNT)
   @return Step table with pct and rpm fields, or nil if invalid
]]--
function M.get_delta_step(step)
    if step < 1 or step > M.DELTA_STEP_COUNT then
        return nil
    end
    return M.DELTA_STEPS[step]
end

return M

--[[
   Inter-Script Communication Contract

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

   Test Modes Outputs (mm_test_ prefix, updated at 100 Hz):
     mm_test_lockout_active     boolean  True if any lockout condition prevents test mode
     mm_test_lockout_reason     string   Human readable lockout reason or empty
     mm_test_rsc_mode_ok        boolean  True if H_RSC_MODE = 3
     mm_test_flight_mode_ok     boolean  True if flight mode allows test modes
     mm_test_altitude_ok        boolean  True if altitude < 1m AGL
     mm_test_arm_flight_ok      boolean  True if not armed or not flying
     mm_test_state              integer  Current state (0=IDLE,1=RAMP,2=PHASE_0,3=PHASE_60,4=FAULT,5=DELTA_RPM)
     mm_test_state_name         string   Human readable state name
     mm_test_active             boolean  True when test mode is controlling RPM
     mm_test_cmd_rpm_fwd        number   Commanded RPM for forward rotor (nil when inactive)
     mm_test_cmd_rpm_aft        number   Commanded RPM for aft rotor (nil when inactive)
     mm_test_phase_target       number   Target phase offset (0 or 60 degrees)
     mm_test_phase_error        number   Current phase error in degrees
     mm_test_rpm_fwd            number   Current forward rotor RPM
     mm_test_rpm_aft            number   Current aft rotor RPM
     mm_test_sync_achieved      boolean  True when RPM and phase targets met
     mm_test_gcs_override       boolean  GCS override active for ground testing

   Delta Sweep Outputs (mm_test_ prefix, updated at 100 Hz when DELTA_RPM state):
     mm_test_delta_active       boolean  True when delta sweep is running
     mm_test_delta_step         integer  Current step (1 to 4) or 0 if inactive
     mm_test_delta_pct          number   Current delta percentage
     mm_test_delta_rpm          number   Current delta RPM value
     mm_test_delta_sign         integer  Direction (+1 or -1)
     mm_test_beat_freq_hz       number   Calculated beat frequency in Hz
     mm_test_step_elapsed_s     number   Time elapsed in current step
     mm_test_step_dwell_s       number   Configured dwell time per step

   Test Modes Command Inputs (set externally to control synchronized mode):
     mm_test_cmd_start          boolean  Set true to start sync mode from SYNC_IDLE
     mm_test_cmd_stop           boolean  Set true to stop and return to SYNC_IDLE
     mm_test_cmd_phase_0        boolean  Set true to change to 0 degree offset
     mm_test_cmd_phase_60       boolean  Set true to change to 60 degree offset
     mm_test_cmd_reset          boolean  Set true to reset from SYNC_FAULT to SYNC_IDLE

   Delta Sweep Command Inputs (set externally to control delta RPM sweep):
     mm_test_cmd_delta_start    boolean  Set true to start delta sweep from SYNC_PHASE_0/60
     mm_test_cmd_delta_next     boolean  Set true to advance to next delta step
     mm_test_cmd_delta_reverse  boolean  Set true to reverse delta direction (+/- to -/+)
     mm_test_cmd_delta_stop     boolean  Set true to stop delta sweep and return to sync mode

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
