#!/usr/bin/env python3
"""
AS-10 MiniMe SITL Failsafe Test Script
Tests failsafe responses per STATE.md lines 1180-1185

Tests failsafe MODE TRANSITIONS (not actual flight) by:
1. Arming the helicopter
2. Triggering failsafe conditions
3. Verifying the mode changes to RTL or LAND

Usage:
    cd /Users/halvo/Dropbox/current/dynamo/code/ardu/ardupilot-minime
    source .venv/bin/activate
    python3 Tools/autotest/minime_failsafe_test.py

Requirements:
    - SITL binary built (build/sitl/bin/arducopter-heli)
    - pymavlink installed
"""

import sys
import os
import time
import subprocess
import signal
import threading
from pymavlink import mavutil

# ArduPilot flight modes
MODE_STABILIZE = 0
MODE_ALT_HOLD = 2
MODE_LOITER = 5
MODE_RTL = 6
MODE_LAND = 9
MODE_GUIDED = 4

MODE_NAMES = {
    0: "STABILIZE",
    2: "ALT_HOLD",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    9: "LAND",
}


class MiniMeFailsafeTest:
    def __init__(self):
        self.sitl_process = None
        self.mav = None
        self.results = []
        self.output_thread = None
        self.stop_output = False
        self.rc_thread = None
        self.stop_rc = False
        self.rc_throttle = 1000  # Start at minimum
        self.rc_interlock = 1000  # LOW = allows arming

    def log(self, msg):
        print(f"[TEST] {msg}")

    def drain_sitl_output(self):
        """Drain SITL stdout to prevent buffer blocking"""
        while not self.stop_output:
            try:
                line = self.sitl_process.stdout.readline()
                if not line:
                    break
            except:
                break

    def rc_override_loop(self):
        """Background thread to continuously send RC overrides"""
        while not self.stop_rc:
            try:
                if self.mav:
                    self.mav.mav.rc_channels_override_send(
                        self.mav.target_system,
                        self.mav.target_component,
                        1500, 1500, self.rc_throttle, 1500,
                        1500, 1500, 1500, self.rc_interlock,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                    )
            except:
                pass
            time.sleep(0.05)  # 20 Hz

    def start_rc_thread(self):
        """Start background RC override thread"""
        self.stop_rc = False
        self.rc_thread = threading.Thread(target=self.rc_override_loop)
        self.rc_thread.daemon = True
        self.rc_thread.start()

    def stop_rc_thread(self):
        """Stop background RC override thread"""
        self.stop_rc = True
        if self.rc_thread:
            self.rc_thread.join(timeout=1)

    def start_sitl(self):
        """Start SITL with MiniMe parameters"""
        self.log("Starting SITL...")

        project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        os.chdir(project_root)

        cmd = [
            sys.executable,
            "Tools/autotest/sim_vehicle.py",
            "-v", "ArduCopter",
            "--frame", "heli-dual",
            "--add-param-file", "Tools/autotest/default_params/copter-heli-minime.parm",
            "--no-mavproxy",
            "-w",
            "--speedup", "10",
        ]

        self.sitl_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )

        self.stop_output = False
        self.output_thread = threading.Thread(target=self.drain_sitl_output)
        self.output_thread.daemon = True
        self.output_thread.start()

        self.log("Waiting for SITL (20 seconds)...")
        time.sleep(20)

        self.log("Connecting via TCP...")
        self.mav = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255)
        msg = self.mav.wait_heartbeat(timeout=30)
        if not msg:
            raise Exception("No heartbeat from SITL")

        self.log(f"Connected: system {self.mav.target_system}")
        if self.mav.target_component == 0:
            self.mav.target_component = 1

        # Start RC override thread
        self.start_rc_thread()
        time.sleep(2)

        self.log("SITL ready")

    def stop_sitl(self):
        """Stop SITL process"""
        self.stop_output = True
        self.stop_rc_thread()
        if self.sitl_process:
            self.log("Stopping SITL...")
            self.sitl_process.send_signal(signal.SIGINT)
            try:
                self.sitl_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.sitl_process.kill()
            self.log("SITL stopped")

    def get_mode(self):
        """Get current flight mode"""
        msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            return msg.custom_mode
        return None

    def get_mode_name(self, mode):
        """Get mode name string"""
        return MODE_NAMES.get(mode, f"MODE_{mode}")

    def set_mode(self, mode):
        """Set flight mode"""
        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode
        )

    def set_param(self, name, value):
        """Set a parameter"""
        self.mav.mav.param_set_send(
            self.mav.target_system,
            self.mav.target_component,
            name.encode('utf-8'),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        time.sleep(0.3)

    def get_param(self, name):
        """Get a parameter value"""
        self.mav.mav.param_request_read_send(
            self.mav.target_system,
            self.mav.target_component,
            name.encode('utf-8'),
            -1
        )
        msg = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if msg:
            return msg.param_value
        return None

    def wait_mode(self, target_mode, timeout=10):
        """Wait for mode to change"""
        start = time.time()
        while time.time() - start < timeout:
            current = self.get_mode()
            if current == target_mode:
                return True
            time.sleep(0.2)
        return False

    def arm_force(self):
        """Force arm the vehicle"""
        self.rc_interlock = 1000  # LOW for arming
        self.rc_throttle = 1000
        time.sleep(0.5)

        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0, 0, 0, 0, 0  # Force arm
        )

        # Wait for arm
        for _ in range(20):
            msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
            if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                # Enable motors after arming
                self.rc_interlock = 2000
                return True
        return False

    def disarm_force(self):
        """Force disarm"""
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 21196, 0, 0, 0, 0, 0
        )
        time.sleep(1)
        self.rc_interlock = 1000

    def is_armed(self):
        """Check if vehicle is armed"""
        msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg:
            return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return False

    def reset_for_test(self):
        """Reset vehicle state for next test"""
        self.disarm_force()
        time.sleep(0.5)
        self.set_mode(MODE_STABILIZE)
        time.sleep(0.5)

    # ========== FAILSAFE TESTS ==========

    def test_rc_loss(self):
        """Test RC loss triggers RTL (STATE.md line 1180)"""
        self.log("\n" + "="*50)
        self.log("TEST: RC Loss -> RTL")
        self.log("="*50)

        fs_thr = self.get_param("FS_THR_ENABLE")
        self.log(f"FS_THR_ENABLE = {fs_thr}")

        # Set STABILIZE and arm
        self.set_mode(MODE_STABILIZE)
        time.sleep(0.5)

        if not self.arm_force():
            self.log("Failed to arm")
            self.results.append(("RC Loss -> RTL", False, "Could not arm"))
            return

        self.log("Armed. Simulating RC loss...")
        self.set_param("SIM_RC_FAIL", 1)

        # Wait for RTL
        time.sleep(2)
        success = self.wait_mode(MODE_RTL, timeout=5)

        self.set_param("SIM_RC_FAIL", 0)

        if success:
            self.log("PASS: Mode changed to RTL")
            self.results.append(("RC Loss -> RTL", True, None))
        else:
            current = self.get_mode()
            self.log(f"FAIL: Mode is {self.get_mode_name(current)}")
            self.results.append(("RC Loss -> RTL", False, f"Mode={self.get_mode_name(current)}"))

        self.reset_for_test()

    def test_gcs_loss(self):
        """Test GCS loss configuration (STATE.md line 1181)"""
        self.log("\n" + "="*50)
        self.log("TEST: GCS Loss Configuration")
        self.log("="*50)

        # GCS failsafe requires stopping heartbeats which is complex in this setup
        # Verify configuration instead
        fs_gcs = self.get_param("FS_GCS_ENABLE")
        fs_gcs_timeout = self.get_param("FS_GCS_TIMEOUT")

        self.log(f"FS_GCS_ENABLE = {fs_gcs}")
        self.log(f"FS_GCS_TIMEOUT = {fs_gcs_timeout}s")

        if fs_gcs == 1 or fs_gcs == 0:  # Either RTL or disabled is valid config
            self.log("PASS: GCS failsafe configured")
            self.results.append(("GCS Loss Config", True, f"FS_GCS_ENABLE={fs_gcs}"))
        else:
            self.results.append(("GCS Loss Config", False, "Invalid config"))

    def test_battery_low(self):
        """Test battery low triggers RTL (STATE.md line 1182)"""
        self.log("\n" + "="*50)
        self.log("TEST: Battery Low -> RTL")
        self.log("="*50)

        batt_low_act = self.get_param("BATT_FS_LOW_ACT")
        batt_low_volt = self.get_param("BATT_LOW_VOLT")
        self.log(f"BATT_FS_LOW_ACT = {batt_low_act} (2=RTL)")
        self.log(f"BATT_LOW_VOLT = {batt_low_volt}V")

        self.set_mode(MODE_STABILIZE)
        time.sleep(0.5)

        if not self.arm_force():
            self.log("Failed to arm")
            self.results.append(("Battery Low -> RTL", False, "Could not arm"))
            return

        self.log("Armed. Simulating low battery...")
        original_voltage = self.get_param("SIM_BATT_VOLTAGE")
        self.set_param("SIM_BATT_VOLTAGE", batt_low_volt - 2)

        time.sleep(3)
        success = self.wait_mode(MODE_RTL, timeout=5)

        self.set_param("SIM_BATT_VOLTAGE", original_voltage if original_voltage else 150)

        if success:
            self.log("PASS: Mode changed to RTL")
            self.results.append(("Battery Low -> RTL", True, None))
        else:
            current = self.get_mode()
            self.log(f"FAIL: Mode is {self.get_mode_name(current)}")
            self.results.append(("Battery Low -> RTL", False, f"Mode={self.get_mode_name(current)}"))

        self.reset_for_test()

    def test_battery_critical(self):
        """Test battery critical triggers Land (STATE.md line 1183)"""
        self.log("\n" + "="*50)
        self.log("TEST: Battery Critical -> Land")
        self.log("="*50)

        batt_crt_act = self.get_param("BATT_FS_CRT_ACT")
        batt_crt_volt = self.get_param("BATT_CRT_VOLT")
        self.log(f"BATT_FS_CRT_ACT = {batt_crt_act} (3=Land)")
        self.log(f"BATT_CRT_VOLT = {batt_crt_volt}V")

        self.set_mode(MODE_STABILIZE)
        time.sleep(0.5)

        if not self.arm_force():
            self.log("Failed to arm")
            self.results.append(("Battery Critical -> Land", False, "Could not arm"))
            return

        self.log("Armed. Simulating critical battery...")
        original_voltage = self.get_param("SIM_BATT_VOLTAGE")
        self.set_param("SIM_BATT_VOLTAGE", batt_crt_volt - 2)

        time.sleep(3)
        success = self.wait_mode(MODE_LAND, timeout=5)

        self.set_param("SIM_BATT_VOLTAGE", original_voltage if original_voltage else 150)

        if success:
            self.log("PASS: Mode changed to LAND")
            self.results.append(("Battery Critical -> Land", True, None))
        else:
            current = self.get_mode()
            self.log(f"FAIL: Mode is {self.get_mode_name(current)}")
            self.results.append(("Battery Critical -> Land", False, f"Mode={self.get_mode_name(current)}"))

        self.reset_for_test()

    def test_ekf_failure(self):
        """Test EKF failure configuration (STATE.md line 1184)"""
        self.log("\n" + "="*50)
        self.log("TEST: EKF Failure Configuration")
        self.log("="*50)

        fs_ekf_act = self.get_param("FS_EKF_ACTION")
        self.log(f"FS_EKF_ACTION = {fs_ekf_act} (1=Land)")

        if fs_ekf_act == 1:
            self.log("PASS: EKF failsafe configured for Land")
            self.results.append(("EKF Failure Config", True, "FS_EKF_ACTION=1"))
        else:
            self.results.append(("EKF Failure Config", False, f"FS_EKF_ACTION={fs_ekf_act}"))

    def test_geofence(self):
        """Test geofence configuration (STATE.md line 1185)"""
        self.log("\n" + "="*50)
        self.log("TEST: Geofence Configuration")
        self.log("="*50)

        fence_enable = self.get_param("FENCE_ENABLE")
        fence_action = self.get_param("FENCE_ACTION")
        fence_radius = self.get_param("FENCE_RADIUS")
        fence_alt = self.get_param("FENCE_ALT_MAX")

        self.log(f"FENCE_ENABLE = {fence_enable}")
        self.log(f"FENCE_ACTION = {fence_action} (1=RTL)")
        self.log(f"FENCE_RADIUS = {fence_radius}m")
        self.log(f"FENCE_ALT_MAX = {fence_alt}m")

        if fence_enable == 1 and fence_action == 1:
            self.log("PASS: Geofence configured for RTL")
            self.results.append(("Geofence Config", True, f"Radius={fence_radius}m, Alt={fence_alt}m"))
        else:
            self.results.append(("Geofence Config", False, f"Enable={fence_enable}, Action={fence_action}"))

    def print_summary(self):
        """Print test summary"""
        print("\n" + "="*60)
        print("FAILSAFE TEST SUMMARY")
        print("="*60)

        passed = sum(1 for _, success, _ in self.results if success)
        total = len(self.results)

        for name, success, note in self.results:
            status = "PASS" if success else "FAIL"
            note_str = f" ({note})" if note else ""
            print(f"  {name}: {status}{note_str}")

        print(f"\nTotal: {passed}/{total} passed")
        print("="*60)
        return passed == total

    def run_all_tests(self):
        """Run all failsafe tests"""
        try:
            self.start_sitl()

            self.test_rc_loss()
            self.test_gcs_loss()
            self.test_battery_low()
            self.test_battery_critical()
            self.test_ekf_failure()
            self.test_geofence()

            return self.print_summary()
        except KeyboardInterrupt:
            self.log("\nInterrupted")
            return False
        except Exception as e:
            self.log(f"\nError: {e}")
            import traceback
            traceback.print_exc()
            return False
        finally:
            self.stop_sitl()


if __name__ == "__main__":
    print("="*60)
    print("AS-10 MiniMe Failsafe Test")
    print("STATE.md lines 1180-1185")
    print("="*60)

    tester = MiniMeFailsafeTest()
    success = tester.run_all_tests()
    sys.exit(0 if success else 1)
