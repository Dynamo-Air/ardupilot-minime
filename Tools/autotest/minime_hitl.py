#!/usr/bin/env python3
"""
AS-10 MiniMe Hardware-In-The-Loop (HITL) Bridge

This script connects the MiniMe physics model to a Cube Orange+ running
SimOnHardWare firmware via MAVLink HIL messages.

Usage:
    1. Flash SimOnHardWare firmware to Cube Orange+
    2. Connect Cube via USB
    3. Run: python3 Tools/autotest/minime_hitl.py --device /dev/cu.usbmodem*
    4. The Cube will receive simulated sensor data from the physics model

Reference: https://ardupilot.org/dev/docs/sim-on-hardware.html
"""

import argparse
import math
import time
import numpy as np
from dataclasses import dataclass
from typing import Optional

try:
    from pymavlink import mavutil
except ImportError:
    print("Error: pymavlink not installed. Run: pip install pymavlink")
    exit(1)


GRAVITY_MSS = 9.80665


@dataclass
class VehicleParams:
    """AS-10 MiniMe vehicle parameters from SPECS.md"""
    mass: float = 97.5
    ixx: float = 8.54
    iyy: float = 180.76
    izz: float = 180.76
    hub_distance: float = 2.5
    hover_rpm: float = 1074.2
    hover_collective_deg: float = 5.0
    max_collective_deg: float = 12.0
    min_collective_deg: float = -2.0
    max_cyclic_deg: float = 7.0
    blade_count: int = 3
    rotor_radius: float = 1.0
    eta_yaw: float = 0.30


class TandemHelicopterPhysics:
    """Tandem counter-rotating helicopter physics for HITL"""

    def __init__(self, params: VehicleParams = None, rate_hz: float = 400.0):
        self.params = params or VehicleParams()
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz

        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.attitude = np.array([0.0, 0.0, 0.0])
        self.angular_velocity = np.array([0.0, 0.0, 0.0])

        self.rpm_fwd = 0.0
        self.rpm_aft = 0.0
        self.time_now = 0.0

        self.thrust_scale = self._calculate_thrust_scale()
        self.rotor_time_constant = 0.068
        self.drag_coefficient = 0.5
        self.angular_drag = 2.0

        self.last_velocity = None

    def _calculate_thrust_scale(self) -> float:
        hover_thrust = self.params.mass * GRAVITY_MSS
        omega_hover = self.params.hover_rpm * 2.0 * math.pi / 60.0
        return hover_thrust / (self.params.hover_collective_deg * omega_hover ** 2)

    def reset(self):
        self.position = np.array([0.0, 0.0, -0.5])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.attitude = np.array([0.0, 0.0, 0.0])
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.rpm_fwd = 0.0
        self.rpm_aft = 0.0
        self.time_now = 0.0
        self.last_velocity = None

    def _parse_swashplate(self, servo1: float, servo2: float, servo3: float):
        s1 = (servo1 - 1000) / 1000.0
        s2 = (servo2 - 1000) / 1000.0
        s3 = (servo3 - 1000) / 1000.0

        collective = (s1 + s2 + s3) / 3.0
        collective_deg = self.params.min_collective_deg + collective * (
            self.params.max_collective_deg - self.params.min_collective_deg
        )

        roll_cyclic = (s1 - s2) * self.params.max_cyclic_deg
        pitch_cyclic = ((s1 + s2) / 2.0 - s3) * self.params.max_cyclic_deg

        return collective_deg, roll_cyclic, pitch_cyclic

    def _update_rotor_rpm(self, current_rpm: float, throttle: float, collective: float) -> float:
        target_rpm = throttle * self.params.hover_rpm * 1.1

        accel_rate = 200.0
        decel_rate = 100.0

        if target_rpm > current_rpm:
            rpm_change = min(accel_rate * self.dt, target_rpm - current_rpm)
        else:
            rpm_change = -min(decel_rate * self.dt, current_rpm - target_rpm)

        return max(0.0, current_rpm + rpm_change)

    def _rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])

    def step(self, servo_outputs: list) -> dict:
        """
        Advance physics by one timestep.

        Args:
            servo_outputs: List of 16 servo PWM values (1000-2000)

        Returns:
            Dictionary with sensor data for HIL messages
        """
        if len(servo_outputs) < 8:
            servo_outputs = servo_outputs + [1500] * (8 - len(servo_outputs))

        servo_fwd_1 = servo_outputs[0] if len(servo_outputs) > 0 else 1500
        servo_fwd_2 = servo_outputs[1] if len(servo_outputs) > 1 else 1500
        servo_fwd_3 = servo_outputs[2] if len(servo_outputs) > 2 else 1500
        servo_aft_1 = servo_outputs[3] if len(servo_outputs) > 3 else 1500
        servo_aft_2 = servo_outputs[4] if len(servo_outputs) > 4 else 1500
        servo_aft_3 = servo_outputs[5] if len(servo_outputs) > 5 else 1500
        motor_interlock = servo_outputs[7] if len(servo_outputs) > 7 else 1000

        rsc = max(0.0, min(1.0, (motor_interlock - 1000) / 1000.0))

        coll_fwd, roll_cyc_fwd, pitch_cyc_fwd = self._parse_swashplate(
            servo_fwd_1, servo_fwd_2, servo_fwd_3
        )
        coll_aft, roll_cyc_aft, pitch_cyc_aft = self._parse_swashplate(
            servo_aft_1, servo_aft_2, servo_aft_3
        )

        self.rpm_fwd = self._update_rotor_rpm(self.rpm_fwd, rsc, coll_fwd)
        self.rpm_aft = self._update_rotor_rpm(self.rpm_aft, rsc, coll_aft)

        omega_fwd = self.rpm_fwd * 2.0 * math.pi / 60.0
        omega_aft = self.rpm_aft * 2.0 * math.pi / 60.0

        thrust_fwd = 0.5 * self.thrust_scale * (omega_fwd ** 2) * coll_fwd
        thrust_aft = 0.5 * self.thrust_scale * (omega_aft ** 2) * coll_aft

        hub_dist = self.params.hub_distance
        Lb1s = 3588.6
        Ma1s = 617.5 / 5.0

        rot_accel = np.array([
            (roll_cyc_fwd + roll_cyc_aft) * Lb1s / 57.3 / self.params.ixx,
            (thrust_fwd - thrust_aft) * hub_dist / self.params.iyy,
            (roll_cyc_fwd - roll_cyc_aft) * self.params.eta_yaw * Lb1s / 57.3 / self.params.izz
        ])

        scale_factor = 9.08 / self.params.mass
        rot_accel *= scale_factor

        R = self._rotation_matrix(*self.attitude)
        gravity_body = R.T @ np.array([0, 0, GRAVITY_MSS])

        total_thrust = thrust_fwd + thrust_aft
        accel_body = np.array([0, 0, -total_thrust / self.params.mass]) + gravity_body

        self.angular_velocity += rot_accel * self.dt
        angular_drag_accel = -self.angular_drag * self.angular_velocity
        self.angular_velocity += angular_drag_accel * self.dt

        max_rate = math.radians(400)
        self.angular_velocity = np.clip(self.angular_velocity, -max_rate, max_rate)

        roll_rate = self.angular_velocity[0]
        pitch_rate = self.angular_velocity[1]
        yaw_rate = self.angular_velocity[2]

        cr, sr = math.cos(self.attitude[0]), math.sin(self.attitude[0])
        cp, tp = math.cos(self.attitude[1]), math.tan(self.attitude[1])

        if abs(cp) > 0.01:
            att_dot = np.array([
                roll_rate + sr * tp * pitch_rate + cr * tp * yaw_rate,
                cr * pitch_rate - sr * yaw_rate,
                (sr / cp) * pitch_rate + (cr / cp) * yaw_rate
            ])
        else:
            att_dot = np.array([roll_rate, pitch_rate, yaw_rate])

        self.attitude += att_dot * self.dt

        self.attitude[0] = math.atan2(math.sin(self.attitude[0]), math.cos(self.attitude[0]))
        self.attitude[1] = max(-math.pi/2 + 0.01, min(math.pi/2 - 0.01, self.attitude[1]))
        self.attitude[2] = math.atan2(math.sin(self.attitude[2]), math.cos(self.attitude[2]))

        R = self._rotation_matrix(*self.attitude)
        accel_world = R @ (accel_body - gravity_body)
        accel_world[2] += GRAVITY_MSS

        self.velocity += accel_world * self.dt
        self.position += self.velocity * self.dt

        if self.position[2] > 0:
            self.position[2] = 0
            self.velocity[2] = min(0, self.velocity[2])
            self.velocity *= 0.9
            self.angular_velocity *= 0.9

        self.time_now += self.dt

        if self.last_velocity is None:
            self.last_velocity = self.velocity.copy()

        accel_imu = (self.velocity - self.last_velocity) / self.dt
        self.last_velocity = self.velocity.copy()
        accel_imu[2] -= GRAVITY_MSS
        accel_body_imu = R.T @ accel_imu

        return {
            "gyro": self.angular_velocity.copy(),
            "accel": accel_body_imu.copy(),
            "position": self.position.copy(),
            "velocity": self.velocity.copy(),
            "attitude": self.attitude.copy(),
            "rpm_fwd": self.rpm_fwd,
            "rpm_aft": self.rpm_aft,
        }


class HITLBridge:
    """MAVLink HITL bridge for SimOnHardWare"""

    def __init__(self, device: str, baudrate: int = 115200, gcs_port: int = 14550):
        self.device = device
        self.baudrate = baudrate
        self.gcs_port = gcs_port
        self.mav: Optional[mavutil.mavlink_connection] = None
        self.gcs_out: Optional[mavutil.mavlink_connection] = None
        self.gcs_in: Optional[mavutil.mavlink_connection] = None
        self.physics = TandemHelicopterPhysics()
        self.last_servo_outputs = [1500] * 16
        self.connected = False
        self.hil_enabled = False
        self.gcs_msg_count = 0
        self.cube_msg_count = 0

    def connect(self) -> bool:
        print(f"Connecting to {self.device} at {self.baudrate} baud...")
        try:
            self.mav = mavutil.mavlink_connection(self.device, baud=self.baudrate)
            self.mav.wait_heartbeat(timeout=10)
            print(f"Connected to system {self.mav.target_system}, component {self.mav.target_component}")
            self.connected = True

            print(f"Starting GCS forwarding on UDP port {self.gcs_port}...")
            self.gcs_out = mavutil.mavlink_connection(
                f"udpout:127.0.0.1:{self.gcs_port}",
                source_system=self.mav.target_system,
                source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
            )
            self.gcs_in = mavutil.mavlink_connection(
                f"udpin:0.0.0.0:{self.gcs_port + 1}",
                source_system=255,
                source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
            )
            print(f"GCS forwarding active:")
            print(f"  Cube -> QGC: UDP out to 127.0.0.1:{self.gcs_port}")
            print(f"  QGC -> Cube: UDP in on port {self.gcs_port + 1}")
            print(f"  Configure QGC: UDP, listen on {self.gcs_port}, send to {self.gcs_port + 1}")

            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def enable_hil_mode(self) -> bool:
        """Enable HIL mode on the flight controller"""
        print("Enabling HIL mode...")

        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED,
            0, 0, 0, 0, 0, 0
        )

        time.sleep(0.5)
        self.hil_enabled = True
        print("HIL mode enabled")
        return True

    def send_hil_sensor(self, sensor_data: dict):
        """Send HIL_SENSOR message with simulated IMU data"""
        time_usec = int(self.physics.time_now * 1e6)

        gyro = sensor_data["gyro"]
        accel = sensor_data["accel"]

        self.mav.mav.hil_sensor_send(
            time_usec,
            accel[0], accel[1], accel[2],
            gyro[0], gyro[1], gyro[2],
            0, 0, 0,
            101325.0,
            0.0,
            -sensor_data["position"][2],
            0x3F
        )

    def send_hil_gps(self, sensor_data: dict):
        """Send HIL_GPS message with simulated GPS data"""
        time_usec = int(self.physics.time_now * 1e6)

        lat = int(47.397742 * 1e7)
        lon = int(8.545594 * 1e7)
        alt = int(-sensor_data["position"][2] * 1000)

        vel = sensor_data["velocity"]
        vn = int(vel[0] * 100)
        ve = int(-vel[1] * 100)
        vd = int(-vel[2] * 100)

        groundspeed = int(math.sqrt(vel[0]**2 + vel[1]**2) * 100)
        cog = int(math.atan2(vel[1], vel[0]) * 18000 / math.pi) % 36000

        self.mav.mav.hil_gps_send(
            time_usec,
            3,
            lat, lon, alt,
            100, 100,
            vn, ve, vd,
            groundspeed, cog,
            10
        )

    def send_hil_state(self, sensor_data: dict):
        """Send HIL_STATE_QUATERNION message"""
        time_usec = int(self.physics.time_now * 1e6)

        att = sensor_data["attitude"]
        cr, sr = math.cos(att[0]/2), math.sin(att[0]/2)
        cp, sp = math.cos(att[1]/2), math.sin(att[1]/2)
        cy, sy = math.cos(att[2]/2), math.sin(att[2]/2)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        gyro = sensor_data["gyro"]
        accel = sensor_data["accel"]
        vel = sensor_data["velocity"]
        pos = sensor_data["position"]

        lat = int(47.397742 * 1e7)
        lon = int(8.545594 * 1e7)
        alt = int(-pos[2] * 1000)

        self.mav.mav.hil_state_quaternion_send(
            time_usec,
            [qw, qx, qy, qz],
            gyro[0], gyro[1], gyro[2],
            lat, lon, alt,
            int(vel[0] * 100), int(-vel[1] * 100), int(-vel[2] * 100),
            0, 0,
            int(accel[0] * 1000), int(accel[1] * 1000), int(accel[2] * 1000)
        )

    def receive_servo_outputs(self) -> bool:
        """Receive SERVO_OUTPUT_RAW messages"""
        msg = self.mav.recv_match(type=['SERVO_OUTPUT_RAW', 'HIL_ACTUATOR_CONTROLS'],
                                   blocking=False)
        if msg is None:
            return False

        if msg.get_type() == 'SERVO_OUTPUT_RAW':
            self.last_servo_outputs = [
                msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw,
                msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw,
                getattr(msg, 'servo9_raw', 1500),
                getattr(msg, 'servo10_raw', 1500),
                getattr(msg, 'servo11_raw', 1500),
                getattr(msg, 'servo12_raw', 1500),
                getattr(msg, 'servo13_raw', 1500),
                getattr(msg, 'servo14_raw', 1500),
                getattr(msg, 'servo15_raw', 1500),
                getattr(msg, 'servo16_raw', 1500),
            ]
            return True

        return False

    def forward_cube_to_gcs(self):
        """Forward all messages from Cube to GCS (except HIL messages we generate)"""
        if self.gcs_out is None:
            return

        while True:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()
            if msg_type in ['BAD_DATA', 'HIL_SENSOR', 'HIL_GPS', 'HIL_STATE_QUATERNION']:
                continue

            if msg_type == 'SERVO_OUTPUT_RAW':
                self.last_servo_outputs = [
                    msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw,
                    msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw,
                    getattr(msg, 'servo9_raw', 1500),
                    getattr(msg, 'servo10_raw', 1500),
                    getattr(msg, 'servo11_raw', 1500),
                    getattr(msg, 'servo12_raw', 1500),
                    getattr(msg, 'servo13_raw', 1500),
                    getattr(msg, 'servo14_raw', 1500),
                    getattr(msg, 'servo15_raw', 1500),
                    getattr(msg, 'servo16_raw', 1500),
                ]

            try:
                self.gcs_out.mav.send(msg)
                self.cube_msg_count += 1
            except Exception:
                pass

    def forward_gcs_to_cube(self):
        """Forward commands from GCS to Cube"""
        if self.gcs_in is None:
            return

        while True:
            msg = self.gcs_in.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()
            if msg_type == 'BAD_DATA':
                continue

            try:
                self.mav.mav.send(msg)
                self.gcs_msg_count += 1
            except Exception:
                pass

    def run(self):
        """Main HITL loop"""
        print("=" * 60)
        print("AS-10 MiniMe HITL Bridge")
        print("=" * 60)

        if not self.connect():
            return

        if not self.enable_hil_mode():
            return

        print("\nHITL loop starting...")
        print("Press Ctrl+C to stop\n")

        self.physics.reset()

        frame_count = 0
        last_print_time = time.time()
        target_dt = 1.0 / 400.0

        try:
            while True:
                loop_start = time.time()

                self.forward_cube_to_gcs()
                self.forward_gcs_to_cube()

                sensor_data = self.physics.step(self.last_servo_outputs)

                self.send_hil_sensor(sensor_data)

                if frame_count % 5 == 0:
                    self.send_hil_gps(sensor_data)

                if frame_count % 2 == 0:
                    self.send_hil_state(sensor_data)

                frame_count += 1

                if time.time() - last_print_time >= 1.0:
                    att = self.physics.attitude
                    print(f"Frame: {frame_count}, "
                          f"Alt: {-self.physics.position[2]:.2f}m, "
                          f"Roll: {math.degrees(att[0]):+.1f}, "
                          f"Pitch: {math.degrees(att[1]):+.1f}, "
                          f"RPM: {self.physics.rpm_fwd:.0f}/{self.physics.rpm_aft:.0f}, "
                          f"GCS: {self.gcs_msg_count}/{self.cube_msg_count}")
                    last_print_time = time.time()

                elapsed = time.time() - loop_start
                if elapsed < target_dt:
                    time.sleep(target_dt - elapsed)

        except KeyboardInterrupt:
            print("\nShutdown requested")


def main():
    parser = argparse.ArgumentParser(
        description="AS-10 MiniMe HITL Bridge",
        epilog="""
QGC Connection:
  The bridge forwards MAVLink between the Cube and QGC.
  Configure QGC with a UDP link:
    - Listen on port 14550 (receives from Cube)
    - Send to port 14551 (commands to Cube)
  Or use MAVProxy: mavproxy.py --master=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
        """
    )
    parser.add_argument("--device", type=str, default="/dev/cu.usbmodem2101",
                        help="Serial device for Cube connection")
    parser.add_argument("--baudrate", type=int, default=115200,
                        help="Serial baud rate")
    parser.add_argument("--gcs-port", type=int, default=14550,
                        help="UDP port for GCS output (input is port+1)")
    args = parser.parse_args()

    bridge = HITLBridge(args.device, args.baudrate, args.gcs_port)
    bridge.run()


if __name__ == "__main__":
    main()
