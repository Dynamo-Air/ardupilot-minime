#!/usr/bin/env python3
"""
AS-10 MiniMe Tandem Helicopter Physics Model

This is a custom physics model for the MiniMe tandem counter-rotating helicopter
that communicates with ArduPilot SITL via the JSON interface.

Vehicle Parameters (from SPECS.md):
    Mass: 97.5 kg (214.94 lbm)
    Ixx: 8.54 kg·m² (very low, responsive roll)
    Iyy: 180.76 kg·m² (high pitch inertia)
    Izz: 180.76 kg·m² (high yaw inertia)
    Hub to hub span: 2.5 m
    Hover RPM: 1074.2
    Frame type: Tandem counter-rotating, hingeless flybarless

Usage:
    Terminal 1: python3 Tools/autotest/minime_physics.py
    Terminal 2: sim_vehicle.py -v ArduCopter --model JSON --add-param-file Tools/autotest/default_params/copter-heli-minime.parm

Author: Dynamo Air
Reference: SPECS.md, STATE.md
"""

import argparse
import json
import math
import socket
import struct
import time
import numpy as np
from dataclasses import dataclass
from typing import Tuple

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
    """
    Tandem counter-rotating helicopter physics model.

    Coordinate frame (FRD):
        +X: Forward (toward forward rotor)
        +Y: Right (starboard)
        +Z: Down

    Servo mapping for heli-dual:
        Servo 1-3: Forward rotor swashplate
        Servo 4-6: Aft rotor swashplate
        Servo 8: Motor interlock / RSC
    """

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

        self.tpp_angle_fwd = np.array([0.0, 0.0])
        self.tpp_angle_aft = np.array([0.0, 0.0])

        self.thrust_scale = self._calculate_thrust_scale()
        self.torque_scale = self._calculate_torque_scale()

        self.rotor_time_constant = 0.068

        self.drag_coefficient = 0.5
        self.angular_drag = 2.0

        self.last_velocity = None

        print(f"MiniMe Physics initialized:")
        print(f"  Mass: {self.params.mass} kg")
        print(f"  Inertia: Ixx={self.params.ixx}, Iyy={self.params.iyy}, Izz={self.params.izz} kg·m²")
        print(f"  Hub distance: {self.params.hub_distance} m")
        print(f"  Hover RPM: {self.params.hover_rpm}")
        print(f"  Rate: {self.rate_hz} Hz")

    def _calculate_thrust_scale(self) -> float:
        """Calculate thrust scale to hover at hover_collective_deg"""
        hover_thrust = self.params.mass * GRAVITY_MSS
        omega_hover = self.params.hover_rpm * 2.0 * math.pi / 60.0
        return hover_thrust / (self.params.hover_collective_deg * omega_hover ** 2)

    def _calculate_torque_scale(self) -> float:
        """Calculate torque scale for rotor drag"""
        omega_hover = self.params.hover_rpm * 2.0 * math.pi / 60.0
        return 0.05 * self.params.mass * GRAVITY_MSS / (omega_hover ** 2)

    def reset(self):
        """Reset vehicle to initial state"""
        self.position = np.array([0.0, 0.0, -0.5])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.attitude = np.array([0.0, 0.0, 0.0])
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.rpm_fwd = 0.0
        self.rpm_aft = 0.0
        self.time_now = 0.0
        self.tpp_angle_fwd = np.array([0.0, 0.0])
        self.tpp_angle_aft = np.array([0.0, 0.0])
        self.last_velocity = None
        print("Vehicle reset")

    def _parse_swashplate(self, servo1: float, servo2: float, servo3: float) -> Tuple[float, float, float]:
        """
        Parse swashplate servo commands to collective and cyclic.

        Returns:
            (collective_deg, roll_cyclic_deg, pitch_cyclic_deg)
        """
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
        """Update rotor RPM based on throttle command and load"""
        target_rpm = throttle * self.params.hover_rpm * 1.1

        accel_rate = 200.0
        decel_rate = 100.0

        if target_rpm > current_rpm:
            rpm_change = min(accel_rate * self.dt, target_rpm - current_rpm)
        else:
            rpm_change = -min(decel_rate * self.dt, current_rpm - target_rpm)

        return max(0.0, current_rpm + rpm_change)

    def _update_tpp_angle(self, gyro: np.ndarray, ctrl_pos: np.ndarray,
                          tpp_angle: np.ndarray) -> np.ndarray:
        """
        Update tip path plane angle for a rotor.

        Args:
            gyro: Body angular rates [p, q, r] in rad/s
            ctrl_pos: Cyclic control position [roll, pitch]
            tpp_angle: Current TPP angle [b1s, a1s]

        Returns:
            Updated TPP angle
        """
        tau_inv = 1.0 / self.rotor_time_constant

        Lflt = 1.7635
        Mflg = 1.9432

        b1s_dot = -gyro[0] - tau_inv * tpp_angle[0] + tau_inv * Lflt * ctrl_pos[0]
        a1s_dot = -gyro[1] - tau_inv * tpp_angle[1] + tau_inv * Mflg * ctrl_pos[1]

        new_tpp = np.array([
            tpp_angle[0] + b1s_dot * self.dt,
            tpp_angle[1] + a1s_dot * self.dt
        ])

        return new_tpp

    def _rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Create rotation matrix from Euler angles (ZYX convention)"""
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])

    def step(self, pwm: list) -> dict:
        """
        Advance physics simulation by one timestep.

        Args:
            pwm: List of 16 servo PWM values (1000-2000)

        Returns:
            JSON data dict for SITL
        """
        servo_fwd_1 = pwm[0]
        servo_fwd_2 = pwm[1]
        servo_fwd_3 = pwm[2]
        servo_aft_1 = pwm[3]
        servo_aft_2 = pwm[4]
        servo_aft_3 = pwm[5]
        motor_interlock = pwm[7] if len(pwm) > 7 else 1000

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
        omega_hover = self.params.hover_rpm * 2.0 * math.pi / 60.0

        cyclic_scalar = 7.2
        ctrl_pos_fwd = np.array([
            roll_cyc_fwd / cyclic_scalar,
            pitch_cyc_fwd / cyclic_scalar
        ])
        ctrl_pos_aft = np.array([
            roll_cyc_aft / cyclic_scalar,
            pitch_cyc_aft / cyclic_scalar
        ])

        self.tpp_angle_fwd = self._update_tpp_angle(
            self.angular_velocity, ctrl_pos_fwd, self.tpp_angle_fwd
        )
        self.tpp_angle_aft = self._update_tpp_angle(
            self.angular_velocity, ctrl_pos_aft, self.tpp_angle_aft
        )

        rpm_ratio_fwd = (self.rpm_fwd / self.params.hover_rpm) if self.params.hover_rpm > 0 else 0
        rpm_ratio_aft = (self.rpm_aft / self.params.hover_rpm) if self.params.hover_rpm > 0 else 0

        thrust_fwd = 0.5 * self.thrust_scale * (omega_fwd ** 2) * coll_fwd
        thrust_aft = 0.5 * self.thrust_scale * (omega_aft ** 2) * coll_aft

        Lb1s = 3588.6
        Ma1s = 617.5 / 5.0
        Mu = 0.003
        Lv = -0.006
        Xu = -0.125
        Yv = -0.375
        Zw = -2.25
        hub_dist = self.params.hub_distance

        rot_accel = np.array([
            (self.tpp_angle_fwd[0] + self.tpp_angle_aft[0]) * Lb1s +
            Lv * self.velocity[1],

            (self.tpp_angle_fwd[1] + self.tpp_angle_aft[1]) * Ma1s +
            (thrust_fwd - thrust_aft) * hub_dist / self.params.iyy +
            Mu * self.velocity[0],

            (self.tpp_angle_fwd[0] * thrust_fwd - self.tpp_angle_aft[0] * thrust_aft) *
            hub_dist / (self.params.izz * 2.0) -
            0.5 * self.angular_velocity[2]
        ])

        scale_factor = 9.08 / self.params.mass
        rot_accel *= scale_factor

        lat_y = (GRAVITY_MSS * (self.tpp_angle_fwd[0] + self.tpp_angle_aft[0]) +
                 Yv * self.velocity[1])
        lat_x = (-GRAVITY_MSS * (self.tpp_angle_fwd[1] + self.tpp_angle_aft[1]) +
                 Xu * self.velocity[0])
        vertical = (-(thrust_fwd + thrust_aft) / self.params.mass +
                   self.velocity[2] * Zw)

        R = self._rotation_matrix(*self.attitude)
        gravity_body = R.T @ np.array([0, 0, GRAVITY_MSS])

        accel_body = np.array([lat_x, lat_y, vertical]) + gravity_body

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

        gyro_body = self.angular_velocity.copy()

        pos_ned = np.array([self.position[0], -self.position[1], -self.position[2]])
        vel_ned = np.array([self.velocity[0], -self.velocity[1], -self.velocity[2]])
        gyro_ap = np.array([gyro_body[0], -gyro_body[1], -gyro_body[2]])
        accel_ap = np.array([accel_body_imu[0], -accel_body_imu[1], -accel_body_imu[2]])
        att_ap = np.array([self.attitude[0], -self.attitude[1], -self.attitude[2]])

        return {
            "timestamp": self.time_now,
            "imu": {
                "gyro": list(gyro_ap),
                "accel_body": list(accel_ap)
            },
            "position": list(pos_ned),
            "attitude": list(att_ap),
            "velocity": list(vel_ned)
        }


def main():
    parser = argparse.ArgumentParser(description="AS-10 MiniMe Physics Model")
    parser.add_argument("--fps", type=float, default=400.0, help="Physics frame rate")
    parser.add_argument("--port", type=int, default=9002, help="UDP port to listen on")
    args = parser.parse_args()

    print("=" * 60)
    print("AS-10 MiniMe Tandem Helicopter Physics Model")
    print("=" * 60)

    params = VehicleParams()
    physics = TandemHelicopterPhysics(params, rate_hz=args.fps)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', args.port))
    sock.settimeout(0.1)

    print(f"Listening on UDP port {args.port}")
    print("Waiting for ArduPilot SITL connection...")
    print("Start SITL with: sim_vehicle.py -v ArduCopter --model JSON --add-param-file Tools/autotest/default_params/copter-heli-minime.parm")

    last_sitl_frame = -1
    connected = False
    frame_count = 0
    frame_time = time.time()
    print_interval = 1000

    while True:
        try:
            data, address = sock.recvfrom(100)
        except socket.timeout:
            continue
        except KeyboardInterrupt:
            print("\nShutdown requested")
            break

        parse_format = 'HHI16H'
        expected_size = struct.calcsize(parse_format)

        if len(data) != expected_size:
            parse_format_32 = 'HHI32H'
            if len(data) == struct.calcsize(parse_format_32):
                parse_format = parse_format_32
            else:
                print(f"Bad packet size: {len(data)} (expected {expected_size})")
                continue

        decoded = struct.unpack(parse_format, data)

        magic_16 = 18458
        magic_32 = 29569
        if decoded[0] not in [magic_16, magic_32]:
            print(f"Incorrect magic: {decoded[0]}")
            continue

        frame_rate_hz = decoded[1]
        frame_number = decoded[2]
        pwm = list(decoded[3:])

        if frame_rate_hz != physics.rate_hz:
            old_rate = physics.rate_hz
            physics.rate_hz = frame_rate_hz
            physics.dt = 1.0 / frame_rate_hz
            if abs(frame_rate_hz - old_rate) > 50 or frame_rate_hz == 400:
                print(f"Physics rate: {frame_rate_hz} Hz")

        if frame_number < last_sitl_frame:
            physics.reset()
        elif frame_number != last_sitl_frame + 1 and connected:
            missed = frame_number - last_sitl_frame - 1
            if missed > 0 and missed < 100:
                print(f"Missed {missed} frames")

        last_sitl_frame = frame_number

        if not connected:
            connected = True
            print(f"Connected to ArduPilot SITL at {address}")

        json_data = physics.step(pwm)

        message = json.dumps(json_data, separators=(',', ':')) + "\n"
        sock.sendto(message.encode('ascii'), address)

        frame_count += 1
        if frame_count % print_interval == 0:
            now = time.time()
            elapsed = now - frame_time
            fps = print_interval / elapsed
            print(f"FPS: {fps:.1f}, T={physics.time_now:.2f}s, "
                  f"Alt={-physics.position[2]:.2f}m, "
                  f"RPM_F={physics.rpm_fwd:.0f}, RPM_A={physics.rpm_aft:.0f}")
            frame_time = now


if __name__ == "__main__":
    main()
