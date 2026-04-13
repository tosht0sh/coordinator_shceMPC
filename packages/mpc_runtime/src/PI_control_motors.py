#!/usr/bin/env python3
from __future__ import annotations

"""Wheel-speed PI helper used by the onboard MPC runtime.

The MPC still decides the desired body motion `(v, omega)`. This helper adds a
low-level correction layer that uses encoder-derived wheel angular speeds to
reduce motor mismatch and straight-line drift before the command is published.
"""

import math
import os

import rospy
from duckietown_msgs.msg import WheelEncoderStamped


class TickToVelocity:
    """Estimate filtered left/right wheel angular speeds from encoder ticks."""

    def __init__(self):
        self.vehicle_name = os.environ.get("VEHICLE_NAME", "duckiebot")
        self.wheel_radius = rospy.get_param(
            f"/{self.vehicle_name}/kinematics_node/radius",
            0.0318,
        )
        self.axis_length = rospy.get_param(
            f"/{self.vehicle_name}/kinematics_node/baseline",
            0.1,
        )
        # Small time constant keeps the estimate smooth without adding too much lag.
        self.tau = rospy.get_param("~lpf_tau", 0.2)

        self.prev = {
            "L": {"t": None, "ticks": None},
            "R": {"t": None, "ticks": None},
        }
        self.w_raw = {"L": 0.0, "R": 0.0}
        self.w_filt = {"L": 0.0, "R": 0.0}
        self._t_last_filter = None

        rospy.Subscriber(
            f"/{self.vehicle_name}/left_wheel_encoder_node/tick",
            WheelEncoderStamped,
            self._cb_left,
            queue_size=10,
        )
        rospy.Subscriber(
            f"/{self.vehicle_name}/right_wheel_encoder_node/tick",
            WheelEncoderStamped,
            self._cb_right,
            queue_size=10,
        )

    def _update_from_tick(self, side: str, msg: WheelEncoderStamped) -> None:
        stamp = msg.header.stamp.to_sec()
        if stamp == 0.0:
            stamp = rospy.Time.now().to_sec()

        ticks = int(msg.data)
        resolution = int(msg.resolution)
        prev = self.prev[side]
        if prev["t"] is not None:
            dt = stamp - prev["t"]
            if dt > 1e-4 and resolution > 0:
                d_ticks = ticks - prev["ticks"]
                self.w_raw[side] = 2.0 * math.pi * (d_ticks / float(resolution)) / dt

        self.prev[side] = {"t": stamp, "ticks": ticks}

    def _cb_left(self, msg: WheelEncoderStamped) -> None:
        self._update_from_tick("L", msg)

    def _cb_right(self, msg: WheelEncoderStamped) -> None:
        self._update_from_tick("R", msg)

    def _update_filter(self) -> None:
        now = rospy.Time.now().to_sec()
        w_left = self.w_raw["L"]
        w_right = self.w_raw["R"]

        if self._t_last_filter is None:
            self._t_last_filter = now
            self.w_filt["L"] = w_left
            self.w_filt["R"] = w_right
            return

        dt = now - self._t_last_filter
        self._t_last_filter = now
        if dt <= 1e-4:
            return

        alpha = dt / (self.tau + dt)
        self.w_filt["L"] += alpha * (w_left - self.w_filt["L"])
        self.w_filt["R"] += alpha * (w_right - self.w_filt["R"])

    def get_filtered_wheels(self) -> tuple[float, float]:
        """Return filtered wheel angular speeds `(w_left, w_right)` in rad/s."""

        self._update_filter()
        return self.w_filt["L"], self.w_filt["R"]

    def is_ready(self) -> bool:
        """Return True once both encoder streams have produced usable updates."""

        # Only require that both tick streams have delivered timestamps. The
        # filter state is initialized on the first call to get_filtered_wheels().
        return all(side["t"] is not None for side in self.prev.values())


class PI:
    """PI correction around the MPC body command.

    The controller converts the MPC body command `(v_ref, omega_ref)` into
    left/right wheel angular speed references, corrects them using filtered
    encoder feedback, and converts the corrected wheel speeds back to a body
    command so the rest of the runtime can keep publishing `Twist2DStamped`.
    """

    def __init__(self):
        self.kp = float(0.05)
        self.ki = float(0.10)
        self.integral_limit = float(5.0)
        self.estimator = TickToVelocity()
        self.wheel_radius = self.estimator.wheel_radius
        self.axis_length = self.estimator.axis_length
        self.integral_left = 0.0
        self.integral_right = 0.0

    def reset(self) -> None:
        self.integral_left = 0.0
        self.integral_right = 0.0

    def _clamp_integral(self, value: float) -> float:
        return max(-self.integral_limit, min(self.integral_limit, value))

    def is_ready(self) -> bool:
        # This uses tick timestamps only, so we avoid a deadlock where PI waits
        # for filtered data before ever calling the filter update.
        return self.estimator.is_ready()

    def pi_controller(self, v_ref: float, omega_ref: float, dt: float) -> tuple[float, float]:
        """Return corrected v_cmd, omega_cmd from the MPC reference command."""

        if abs(v_ref) < 1e-6 and abs(omega_ref) < 1e-6:
            self.reset()
            return 0.0, 0.0

        dt = max(float(dt), 1e-3)
        w_left_meas, w_right_meas = self.estimator.get_filtered_wheels()

        w_left_ref = (v_ref - 0.5 * self.axis_length * omega_ref) / self.wheel_radius
        w_right_ref = (v_ref + 0.5 * self.axis_length * omega_ref) / self.wheel_radius

        error_left = w_left_ref - w_left_meas
        error_right = w_right_ref - w_right_meas

        self.integral_left = self._clamp_integral(self.integral_left + error_left * dt)
        self.integral_right = self._clamp_integral(self.integral_right + error_right * dt)

        # Treat the PI term as a trim around the MPC wheel-speed reference.
        w_left_cmd = w_left_ref + self.kp * error_left + self.ki * self.integral_left
        w_right_cmd = w_right_ref + self.kp * error_right + self.ki * self.integral_right

        v_cmd = 0.5 * self.wheel_radius * (w_right_cmd + w_left_cmd)
        omega_cmd = self.wheel_radius * (w_right_cmd - w_left_cmd) / self.axis_length
        return v_cmd, omega_cmd
