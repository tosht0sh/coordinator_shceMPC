#!/usr/bin/env python3
from __future__ import annotations

"""Publish a repeatable body-command sequence for wheel identification.

This node is meant for dedicated identification runs where we want to excite
the downstream Duckiebot drive stack without running the full MPC/scheduler
pipeline. It publishes `Twist2DStamped` commands directly to the same final
command topic used by the runtime node.
"""

import os

import rospy
from duckietown_msgs.msg import Twist2DStamped


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _parse_sequence(raw: str) -> list[tuple[float, float, float]]:
    """Parse a `duration,v,omega;...` command sequence string."""

    phases: list[tuple[float, float, float]] = []
    for chunk in raw.split(";"):
        item = chunk.strip()
        if not item:
            continue

        parts = [part.strip() for part in item.split(",")]
        if len(parts) != 3:
            raise ValueError(
                "Each phase must have exactly 3 comma-separated values: duration,v,omega"
            )

        duration, v_cmd, omega_cmd = (float(part) for part in parts)
        if duration <= 0.0:
            raise ValueError("Each phase duration must be positive.")
        phases.append((duration, v_cmd, omega_cmd))

    if not phases:
        raise ValueError("Command sequence must contain at least one phase.")
    return phases


class WheelIdCommandPlayer:
    """Publish a reusable body-command sequence for identification runs."""

    def __init__(self) -> None:
        self.vehicle_name = os.environ.get("VEHICLE_NAME", "duckiebot")
        self.cmd_topic = os.getenv(
            "WHEEL_ID_PLAYER_TOPIC",
            f"/{self.vehicle_name}/car_cmd_switch_node/cmd",
        )
        self.publish_hz = float(os.getenv("WHEEL_ID_PLAYER_HZ", "25.0"))
        self.repeat = _env_bool("WHEEL_ID_PLAYER_REPEAT", False)

        # Default sequence approximates the rectangle-like schedule used for the
        # real MPC tuning, but with explicit point turns between the straight
        # segments. The quarter-turn durations are slightly generous to account
        # for real robot lag during spin-in-place commands.
        # Format: duration, v_cmd, omega_cmd
        straight_speed = 0.32
        turn_omega = 8.00
        quarter_turn_sec = 0.7
        half_turn_sec = 1.40
        settle_sec = 0.60

        # Use this for traning
        # default_sequence = (
        #     "1.0,0.00,0.00;"
        #     "3.0,0.12,0.00;"
        #     "0.8,0.00,0.00;"
        #     "4.5,0.20,0.00;"
        #     "0.8,0.00,0.00;"
        #     "2.5,0.28,0.00;"
        #     "0.8,0.00,0.00;"
        #     "0.7,0.00,2.50;"
        #     "0.8,0.00,0.00;"
        #     "0.9,0.00,3.50;"
        #     "0.8,0.00,0.00;"
        #     "0.7,0.00,-2.50;"
        #     "0.8,0.00,0.00;"
        #     "0.9,0.00,-3.50;"
        #     "0.8,0.00,0.00;"
        #     "2.5,0.14,1.50;"
        #     "0.8,0.00,0.00;"
        #     "2.5,0.14,-1.50;"
        #     "0.8,0.00,0.00;"
        #     "3.5,0.22,0.00;"
        #     "0.8,0.00,0.00;"
        #     "2.0,0.18,2.00;"
        #     "0.8,0.00,0.00;"
        #     "2.0,0.18,-2.00;"
        #     "1.0,0.00,0.00"
        # )

        # Use this for validation
        default_sequence = (
            "1.0,0.00,0.00;"
            f"6.0,{straight_speed:.2f},0.00;"
            f"{settle_sec:.2f},0.00,0.00;"
            f"{quarter_turn_sec:.2f},0.00,{turn_omega:.2f};"
            f"6.0,{straight_speed:.2f},0.00;"
            f"{settle_sec:.2f},0.00,0.00;"
            f"{quarter_turn_sec:.2f},0.00,{turn_omega:.2f};"
            f"6.0,{straight_speed:.2f},0.00;"
            f"{settle_sec:.2f},0.00,0.00;"
            f"{quarter_turn_sec:.2f},0.00,{turn_omega:.2f};"
            f"6.0,{straight_speed:.2f},0.00;"
            "1.0,0.00,0.00;"
            f"{half_turn_sec:.2f},0.00,{turn_omega:.2f};"
            "1.0,0.00,0.00;"
            f"6.0,{straight_speed:.2f},0.00;"
            f"{settle_sec:.2f},0.00,0.00;"
            f"{quarter_turn_sec:.2f},0.00,{-turn_omega:.2f};"
            f"6.0,{straight_speed:.2f},0.00;"
            f"{settle_sec:.2f},0.00,0.00;"
            f"{quarter_turn_sec:.2f},0.00,{-turn_omega:.2f};"
            f"6.0,{straight_speed:.2f},0.00;"
            f"{settle_sec:.2f},0.00,0.00;"
            f"{quarter_turn_sec:.2f},0.00,{-turn_omega:.2f};"
            f"6.0,{straight_speed:.2f},0.00;"
            "1.0,0.00,0.00"
        )
        raw_sequence = os.getenv("WHEEL_ID_PLAYER_SEQUENCE", default_sequence)
        self.sequence = _parse_sequence(raw_sequence)

        self.publisher = rospy.Publisher(self.cmd_topic, Twist2DStamped, queue_size=1)
        rospy.on_shutdown(self._publish_stop)

        rospy.loginfo("Wheel ID command player publishing to %s", self.cmd_topic)
        rospy.loginfo("Wheel ID command player repeat=%s publish_hz=%.2f", self.repeat, self.publish_hz)
        for index, (duration, v_cmd, omega_cmd) in enumerate(self.sequence):
            rospy.loginfo(
                "Phase %d: duration=%.2fs v=%.3f omega=%.3f",
                index,
                duration,
                v_cmd,
                omega_cmd,
            )

    def _publish(self, v_cmd: float, omega_cmd: float) -> None:
        self.publisher.publish(Twist2DStamped(v=v_cmd, omega=omega_cmd))

    def _publish_stop(self) -> None:
        try:
            self._publish(0.0, 0.0)
        except Exception:
            pass

    def run(self) -> None:
        rate = rospy.Rate(max(1.0, self.publish_hz))

        while not rospy.is_shutdown():
            for index, (duration, v_cmd, omega_cmd) in enumerate(self.sequence):
                phase_start = rospy.Time.now().to_sec()
                rospy.loginfo(
                    "Wheel ID command player entering phase %d with v=%.3f omega=%.3f for %.2fs",
                    index,
                    v_cmd,
                    omega_cmd,
                    duration,
                )

                while not rospy.is_shutdown():
                    elapsed = rospy.Time.now().to_sec() - phase_start
                    if elapsed >= duration:
                        break
                    self._publish(v_cmd, omega_cmd)
                    rate.sleep()

            self._publish_stop()
            if not self.repeat:
                rospy.loginfo("Wheel ID command player finished one sequence and is stopping.")
                return


if __name__ == "__main__":
    rospy.init_node("wheel_id_command_player")
    WheelIdCommandPlayer().run()
