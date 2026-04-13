#!/usr/bin/env python3
"""Manual command passthrough with wheel-speed PI correction.

Use this node to tune the low-level wheel PI without the MPC in the loop.
Publish a manual `Twist2DStamped` command to the input topic and the node will
apply the same wheel-speed PI correction that `bot_mpc_node` uses before
forwarding the command to the robot.
"""

from __future__ import annotations

import os
from typing import Optional

import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped

try:
    from .PI_control_motors import PI
except ImportError:
    from PI_control_motors import PI


class ManualPiNode(DTROS):
    """Forward manual body commands through the wheel PI helper."""

    def __init__(self, node_name: str):
        super(ManualPiNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        self.vehicle_name = os.environ["VEHICLE_NAME"]
        topic_template = os.getenv(
            "MANUAL_PI_INPUT_TOPIC",
            f"/{self.vehicle_name}/joy_mapper_node/car_cmd",
        )
        self.input_topic = (
            topic_template.replace("{vehicle}", self.vehicle_name)
            .replace("{VEHICLE_NAME}", self.vehicle_name)
        )
        self.output_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
        self.cmd_timeout = float(os.getenv("MANUAL_PI_TIMEOUT", "0.5"))
        self.publish_hz = float(os.getenv("MANUAL_PI_HZ", "20.0"))

        self.pi_controller = PI()
        self.cmd_pub = rospy.Publisher(self.output_topic, Twist2DStamped, queue_size=1)
        self.cmd_sub = rospy.Subscriber(self.input_topic, Twist2DStamped, self._on_manual_cmd, queue_size=10)

        self._latest_cmd: Optional[tuple[float, float]] = None
        self._latest_cmd_rx_time: Optional[float] = None
        self._seen_first_cmd = False

        self.loginfo(f"Reading manual commands from {self.input_topic}")
        self.loginfo(f"Publishing corrected commands to {self.output_topic}")
        self.loginfo(f"Waiting for Twist2DStamped input on {self.input_topic}")

    def _on_manual_cmd(self, msg: Twist2DStamped) -> None:
        self._latest_cmd = (float(msg.v), float(msg.omega))
        self._latest_cmd_rx_time = rospy.get_time()
        if not self._seen_first_cmd:
            self._seen_first_cmd = True
            self.loginfo(
                f"First manual command received on {self.input_topic}: "
                f"v={self._latest_cmd[0]:.3f}, omega={self._latest_cmd[1]:.3f}"
            )

    def _publish_twist(self, v: float, omega: float) -> None:
        self.cmd_pub.publish(Twist2DStamped(v=v, omega=omega))

    def run(self) -> None:
        rate = rospy.Rate(self.publish_hz)
        dt = 1.0 / max(self.publish_hz, 1.0)

        while not rospy.is_shutdown():
            if self._latest_cmd is None or self._latest_cmd_rx_time is None:
                rospy.loginfo_throttle(
                    2.0,
                    "Manual PI waiting for commands on %s (connections=%d)",
                    self.input_topic,
                    self.cmd_sub.get_num_connections(),
                )
                self._publish_twist(0.0, 0.0)
                rate.sleep()
                continue

            age = rospy.get_time() - self._latest_cmd_rx_time
            if age > self.cmd_timeout:
                rospy.logwarn_throttle(
                    2.0,
                    "Manual PI command stream stale: age=%.3fs timeout=%.3fs. Publishing stop.",
                    age,
                    self.cmd_timeout,
                )
                self.pi_controller.reset()
                self._publish_twist(0.0, 0.0)
                rate.sleep()
                continue

            v_ref, omega_ref = self._latest_cmd
            if self.pi_controller.is_ready():
                v_cmd, omega_cmd = self.pi_controller.pi_controller(v_ref, omega_ref, dt)
            else:
                rospy.loginfo_throttle(2.0, "Manual PI waiting for valid encoder updates; publishing raw command.")
                v_cmd, omega_cmd = v_ref, omega_ref

            self._publish_twist(v_cmd, omega_cmd)
            rospy.loginfo_throttle(
                1.0,
                "manual_pi ref=(%.3f, %.3f) cmd=(%.3f, %.3f)",
                v_ref,
                omega_ref,
                v_cmd,
                omega_cmd,
            )
            rate.sleep()

    def on_shutdown(self) -> None:
        self.pi_controller.reset()
        self._publish_twist(0.0, 0.0)


if __name__ == "__main__":
    node = ManualPiNode(node_name="manual_pi_node")
    node.run()
