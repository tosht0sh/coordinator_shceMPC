#!/usr/bin/env python3
from __future__ import annotations

"""Log wheel-speed identification data from the running bot stack.

This node subscribes to the final body command topic published to the Duckiebot
motor stack and records:

- published body command `(v_cmd, omega_cmd)`
- equivalent left/right wheel command `(wL_cmd, wR_cmd)`
- encoder-derived left/right wheel speeds, both raw and filtered

The resulting CSV is intended for offline first-order wheel identification.
"""

import csv
import os
import pathlib
import socket

import rospy
from duckietown_msgs.msg import Twist2DStamped

try:
    from .PI_control_motors import TickToVelocity
except ImportError:
    from PI_control_motors import TickToVelocity


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


class TcpCsvSender:
    """Best-effort TCP CSV streamer for host-side logging."""

    def __init__(self, header: list[str]) -> None:
        self.host = "192.168.1.10" #os.getenv("WHEEL_ID_TCP_HOST", "").strip()
        self.port = "5015" #int(os.getenv("WHEEL_ID_TCP_PORT", "5015"))
        self.timeout = float(os.getenv("WHEEL_ID_TCP_TIMEOUT", "1.0"))
        self.retry_sec = float(os.getenv("WHEEL_ID_TCP_RETRY_SEC", "2.0"))
        self.enabled = bool(self.host) and _env_bool("WHEEL_ID_TCP_ENABLE", True)
        self._sock: socket.socket | None = None
        self._last_attempt = 0.0
        self._header_line = ",".join(header) + "\n"

    def _disconnect(self) -> None:
        if self._sock is None:
            return
        try:
            self._sock.close()
        except OSError:
            pass
        self._sock = None

    def _maybe_connect(self, now: float) -> None:
        if not self.enabled or self._sock is not None:
            return
        if now - self._last_attempt < self.retry_sec:
            return

        self._last_attempt = now
        try:
            sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
            try:
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass
            sock.sendall(self._header_line.encode("utf-8"))
            self._sock = sock
            rospy.loginfo("Wheel ID TCP connected to %s:%d", self.host, self.port)
        except OSError as exc:
            rospy.logwarn_throttle(5.0, "Wheel ID TCP connect failed: %s", exc)

    def send_row(self, row: list[object], now: float) -> None:
        if not self.enabled:
            return

        self._maybe_connect(now)
        if self._sock is None:
            return

        payload = (",".join(str(value) for value in row) + "\n").encode("utf-8")
        try:
            self._sock.sendall(payload)
        except OSError as exc:
            rospy.logwarn_throttle(5.0, "Wheel ID TCP send failed: %s", exc)
            self._disconnect()

    def close(self) -> None:
        self._disconnect()


class WheelIdLogger:
    """Capture final commands and wheel-speed measurements to CSV."""

    def __init__(self) -> None:
        self.vehicle_name = os.environ.get("VEHICLE_NAME", "duckiebot")
        self.cmd_topic = os.getenv(
            "WHEEL_ID_CMD_TOPIC",
            f"/{self.vehicle_name}/car_cmd_switch_node/cmd",
        )

        # User-requested hardcoded host-side path.
        default_dir = os.getenv(
            "WHEEL_ID_LOG_DIR",
            "/mnt/intenso/code/dev/coordinator_shceMPC/tools",
        )
        self.estimator = TickToVelocity()
        self.t0: float | None = None
        self.sample_count = 0
        self.csv_header = [
            "t",
            "v_cmd",
            "omega_cmd",
            "wL_cmd",
            "wR_cmd",
            "wL_meas",
            "wR_meas",
            "wL_raw",
            "wR_raw",
            "wheel_radius",
            "axle_length",
            "enc_ready",
        ]
        self.tcp_sender = TcpCsvSender(self.csv_header)
        self.csv_file = None
        self.writer = None

        self.output_dir = pathlib.Path(default_dir).expanduser()
        stamp = rospy.Time.now().to_sec()
        filename = f"wheel_id_{self.vehicle_name}_{int(stamp)}.csv"
        self.csv_path = self.output_dir / filename
        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            self.csv_file = self.csv_path.open("w", newline="")
            self.writer = csv.writer(self.csv_file)
            self.writer.writerow(self.csv_header)
            self.csv_file.flush()
        except OSError as exc:
            self.csv_file = None
            self.writer = None
            rospy.logwarn("Wheel ID local CSV disabled: %s", exc)

        rospy.Subscriber(
            self.cmd_topic,
            Twist2DStamped,
            self._on_cmd,
            queue_size=100,
        )
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("Wheel ID logger listening on %s", self.cmd_topic)
        if self.writer is not None:
            rospy.loginfo("Wheel ID CSV path: %s", str(self.csv_path))
        if self.tcp_sender.enabled:
            rospy.loginfo(
                "Wheel ID TCP streaming enabled for %s:%s",
                self.tcp_sender.host,
                self.tcp_sender.port,
            )
        print(",".join(self.csv_header), flush=True)

    def _on_cmd(self, msg: Twist2DStamped) -> None:
        now = rospy.Time.now().to_sec()
        if self.t0 is None:
            self.t0 = now

        v_cmd = float(msg.v)
        omega_cmd = float(msg.omega)

        w_left_meas, w_right_meas = self.estimator.get_filtered_wheels()
        w_left_raw = float(self.estimator.w_raw["L"])
        w_right_raw = float(self.estimator.w_raw["R"])

        wheel_radius = float(self.estimator.wheel_radius)
        axle_length = float(self.estimator.axis_length)
        w_left_cmd = (v_cmd - 0.5 * axle_length * omega_cmd) / wheel_radius
        w_right_cmd = (v_cmd + 0.5 * axle_length * omega_cmd) / wheel_radius

        row = [
            now - self.t0,
            v_cmd,
            omega_cmd,
            w_left_cmd,
            w_right_cmd,
            w_left_meas,
            w_right_meas,
            w_left_raw,
            w_right_raw,
            wheel_radius,
            axle_length,
            int(self.estimator.is_ready()),
        ]
        if self.writer is not None and self.csv_file is not None:
            self.writer.writerow(row)
            self.csv_file.flush()
        self.sample_count += 1
        print(",".join(str(value) for value in row), flush=True)
        self.tcp_sender.send_row(row, now)

        rospy.loginfo_throttle(
            2.0,
            "Wheel ID logger samples=%d encoder_ready=%s",
            self.sample_count,
            self.estimator.is_ready(),
        )

    def _on_shutdown(self) -> None:
        self.tcp_sender.close()
        try:
            if self.csv_file is not None:
                self.csv_file.close()
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("wheel_id_logger")
    WheelIdLogger()
    rospy.spin()
