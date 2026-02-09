#!/usr/bin/env python3
import os, math
import rospy
from duckietown_msgs.msg import WheelEncoderStamped

class TickToVelocity:
    def __init__(self):
        self.veh = os.environ.get("VEHICLE_NAME", "duckiebot")

        # Pull DB21 kinematics from ROS params (recommended)
        self.r = rospy.get_param(f"/{self.veh}/kinematics_node/radius")     # wheel radius [m]
        self.b = rospy.get_param(f"/{self.veh}/kinematics_node/baseline")   # baseline [m]

        # radius_param = f"/{self.veh}/kinematics_node/radius"
        # baseline_param = f"/{self.veh}/kinematics_node/baseline"

        # self.r = rospy.get_param(radius_param, 0.0318)   # fallback default
        # self.b = rospy.get_param(baseline_param, 0.10)   # fallback default

        # LPF time constant (seconds). Tune: 0.1 (responsive) ... 0.3 (smooth)
        self.tau = rospy.get_param("~lpf_tau", 0.2)

        # previous samples for tick differentiation
        self.prev = {
            "L": {"t": None, "ticks": None, "N": None},
            "R": {"t": None, "ticks": None, "N": None},
        }
        self.w = {"L": 0.0, "R": 0.0}  # wheel angular speeds [rad/s]

        # Raw and filtered robot velocities
        self.v_raw = 0.0
        self.wz_raw = 0.0
        self.v_filt = 0.0
        self.wz_filt = 0.0
        self._t_last_vel = None  # last time we updated v/wz (for dt in filter)

        rospy.Subscriber(f"/{self.veh}/left_wheel_encoder_node/tick",
                         WheelEncoderStamped, self.cb_left, queue_size=10)
        rospy.Subscriber(f"/{self.veh}/right_wheel_encoder_node/tick",
                         WheelEncoderStamped, self.cb_right, queue_size=10)

    def _update(self, side, msg):
        t = msg.header.stamp.to_sec()
        if t == 0.0:
            t = rospy.Time.now().to_sec()

        ticks = int(msg.data)
        N = int(msg.resolution)  # ticks per revolution

        p = self.prev[side]
        if p["t"] is not None:
            dt = t - p["t"]
            if dt > 1e-4:
                d_ticks = ticks - p["ticks"]
                self.w[side] = 2.0 * math.pi * (d_ticks / float(N)) / dt

        self.prev[side] = {"t": t, "ticks": ticks, "N": N}

    def cb_left(self, msg):  self._update("L", msg)
    def cb_right(self, msg): self._update("R", msg)

    def _update_robot_velocity_and_filter(self):
        """
        Compute raw (v, wz) from current wheel speeds, then low-pass filter them.
        Filter: y = y + alpha*(x - y), alpha = dt/(tau + dt)
        """
        now = rospy.Time.now().to_sec()

        # Compute raw robot velocity from current wheel angular speeds
        wL, wR = self.w["L"], self.w["R"]
        vL, vR = self.r * wL, self.r * wR
        self.v_raw = 0.5 * (vR + vL)       # [m/s]
        self.wz_raw = (vR - vL) / self.b   # [rad/s]

        # Initialize filter time on first run
        if self._t_last_vel is None:
            self._t_last_vel = now
            self.v_filt = self.v_raw
            self.wz_filt = self.wz_raw
            return

        dt = now - self._t_last_vel
        self._t_last_vel = now
        if dt <= 1e-4:
            return

        # EMA / 1st-order LPF coefficient
        alpha = dt / (self.tau + dt)

        # Filter v and wz
        self.v_filt  = self.v_filt  + alpha * (self.v_raw  - self.v_filt)
        self.wz_filt = self.wz_filt + alpha * (self.wz_raw - self.wz_filt)

    def spin(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self._update_robot_velocity_and_filter()

            rospy.loginfo_throttle(
                0.2,
                f"raw:  v={self.v_raw:+.3f} m/s, wz={self.wz_raw:+.3f} rad/s | "
                f"filt: v={self.v_filt:+.3f} m/s, wz={self.wz_filt:+.3f} rad/s"
            )
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("tick_to_velocity")
    TickToVelocity().spin()
