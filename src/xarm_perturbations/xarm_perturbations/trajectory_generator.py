#!/usr/bin/env python3
"""
Lissajous Cartesian trajectory generator for xArm Lite 6.

Publishes the desired end-effector position as PointStamped on /desired_position.
The trajectory is a Lissajous figure — with nx=1, ny=2 (default) this traces
a figure-8 (infinity symbol) in the specified plane.

Parametric equations (in plane axes a, b):
  a(t) = rx * sin(nx * omega * t + phase)
  b(t) = ry * sin(ny * omega * t)
  omega = 2*pi * base_freq

A 2-second soft-start ramp prevents jerky motion at startup.

Example run (figure-8 in XY plane):
  ros2 run xarm_perturbations trajectory_generator --ros-args \\
    -p rx:=0.06 -p ry:=0.03 \\
    -p base_freq:=0.05 \\
    -p nx:=1.0 -p ny:=2.0 \\
    -p phase_deg:=90.0 \\
    -p plane:=xy

Architecture:
  trajectory_generator --> /desired_position --> position_controller
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class LissajousTrajectoryGenerator(Node):
    """Publishes a Lissajous-curve desired position at a fixed rate."""

    def __init__(self):
        super().__init__("lissajous_trajectory_generator")

        # ── Parameters ────────────────────────────────────────────
        self.rx = float(self.declare_parameter("rx", 0.06).value)
        self.ry = float(self.declare_parameter("ry", 0.03).value)
        self.base_freq = float(self.declare_parameter("base_freq", 0.05).value)
        self.nx = float(self.declare_parameter("nx", 1.0).value)
        self.ny = float(self.declare_parameter("ny", 2.0).value)
        self.phase_deg = float(self.declare_parameter("phase_deg", 90.0).value)
        self.plane = str(
            self.declare_parameter("plane", "xy").value
        ).lower().strip()
        topic = str(
            self.declare_parameter("desired_pos_topic", "/desired_position").value
        )
        rate_hz = float(self.declare_parameter("rate_hz", 50.0).value)

        self.phase_rad = math.radians(self.phase_deg)
        self.omega = 2.0 * math.pi * self.base_freq

        # ── State ─────────────────────────────────────────────────
        self.center: np.ndarray | None = None
        self.start_time = self.get_clock().now()
        self._last_warn = self.get_clock().now()

        # ── TF ────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Publisher ─────────────────────────────────────────────
        self.pub = self.create_publisher(PointStamped, topic, 10)

        # ── Timer ─────────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"LissajousTrajectoryGenerator ready\n"
            f"  topic   : {topic}\n"
            f"  rx={self.rx} m  ry={self.ry} m  base_freq={self.base_freq} Hz\n"
            f"  nx={self.nx}  ny={self.ny}  phase={self.phase_deg} deg  plane={self.plane}"
        )

    # ── TF helper ──────────────────────────────────────────────────
    def _read_tf(self) -> np.ndarray | None:
        try:
            t = self.tf_buffer.lookup_transform(
                "link_base", "link_eef", rclpy.time.Time()
            )
            return np.array(
                [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z,
                ],
                dtype=float,
            )
        except Exception as exc:
            now = self.get_clock().now()
            if (now - self._last_warn).nanoseconds > 2_000_000_000:
                self.get_logger().warn(f"TF not ready: {exc}")
                self._last_warn = now
            return None

    # ── Trajectory ─────────────────────────────────────────────────
    def _lissajous(self, t: float) -> np.ndarray:
        ramp = min(t / 2.0, 1.0)  # soft start: 0 → 1 over first 2 s
        cx, cy, cz = self.center

        a = ramp * self.rx * math.sin(self.nx * self.omega * t + self.phase_rad)
        b = ramp * self.ry * math.sin(self.ny * self.omega * t)

        if self.plane == "xy":
            return np.array([cx + a, cy + b, cz], dtype=float)
        elif self.plane == "xz":
            return np.array([cx + a, cy, cz + b], dtype=float)
        elif self.plane == "yz":
            return np.array([cx, cy + a, cz + b], dtype=float)
        else:
            return np.array([cx + a, cy + b, cz], dtype=float)

    # ── Main tick ──────────────────────────────────────────────────
    def _tick(self):
        # Wait until TF is available to initialise the center
        if self.center is None:
            p = self._read_tf()
            if p is None:
                return
            self.center = p.copy()
            self.start_time = self.get_clock().now()
            self.get_logger().info(f"Center initialised at {self.center.round(4)}")
            return

        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        target = self._lissajous(t)

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_base"
        msg.point.x = float(target[0])
        msg.point.y = float(target[1])
        msg.point.z = float(target[2])
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LissajousTrajectoryGenerator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
