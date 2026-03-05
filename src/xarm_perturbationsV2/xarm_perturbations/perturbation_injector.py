#!/usr/bin/env python3
"""
Perturbation injector for xArm Lite 6 servo experiments.

Modes
-----
  standalone (default):
    Publishes  base_linear + perturbation  directly to output_topic.
    Use this to test the injector in isolation or when no position
    controller is running.

  relay (input_topic != ""):
    Subscribes to input_topic (TwistStamped, e.g. /controller_output),
    adds the perturbation signal to the received linear velocity, and
    republishes to output_topic.
    Use this to inject disturbances into a running position controller.

Perturbation types (mode parameter)
-------------------------------------
  off       -- no perturbation (pure relay / pure base_linear)
  sine      -- sinusoidal disturbance on sine_axis
  gaussian  -- zero-mean Gaussian noise on gauss_axis (or all axes)

Example -- standalone sine perturbation:
  ros2 run xarm_perturbations perturbation_injector --ros-args \\
    -p output_topic:=/servo_server/delta_twist_cmds \\
    -p pub_reliability:=reliable \\
    -p enabled:=true \\
    -p mode:=sine \\
    -p sine_freq_hz:=1.0 \\
    -p sine_amp_linear:=0.01 \\
    -p sine_axis:=x \\
    -p base_linear:="[0.02, -0.02, 0.02]" \\
    -p debug:=true

Example -- relay Gaussian perturbation (controller feeds injector):
  ros2 run xarm_perturbations perturbation_injector --ros-args \\
    -p input_topic:=/controller_output \\
    -p output_topic:=/servo_server/delta_twist_cmds \\
    -p pub_reliability:=reliable \\
    -p enabled:=true \\
    -p mode:=gaussian \\
    -p gauss_std_linear:=0.01 \\
    -p gauss_axis:=x \\
    -p debug:=true
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


class PerturbationGenerator(Node):
    def __init__(self):
        super().__init__("xarm_perturbation_injector")

        # ── Output / mode parameters ──────────────────────────────
        self.output_topic = str(
            self.declare_parameter("output_topic", "/servo_server/delta_twist_cmds").value
        )
        self.enabled = bool(self.declare_parameter("enabled", True).value)
        self.mode = str(self.declare_parameter("mode", "sine").value).lower().strip()

        self.publish_period_s = float(self.declare_parameter("publish_period_s", 0.02).value)
        self.max_lin = float(self.declare_parameter("max_linear_speed", 0.20).value)

        # ── Sine parameters ───────────────────────────────────────
        self.sine_freq_hz = float(self.declare_parameter("sine_freq_hz", 8.0).value)
        self.sine_amp_linear = float(self.declare_parameter("sine_amp_linear", 0.02).value)
        self.sine_axis = str(
            self.declare_parameter("sine_axis", "x").value
        ).lower().strip()

        # ── Gaussian parameters ───────────────────────────────────
        self.gauss_std_linear = float(
            self.declare_parameter("gauss_std_linear", 0.01).value
        )
        self.gauss_axis = str(
            self.declare_parameter("gauss_axis", "all").value
        ).lower().strip()

        # ── Standalone base velocity (used when NOT in relay mode) ─
        base = self.declare_parameter("base_linear", [0.0, 0.0, 0.0]).value
        try:
            self.base_linear = np.array(
                [float(base[0]), float(base[1]), float(base[2])], dtype=float
            )
        except Exception:
            self.base_linear = np.zeros(3, dtype=float)

        # ── Relay mode ────────────────────────────────────────────
        input_topic = str(self.declare_parameter("input_topic", "").value).strip()
        self.relay_mode = bool(input_topic)
        self.last_relay_v = np.zeros(3, dtype=float)

        # ── Debug ─────────────────────────────────────────────────
        self.debug = bool(self.declare_parameter("debug", True).value)
        self.debug_period_s = float(self.declare_parameter("debug_period_s", 1.0).value)
        self._out_count = 0
        self._last_dbg_wall = time.time()

        # ── QoS for publisher ─────────────────────────────────────
        qos_name = str(
            self.declare_parameter("pub_reliability", "reliable").value
        ).lower().strip()
        reliability = (
            ReliabilityPolicy.RELIABLE
            if qos_name in ("reliable", "r")
            else ReliabilityPolicy.BEST_EFFORT
        )
        self.pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(TwistStamped, self.output_topic, self.pub_qos)

        # ── Relay subscription ────────────────────────────────────
        if self.relay_mode:
            self.create_subscription(TwistStamped, input_topic, self._relay_cb, 10)

        self.t0 = time.time()
        self.rng = np.random.default_rng(7)

        self.timer = self.create_timer(self.publish_period_s, self.tick)

        self.get_logger().info(
            "xarm_perturbation_injector ready\n"
            f"   OUT: {self.output_topic}\n"
            f"   mode={self.mode}, enabled={self.enabled}\n"
            f"   relay_mode={self.relay_mode}"
            + (f", input_topic={input_topic}" if self.relay_mode else "") + "\n"
            f"   PUB reliability="
            f"{'RELIABLE' if reliability == ReliabilityPolicy.RELIABLE else 'BEST_EFFORT'}"
        )

    # ── Relay callback ─────────────────────────────────────────────
    def _relay_cb(self, msg: TwistStamped):
        self.last_relay_v = np.array(
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
            dtype=float,
        )

    # ── Perturbation signal ────────────────────────────────────────
    def _dp(self) -> np.ndarray:
        if self.mode == "off":
            return np.zeros(3, dtype=float)

        if self.mode == "gaussian":
            dp = np.zeros(3, dtype=float)
            if self.gauss_axis == "all":
                dp = self.rng.normal(0.0, self.gauss_std_linear, size=3)
            else:
                axis_idx = {"x": 0, "y": 1, "z": 2}.get(self.gauss_axis, 0)
                dp[axis_idx] = float(self.rng.normal(0.0, self.gauss_std_linear))
            return dp

        # sine (default)
        s = math.sin(2.0 * math.pi * self.sine_freq_hz * (time.time() - self.t0))
        dp = np.zeros(3, dtype=float)
        axis_idx = {"x": 0, "y": 1, "z": 2}.get(self.sine_axis, 0)
        dp[axis_idx] = self.sine_amp_linear * s
        return dp

    # ── Timer callback ─────────────────────────────────────────────
    def tick(self):
        if not self.enabled:
            self._publish(np.zeros(3, dtype=float), note="DISABLED")
            return

        base = self.last_relay_v if self.relay_mode else self.base_linear
        v = np.clip(base + self._dp(), -self.max_lin, self.max_lin)
        self._publish(v, note="RUN")

    def _publish(self, v_xyz: np.ndarray, note: str = ""):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "link_base"
        out.twist.linear.x = float(v_xyz[0])
        out.twist.linear.y = float(v_xyz[1])
        out.twist.linear.z = float(v_xyz[2])
        out.twist.angular.x = 0.0
        out.twist.angular.y = 0.0
        out.twist.angular.z = 0.0
        self.pub.publish(out)
        self._out_count += 1

        if self.debug:
            now = time.time()
            if now - self._last_dbg_wall >= self.debug_period_s:
                self.get_logger().info(
                    f"[dbg] out={self._out_count} {note} v={np.round(v_xyz, 3)}"
                )
                self._last_dbg_wall = now


def main():
    rclpy.init()
    n = PerturbationGenerator()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
