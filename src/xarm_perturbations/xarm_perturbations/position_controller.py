#!/usr/bin/env python3
"""
PD / PID Cartesian position controller for xArm Lite 6 via MoveIt Servo.

Architecture
------------
  trajectory_generator  -->  /desired_position  -->  position_controller
                                                            |
                                                  TwistStamped (output_topic)
                                                            |
                                        [baseline]  /servo_server/delta_twist_cmds
                                     OR [perturb]   /controller_output  --> perturbation_injector

Controller
----------
  v = Kp * e  +  Kd * ef_dot  +  Ki * integral(e dt)

  where ef_dot is the FILTERED derivative (first-order low-pass, cutoff derivative_filter_freq Hz):
    ef_dot[k] = alpha * e_dot_raw[k]  +  (1-alpha) * ef_dot[k-1]
    alpha      = dt / (tau_d + dt),   tau_d = 1 / (2*pi*fc)

  This attenuates high-frequency noise (Gaussian) while keeping responsiveness
  to trajectory-frequency errors.

  Additional features:
    * per-axis deadband         (no command if |e_i| < epsilon)
    * per-axis velocity sat.    |v_i| <= max_speed
    * integral anti-windup      (clamp integral contribution to +/- integral_max)
    * dt guard                  (clamp dt to [1 us, 100 ms])

Recommended gains (tuned from experimental data — fixed across all experiments):

  Gain choice rationale:
    Kp 2.5 -> 4.0  : faster recovery from perturbation-induced position offsets
    Kd 0.6 -> 0.3  : lower because derivative filter handles noise attenuation
    Ki 0.0 -> 0.2  : eliminates +1.75 cm bias observed in sine/gaussian experiments
    fc = 8 Hz      : passes trajectory (0.05-0.10 Hz), blocks Gaussian noise (> 8 Hz)

Example — baseline:
  ros2 run xarm_perturbations position_controller --ros-args \\
    -p output_topic:=/servo_server/delta_twist_cmds \\
    -p kp:="[4.0, 4.0, 4.0]" -p kd:="[0.3, 0.3, 0.3]" \\
    -p ki:="[0.2, 0.2, 0.2]" \\
    -p derivative_filter_freq:=8.0 \\
    -p max_speed:=0.12 -p deadband:=0.001

Example — perturbation (controller feeds perturbation_injector):
  ros2 run xarm_perturbations position_controller --ros-args \\
    -p output_topic:=/controller_output \\
    -p kp:="[4.0, 4.0, 4.0]" -p kd:="[0.3, 0.3, 0.3]" \\
    -p ki:="[0.2, 0.2, 0.2]" \\
    -p derivative_filter_freq:=8.0 \\
    -p max_speed:=0.12 -p deadband:=0.001
"""
import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class PositionController(Node):

    def __init__(self):
        super().__init__("position_controller")

        # ── Parameters ────────────────────────────────────────────
        desired_topic = str(
            self.declare_parameter("desired_pos_topic", "/desired_position").value
        )
        output_topic = str(
            self.declare_parameter("output_topic", "/servo_server/delta_twist_cmds").value
        )

        kp = list(self.declare_parameter("kp", [4.0, 4.0, 4.0]).value)
        kd = list(self.declare_parameter("kd", [0.15, 0.15, 0.15]).value)
        ki = list(self.declare_parameter("ki", [0.2, 0.2, 0.2]).value)

        self.kp = np.array(kp, dtype=float)
        self.kd = np.array(kd, dtype=float)
        self.ki = np.array(ki, dtype=float)

        self.max_speed = float(self.declare_parameter("max_speed", 0.12).value)
        deadband = float(self.declare_parameter("deadband", 0.001).value)
        self.epsilon = np.full(3, deadband, dtype=float)

        # Anti-windup: integral contribution clamped to +/- integral_max
        self.integral_max = float(self.declare_parameter("integral_max", 0.05).value)

        # Derivative low-pass filter cutoff (Hz)
        # tau_d = 1 / (2*pi*fc)  =>  alpha = dt/(tau_d + dt)
        fc = float(self.declare_parameter("derivative_filter_freq", 8.0).value)
        self.tau_d = 1.0 / (2.0 * math.pi * fc)

        # ── State ─────────────────────────────────────────────────
        self.desired_pos: Optional[np.ndarray] = None
        self.prev_error = np.zeros(3, dtype=float)
        self.integral = np.zeros(3, dtype=float)
        self.d_filtered = np.zeros(3, dtype=float)   # low-pass filtered derivative
        self.prev_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()

        # ── TF ────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Publishers ────────────────────────────────────────────
        self.servo_pub = self.create_publisher(TwistStamped, output_topic, 10)
        self.actual_pos_pub = self.create_publisher(PointStamped, "/actual_position", 10)

        # ── Subscription ──────────────────────────────────────────
        self.create_subscription(PointStamped, desired_topic, self._desired_cb, 10)

        # ── Control loop at 50 Hz ─────────────────────────────────
        self.timer = self.create_timer(0.02, self._loop)

        self.get_logger().info(
            f"PositionController ready\n"
            f"  desired_topic : {desired_topic}\n"
            f"  output_topic  : {output_topic}\n"
            f"  kp={list(self.kp)}  kd={list(self.kd)}  ki={list(self.ki)}\n"
            f"  max_speed={self.max_speed}  deadband={deadband}\n"
            f"  derivative_filter_freq={fc} Hz  (tau={self.tau_d*1000:.1f} ms)"
        )

    # ── Callbacks ──────────────────────────────────────────────────
    def _desired_cb(self, msg: PointStamped):
        self.desired_pos = np.array(
            [msg.point.x, msg.point.y, msg.point.z], dtype=float
        )

    # ── Helpers ────────────────────────────────────────────────────
    def _read_pose(self) -> Optional[np.ndarray]:
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
        except Exception:
            return None

    def _publish_twist(self, v: np.ndarray):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_base"
        msg.twist.linear.x = float(v[0])
        msg.twist.linear.y = float(v[1])
        msg.twist.linear.z = float(v[2])
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.servo_pub.publish(msg)

    def _publish_actual(self, pos: np.ndarray):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_base"
        msg.point.x = float(pos[0])
        msg.point.y = float(pos[1])
        msg.point.z = float(pos[2])
        self.actual_pos_pub.publish(msg)

    # ── Control loop ───────────────────────────────────────────────
    def _loop(self):
        if self.desired_pos is None:
            return

        current = self._read_pose()
        if current is None:
            return

        self._publish_actual(current)

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        dt = max(dt, 1e-6)
        dt = min(dt, 0.10)

        error = self.desired_pos - current

        # Filtered derivative: first-order low-pass on d_error/dt
        # alpha = 1 means no filter; alpha < 1 attenuates high-freq noise
        alpha = dt / (self.tau_d + dt)
        d_raw = (error - self.prev_error) / dt
        self.d_filtered = alpha * d_raw + (1.0 - alpha) * self.d_filtered

        # Integral with anti-windup clamp
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_max, self.integral_max)

        # PID command with filtered derivative
        v = self.kp * error + self.kd * self.d_filtered + self.ki * self.integral

        # Deadband
        v = np.where(np.abs(error) > self.epsilon, v, 0.0)

        # Velocity saturation
        v = np.clip(v, -self.max_speed, self.max_speed)

        self._publish_twist(v)

        self.prev_error = error.copy()
        self.prev_time = now

        if (now - self.last_log_time).nanoseconds > 1_000_000_000:
            err_norm = float(np.linalg.norm(error))
            self.get_logger().info(
                f"pos={current.round(3)}  tgt={self.desired_pos.round(3)}"
                f"  |err|={err_norm:.4f}  v={v.round(3)}"
            )
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
