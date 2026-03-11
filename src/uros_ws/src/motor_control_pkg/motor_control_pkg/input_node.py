# !/usr/bin/env python3
# ros2 run motor_control_pkg input_node --ros-args --params-file ~/ros2_ws/src/motor_control_pkg/config/input_params.yaml

import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')

        # =========================
        # Parameters
        # =========================
        self.declare_parameter('signal_type', 'step')      # step | square | sine
        self.declare_parameter('publish_rate', 100.0)      # Hz

        # Common parameters
        self.declare_parameter('amplitude', 30.0)          # signal amplitude
        self.declare_parameter('offset', 0.0)              # DC offset
        self.declare_parameter('min_output', -1.0)       # clamp lower bound
        self.declare_parameter('max_output', 1.0)        # clamp upper bound

        # Step parameters
        self.declare_parameter('step_time', 1.0)           # seconds
        self.declare_parameter('step_initial', 0.0)        # value before step
        self.declare_parameter('step_final', 30.0)         # value after step

        # Square / sine parameters
        self.declare_parameter('frequency', 0.2)           # Hz
        self.declare_parameter('phase', 0.0)               # radians

        # Square-specific
        self.declare_parameter('duty_cycle', 0.5)          # 0 to 1

        # Optional duration
        self.declare_parameter('duration', -1.0)           # seconds, -1 => infinite

        # =========================
        # Read parameters
        # =========================
        self.signal_type = self.get_parameter('signal_type').get_parameter_value().string_value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.amplitude = float(self.get_parameter('amplitude').value)
        self.offset = float(self.get_parameter('offset').value)
        self.min_output = float(self.get_parameter('min_output').value)
        self.max_output = float(self.get_parameter('max_output').value)

        self.step_time = float(self.get_parameter('step_time').value)
        self.step_initial = float(self.get_parameter('step_initial').value)
        self.step_final = float(self.get_parameter('step_final').value)

        self.frequency = float(self.get_parameter('frequency').value)
        self.phase = float(self.get_parameter('phase').value)
        self.duty_cycle = float(self.get_parameter('duty_cycle').value)

        self.duration = float(self.get_parameter('duration').value)

        # =========================
        # ROS interfaces
        # =========================
        self.publisher_ = self.create_publisher(Float32, '/set_point', 10)

        if self.publish_rate <= 0.0:
            self.publish_rate = 100.0

        self.dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.t0 = self.get_clock().now()
        self.last_log_time = 0.0

        self.get_logger().info('Input node started')
        self.get_logger().info(f'signal_type = {self.signal_type}')
        self.get_logger().info(f'publish_rate = {self.publish_rate:.2f} Hz')

    def clamp(self, value: float) -> float:
        return float(np.clip(value, self.min_output, self.max_output))

    def elapsed_time(self) -> float:
        now = self.get_clock().now()
        return (now - self.t0).nanoseconds * 1e-9

    def generate_step(self, t: float) -> float:
        if t < self.step_time:
            return self.step_initial
        return self.step_final

    def generate_square(self, t: float) -> float:
        if self.frequency <= 0.0:
            return self.offset

        period = 1.0 / self.frequency
        t_shifted = t + self.phase / (2.0 * math.pi * self.frequency)
        tau = t_shifted % period

        high_time = self.duty_cycle * period
        if tau < high_time:
            return self.offset + self.amplitude
        return self.offset - self.amplitude

    def generate_sine(self, t: float) -> float:
        return self.offset + self.amplitude * math.sin(2.0 * math.pi * self.frequency * t + self.phase)

    def compute_signal(self, t: float) -> float:
        signal_type = self.signal_type.lower()

        if signal_type == 'step':
            y = self.generate_step(t)
        elif signal_type == 'square':
            y = self.generate_square(t)
        elif signal_type == 'sine':
            y = self.generate_sine(t)
        else:
            self.get_logger().warn(
                f'Unknown signal_type "{self.signal_type}". Using step.'
            )
            y = self.generate_step(t)

        return self.clamp(y)

    def timer_callback(self):
        t = self.elapsed_time()

        # If duration is defined and exceeded, keep publishing zero
        if self.duration > 0.0 and t > self.duration:
            value = 0.0
        else:
            value = self.compute_signal(t)

        msg = Float32()
        msg.data = float(value)
        self.publisher_.publish(msg)

        # Debug log every ~0.5 s
        if (t - self.last_log_time) >= 0.5:
            self.last_log_time = t
            self.get_logger().info(f't={t:.2f} s | set_point={value:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = InputNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()