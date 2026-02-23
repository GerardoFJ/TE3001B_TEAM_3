# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult


# Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('sp_gen')

        # Declare parameters
        self.declare_parameter('amplitude', 2.0)
        self.declare_parameter('omega', 1.0)
        self.declare_parameter('timer_period', 0.1)
        # signal_type: 'sine' or 'square'
        self.declare_parameter('signal_type', 'sine')

        # Get parameters
        self.amplitude = self.get_parameter('amplitude').value
        self.omega = self.get_parameter('omega').value
        self.timer_period = self.get_parameter('timer_period').value
        self.signal_type = self.get_parameter('signal_type').value

        # Publisher and timer
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        # Message and time reference
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info(f'SetPoint Node Started \U0001F680 | signal_type: {self.signal_type}')

    # Timer Callback: Generate and publish set point signal
    def timer_cb(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if self.signal_type == 'square':
            # Square wave via sign of sine
            self.signal_msg.data = float(self.amplitude * np.sign(np.sin(self.omega * elapsed_time)))
        else:
            # Default: sine wave
            self.signal_msg.data = float(self.amplitude * np.sin(self.omega * elapsed_time))

        self.signal_publisher.publish(self.signal_msg)

    # Parameter update callback
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'amplitude':
                self.amplitude = param.value
                self.get_logger().info(f'amplitude updated to {self.amplitude}')

            elif param.name == 'omega':
                if param.value <= 0.0:
                    self.get_logger().warn('Invalid omega! It must be positive.')
                    return SetParametersResult(successful=False, reason='omega must be positive')
                self.omega = param.value
                self.get_logger().info(f'omega updated to {self.omega}')

            elif param.name == 'signal_type':
                if param.value not in ('sine', 'square'):
                    self.get_logger().warn(f'Invalid signal_type "{param.value}"! Use "sine" or "square".')
                    return SetParametersResult(successful=False, reason='signal_type must be "sine" or "square"')
                self.signal_type = param.value
                self.get_logger().info(f'signal_type updated to {self.signal_type}')

        return SetParametersResult(successful=True)


# Main
def main(args=None):
    rclpy.init(args=args)

    set_point = SetPointPublisher()

    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()


# Execute Node
if __name__ == '__main__':
    main()
