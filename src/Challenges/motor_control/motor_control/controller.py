# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult


# Class Definition
class PIDController(Node):
    def __init__(self):
        super().__init__('ctrl')

        # Declare parameters
        self.declare_parameter('gain_Kp', 0.2)
        self.declare_parameter('gain_Ki', 2.0)
        self.declare_parameter('gain_Kd', 0.0)
        self.declare_parameter('sample_time', 0.01)

        # Get parameters
        self.Kp = self.get_parameter('gain_Kp').value
        self.Ki = self.get_parameter('gain_Ki').value
        self.Kd = self.get_parameter('gain_Kd').value
        self.sample_time = self.get_parameter('sample_time').value

        # Control variables
        self.set_point = 0.0
        self.motor_output = 0.0
        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0

        # Message
        self.control_msg = Float32()

        # Publishers, subscribers and timer
        self.ctrl_pub = self.create_publisher(Float32, 'motor_input_u', 10)
        self.sp_sub = self.create_subscription(Float32, 'set_point', self.sp_callback, 10)
        self.motor_sub = self.create_subscription(Float32, 'motor_speed_y', self.motor_callback, 10)
        self.timer = self.create_timer(self.sample_time, self.timer_cb)

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info('Controller Node Started \U0001F680')

    # Set point subscriber callback
    def sp_callback(self, msg):
        self.set_point = msg.data

    # Motor output subscriber callback
    def motor_callback(self, msg):
        self.motor_output = msg.data

    # Timer callback: compute and publish control signal
    def timer_cb(self):
        # Compute error: e[k] = sp[k] - y[k]
        self.error = self.set_point - self.motor_output

        # Discrete PID (position form):
        # u[k] = Kp*e[k] + Ki*Ts*sum(e) + Kd/Ts*(e[k] - e[k-1])
        self.integral += self.error * self.sample_time
        derivative = (self.error - self.prev_error) / self.sample_time

        control_u = (self.Kp * self.error +
                     self.Ki * self.integral +
                     self.Kd * derivative)

        # Publish control signal
        self.control_msg.data = control_u
        self.ctrl_pub.publish(self.control_msg)

        # Update previous error
        self.prev_error = self.error

    # Parameter update callback
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'gain_Kp':
                if param.value < 0.0:
                    self.get_logger().warn('Invalid gain_Kp! It cannot be negative.')
                    return SetParametersResult(successful=False, reason='gain_Kp cannot be negative')
                self.Kp = param.value
                self.get_logger().info(f'gain_Kp updated to {self.Kp}')

            elif param.name == 'gain_Ki':
                if param.value < 0.0:
                    self.get_logger().warn('Invalid gain_Ki! It cannot be negative.')
                    return SetParametersResult(successful=False, reason='gain_Ki cannot be negative')
                self.Ki = param.value
                self.get_logger().info(f'gain_Ki updated to {self.Ki}')

            elif param.name == 'gain_Kd':
                if param.value < 0.0:
                    self.get_logger().warn('Invalid gain_Kd! It cannot be negative.')
                    return SetParametersResult(successful=False, reason='gain_Kd cannot be negative')
                self.Kd = param.value
                self.get_logger().info(f'gain_Kd updated to {self.Kd}')

            elif param.name == 'sample_time':
                if param.value <= 0.0:
                    self.get_logger().warn('Invalid sample_time! It must be positive.')
                    return SetParametersResult(successful=False, reason='sample_time must be positive')
                self.sample_time = param.value
                self.get_logger().info(f'sample_time updated to {self.sample_time}')

        return SetParametersResult(successful=True)


# Main
def main(args=None):
    rclpy.init(args=args)

    node = PIDController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


# Execute Node
if __name__ == '__main__':
    main()
