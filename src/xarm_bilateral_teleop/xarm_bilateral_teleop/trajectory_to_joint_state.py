"""
Bridge node: converts JointTrajectory → JointState.

MoveIt Servo publishes trajectory_msgs/msg/JointTrajectory, but Isaac Sim's
ROS2 bridge only subscribes to sensor_msgs/msg/JointState. This node extracts
the last waypoint from each incoming JointTrajectory and republishes it as a
JointState on the /{arm_ns}/joint_command topic that Isaac Sim listens to.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class TrajectoryToJointState(Node):

    def __init__(self):
        super().__init__('trajectory_to_joint_state')

        arm_ns = self.declare_parameter('arm_ns', 'master').value

        traj_topic = f'/{arm_ns}/lite6_traj_controller/joint_trajectory'
        cmd_topic = f'/{arm_ns}/joint_command'

        self._pub = self.create_publisher(JointState, cmd_topic, 10)
        self.create_subscription(
            JointTrajectory, traj_topic, self._traj_cb, 10)

        self.get_logger().info(
            f'Bridging {traj_topic} -> {cmd_topic}')

    def _traj_cb(self, msg: JointTrajectory):
        if not msg.points:
            return

        # Use the last (or only) trajectory point
        point = msg.points[-1]

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(msg.joint_names)
        js.position = list(point.positions) if point.positions else []
        js.velocity = list(point.velocities) if point.velocities else []
        js.effort = list(point.effort) if point.effort else []

        self._pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryToJointState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
