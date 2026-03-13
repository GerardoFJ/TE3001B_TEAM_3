"""
Slave impedance tracking node for bilateral teleoperation.

Tracks the master arm's EE pose with PD position control in Cartesian space,
publishing velocity commands (TwistStamped) to MoveIt Servo.

When the slave contacts an obstacle, the force sensor node (separate)
detects it and publishes the wrench -- the master node reads that wrench
to create haptic feedback.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState

from xarm_bilateral_teleop.kinematics import (
    forward_kinematics,
    position_jacobian,
)


class SlaveImpedanceNode(Node):

    def __init__(self):
        super().__init__('slave_impedance')

        # Parameters
        self._Kp = np.array(self.declare_parameter(
            'Kp', [2.0, 2.0, 2.0]).value, dtype=float)
        self._deadband = float(self.declare_parameter('deadband_m', 0.003).value)
        self._max_speed = float(self.declare_parameter(
            'max_cart_speed', 0.05).value)
        control_hz = float(self.declare_parameter('control_hz', 50.0).value)
        self._timeout = float(self.declare_parameter('timeout_s', 0.5).value)

        # Topic names
        servo_topic = self.declare_parameter(
            'servo_twist_topic',
            '/slave/servo_server/delta_twist_cmds').value
        master_pose_topic = self.declare_parameter(
            'master_pose_topic', '/teleop/master_pose').value
        master_vel_topic = self.declare_parameter(
            'master_velocity_topic', '/teleop/master_velocity').value
        joint_states_topic = self.declare_parameter(
            'joint_states_topic', '/slave/joint_states').value

        # State
        self._dt = 1.0 / control_hz
        self._q = np.zeros(6)
        self._qd = np.zeros(6)
        self._x_des = None  # desired EE position from master
        self._v_ff = np.zeros(3)  # feedforward velocity from master
        self._state_ready = False
        self._last_pose_time = None

        # Publishers
        self._pub_twist = self.create_publisher(TwistStamped, servo_topic, 10)

        # Subscribers
        self.create_subscription(
            PoseStamped, master_pose_topic, self._pose_cb, 10)
        self.create_subscription(
            TwistStamped, master_vel_topic, self._vel_cb, 10)
        self.create_subscription(
            JointState, joint_states_topic, self._js_cb, 10)

        # Control timer
        self._timer = self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f'SlaveImpedance ready  Kp={self._Kp.tolist()}  hz={control_hz}')

    # -- Callbacks --

    def _pose_cb(self, msg: PoseStamped):
        p = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        if np.isfinite(p).all():
            self._x_des = p
            self._last_pose_time = self.get_clock().now()

    def _vel_cb(self, msg: TwistStamped):
        v = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
        ])
        if np.isfinite(v).all():
            self._v_ff = v

    def _js_cb(self, msg: JointState):
        names = list(msg.name)
        pos = list(msg.position)
        vel = list(msg.velocity) if msg.velocity else [0.0] * len(pos)

        q = np.zeros(6)
        qd = np.zeros(6)
        found = 0
        for i, name in enumerate(names):
            for j in range(1, 7):
                if name.endswith(f'joint{j}'):
                    q[j - 1] = pos[i]
                    qd[j - 1] = vel[i] if i < len(vel) else 0.0
                    found += 1
        if found < 6:
            q = np.array(pos[:6], dtype=float)
            qd = np.array(vel[:6], dtype=float)

        self._q = q
        self._qd = qd
        self._state_ready = True

    # -- Control loop --

    def _control_loop(self):
        if not self._state_ready or self._x_des is None:
            return

        now = self.get_clock().now()

        # Timeout: stop if master pose is stale
        if self._last_pose_time is not None:
            dt_since = (now - self._last_pose_time).nanoseconds / 1e9
            if dt_since > self._timeout:
                self._publish_zero_twist(now)
                self.get_logger().warn(
                    f'Master pose timeout ({dt_since:.2f}s), halting',
                    throttle_duration_sec=2.0)
                return

        # 1. Current EE position via FK
        T = forward_kinematics(self._q)
        x_actual = T[:3, 3]

        # 2. Position error with deadband and hard clamp (max 1 cm correction)
        e = self._x_des - x_actual
        e = np.where(np.abs(e) < self._deadband, 0.0, e)
        e = np.clip(e, -0.01, 0.01)

        # 3. Feedforward velocity + proportional correction
        # v_ff drives the slave at master speed; Kp*e corrects position drift
        v_cmd = self._v_ff + self._Kp * e

        # 5. Saturate
        v_cmd = np.clip(v_cmd, -self._max_speed, self._max_speed)

        # 6. Publish
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'link_base'
        msg.twist.linear.x = float(v_cmd[0])
        msg.twist.linear.y = float(v_cmd[1])
        msg.twist.linear.z = float(v_cmd[2])
        self._pub_twist.publish(msg)

    def _publish_zero_twist(self, now):
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'link_base'
        self._pub_twist.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlaveImpedanceNode()
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
