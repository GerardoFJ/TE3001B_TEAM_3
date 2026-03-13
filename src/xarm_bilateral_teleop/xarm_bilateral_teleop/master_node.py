"""
Master admittance controller for bilateral teleoperation.

The operator applies force on the master arm's end effector (detected by
the virtual F/T sensor). An admittance model converts force to velocity:

    Md * v_dot + Bd * v = F_operator - alpha * F_slave

When the slave reports contact force, it opposes the operator's input,
creating a sensation of resistance.

Publishes:
  - TwistStamped to MoveIt Servo (master arm motion)
  - PoseStamped of current EE pose (for slave tracking)
  - TwistStamped of current EE velocity (feedforward for slave)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from sensor_msgs.msg import JointState

from xarm_bilateral_teleop.kinematics import (
    forward_kinematics,
    condition_number,
    position_jacobian,
)


class MasterAdmittanceNode(Node):

    def __init__(self):
        super().__init__('master_admittance')

        # Parameters
        self._Md = np.array(self.declare_parameter(
            'Md', [2.0, 2.0, 2.0]).value, dtype=float)
        self._Bd = np.array(self.declare_parameter(
            'Bd', [10.0, 10.0, 10.0]).value, dtype=float)
        self._alpha = float(self.declare_parameter('alpha', 0.3).value)
        self._deadband = float(self.declare_parameter(
            'force_deadband_n', 1.5).value)
        self._force_clamp = float(self.declare_parameter(
            'force_clamp_n', 20.0).value)
        self._max_speed = float(self.declare_parameter(
            'max_cart_speed', 0.10).value)
        self._kappa_thr = float(self.declare_parameter(
            'singularity_threshold', 50.0).value)
        control_hz = float(self.declare_parameter('control_hz', 50.0).value)

        # Topic names (configurable for namespacing)
        servo_topic = self.declare_parameter(
            'servo_twist_topic',
            '/master/servo_server/delta_twist_cmds').value
        master_force_topic = self.declare_parameter(
            'master_force_topic',
            '/master/force_sensor/force_estimate').value
        slave_force_topic = self.declare_parameter(
            'slave_force_topic',
            '/slave/force_sensor/force_estimate').value
        joint_states_topic = self.declare_parameter(
            'joint_states_topic', '/master/joint_states').value
        # Direct operator override (e.g. keyboard teleop).
        # When a message arrives here it takes priority over the force sensor.
        override_topic = self.declare_parameter(
            'operator_override_topic',
            '/teleop/operator_force_direct').value
        slave_override_topic = self.declare_parameter(
            'slave_force_override_topic',
            '/teleop/slave_force_direct').value
        self._override_timeout = float(self.declare_parameter(
            'override_timeout_s', 0.5).value)

        # State
        self._dt = 1.0 / control_hz
        self._v = np.zeros(3)  # current EE velocity from admittance
        self._q = np.zeros(6)
        self._qd = np.zeros(6)
        self._F_op = np.zeros(3)
        self._F_override = None  # set while keyboard/test publisher is active
        self._F_override_time = None
        self._F_slave = np.zeros(3)
        self._F_slave_override = None
        self._F_slave_override_time = None
        self._state_ready = False

        # Publishers
        self._pub_twist = self.create_publisher(TwistStamped, servo_topic, 10)
        self._pub_pose = self.create_publisher(
            PoseStamped, '/teleop/master_pose', 10)
        self._pub_vel = self.create_publisher(
            TwistStamped, '/teleop/master_velocity', 10)

        # QoS for micro-ROS / ESP32 publishers (BEST_EFFORT)
        _be_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        # Subscribers
        self.create_subscription(
            WrenchStamped, master_force_topic, self._master_force_cb, 10)
        self.create_subscription(
            WrenchStamped, override_topic, self._override_force_cb, _be_qos)
        self.create_subscription(
            WrenchStamped, slave_force_topic, self._slave_force_cb, 10)
        self.create_subscription(
            WrenchStamped, slave_override_topic, self._slave_override_force_cb, 10)
        self.create_subscription(
            JointState, joint_states_topic, self._js_cb, 10)

        # Control timer
        self._timer = self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f'MasterAdmittance ready  Md={self._Md.tolist()}  '
            f'Bd={self._Bd.tolist()}  alpha={self._alpha}  '
            f'hz={control_hz}')

    # -- Callbacks --

    def _master_force_cb(self, msg: WrenchStamped):
        # Only update F_op from the sensor when no active override
        if self._F_override is not None and self._F_override_time is not None:
            age = (self.get_clock().now() - self._F_override_time).nanoseconds / 1e9
            if age < self._override_timeout:
                return  # override is active, ignore sensor
        f = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
        ])
        if np.isfinite(f).all():
            self._F_op = f

    def _override_force_cb(self, msg: WrenchStamped):
        f = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
        ])
        if np.isfinite(f).all():
            self._F_override = f
            self._F_override_time = self.get_clock().now()

    def _slave_force_cb(self, msg: WrenchStamped):
        # Ignore real sensor when override is active
        if self._F_slave_override is not None and self._F_slave_override_time is not None:
            age = (self.get_clock().now() - self._F_slave_override_time).nanoseconds / 1e9
            if age < self._override_timeout:
                return
        f = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
        ])
        if np.isfinite(f).all():
            self._F_slave = f

    def _slave_override_force_cb(self, msg: WrenchStamped):
        f = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
        ])
        if np.isfinite(f).all():
            self._F_slave_override = f
            self._F_slave_override_time = self.get_clock().now()

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
        if not self._state_ready:
            return

        now = self.get_clock().now()

        # 1. Net force: operator force minus reflected slave force
        #    Use override (keyboard/test) if recently active, else sensor
        if self._F_override is not None and self._F_override_time is not None:
            age = (now - self._F_override_time).nanoseconds / 1e9
            F_op = self._F_override if age < self._override_timeout else self._F_op
        else:
            F_op = self._F_op

        # Use slave force override (keyboard) if recently active, else real sensor
        if self._F_slave_override is not None and self._F_slave_override_time is not None:
            age = (now - self._F_slave_override_time).nanoseconds / 1e9
            F_slave = self._F_slave_override if age < self._override_timeout else self._F_slave
        else:
            F_slave = self._F_slave
        F_slave_clamped = np.clip(F_slave, -self._force_clamp, self._force_clamp)
        F_net = F_op - self._alpha * F_slave_clamped

        # 2. Deadband: ignore small forces (noise)
        for i in range(3):
            if abs(F_net[i]) < self._deadband:
                F_net[i] = 0.0

        # 3. Singularity check (use 3x6 position Jacobian since we only
        #    command linear velocity -- the 6x6 is degenerate at home)
        J = position_jacobian(self._q)
        kappa = condition_number(J)
        if kappa > self._kappa_thr:
            # Still publish pose so slave can track, just zero velocity
            self._v = np.zeros(3)
            self._publish_zero_twist(now)
            self._publish_pose(now)
            self.get_logger().warn(
                f'Near singularity (kappa={kappa:.0f}), halting',
                throttle_duration_sec=5.0)
            return

        # 4. Admittance model: Md * v_dot + Bd * v = F_net
        #    Discrete Euler: v += dt * (F_net - Bd * v) / Md
        v_dot = (F_net - self._Bd * self._v) / self._Md
        self._v += v_dot * self._dt

        # 5. Saturate velocity
        self._v = np.clip(self._v, -self._max_speed, self._max_speed)

        # 6. Publish TwistStamped to MoveIt Servo
        twist_msg = TwistStamped()
        twist_msg.header.stamp = now.to_msg()
        twist_msg.header.frame_id = 'link_base'
        twist_msg.twist.linear.x = float(self._v[0])
        twist_msg.twist.linear.y = float(self._v[1])
        twist_msg.twist.linear.z = float(self._v[2])
        self._pub_twist.publish(twist_msg)

        # 7. Publish current EE pose for slave tracking
        self._publish_pose(now)

        # 8. Publish EE velocity for slave feedforward
        vel_msg = TwistStamped()
        vel_msg.header.stamp = now.to_msg()
        vel_msg.header.frame_id = 'link_base'
        vel_msg.twist.linear.x = float(self._v[0])
        vel_msg.twist.linear.y = float(self._v[1])
        vel_msg.twist.linear.z = float(self._v[2])
        self._pub_vel.publish(vel_msg)

    def _publish_pose(self, now):
        T = forward_kinematics(self._q)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = 'link_base'
        pose_msg.pose.position.x = float(T[0, 3])
        pose_msg.pose.position.y = float(T[1, 3])
        pose_msg.pose.position.z = float(T[2, 3])
        quat = self._rot_to_quat(T[:3, :3])
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        self._pub_pose.publish(pose_msg)

    def _publish_zero_twist(self, now):
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'link_base'
        self._pub_twist.publish(msg)

    @staticmethod
    def _rot_to_quat(R: np.ndarray) -> tuple:
        """Convert 3x3 rotation matrix to quaternion (x, y, z, w)."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return (float(x), float(y), float(z), float(w))


def main(args=None):
    rclpy.init(args=args)
    node = MasterAdmittanceNode()
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
