"""
Virtual F/T sensor ROS2 node.

Reads /joint_states, estimates external wrench via motor torque residuals,
publishes WrenchStamped. One instance runs per arm (namespaced).

When physical sensors are available, replace this node only --
the WrenchStamped interface stays the same for downstream controllers.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool

from xarm_bilateral_teleop.force_estimator import ForceEstimator


class ForceSensorNode(Node):

    def __init__(self):
        super().__init__('force_sensor')

        # Parameters
        control_hz = self.declare_parameter('control_hz', 100.0).value
        cutoff_hz = self.declare_parameter('cutoff_hz', 5.0).value
        threshold = self.declare_parameter('force_threshold_n', 2.0).value
        self._cal_duration = self.declare_parameter(
            'calibration_duration_s', 2.0).value

        self._estimator = ForceEstimator(
            control_hz=control_hz,
            cutoff_hz=cutoff_hz,
            force_threshold_n=threshold,
        )

        # State
        self._q = np.zeros(6)
        self._tau = np.zeros(6)
        self._state_ready = False

        # Calibration state
        self._calibrating = True
        self._cal_samples = []
        self._cal_q = None
        self._cal_count = int(control_hz * self._cal_duration)

        # Pubs / Subs
        self._pub_wrench = self.create_publisher(
            WrenchStamped, '~/force_estimate', 10)
        self._pub_contact = self.create_publisher(
            Bool, '~/contact_detected', 10)

        self.create_subscription(
            JointState, 'joint_states', self._js_cb, 10)

        self._dt = 1.0 / control_hz
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'ForceSensor ready  hz={control_hz}  cutoff={cutoff_hz} Hz  '
            f'threshold={threshold} N  cal={self._cal_duration} s')

    def _js_cb(self, msg: JointState):
        names = list(msg.name)
        pos = list(msg.position)
        eff = list(msg.effort) if msg.effort else [0.0] * len(pos)

        q = np.zeros(6)
        tau = np.zeros(6)
        found = 0
        for i, name in enumerate(names):
            for j in range(1, 7):
                if name.endswith(f'joint{j}'):
                    q[j - 1] = pos[i]
                    tau[j - 1] = eff[i] if i < len(eff) else 0.0
                    found += 1
        if found < 6:
            q = np.array(pos[:6], dtype=float)
            tau = np.array(eff[:6], dtype=float)

        self._q = q
        self._tau = tau
        self._state_ready = True

    def _tick(self):
        if not self._state_ready:
            return

        # Calibration phase: collect noise samples at rest
        if self._calibrating:
            if self._cal_q is None:
                self._cal_q = self._q.copy()
            self._cal_samples.append(self._tau.copy())
            if len(self._cal_samples) >= self._cal_count:
                samples = np.array(self._cal_samples)
                self._estimator.calibrate_noise(self._cal_q, samples)
                new_thr = self._estimator.suggest_threshold()
                self.get_logger().info(
                    f'Calibration done ({len(samples)} samples). '
                    f'Suggested threshold: {new_thr:.2f} N')
                self._calibrating = False
            return

        # Normal operation
        F_est = self._estimator.update(self._q, self._tau)
        contact = self._estimator.is_contact(F_est)

        # Publish wrench
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link_base'
        msg.wrench.force.x = float(F_est[0])
        msg.wrench.force.y = float(F_est[1])
        msg.wrench.force.z = float(F_est[2])
        msg.wrench.torque.x = float(F_est[3])
        msg.wrench.torque.y = float(F_est[4])
        msg.wrench.torque.z = float(F_est[5])
        self._pub_wrench.publish(msg)

        # Publish contact flag
        cmsg = Bool()
        cmsg.data = contact
        self._pub_contact.publish(cmsg)


def main(args=None):
    rclpy.init(args=args)
    node = ForceSensorNode()
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
