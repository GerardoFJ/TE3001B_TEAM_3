#!/usr/bin/env python3
"""
Joint-Space PID / CTC Controller for xArm Lite 6 (TE300XB-5 Challenge).

Takes joint-space references from the IK Reference Generator and produces
Cartesian velocity commands (TwistStamped) to MoveIt Servo.

Controller selection (parameter controller_type)
-------------------------------------------------
  "pid"  -- Joint-space PID with anti-windup integral clamping.
            tau = -(Kp @ e + Kd @ ed + Ki @ e_int)
            qd_cmd = qd_des + tau / joint_vel_scale

  "ctc"  -- Computed Torque Control (dynamics-model-based).
            v   = qdd_des - Kp_ctc @ e - Kd_ctc @ ed   (desired acceleration)
            tau = M(q) @ v + G(q) + F(qd)               (CTC torque law)
            qd_cmd = qd_des + v * dt                    (velocity interface)

Both controllers then map qd_cmd to Cartesian velocity:
    v_cart = J(q) @ qd_cmd

v_cart is saturated and published as TwistStamped to output_topic.

Safety features
---------------
  * Actuator saturation:   velocity clamp per axis [max_cart_speed m/s]
  * Rate limiting:         dt guarded to [1 us, 100 ms]
  * Emergency stop:        if |e| > estop_threshold [rad] for any joint

Data logging
------------
  Writes a CSV file on shutdown to:
    ~/xarm_trials/<trial_name>_<YYYY-MM-DD_HH-MM-SS>.csv

  Columns: time_abs, time_rel, q1..q6, qd1..qd6,
           q_des1..q_des6, qd_des1..qd_des6, qdd_des1..qdd_des6,
           tau1..tau6, p_x, p_y, p_z, p_des_x, p_des_y, p_des_z,
           v_cart_x, v_cart_y, v_cart_z, perturb_active

Usage (PID, no perturbation, baseline trial)
--------------------------------------------
  ros2 run xarm_perturbations joint_space_controller --ros-args \\
    -p controller_type:=pid \\
    -p output_topic:=/servo_server/delta_twist_cmds \\
    -p trial_name:=trial_pdpid_nopert

Usage (CTC, with perturbation relay through perturbation_injector)
------------------------------------------------------------------
  ros2 run xarm_perturbations joint_space_controller --ros-args \\
    -p controller_type:=ctc \\
    -p output_topic:=/controller_output \\
    -p trial_name:=trial_ctc_pert
"""
import csv
import json
import os
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import Buffer, TransformListener

from xarm_perturbations.xarm_kinematics import (
    friction_torques,
    get_eef_position,
    gravity_torques,
    inertia_matrix,
    position_jacobian,
)

# ---------------------------------------------------------------------------
# Defaults  (conservative for real-robot safety; tune upward after validation)
# ---------------------------------------------------------------------------
# PID: start low, increase once tracking is confirmed
_KP_PID   = np.diag([15., 15., 15., 10., 8., 5.])
_KD_PID   = np.diag([ 5.,  5.,  5.,  4., 3., 2.])
_KI_PID   = np.diag([ 2.,  2.,  2.,  1., 1., 0.5])
_I_LIMIT  = 0.3    # anti-windup clamp [rad]

# CTC: inner-loop gains (applied to error in acceleration space)
_KP_CTC   = np.diag([30., 30., 30., 20., 15., 10.])
_KD_CTC   = np.diag([10., 10., 10.,  8.,  6.,  4.])

_TORQUE_LIMIT     = 10.0   # N·m per joint (saturation)
_VEL_LIMIT        = 1.5    # rad/s per joint (safety clamp)
_MAX_CART_SPEED   = 0.10   # m/s per Cartesian axis
_ESTOP_THRESHOLD  = 0.8    # rad -- emergency stop if any joint error > this


class JointSpaceController(Node):

    def __init__(self):
        super().__init__('joint_space_controller')

        # ── Parameters ─────────────────────────────────────────────
        self.ctrl_type  = str(self.declare_parameter(
            'controller_type', 'pid').value).lower().strip()
        output_topic    = str(self.declare_parameter(
            'output_topic', '/servo_server/delta_twist_cmds').value)
        # Default 50 Hz matches the existing position_controller and the servo
        # publish_period (0.067 s ≈ 15 Hz). Higher rates are allowed but the
        # servo only outputs at its own publish_period regardless.
        self.rate_hz    = float(self.declare_parameter('control_rate_hz', 50.0).value or 50.0)
        self.trial_name = str(self.declare_parameter('trial_name', 'trial').value)
        self.max_cart   = float(self.declare_parameter('max_cart_speed', _MAX_CART_SPEED).value)
        self.estop_thr  = float(self.declare_parameter('estop_threshold', _ESTOP_THRESHOLD).value)
        self.vel_scale  = float(self.declare_parameter('joint_vel_scale', 8.0).value)
        self.dt         = 1.0 / self.rate_hz
        self.use_pid    = (self.ctrl_type != 'ctc')

        # ── Gains ──────────────────────────────────────────────────
        # Allow per-param override via flat lists
        kp_p = list(self.declare_parameter('kp_pid', list(np.diag(_KP_PID))).value)
        kd_p = list(self.declare_parameter('kd_pid', list(np.diag(_KD_PID))).value)
        ki_p = list(self.declare_parameter('ki_pid', list(np.diag(_KI_PID))).value)
        kp_c = list(self.declare_parameter('kp_ctc', list(np.diag(_KP_CTC))).value)
        kd_c = list(self.declare_parameter('kd_ctc', list(np.diag(_KD_CTC))).value)

        self.Kp_pid = np.diag(kp_p)
        self.Kd_pid = np.diag(kd_p)
        self.Ki_pid = np.diag(ki_p)
        self.Kp_ctc = np.diag(kp_c)
        self.Kd_ctc = np.diag(kd_c)
        self.i_limit = float(self.declare_parameter('i_limit', _I_LIMIT).value)

        # ── State ──────────────────────────────────────────────────
        self.q_meas   = np.zeros(6)
        self.qd_meas  = np.zeros(6)
        self.q_des    = np.zeros(6)
        self.qd_des   = np.zeros(6)
        self.qdd_des  = np.zeros(6)
        self.p_des    = np.zeros(3)

        self.e_int      = np.zeros(6)   # integral for PID
        self.prev_t     = None
        self.prev_err   = np.zeros(6)
        self._first_step = True          # prevents derivative spike on step 0

        self.refs_ready  = False
        self.state_ready = False
        self.estop       = False
        self.perturb_active = False

        # Grace period: skip ESTOP check for the first N steps so the robot
        # can converge from its initial position to the reference trajectory.
        # IK generator holds at start for 5 s + approach segment 2.5 s = 7.5 s.
        # At 50 Hz, 400 steps = 8 seconds of warm-up.
        self._warmup_steps = int(8.0 * self.rate_hz)
        self._steps_active = 0

        # ── Logging state ──────────────────────────────────────────
        self._log_rows = []
        self._t_start  = None

        # ── TF (for FK cross-check and p logging) ──────────────────
        self.tf_buf = Buffer()
        self.tf_lis = TransformListener(self.tf_buf, self)

        # ── Publishers ─────────────────────────────────────────────
        self.pub = self.create_publisher(TwistStamped, output_topic, 10)
        self.pub_actual = self.create_publisher(PointStamped, '/actual_position', 10)
        self.pub_error  = self.create_publisher(PointStamped, '/controller/error', 10)
        self.pub_jerr   = self.create_publisher(Float64MultiArray, '/controller/joint_error', 10)

        # ── Subscriptions ──────────────────────────────────────────
        self.create_subscription(JointState, '/joint_states',    self._js_cb,    10)
        self.create_subscription(Float64MultiArray, '/ik/q_des', self._qdes_cb,  10)
        self.create_subscription(Float64MultiArray, '/ik/qd_des',self._qddes_cb, 10)
        self.create_subscription(Float64MultiArray, '/ik/qdd_des',self._qdddes_cb,10)
        self.create_subscription(PointStamped,      '/ik/p_des', self._pdes_cb,  10)
        self.create_subscription(String,            '/ik/phase', self._phase_cb, 10)

        # ── Control timer ──────────────────────────────────────────
        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f'JointSpaceController ready\n'
            f'  controller_type = {self.ctrl_type.upper()}\n'
            f'  output_topic    = {output_topic}\n'
            f'  trial_name      = {self.trial_name}\n'
            f'  rate            = {self.rate_hz} Hz\n'
            f'  max_cart_speed  = {self.max_cart} m/s'
        )

    # ── Subscriptions ──────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        names = list(msg.name)
        pos   = list(msg.position)
        vel   = list(msg.velocity) if msg.velocity else [0.0] * len(pos)

        q  = np.zeros(6)
        qd = np.zeros(6)
        found = 0
        for i, name in enumerate(names):
            for j in range(1, 7):
                if name.endswith(f'joint{j}'):
                    q[j - 1]  = pos[i]
                    qd[j - 1] = vel[i] if i < len(vel) else 0.0
                    found += 1
        if found < 6:
            q  = np.array(pos[:6], dtype=float)
            qd = np.array(vel[:6], dtype=float)

        self.q_meas  = q
        self.qd_meas = qd
        self.state_ready = True

    def _qdes_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            self.q_des = np.array(msg.data[:6], dtype=float)
            self.refs_ready = True

    def _qddes_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            self.qd_des = np.array(msg.data[:6], dtype=float)

    def _qdddes_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            self.qdd_des = np.array(msg.data[:6], dtype=float)

    def _pdes_cb(self, msg: PointStamped):
        self.p_des = np.array([msg.point.x, msg.point.y, msg.point.z])

    def _phase_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.perturb_active = data.get('in_dwell', False)
        except Exception:
            pass

    # ── FK via TF ──────────────────────────────────────────────────

    def _read_eef_tf(self) -> np.ndarray | None:
        try:
            t = self.tf_buf.lookup_transform('link_base', 'link_eef', rclpy.time.Time())
            return np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ])
        except Exception:
            return None

    # ── PID controller ─────────────────────────────────────────────

    def _pid(self, e: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """Returns (qd_cmd, tau_logged)."""
        # Initialise prev_err to current error to avoid a large spike at step 0
        if self._first_step:
            self.prev_err = e.copy()
            self._first_step = False
        ed = (e - self.prev_err) / dt

        # Anti-windup integral
        self.e_int = np.clip(self.e_int + e * dt, -self.i_limit, self.i_limit)

        tau = -(self.Kp_pid @ e + self.Kd_pid @ ed + self.Ki_pid @ self.e_int)
        tau = np.clip(tau, -_TORQUE_LIMIT, _TORQUE_LIMIT)

        # For velocity interface: τ maps to velocity correction
        qd_cmd = self.qd_des + tau / self.vel_scale
        return qd_cmd, tau

    # ── CTC controller ─────────────────────────────────────────────

    def _ctc(self, e: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """Returns (qd_cmd, tau_logged)."""
        if self._first_step:
            self.prev_err = e.copy()
            self._first_step = False
        ed = (e - self.prev_err) / dt

        # Desired acceleration (feedback linearisation)
        v = self.qdd_des - self.Kp_ctc @ e - self.Kd_ctc @ ed

        # CTC law: tau = M(q)*v + G(q) + F(qd)
        M = inertia_matrix(self.q_meas)
        G = gravity_torques(self.q_meas)
        F = friction_torques(self.qd_meas)
        tau = M @ v + G + F
        tau = np.clip(tau, -_TORQUE_LIMIT, _TORQUE_LIMIT)

        # Velocity interface: integrate desired acceleration
        qd_cmd = self.qd_des + v * dt
        return qd_cmd, tau

    # ── Control loop ───────────────────────────────────────────────

    def _control_loop(self):
        if not (self.refs_ready and self.state_ready):
            return
        if self.estop:
            self._publish_zero()
            return

        now = self.get_clock().now()
        if self._t_start is None:
            self._t_start = now

        if self.prev_t is None:
            self.prev_t = now
            return

        dt = (now - self.prev_t).nanoseconds / 1e9
        dt = float(np.clip(dt, 1e-6, 0.10))

        # Tracking error
        e  = self.q_meas - self.q_des
        ed = self.qd_meas - self.qd_des

        self._steps_active += 1

        # Emergency stop check — skipped during warm-up grace period
        if self._steps_active <= self._warmup_steps:
            if np.any(np.abs(e) > self.estop_thr):
                self.get_logger().warn(
                    f'[WARMUP {self._steps_active}/{self._warmup_steps}]'
                    f' large error {np.round(e, 3)} rad — converging...',
                    throttle_duration_sec=1.0,
                )
        elif np.any(np.abs(e) > self.estop_thr):
            self.get_logger().error(
                f'ESTOP: joint error {np.round(e, 3)} exceeds {self.estop_thr} rad'
            )
            self.estop = True
            self._publish_zero()
            return

        # Compute joint velocity command
        if self.use_pid:
            qd_cmd, tau = self._pid(e, dt)
        else:
            qd_cmd, tau = self._ctc(e, dt)

        # Clip joint velocity
        qd_cmd = np.clip(qd_cmd, -_VEL_LIMIT, _VEL_LIMIT)

        # Map to Cartesian via Jacobian
        J = position_jacobian(self.q_meas)
        v_cart = J @ qd_cmd
        v_cart = np.clip(v_cart, -self.max_cart, self.max_cart)

        # Publish Cartesian velocity
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'link_base'
        msg.twist.linear.x = float(v_cart[0])
        msg.twist.linear.y = float(v_cart[1])
        msg.twist.linear.z = float(v_cart[2])
        self.pub.publish(msg)

        # Publish actual EEF position (from TF or FK)
        p_actual = self._read_eef_tf()
        if p_actual is None:
            p_actual = get_eef_position(self.q_meas)
        ap = PointStamped()
        ap.header.stamp = now.to_msg()
        ap.header.frame_id = 'link_base'
        ap.point.x = float(p_actual[0])
        ap.point.y = float(p_actual[1])
        ap.point.z = float(p_actual[2])
        self.pub_actual.publish(ap)

        # Publish Cartesian tracking error
        ee_error = p_actual - self.p_des
        err_msg = PointStamped()
        err_msg.header.stamp = now.to_msg()
        err_msg.header.frame_id = 'link_base'
        err_msg.point.x = float(ee_error[0])
        err_msg.point.y = float(ee_error[1])
        err_msg.point.z = float(ee_error[2])
        self.pub_error.publish(err_msg)

        # Publish per-joint tracking error
        jerr_msg = Float64MultiArray()
        jerr_msg.data = e.tolist()
        self.pub_jerr.publish(jerr_msg)

        # Log data
        t_abs = now.nanoseconds / 1e9
        t_rel = (now - self._t_start).nanoseconds / 1e9
        row = ([t_abs, t_rel]
               + self.q_meas.tolist()
               + self.qd_meas.tolist()
               + self.q_des.tolist()
               + self.qd_des.tolist()
               + self.qdd_des.tolist()
               + tau.tolist()
               + p_actual.tolist()
               + self.p_des.tolist()
               + v_cart.tolist()
               + [int(self.perturb_active)])
        self._log_rows.append(row)

        self.prev_err = e.copy()
        self.prev_t   = now

        # Periodic status log
        if len(self._log_rows) % int(self.rate_hz) == 0:
            ee_err = float(np.linalg.norm(p_actual - self.p_des))
            self.get_logger().info(
                f'[{self.ctrl_type.upper()}] t={t_rel:.1f}s'
                f'  |e_joint|={np.linalg.norm(e):.4f} rad'
                f'  |e_EE|={ee_err:.4f} m'
                f'  |v_cart|={np.linalg.norm(v_cart):.4f} m/s'
            )

    def _publish_zero(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link_base'
        self.pub.publish(msg)

    # ── Shutdown / save CSV ─────────────────────────────────────────

    def destroy_node(self):
        self._save_csv()
        super().destroy_node()

    def _save_csv(self):
        if not self._log_rows:
            return

        ts   = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        name = f'{self.trial_name}_{ts}'
        out_dir = os.path.expanduser(
            f'~/dev_ws/src/xarm_ros2/xarm_perturbations/results/{self.trial_name}')
        os.makedirs(out_dir, exist_ok=True)
        csv_path  = os.path.join(out_dir, f'{name}.csv')
        meta_path = os.path.join(out_dir, f'{name}_metadata.txt')

        header = (
            ['time_abs', 'time_rel']
            + [f'q{i+1}'    for i in range(6)]
            + [f'qd{i+1}'   for i in range(6)]
            + [f'q_des{i+1}'  for i in range(6)]
            + [f'qd_des{i+1}' for i in range(6)]
            + [f'qdd_des{i+1}'for i in range(6)]
            + [f'tau{i+1}'  for i in range(6)]
            + ['p_x', 'p_y', 'p_z']
            + ['p_des_x', 'p_des_y', 'p_des_z']
            + ['v_cart_x', 'v_cart_y', 'v_cart_z']
            + ['perturb_active']
        )
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self._log_rows)

        # Metadata
        with open(meta_path, 'w') as f:
            f.write(f'trial_name: {self.trial_name}\n')
            f.write(f'timestamp:  {ts}\n')
            f.write(f'controller: {self.ctrl_type}\n')
            f.write(f'rate_hz:    {self.rate_hz}\n')
            f.write(f'samples:    {len(self._log_rows)}\n')
            if self.use_pid:
                f.write(f'Kp_pid: {np.diag(self.Kp_pid).tolist()}\n')
                f.write(f'Kd_pid: {np.diag(self.Kd_pid).tolist()}\n')
                f.write(f'Ki_pid: {np.diag(self.Ki_pid).tolist()}\n')
                f.write(f'i_limit: {self.i_limit}\n')
            else:
                f.write(f'Kp_ctc: {np.diag(self.Kp_ctc).tolist()}\n')
                f.write(f'Kd_ctc: {np.diag(self.Kd_ctc).tolist()}\n')
            f.write(f'max_cart_speed: {self.max_cart}\n')
            f.write(f'torque_limit:   {_TORQUE_LIMIT}\n')
            f.write(f'vel_limit:      {_VEL_LIMIT}\n')

        self.get_logger().info(f'Saved {len(self._log_rows)} rows → {csv_path}')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = JointSpaceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
