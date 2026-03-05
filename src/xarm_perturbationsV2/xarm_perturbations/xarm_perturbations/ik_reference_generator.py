#!/usr/bin/env python3
"""
IK Reference Generator for xArm Lite 6 (TE300XB-5 Challenge).

Generates a smooth Cartesian waypoint trajectory and produces joint-space
references q_des(t), qd_des(t), qdd_des(t) via weighted resolved-rate IK.

IK method (Section 5 of the handout)
--------------------------------------
  W  = diag(1, 1, wz)                   Z-priority weight matrix
  Jw = W @ J                            weighted Jacobian (3x6)
  J# = Jw.T @ inv(Jw @ Jw.T + lam^2*I) DLS pseudo-inverse
  q_null = (I - J# @ Jw)(-k_null*(q - q_home))
  qd_des = J# @ (pd_des + k_task*(p_des - p)) + q_null

Published topics
----------------
  /ik/q_des        std_msgs/Float64MultiArray  (6,) [rad]
  /ik/qd_des       std_msgs/Float64MultiArray  (6,) [rad/s]
  /ik/qdd_des      std_msgs/Float64MultiArray  (6,) [rad/s^2]
  /ik/p_des        geometry_msgs/PointStamped  [m] in link_base
  /ik/phase        std_msgs/String             JSON waypoint info

Usage
-----
  ros2 run xarm_perturbations ik_reference_generator --ros-args \\
    -p wz:=2.5 -p lam:=0.015 -p k_task:=14.0 -p k_null:=1.5 \\
    -p dwell_sec:=1.5 -p segment_sec:=2.0 -p control_rate_hz:=200.0
"""
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from xarm_perturbations.xarm_kinematics import (
    get_eef_position,
    position_jacobian,
)

# Task definition — Pick & Place doble (8 waypoints distintos + retorno)

_WAYPOINTS = np.array([
    [0.30,  0.15, 0.30],   # WP1 alto  — aproximación sobre A
    [0.30,  0.15, 0.20],   # WP2 bajo  — agarre en A
    [0.30, -0.15, 0.30],   # WP3 alto  — transporte hacia B
    [0.30, -0.15, 0.20],   # WP4 bajo  — depósito en B
    [0.35,  0.10, 0.30],   # WP5 alto  — aproximación sobre C
    [0.35,  0.10, 0.20],   # WP6 bajo  — agarre en C
    [0.35, -0.10, 0.30],   # WP7 alto  — transporte hacia D
    [0.35, -0.10, 0.20],   # WP8 bajo  — depósito en D
    [0.30,  0.15, 0.30],   # WP9 = WP1 — retorno (cierra loop)
])
_LAYERS = ['high', 'low', 'high', 'low', 'high', 'low', 'high', 'low', 'high']


# ---------------------------------------------------------------------------
# Trajectory helpers
# ---------------------------------------------------------------------------

def _quintic_coeffs(p0: float, p1: float, T: float) -> np.ndarray:
    """
    Quintic polynomial boundary-value problem.
    BC: pos/vel/acc = 0 at t=0 and t=T.
    Returns coefficients [a0..a5] for p(t) = a0 + a1*t + ... + a5*t^5.
    """
    a0, a1, a2 = p0, 0.0, 0.0
    h = p1 - p0
    a3 =  10*h / T**3
    a4 = -15*h / T**4
    a5 =   6*h / T**5
    return np.array([a0, a1, a2, a3, a4, a5])


def _eval_quintic(c: np.ndarray, t: float):
    """Evaluate quintic: returns (pos, vel, acc)."""
    t2, t3, t4, t5 = t**2, t**3, t**4, t**5
    pos = c[0] + c[1]*t + c[2]*t2 + c[3]*t3 + c[4]*t4 + c[5]*t5
    vel = c[1] + 2*c[2]*t + 3*c[3]*t2 + 4*c[4]*t3 + 5*c[5]*t4
    acc = 2*c[2] + 6*c[3]*t + 12*c[4]*t2 + 20*c[5]*t3
    return pos, vel, acc


def build_cartesian_trajectory(waypoints: np.ndarray,
                                dwell_sec: float,
                                segment_sec: float,
                                dt: float):
    """
    Generate p_des(t), pd_des(t), pdd_des(t) time series.

    Structure per segment:
      [segment: quintic spline, duration=segment_sec]
      [dwell:   constant at waypoint, duration=dwell_sec]

    Returns:
        ts:       (N,)   time stamps [s]
        p_des:    (N, 3) desired positions
        pd_des:   (N, 3) desired velocities
        pdd_des:  (N, 3) desired accelerations
        wp_idx:   (N,)   index of active waypoint (-1 during transitions)
        dwell_mask:(N,)  bool, True during dwell windows
    """
    n_wp = len(waypoints)
    ts_list, p_list, pd_list, pdd_list, wpi_list, dwell_list = [], [], [], [], [], []

    t = 0.0
    for seg in range(n_wp - 1):
        p_start = waypoints[seg]
        p_end   = waypoints[seg + 1]

        # --- quintic spline segment ---
        n_seg = max(1, int(round(segment_sec / dt)))
        coeffs = np.array([_quintic_coeffs(p_start[ax], p_end[ax], segment_sec)
                           for ax in range(3)])   # (3, 6)
        for k in range(n_seg):
            t_local = k * dt
            p_k  = np.array([_eval_quintic(coeffs[ax], t_local)[0] for ax in range(3)])
            pd_k = np.array([_eval_quintic(coeffs[ax], t_local)[1] for ax in range(3)])
            pa_k = np.array([_eval_quintic(coeffs[ax], t_local)[2] for ax in range(3)])
            ts_list.append(t + t_local)
            p_list.append(p_k)
            pd_list.append(pd_k)
            pdd_list.append(pa_k)
            wpi_list.append(-1)
            dwell_list.append(False)
        t += segment_sec

        # --- dwell at waypoint seg+1 ---
        if seg + 1 < n_wp:
            n_dw = max(1, int(round(dwell_sec / dt)))
            for k in range(n_dw):
                ts_list.append(t + k * dt)
                p_list.append(p_end.copy())
                pd_list.append(np.zeros(3))
                pdd_list.append(np.zeros(3))
                wpi_list.append(seg + 1)
                dwell_list.append(True)
            t += dwell_sec

    return (np.array(ts_list),
            np.array(p_list),
            np.array(pd_list),
            np.array(pdd_list),
            np.array(wpi_list, dtype=int),
            np.array(dwell_list, dtype=bool))


# ---------------------------------------------------------------------------
# Weighted resolved-rate IK
# ---------------------------------------------------------------------------

def weighted_resolved_rate_ik(ts: np.ndarray,
                                p_des: np.ndarray,
                                pd_des: np.ndarray,
                                pdd_des: np.ndarray,
                                q_init: np.ndarray,
                                q_home: np.ndarray,
                                wz: float = 2.5,
                                lam: float = 0.015,
                                k_task: float = 14.0,
                                k_null: float = 1.5,
                                max_qd: float = 2.5):
    """
    Compute joint-space references via weighted resolved-rate IK.

    W = diag(1, 1, wz)
    Jw = W @ J
    J# = Jw^T (Jw Jw^T + lam^2 I)^-1   (DLS pseudo-inverse)
    qd_null = (I - J# Jw)(-k_null*(q - q_home))
    qd_des[k] = J# @ (pd_des[k] + k_task*(p_des[k] - p(q))) + qd_null

    Args:
        ts:      (N,) time array
        p_des:   (N,3) desired EEF positions
        pd_des:  (N,3) desired EEF velocities
        pdd_des: (N,3) desired EEF accelerations (used for qdd estimate)
        q_init:  (6,) initial joint configuration
        q_home:  (6,) home configuration for nullspace stabilisation
        wz:      Z-axis weight (>1.0)
        lam:     DLS damping factor
        k_task:  task-space error gain [1/s]
        k_null:  nullspace posture gain [1/s]

    Returns:
        q_des:   (N,6) desired joint positions
        qd_des:  (N,6) desired joint velocities
        qdd_des: (N,6) desired joint accelerations (numerical diff of qd_des)
    """
    N = len(ts)
    q_des   = np.zeros((N, 6))
    qd_des  = np.zeros((N, 6))
    qdd_des = np.zeros((N, 6))

    W = np.diag([1.0, 1.0, wz])          # Z-priority weight matrix
    I6 = np.eye(6)
    I3 = np.eye(3)

    q = q_init.copy()
    J_prev = None

    for k in range(N):
        dt = (ts[k] - ts[k - 1]) if k > 0 else (ts[1] - ts[0])

        # Forward kinematics and Jacobian at current q
        p = get_eef_position(q)
        J = position_jacobian(q)

        # Weighted Jacobian
        Jw = W @ J                                       # (3, 6)

        # DLS pseudo-inverse
        A  = Jw @ Jw.T + (lam ** 2) * I3               # (3, 3)
        J_hash = Jw.T @ np.linalg.inv(A)                # (6, 3)

        # Nullspace posture stabilisation
        P_null = I6 - J_hash @ Jw                       # (6, 6)
        q_null = P_null @ (-k_null * (q - q_home))      # (6,)

        # Task-space velocity command
        p_err  = p_des[k] - p
        qd_k   = J_hash @ (pd_des[k] + k_task * p_err) + q_null

        # Clamp joint velocities to safe range
        qd_k = np.clip(qd_k, -max_qd, max_qd)

        # Integrate q
        q = q + qd_k * dt

        q_des[k]  = q.copy()
        qd_des[k] = qd_k.copy()

        # Lightweight J_dot estimate for qdd
        if J_prev is not None:
            J_dot = (J - J_prev) / dt
            Jw_dot = W @ J_dot
            # qdd ~ J# @ (pdd_des - J_dot @ qd_k) + nullspace term
            qdd_des[k] = J_hash @ (pdd_des[k] - Jw_dot @ qd_k)
        else:
            qdd_des[k] = np.zeros(6)

        J_prev = J.copy()

    # Smooth qdd by numerical differentiation as a sanity check
    for k in range(1, N - 1):
        dt_k = ts[k + 1] - ts[k - 1]
        if dt_k > 1e-9:
            qdd_des[k] = (qd_des[k + 1] - qd_des[k - 1]) / dt_k
    qdd_des[0]  = qdd_des[1]
    qdd_des[-1] = qdd_des[-2]

    return q_des, qd_des, qdd_des


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class IKReferenceGenerator(Node):
    """
    Computes and plays back IK-based joint references for the xArm Lite 6.

    On startup, waits for the first JointState message to determine q_init,
    then computes the full trajectory offline and publishes it in real time.
    """

    def __init__(self):
        super().__init__('ik_reference_generator')

        # ── Parameters ─────────────────────────────────────────────
        self.wz        = float(self.declare_parameter('wz',        2.5).value)
        self.lam       = float(self.declare_parameter('lam',       0.015).value)
        self.k_task    = float(self.declare_parameter('k_task',    14.0).value)
        self.k_null    = float(self.declare_parameter('k_null',    1.5).value)
        # max_qd: start conservative (rad/s); increase to 2.5 after validation
        self.max_qd    = float(self.declare_parameter('max_qd',           1.0).value or 1.0)
        self.dwell_sec = float(self.declare_parameter('dwell_sec',        1.5).value or 1.5)
        self.seg_sec   = float(self.declare_parameter('segment_sec',      2.5).value or 2.5)
        # 50 Hz matches existing position_controller and servo publish period
        self.rate_hz   = float(self.declare_parameter('control_rate_hz', 50.0).value or 50.0)
        self.dt        = 1.0 / self.rate_hz

        # q_home for nullspace (all zeros = robot upright zero config)
        q_home_param = list(self.declare_parameter(
            'q_home', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).value)
        self.q_home = np.array(q_home_param, dtype=float)

        # ── State ──────────────────────────────────────────────────
        self.q_init: np.ndarray | None = None
        self.q_meas: np.ndarray | None = None   # live robot state for online IK
        self._q_samples: list[np.ndarray] = []   # collect multiple readings
        self._N_SAMPLES = 15   # require 15 consistent readings before using q_init
        self.trajectory_ready = False
        self.play_idx = 0
        self._prev_qd = np.zeros(6)   # for qdd finite-difference estimate

        # start_hold_sec: hold (qd_des=0) before advancing playback.
        # Gives the user time to launch the controller in sync.
        self.start_hold_sec = float(
            self.declare_parameter('start_hold_sec', 5.0).value or 5.0)
        self._hold_steps = 0   # filled after trajectory is computed
        self._hold_count = 0

        # Only the Cartesian trajectory is pre-computed (joint IK is online).
        self.ts      = None
        self.p_des   = None
        self.pd_des  = None
        self.pdd_des = None
        self.wp_idx  = None
        self.dwell   = None

        # ── Publishers ─────────────────────────────────────────────
        self.pub_q    = self.create_publisher(Float64MultiArray, '/ik/q_des',   10)
        self.pub_qd   = self.create_publisher(Float64MultiArray, '/ik/qd_des',  10)
        self.pub_qdd  = self.create_publisher(Float64MultiArray, '/ik/qdd_des', 10)
        self.pub_p    = self.create_publisher(PointStamped,      '/ik/p_des',   10)
        self.pub_phase = self.create_publisher(String,           '/ik/phase',   10)

        # ── Subscriptions ──────────────────────────────────────────
        self.create_subscription(JointState, '/joint_states',
                                 self._joint_state_cb, 10)

        # ── Timer ──────────────────────────────────────────────────
        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            'IKReferenceGenerator ready\n'
            f'  wz={self.wz}  lam={self.lam}  k_task={self.k_task}'
            f'  k_null={self.k_null}\n'
            f'  dwell={self.dwell_sec}s  segment={self.seg_sec}s'
            f'  rate={self.rate_hz}Hz\n'
            f'  Waiting for /joint_states to initialise q_init...'
        )

    # ── Callbacks ──────────────────────────────────────────────────

    def _joint_state_cb(self, msg: JointState):
        if len(msg.position) < 6:
            return

        # xArm joint names: joint1..joint6 (or with prefix)
        names = list(msg.name)
        positions = list(msg.position)
        q = np.zeros(6)
        found = 0
        for i, name in enumerate(names):
            for j in range(1, 7):
                if name.endswith(f'joint{j}'):
                    q[j - 1] = positions[i]
                    found += 1
        if found < 6:
            # fallback: assume ordered
            q = np.array(positions[:6], dtype=float)

        # Always update live measurement for online IK
        self.q_meas = q.copy()

        if self.q_init is not None:
            return  # already initialised; only needed for trajectory computation

        # Accumulate samples to filter out zero-initialisation transients
        # (robot driver sometimes publishes zeros before joint states are valid)
        self._q_samples.append(q.copy())
        n = len(self._q_samples)
        if n < self._N_SAMPLES:
            if n == 1 or n % 5 == 0:
                self.get_logger().info(
                    f'Collecting joint_states samples: {n}/{self._N_SAMPLES}'
                )
            return

        # Use the median across samples (robust to outlier zero readings)
        q_med = np.median(self._q_samples, axis=0)

        self.q_init = q_med
        self.get_logger().info(
            f'q_init = {np.round(q_med, 4)} (median of {n} samples)\n'
            'Computing IK trajectory (offline)...'
        )
        self._compute_trajectory()

    # ── Trajectory computation ──────────────────────────────────────

    def _compute_trajectory(self):
        t0 = time.time()

        # Prepend the robot's actual EEF position as the start of the trajectory
        # so the approach begins from where the robot actually is.
        p_robot = get_eef_position(self.q_init)
        all_waypoints = np.vstack([p_robot.reshape(1, 3), _WAYPOINTS])
        self.get_logger().info(
            f'Robot EEF at startup: {np.round(p_robot, 4)} m\n'
            f'Approach: {np.round(p_robot, 3)} → WP1 {_WAYPOINTS[0].tolist()}'
        )

        # Only compute the Cartesian trajectory (timestamps + desired positions/
        # velocities/accelerations). Joint-space references are generated ONLINE
        # in _tick() from the live q_meas so the references always stay close
        # to the robot's actual configuration and never accumulate drift.
        ts, p_des, pd_des, pdd_des, wp_idx, dwell = build_cartesian_trajectory(
            all_waypoints, self.dwell_sec, self.seg_sec, self.dt
        )

        elapsed = time.time() - t0
        self.ts      = ts
        self.p_des   = p_des
        self.pd_des  = pd_des
        self.pdd_des = pdd_des
        self.wp_idx  = wp_idx
        self.dwell   = dwell

        total_dur = ts[-1]
        self.trajectory_ready = True
        self.play_idx = 0
        self._prev_qd = np.zeros(6)

        self._hold_steps = int(self.start_hold_sec * self.rate_hz)
        self._hold_count = 0

        self.get_logger().info(
            f'Cartesian trajectory built in {elapsed:.2f}s\n'
            f'  N={len(ts)} steps, total duration={total_dur:.1f}s\n'
            f'  Holding at start for {self.start_hold_sec}s — START THE CONTROLLER NOW'
        )

    # ── Playback tick ───────────────────────────────────────────────

    def _tick(self):
        if not self.trajectory_ready or self.q_meas is None:
            return
        try:
            self._tick_impl()
        except Exception as exc:
            self.get_logger().error(f'_tick error: {exc}', throttle_duration_sec=1.0)

    def _tick_impl(self):

        # Hold before advancing playback so the controller can connect while
        # the reference is at (or very near) the robot's current configuration.
        if self._hold_count < self._hold_steps:
            self._hold_count += 1
            k = 0
            is_holding = True
        else:
            k = self.play_idx
            if k >= len(self.ts):
                self.play_idx = 0
                self._hold_count = 0
                return
            self.play_idx += 1
            is_holding = False

        # ── Online weighted resolved-rate IK ───────────────────────
        # Compute qd_des from the CURRENT robot state (q_meas).
        # This ensures q_des = q_meas + qd_des*dt is always one step ahead
        # of the actual robot, so the joint-space tracking error stays small
        # regardless of deviations in the robot's Cartesian path.
        q = self.q_meas
        p = get_eef_position(q)
        J = position_jacobian(q)

        W     = np.diag([1.0, 1.0, self.wz])
        I6    = np.eye(6)
        I3    = np.eye(3)
        Jw    = W @ J
        A     = Jw @ Jw.T + (self.lam ** 2) * I3
        J_hash = Jw.T @ np.linalg.inv(A)
        P_null = I6 - J_hash @ Jw

        if is_holding:
            qd_k = np.zeros(6)
        else:
            p_err = self.p_des[k] - p
            qd_k  = (J_hash @ (self.pd_des[k] + self.k_task * p_err)
                     + P_null @ (-self.k_null * (q - self.q_home)))
            qd_k  = np.clip(qd_k, -self.max_qd, self.max_qd)

        # One step ahead reference
        q_des_k  = q + qd_k * self.dt

        # Acceleration via finite difference of qd
        qdd_des_k = (qd_k - self._prev_qd) / self.dt
        self._prev_qd = qd_k.copy()

        now = self.get_clock().now().to_msg()

        # Publish q_des
        m_q = Float64MultiArray()
        m_q.data = q_des_k.tolist()
        self.pub_q.publish(m_q)

        # Publish qd_des
        m_qd = Float64MultiArray()
        m_qd.data = qd_k.tolist()
        self.pub_qd.publish(m_qd)

        # Publish qdd_des
        m_qdd = Float64MultiArray()
        m_qdd.data = qdd_des_k.tolist()
        self.pub_qdd.publish(m_qdd)

        # Publish p_des
        m_p = PointStamped()
        m_p.header.stamp = now
        m_p.header.frame_id = 'link_base'
        m_p.point.x = float(self.p_des[k, 0])
        m_p.point.y = float(self.p_des[k, 1])
        m_p.point.z = float(self.p_des[k, 2])
        self.pub_p.publish(m_p)

        # Publish phase info (once per second to reduce overhead)
        if k % max(1, int(self.rate_hz)) == 0:
            wpi = int(self.wp_idx[k])
            # wp_idx is 1-based after prepending p_robot; _LAYERS is 0-based for
            # the 9 original waypoints (WP1..WP9 → index 0..8).
            if wpi >= 1 and wpi <= len(_LAYERS):
                layer = _LAYERS[wpi - 1]
            else:
                layer = 'transition'
            phase = {
                'step': k,
                'time': float(self.ts[k]),
                'waypoint': wpi,
                'in_dwell': bool(self.dwell[k]),
                'layer': layer,
                'p_des': self.p_des[k].tolist(),
            }
            m_phase = String()
            m_phase.data = json.dumps(phase)
            self.pub_phase.publish(m_phase)

        self.play_idx += 1


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = IKReferenceGenerator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
