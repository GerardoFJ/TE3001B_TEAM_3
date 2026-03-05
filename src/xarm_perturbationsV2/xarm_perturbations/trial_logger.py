#!/usr/bin/env python3
"""
Trial Logger for xArm Lite 6 TE300XB-5 Challenge.

Subscribes to all required data streams and saves a timestamped CSV
on shutdown. Also computes and prints the required metrics (Section 9).

Required columns (Section 8.1):
  time_abs, time_rel,
  q1..q6, qd1..qd6,
  q_des1..q_des6, qd_des1..qd_des6, qdd_des1..qdd_des6,
  p_x, p_y, p_z, p_des_x, p_des_y, p_des_z,
  v_cart_x, v_cart_y, v_cart_z,
  perturb_flag,
  waypoint_idx, in_dwell

Computed metrics (Section 9):
  - Joint RMSE (per joint)
  - Joint max absolute error (per joint)
  - Dwell-window mean error (per waypoint)
  - EE RMSE, EE max error
  - Waypoint success rate (threshold eps_thresh = 0.005 m)

Usage
-----
  ros2 run xarm_perturbations trial_logger --ros-args \\
    -p trial_name:=trial_ctc_nopert \\
    -p eps_thresh:=0.005

File naming:  ~/xarm_trials/<trial_name>_<YYYY-MM-DD_HH-MM-SS>.csv
"""
import csv
import json
import os
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


class TrialLogger(Node):

    def __init__(self):
        super().__init__('trial_logger')

        # ── Parameters ─────────────────────────────────────────────
        self.trial_name  = str(self.declare_parameter('trial_name', 'trial').value)
        self.eps_thresh  = float(self.declare_parameter('eps_thresh', 0.005).value)
        self.rate_hz     = float(self.declare_parameter('log_rate_hz', 200.0).value)

        # ── State ──────────────────────────────────────────────────
        self.q_meas   = np.zeros(6)
        self.qd_meas  = np.zeros(6)
        self.q_des    = np.zeros(6)
        self.qd_des   = np.zeros(6)
        self.qdd_des  = np.zeros(6)
        self.p_actual = np.zeros(3)
        self.p_des    = np.zeros(3)
        self.v_cart   = np.zeros(3)
        self.perturb  = 0
        self.wp_idx   = -1
        self.in_dwell = False

        self._rows    = []
        self._t_start = None
        self._has_data = False

        # ── Subscriptions ──────────────────────────────────────────
        self.create_subscription(JointState, '/joint_states',
                                 self._js_cb, 10)
        self.create_subscription(Float64MultiArray, '/ik/q_des',
                                 self._qdes_cb, 10)
        self.create_subscription(Float64MultiArray, '/ik/qd_des',
                                 self._qddes_cb, 10)
        self.create_subscription(Float64MultiArray, '/ik/qdd_des',
                                 self._qdddes_cb, 10)
        self.create_subscription(PointStamped, '/actual_position',
                                 self._actual_cb, 10)
        self.create_subscription(PointStamped, '/ik/p_des',
                                 self._pdes_cb, 10)
        self.create_subscription(String, '/ik/phase',
                                 self._phase_cb, 10)

        from geometry_msgs.msg import TwistStamped
        self.create_subscription(TwistStamped, '/servo_server/delta_twist_cmds',
                                 self._vcart_cb, 10)

        # ── Log timer ──────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / self.rate_hz, self._log_tick)

        self.get_logger().info(
            f'TrialLogger ready\n'
            f'  trial_name  = {self.trial_name}\n'
            f'  eps_thresh  = {self.eps_thresh} m\n'
            f'  log_rate    = {self.rate_hz} Hz\n'
            f'  Ctrl+C to stop and save.'
        )

    # ── Callbacks ──────────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        names = list(msg.name)
        pos   = list(msg.position)
        vel   = list(msg.velocity) if msg.velocity else [0.0] * len(pos)
        q, qd = np.zeros(6), np.zeros(6)
        found = 0
        for i, name in enumerate(names):
            for j in range(1, 7):
                if name.endswith(f'joint{j}'):
                    q[j-1]  = pos[i]
                    qd[j-1] = vel[i] if i < len(vel) else 0.0
                    found += 1
        if found < 6:
            q  = np.array(pos[:6], dtype=float)
            qd = np.array(vel[:6], dtype=float)
        self.q_meas  = q
        self.qd_meas = qd
        self._has_data = True

    def _qdes_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            self.q_des = np.array(msg.data[:6])

    def _qddes_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            self.qd_des = np.array(msg.data[:6])

    def _qdddes_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            self.qdd_des = np.array(msg.data[:6])

    def _actual_cb(self, msg: PointStamped):
        self.p_actual = np.array([msg.point.x, msg.point.y, msg.point.z])

    def _pdes_cb(self, msg: PointStamped):
        self.p_des = np.array([msg.point.x, msg.point.y, msg.point.z])

    def _phase_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.wp_idx   = int(d.get('waypoint', -1))
            self.in_dwell = bool(d.get('in_dwell', False))
        except Exception:
            pass

    def _vcart_cb(self, msg):
        try:
            self.v_cart = np.array([
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z,
            ])
        except Exception:
            pass

    # ── Log tick ───────────────────────────────────────────────────

    def _log_tick(self):
        if not self._has_data:
            return

        now = self.get_clock().now()
        if self._t_start is None:
            self._t_start = now

        t_abs = now.nanoseconds / 1e9
        t_rel = (now - self._t_start).nanoseconds / 1e9

        row = (
            [t_abs, t_rel]
            + self.q_meas.tolist()
            + self.qd_meas.tolist()
            + self.q_des.tolist()
            + self.qd_des.tolist()
            + self.qdd_des.tolist()
            + self.p_actual.tolist()
            + self.p_des.tolist()
            + self.v_cart.tolist()
            + [int(self.perturb), self.wp_idx, int(self.in_dwell)]
        )
        self._rows.append(row)

    # ── Metrics computation ────────────────────────────────────────

    def _compute_metrics(self, data: np.ndarray):
        """Compute all required metrics from logged data array."""
        # Column indices (must match header order in _log_tick / _save)
        # [0]=time_abs [1]=time_rel
        # [2-7]=q1..q6  [8-13]=qd1..qd6
        # [14-19]=q_des  [20-25]=qd_des  [26-31]=qdd_des
        # [32-34]=p_xyz  [35-37]=p_des_xyz  [38-40]=v_cart_xyz
        # [41]=perturb  [42]=wp_idx  [43]=in_dwell
        t_rel_col  = 1
        q_cols     = slice(2,  8)
        qd_cols    = slice(8,  14)
        qdes_cols  = slice(14, 20)
        p_cols     = slice(32, 35)
        pdes_cols  = slice(35, 38)
        dwell_col  = 43
        wp_col     = 42

        q     = data[:, q_cols]
        q_des = data[:, qdes_cols]
        p     = data[:, p_cols]
        p_des = data[:, pdes_cols]
        in_dw = data[:, dwell_col].astype(bool)
        wp    = data[:, wp_col].astype(int)
        t_rel = data[:, t_rel_col]

        e_joint = q - q_des
        e_ee    = np.linalg.norm(p - p_des, axis=1)

        metrics = {}

        # Joint RMSE and max error
        metrics['joint_rmse']      = np.sqrt(np.mean(e_joint**2, axis=0))  # (6,)
        metrics['joint_max_error'] = np.max(np.abs(e_joint), axis=0)       # (6,)

        # EE RMSE and max error
        metrics['ee_rmse']      = float(np.sqrt(np.mean(e_ee**2)))
        metrics['ee_max_error'] = float(np.max(e_ee))

        # Dwell-window mean error per waypoint
        wp_ids  = [i for i in np.unique(wp) if i >= 0]
        dw_mean = {}
        for wid in wp_ids:
            mask = in_dw & (wp == wid)
            if mask.sum() > 0:
                dw_mean[int(wid)] = float(np.mean(np.abs(e_joint[mask]).mean(axis=1)))
        metrics['dwell_mean_error'] = dw_mean

        # Waypoint success rate
        # Success: |e_EE| < eps_thresh during the LAST 0.5 s of each dwell
        success_count = 0
        total_count   = len(wp_ids)
        for wid in wp_ids:
            mask  = in_dw & (wp == wid)
            if mask.sum() == 0:
                continue
            t_dwell = t_rel[mask]
            t_end   = t_dwell.max()
            last_half = mask & (t_rel >= t_end - 0.5)
            if last_half.sum() > 0:
                if np.max(e_ee[last_half]) < self.eps_thresh:
                    success_count += 1
        metrics['success_rate'] = (success_count / total_count * 100.0
                                   if total_count > 0 else 0.0)
        metrics['eps_thresh']   = self.eps_thresh
        metrics['n_waypoints']  = total_count

        return metrics

    # ── Save on shutdown ───────────────────────────────────────────

    def destroy_node(self):
        if self._rows:
            self._save()
        super().destroy_node()

    def _save(self):
        ts   = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        name = f'{self.trial_name}_{ts}'
        out_dir = os.path.expanduser('~/dev_ws/results')
        os.makedirs(out_dir, exist_ok=True)

        csv_path     = os.path.join(out_dir, f'{name}.csv')
        metrics_path = os.path.join(out_dir, f'{name}_metrics.txt')

        header = (
            ['time_abs', 'time_rel']
            + [f'q{i+1}'     for i in range(6)]
            + [f'qd{i+1}'    for i in range(6)]
            + [f'q_des{i+1}' for i in range(6)]
            + [f'qd_des{i+1}'for i in range(6)]
            + [f'qdd_des{i+1}'for i in range(6)]
            + ['p_x', 'p_y', 'p_z']
            + ['p_des_x', 'p_des_y', 'p_des_z']
            + ['v_cart_x', 'v_cart_y', 'v_cart_z']
            + ['perturb_active', 'waypoint_idx', 'in_dwell']
        )
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self._rows)

        data = np.array(self._rows, dtype=float)
        met  = self._compute_metrics(data)

        with open(metrics_path, 'w') as f:
            f.write(f'Trial:  {self.trial_name}\n')
            f.write(f'Time:   {ts}\n')
            f.write(f'Samples:{len(self._rows)}\n\n')
            f.write('=== Joint-Space Metrics ===\n')
            f.write('Joint RMSE [rad]:      '
                    + '  '.join(f'{v:.5f}' for v in met['joint_rmse']) + '\n')
            f.write('Joint MaxErr [rad]:    '
                    + '  '.join(f'{v:.5f}' for v in met['joint_max_error']) + '\n')
            f.write('\n=== Task-Space Metrics ===\n')
            f.write(f'EE RMSE    [m]: {met["ee_rmse"]:.5f}\n')
            f.write(f'EE MaxErr  [m]: {met["ee_max_error"]:.5f}\n')
            f.write(f'\n=== Waypoint Success (eps={met["eps_thresh"]} m) ===\n')
            f.write(f'Success rate: {met["success_rate"]:.1f}%  '
                    f'({met["n_waypoints"]} waypoints)\n')
            f.write('\nDwell-window mean joint error [rad] per waypoint:\n')
            for wid, val in sorted(met['dwell_mean_error'].items()):
                f.write(f'  WP{wid}: {val:.5f}\n')

        self.get_logger().info(
            f'Saved {len(self._rows)} rows → {csv_path}\n'
            f'EE RMSE={met["ee_rmse"]*1000:.2f} mm  '
            f'EE MaxErr={met["ee_max_error"]*1000:.2f} mm  '
            f'Success={met["success_rate"]:.1f}%'
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = TrialLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
