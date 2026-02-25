#!/usr/bin/env python3
"""
Tracking evaluator for xArm Lite 6 position-controller experiments.

Subscribes to
-------------
  /desired_position   (PointStamped)  from trajectory_generator
  /actual_position    (PointStamped)  from position_controller
  <vel_topic>         (TwistStamped)  commanded velocity (default: /servo_server/delta_twist_cmds)

Press Ctrl+C to stop recording and generate the evaluation report.

Outputs (saved to ~/xarm_eval_<label>_<timestamp>/)
-----------------------------------------------------
  position_tracking.png   -- desired vs actual per axis
  position_error.png      -- tracking error per axis over time
  velocity_magnitude.png  -- commanded |v| over time
  tracking_data.csv       -- raw time-series data
  metrics.txt             -- RMSE and max-error summary

Example -- baseline:
  ros2 run xarm_perturbations evaluator --ros-args \\
    -p label:=baseline \\
    -p vel_topic:=/servo_server/delta_twist_cmds

Example -- sine perturbation (velocity after injector):
  ros2 run xarm_perturbations evaluator --ros-args \\
    -p label:=sine \\
    -p vel_topic:=/servo_server/delta_twist_cmds
"""
import os
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, TwistStamped
from rclpy.node import Node


class TrackingEvaluator(Node):

    def __init__(self):
        super().__init__("tracking_evaluator")

        # ── Parameters ────────────────────────────────────────────
        desired_topic = str(
            self.declare_parameter("desired_topic", "/desired_position").value
        )
        actual_topic = str(
            self.declare_parameter("actual_topic", "/actual_position").value
        )
        vel_topic = str(
            self.declare_parameter("vel_topic", "/servo_server/delta_twist_cmds").value
        )
        self.label = str(self.declare_parameter("label", "experiment").value)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # Resolve the source package directory from the share path:
        #   install/<pkg>/share/<pkg>  -->  src/xarm_ros2/<pkg>
        try:
            share_dir = Path(get_package_share_directory("xarm_perturbations"))
            ws_root = share_dir.parents[3]          # <ws>/install/<pkg>/share/<pkg>
            src_pkg_dir = ws_root / "src" / "xarm_ros2" / "xarm_perturbations"
            if not src_pkg_dir.is_dir():
                src_pkg_dir = share_dir              # fallback: use share dir
        except Exception:
            src_pkg_dir = Path.home()
        default_dir = str(src_pkg_dir / "results" / f"{self.label}_{timestamp}")
        self.save_dir = str(self.declare_parameter("save_dir", default_dir).value)

        # ── Data buffers ──────────────────────────────────────────
        self._t_des: list = []
        self._des: list = []
        self._t_act: list = []
        self._act: list = []
        self._t_vel: list = []
        self._vel_mag: list = []

        self._t0 = time.time()

        # ── Subscriptions ─────────────────────────────────────────
        self.create_subscription(PointStamped, desired_topic, self._des_cb, 10)
        self.create_subscription(PointStamped, actual_topic, self._act_cb, 10)
        self.create_subscription(TwistStamped, vel_topic, self._vel_cb, 10)

        self.get_logger().info(
            f"TrackingEvaluator ready\n"
            f"  desired : {desired_topic}\n"
            f"  actual  : {actual_topic}\n"
            f"  velocity: {vel_topic}\n"
            f"  label   : {self.label}\n"
            f"  save_dir: {self.save_dir}\n"
            f"Press Ctrl+C to stop and generate report."
        )

    # ── Callbacks ──────────────────────────────────────────────────
    def _des_cb(self, msg: PointStamped):
        self._t_des.append(time.time() - self._t0)
        self._des.append(np.array([msg.point.x, msg.point.y, msg.point.z]))

    def _act_cb(self, msg: PointStamped):
        self._t_act.append(time.time() - self._t0)
        self._act.append(np.array([msg.point.x, msg.point.y, msg.point.z]))

    def _vel_cb(self, msg: TwistStamped):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        vz = msg.twist.linear.z
        self._t_vel.append(time.time() - self._t0)
        self._vel_mag.append(float(np.sqrt(vx**2 + vy**2 + vz**2)))

    # ── Report generation ──────────────────────────────────────────
    def evaluate_and_save(self):
        """Compute metrics and save plots + CSV.  Called on shutdown."""
        n_des = len(self._t_des)
        n_act = len(self._t_act)

        if n_des < 5 or n_act < 5:
            self.get_logger().warn(
                f"Not enough data to evaluate (des={n_des}, act={n_act}). "
                "Did you let the experiment run long enough?"
            )
            return

        import matplotlib
        matplotlib.use("Agg")  # non-interactive backend (safe for ROS nodes)
        import matplotlib.pyplot as plt

        t_des = np.array(self._t_des)
        des = np.array(self._des)       # (N, 3)
        t_act = np.array(self._t_act)
        act = np.array(self._act)       # (M, 3)

        # ── Align on common time range ────────────────────────────
        t_start = max(t_des[0], t_act[0])
        t_end = min(t_des[-1], t_act[-1])

        mask = (t_des >= t_start) & (t_des <= t_end)
        if mask.sum() < 3:
            self.get_logger().warn("Time ranges do not overlap enough.")
            return

        t_eval = t_des[mask]
        des_eval = des[mask]

        # Interpolate actual onto desired timestamps
        act_interp = np.column_stack(
            [np.interp(t_eval, t_act, act[:, i]) for i in range(3)]
        )

        error = des_eval - act_interp       # (N, 3)
        t_rel = t_eval - t_eval[0]          # seconds from experiment start

        # ── Metrics ───────────────────────────────────────────────
        rmse_axis = np.sqrt(np.mean(error**2, axis=0))
        rmse_total = float(np.sqrt(np.mean(np.sum(error**2, axis=1))))
        max_err_axis = np.max(np.abs(error), axis=0)

        labels_ax = ["x", "y", "z"]
        summary_lines = [
            f"Experiment : {self.label}",
            f"Duration   : {t_rel[-1]:.1f} s  ({len(t_rel)} samples)",
            "",
            "RMSE per axis:",
            f"  x = {rmse_axis[0]*100:.3f} cm",
            f"  y = {rmse_axis[1]*100:.3f} cm",
            f"  z = {rmse_axis[2]*100:.3f} cm",
            f"Total RMSE : {rmse_total*100:.3f} cm",
            "",
            "Max |error| per axis:",
            f"  x = {max_err_axis[0]*100:.3f} cm",
            f"  y = {max_err_axis[1]*100:.3f} cm",
            f"  z = {max_err_axis[2]*100:.3f} cm",
        ]

        for line in summary_lines:
            self.get_logger().info(line)
        print("\n" + "\n".join(summary_lines) + "\n")

        # ── Create output directory ───────────────────────────────
        Path(self.save_dir).mkdir(parents=True, exist_ok=True)

        colors = ["C0", "C1", "C2"]

        # ── Plot 1: desired vs actual position ────────────────────
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        fig.suptitle(f"Desired vs Actual Position — {self.label}", fontsize=13)
        for i, ax in enumerate(axes):
            ax.plot(
                t_rel, des_eval[:, i] * 100,
                label="Desired", linewidth=1.8, color=colors[i],
            )
            ax.plot(
                t_rel, act_interp[:, i] * 100,
                label="Actual", linewidth=1.4, linestyle="--",
                color=colors[i], alpha=0.75,
            )
            ax.set_ylabel(f"{labels_ax[i]} [cm]")
            ax.legend(loc="upper right", fontsize=9)
            ax.grid(True, alpha=0.35)
        axes[-1].set_xlabel("Time [s]")
        fig.tight_layout()
        fig.savefig(os.path.join(self.save_dir, "position_tracking.png"), dpi=150)
        plt.close(fig)

        # ── Plot 2: error over time ───────────────────────────────
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        fig.suptitle(f"Position Error — {self.label}", fontsize=13)
        for i, ax in enumerate(axes):
            ax.plot(t_rel, error[:, i] * 100, linewidth=1.0, color=colors[i])
            ax.axhline(0.0, color="k", linewidth=0.8, linestyle=":")
            ax.set_ylabel(f"e_{labels_ax[i]} [cm]")
            ax.grid(True, alpha=0.35)
        axes[-1].set_xlabel("Time [s]")
        fig.tight_layout()
        fig.savefig(os.path.join(self.save_dir, "position_error.png"), dpi=150)
        plt.close(fig)

        # ── Plot 3: commanded velocity magnitude ──────────────────
        if len(self._t_vel) > 2:
            t_vel = np.array(self._t_vel)
            vel_mag = np.array(self._vel_mag)
            mask_v = (t_vel >= t_start) & (t_vel <= t_end)
            if mask_v.sum() > 2:
                fig, ax = plt.subplots(figsize=(12, 4))
                ax.plot(
                    t_vel[mask_v] - t_start, vel_mag[mask_v] * 100,
                    linewidth=1.0, color="C3",
                )
                ax.set_xlabel("Time [s]")
                ax.set_ylabel("Velocity magnitude [cm/s]")
                ax.set_title(f"Commanded Velocity Magnitude — {self.label}", fontsize=13)
                ax.grid(True, alpha=0.35)
                fig.tight_layout()
                fig.savefig(
                    os.path.join(self.save_dir, "velocity_magnitude.png"), dpi=150
                )
                plt.close(fig)

        # ── Save CSV ──────────────────────────────────────────────
        csv_path = os.path.join(self.save_dir, "tracking_data.csv")
        header = "t_s,des_x,des_y,des_z,act_x,act_y,act_z,err_x,err_y,err_z"
        data = np.column_stack([t_rel, des_eval, act_interp, error])
        np.savetxt(csv_path, data, delimiter=",", header=header, comments="")

        # ── Save metrics text ─────────────────────────────────────
        metrics_path = os.path.join(self.save_dir, "metrics.txt")
        with open(metrics_path, "w") as f:
            f.write("\n".join(summary_lines) + "\n")

        self.get_logger().info(
            f"Results saved to: {self.save_dir}/\n"
            f"  position_tracking.png\n"
            f"  position_error.png\n"
            f"  velocity_magnitude.png\n"
            f"  tracking_data.csv\n"
            f"  metrics.txt"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrackingEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.evaluate_and_save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
