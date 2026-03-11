#!/usr/bin/env python3
"""
gazebo_replay_node.py — ROS 2 node that replays a commanded-trajectory CSV
                        (produced by run_experiments.py) in Gazebo.

The node reads the cmd.csv file exported by run_experiments.py and publishes
JointTrajectory messages to /so101/joint_trajectory_controller/joint_trajectory.
Gazebo must already be running with the SO101 spawned and the
joint_trajectory_controller active.

Usage:
    ros2 run s0s1_gazebo gazebo_replay_node --ros-args \
        -p csv_path:=/abs/path/to/results/PID5/cmd.csv \
        -p speed_factor:=1.0 \
        -p loop:=false

Parameters:
    csv_path      (str)   — absolute path to cmd.csv
    speed_factor  (float) — playback speed multiplier (1.0 = real-time)
    loop          (bool)  — replay continuously until Ctrl-C
    topic         (str)   — JointTrajectory topic to publish on
    frame_id      (str)   — header frame_id (default: "world")
"""
from __future__ import annotations

import sys
import os

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as RosDuration

import numpy as np
import pandas as pd


JOINT_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# cmd.csv stores joint angles in RADIANS (CommandLogger converts deg→rad at write time)
# The trajectory controller also expects radians.


def _secs_to_ros_duration(secs: float) -> RosDuration:
    whole = int(secs)
    frac  = secs - whole
    return RosDuration(sec=whole, nanosec=int(frac * 1e9))


class GazeboReplayNode(Node):

    def __init__(self):
        super().__init__("gazebo_replay_node")

        self.declare_parameter("csv_path",     "")
        self.declare_parameter("speed_factor", 1.0)
        self.declare_parameter("loop",         False)
        self.declare_parameter("topic",
                               "/so101/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("frame_id",     "world")
        # How many trajectory points to pack per message (1 = streaming, N = batch)
        self.declare_parameter("batch_size",   1)

        csv_path     = self.get_parameter("csv_path").value
        speed_factor = float(self.get_parameter("speed_factor").value)
        self.loop    = bool(self.get_parameter("loop").value)
        topic        = self.get_parameter("topic").value
        frame_id     = self.get_parameter("frame_id").value
        batch_size   = int(self.get_parameter("batch_size").value)

        if not csv_path or not os.path.isfile(csv_path):
            self.get_logger().error(f"csv_path not found: '{csv_path}'")
            raise SystemExit(1)

        self._pub = self.create_publisher(JointTrajectory, topic, 10)
        self._frame_id    = frame_id
        self._speed_factor = speed_factor
        self._batch_size  = max(1, batch_size)

        # Load CSV
        df = pd.read_csv(csv_path)
        self._t   = df["t"].values
        self._pos = {jn: df[f"{jn}_cmd"].values
                     for jn in JOINT_ORDER if f"{jn}_cmd" in df.columns}

        # Joints actually present in the CSV
        self._joint_names = [jn for jn in JOINT_ORDER if jn in self._pos]
        self._n_steps = len(self._t)

        if self._n_steps == 0:
            self.get_logger().error("CSV is empty.")
            raise SystemExit(1)

        self.get_logger().info(
            f"Loaded {self._n_steps} steps from {csv_path}\n"
            f"  joints : {self._joint_names}\n"
            f"  t_span : {self._t[0]:.3f} → {self._t[-1]:.3f} s\n"
            f"  speed  : ×{speed_factor}\n"
            f"  topic  : {topic}"
        )

        # Use a timer driven at the CSV sample rate
        dt_nominal = float(np.median(np.diff(self._t)))
        timer_period = max(0.001, dt_nominal / speed_factor)

        self._step = 0
        self._timer = self.create_timer(timer_period, self._timer_cb)

    # ------------------------------------------------------------------
    def _timer_cb(self) -> None:
        if self._step >= self._n_steps:
            if self.loop:
                self._step = 0
                self.get_logger().info("Replay looping.")
            else:
                self._timer.cancel()
                self.get_logger().info("Replay finished.")
                return

        # Build a JointTrajectory with batch_size consecutive waypoints
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.joint_names = self._joint_names

        t_base = self._t[self._step]
        for k in range(self._batch_size):
            idx = self._step + k
            if idx >= self._n_steps:
                break

            pt = JointTrajectoryPoint()
            pt.positions = [float(self._pos[jn][idx]) for jn in self._joint_names]
            pt.velocities = [0.0] * len(self._joint_names)
            pt.accelerations = [0.0] * len(self._joint_names)

            t_rel = (self._t[idx] - t_base) / self._speed_factor
            pt.time_from_start = _secs_to_ros_duration(max(0.0, t_rel))
            msg.points.append(pt)

        self._pub.publish(msg)
        self._step += self._batch_size


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GazeboReplayNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
