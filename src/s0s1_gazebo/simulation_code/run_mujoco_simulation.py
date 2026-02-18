# run_mujoco_simulation.py
"""
Fixed runner for:
- MuJoCo viewer
- PID control loop
- (optional) Dash/Plotly realtime plotter
- (optional) ROS/Gazebo CSV logger

Fixes included:
- Convert degree-based pose dicts to radians for PID (your PID utils expect degrees in input)
- Robust "viewer still running" handling (safe early exit)
- Optional disabling of Dash if it crashes / is not installed
- Optional headless mode to avoid GLXBadWindow issues on some Linux setups
"""

from __future__ import annotations

import os
import sys
import time
import argparse

import mujoco
import mujoco.viewer

from so101_mujoco_utils2 import set_initial_pose, get_positions_dict
from so101_mujoco_pid_utils import move_to_pose_pid, hold_position_pid

# Optional: Dash plotter
try:
    from so101_mujoco_utils2 import RealtimeJointPlotter
except Exception:
    RealtimeJointPlotter = None  # type: ignore

# Optional: CSV logger for ROS/Gazebo replay
try:
    from so101_ros_bridge import CommandLogger
except Exception:
    CommandLogger = None  # type: ignore


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="model/scene_urdf.xml")
    parser.add_argument("--no-plot", action="store_true", help="Disable Dash plotter")
    parser.add_argument("--log-csv", default="", help="Write commanded joint refs to CSV for ROS replay")
    parser.add_argument("--headless", action="store_true", help="Run without MuJoCo viewer (avoids GLX issues)")
    parser.add_argument("--plot-host", default="127.0.0.1")
    parser.add_argument("--plot-port", type=int, default=8050)
    args = parser.parse_args()

    # If you keep getting GLXBadWindow, headless is the simplest fix:
    # python run_mujoco_simulation.py --headless --no-plot
    # Or run inside a proper X server / disable compositor / avoid remote GL.

    MODEL_PATH = args.model
    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)

    # Poses are in DEGREES for revolute joints + gripper 0..100 (your convention)
    starting_position = {
        "shoulder_pan":  -4.4003158666,
        "shoulder_lift": -92.2462050161,
        "elbow_flex":     89.9543738355,
        "wrist_flex":     55.1185398916,
        "wrist_roll":      0.0,
        "gripper":         0.0,
    }

    desired_zero = {
        "shoulder_pan":  0.0,
        "shoulder_lift": 0.0,
        "elbow_flex":    0.0,
        "wrist_flex":    0.0,
        "wrist_roll":    0.0,
        "gripper":       0.0,
    }

    # Put robot at starting pose (this uses your util conversion and mj_forward)
    set_initial_pose(m, d, starting_position)

    # Optional plotter
    plotter = None
    if (not args.no_plot) and (RealtimeJointPlotter is not None):
        try:
            plotter = RealtimeJointPlotter(max_points=4000)
            plotter.start(host=args.plot_host, port=args.plot_port, update_ms=100)
            print(f"[plot] Realtime plot: http://{args.plot_host}:{args.plot_port}")
        except Exception as e:
            print(f"[plot] Disabled (failed to start Dash): {e}")
            plotter = None

    # Optional CSV logger
    logger = None
    if args.log_csv:
        if CommandLogger is None:
            print("[log] so101_ros_bridge.CommandLogger not available; skipping CSV logging.")
        else:
            logger = CommandLogger(args.log_csv)
            print(f"[log] Writing commanded references to: {args.log_csv}")

    def run_sequence(viewer):
        # Helpful: show where the sim actually starts (debug)
        print("[sim] start q (deg/0..100):", get_positions_dict(m, d))

        # Move to zero, hold, return, hold
        move_to_pose_pid(m, d, viewer, desired_zero, duration=2.0, realtime=True, plotter=plotter, logger=logger)
        hold_position_pid(m, d, viewer, desired_zero, duration=2.0, realtime=True, plotter=plotter, logger=logger)

        move_to_pose_pid(m, d, viewer, starting_position, duration=2.0, realtime=True, plotter=plotter, logger=logger)
        hold_position_pid(m, d, viewer, starting_position, duration=2.0, realtime=True, plotter=plotter, logger=logger)

        print("[sim] end q (deg/0..100):", get_positions_dict(m, d))

    try:
        if args.headless:
            # Headless loop: no OpenGL viewer; steps still run.
            # (Useful for servers or when GLXBadWindow appears.)
            run_sequence(viewer=None)

        else:
            # Viewer mode
            with mujoco.viewer.launch_passive(m, d) as viewer:
                # If the user closes the window early, stop cleanly.
                if not viewer.is_running():
                    print("[sim] Viewer not running; exiting.")
                    return

                run_sequence(viewer)

                # Optional: keep open a bit at the end so you can see final pose
                t_end = time.time() + 1.0
                while viewer.is_running() and time.time() < t_end:
                    viewer.sync()
                    time.sleep(m.opt.timestep)

    finally:
        if logger is not None:
            try:
                logger.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
