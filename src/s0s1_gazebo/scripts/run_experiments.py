#!/usr/bin/env python3
"""
run_experiments.py — Standalone batch runner for SO101 PID tuning study.

Runs all 20 gain configurations (P × 5, PD × 5, PI × 5, PID × 5) in
headless MuJoCo, each executing the full trajectory:
  1. move_to_zero     (2 s)
  2. hold_zero        (2 s)
  3. return_to_start  (2 s)
  4. hold_start       (2 s)

Outputs (saved under <results_dir>/<combo_id>/):
  state.csv    — actual + desired joint positions every sim step
  cmd.csv      — commanded reference trajectory (for Gazebo replay)

Usage (from the package scripts/ directory, or anywhere on PYTHONPATH):
    python3 run_experiments.py [--results-dir results] [--ids P1 PD3 ...]
"""
from __future__ import annotations

import argparse
import os
import sys
import time

# Allow running directly from the scripts/ directory
_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

import mujoco

from so101_mujoco_utils2 import set_initial_pose, get_positions_dict
from so101_mujoco_pid_utils import (
    move_to_pose_pid,
    hold_position_pid,
    build_default_perturbations,
)
from so101_ros_bridge import CommandLogger
from state_logger import StateLogger
from experiment_configs import ALL_CONFIGS, CONFIG_BY_ID, ExperimentConfig

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
_URDF_DIR = os.path.join(_SCRIPTS_DIR, "..", "urdf")
DEFAULT_SCENE = os.path.join(_URDF_DIR, "scene_urdf.xml")

# ---------------------------------------------------------------------------
# Fixed poses (degrees / gripper 0-100)
# ---------------------------------------------------------------------------
STARTING_POSITION = {
    "shoulder_pan":  -4.4003158666,
    "shoulder_lift": -92.2462050161,
    "elbow_flex":     89.9543738355,
    "wrist_flex":     55.1185398916,
    "wrist_roll":      0.0,
    "gripper":         0.0,
}

ZERO_POSITION = {
    "shoulder_pan":  0.0,
    "shoulder_lift": 0.0,
    "elbow_flex":    0.0,
    "wrist_flex":    0.0,
    "wrist_roll":    0.0,
    "gripper":       0.0,
}

MOVE_DURATION  = 2.0   # seconds per move phase
HOLD_DURATION  = 2.0   # seconds per hold phase


# ---------------------------------------------------------------------------
# Single-experiment runner
# ---------------------------------------------------------------------------
def run_one(
    cfg: ExperimentConfig,
    scene_path: str,
    results_dir: str,
) -> None:
    out_dir = os.path.join(results_dir, cfg.combo_id)
    os.makedirs(out_dir, exist_ok=True)

    state_path = os.path.join(out_dir, "state.csv")
    cmd_path   = os.path.join(out_dir, "cmd.csv")

    print(f"[{cfg.combo_id}] {cfg.label}")
    print(f"         {cfg.gains_summary()}")
    print(f"         -> {out_dir}")

    # Build model & data fresh for every experiment
    m = mujoco.MjModel.from_xml_path(scene_path)
    d = mujoco.MjData(m)
    set_initial_pose(m, d, STARTING_POSITION)

    # Build controller objects
    pid     = cfg.build_pid()
    perturb = build_default_perturbations()   # same seed=7 for all runs → fair comparison

    # Open loggers
    slog = StateLogger(state_path)
    clog = CommandLogger(cmd_path)

    t_wall_start = time.time()

    # ---------- Trajectory ----------
    move_to_pose_pid(
        m, d, viewer=None,
        target_pose_deg=ZERO_POSITION,
        duration=MOVE_DURATION, realtime=False,
        pid=pid, perturb=perturb,
        logger=clog, state_logger=slog, phase="move_to_zero",
    )
    hold_position_pid(
        m, d, viewer=None,
        hold_pose_deg=ZERO_POSITION,
        duration=HOLD_DURATION, realtime=False,
        pid=pid, perturb=perturb,
        logger=clog, state_logger=slog, phase="hold_zero",
    )
    move_to_pose_pid(
        m, d, viewer=None,
        target_pose_deg=STARTING_POSITION,
        duration=MOVE_DURATION, realtime=False,
        pid=pid, perturb=perturb,
        logger=clog, state_logger=slog, phase="return_to_start",
    )
    hold_position_pid(
        m, d, viewer=None,
        hold_pose_deg=STARTING_POSITION,
        duration=HOLD_DURATION, realtime=False,
        pid=pid, perturb=perturb,
        logger=clog, state_logger=slog, phase="hold_start",
    )
    # --------------------------------

    slog.close()
    clog.close()

    elapsed = time.time() - t_wall_start
    print(f"         done in {elapsed:.1f}s wall-clock\n")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(description="SO101 PID batch experiment runner")
    parser.add_argument(
        "--results-dir", default="results",
        help="Directory to write CSV outputs (default: ./results)",
    )
    parser.add_argument(
        "--scene", default=DEFAULT_SCENE,
        help="Path to MuJoCo scene XML",
    )
    parser.add_argument(
        "--ids", nargs="*", default=None,
        help="Subset of combo IDs to run (e.g. P1 PD3 PID5). Runs all if omitted.",
    )
    args = parser.parse_args()

    if args.ids:
        configs = []
        for cid in args.ids:
            if cid not in CONFIG_BY_ID:
                print(f"ERROR: unknown combo ID '{cid}'. Valid IDs: {list(CONFIG_BY_ID)}")
                sys.exit(1)
            configs.append(CONFIG_BY_ID[cid])
    else:
        configs = ALL_CONFIGS

    scene = os.path.abspath(args.scene)
    if not os.path.exists(scene):
        print(f"ERROR: scene file not found: {scene}")
        sys.exit(1)

    results = os.path.abspath(args.results_dir)
    os.makedirs(results, exist_ok=True)

    print(f"Scene  : {scene}")
    print(f"Results: {results}")
    print(f"Running {len(configs)} experiment(s)...\n")

    t0 = time.time()
    for cfg in configs:
        run_one(cfg, scene, results)

    print(f"All done in {time.time() - t0:.1f}s total.")
    print(f"CSV files written to: {results}")


if __name__ == "__main__":
    main()
