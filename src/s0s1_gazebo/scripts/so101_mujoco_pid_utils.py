# so101_mujoco_pid_utils.py
from __future__ import annotations

import time
from typing import Protocol, Optional

import numpy as np
import mujoco

from so101_control import (
    JointPID, PIDGains,
    PerturbationModel, PerturbationConfig,
    get_q_qd_dict,
    apply_joint_torques_qfrc,
)

# NEW: ROS bridge logger (CSV trajectory export for Gazebo replay)
# (see earlier file: so101_ros_bridge.py)
try:
    from so101_ros_bridge import CommandLogger
except Exception:
    CommandLogger = None  # type: ignore


class _PlotterProto(Protocol):
    def sample(self, m, d, now: float | None = None) -> None: ...


DEFAULT_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def lerp_pose(p0: dict[str, float], p1: dict[str, float], s: float) -> dict[str, float]:
    s = float(np.clip(s, 0.0, 1.0))
    return {k: (1.0 - s) * p0[k] + s * p1[k] for k in p0.keys()}


def build_default_pid(joint_names=DEFAULT_JOINTS) -> JointPID:
    # Start with zeros so students explicitly tune.
    gains = {
        "shoulder_pan":  PIDGains(kp=55.0, ki=0.9, kd=2.0, i_limit=2.0, tau_limit=8.0),
        "shoulder_lift": PIDGains(kp=30.0, ki=0.7, kd=1.0, i_limit=2.0, tau_limit=18.0),
        "elbow_flex":    PIDGains(kp=25.0, ki=0.5, kd=0.5, i_limit=2.0, tau_limit=15.0),
        "wrist_flex":    PIDGains(kp=15.0, ki=0.3, kd=0.5, i_limit=2.0, tau_limit=6.0),
        "wrist_roll":    PIDGains(kp=10.0, ki=0.1, kd=0.8, i_limit=2.0, tau_limit=3.0),
    }
    for jn in joint_names:
        if jn not in gains:
            gains[jn] = PIDGains(kp=25.0, ki=0.3, kd=1.0, i_limit=2.0, tau_limit=6.0)
    return JointPID(joint_names, gains)


def build_default_perturbations(joint_names=DEFAULT_JOINTS) -> PerturbationModel:
    # If you want your "strong perturbations", replace with your PerturbationConfig values.
    cfg = PerturbationConfig(
        sinus_amp=0.8,
        sinus_freq_hz=0.5,
        noise_std=0.25,
        noise_tau=0.25,
        impulse_prob_per_s=0.12,
        impulse_mag=2.0,
        impulse_dur=0.05,
        meas_q_std=0.0,
        meas_qd_std=0.0,
        seed=7,
    )
    return PerturbationModel(joint_names, cfg)


def step_sim(m, d, viewer, realtime: bool, plotter: Optional[_PlotterProto] = None):
    """
    Single MuJoCo step + optional viewer sync + optional realtime pacing + optional plotting.
    We sample AFTER mj_step so the plotted state is the realized state.
    """
    mujoco.mj_step(m, d)

    if plotter is not None:
        plotter.sample(m, d)

    if viewer is not None:
        viewer.sync()

    if realtime:
        time.sleep(m.opt.timestep)


def _cmd_dict_deg_for_logging(q_des_rad: dict[str, float], hold_or_target_pose_deg: dict[str, float]) -> dict[str, float]:
    """
    Build a command dict in the "external" convention:
    - revolute joints: degrees
    - gripper: 0..100 (if provided, else 0)
    """
    cmd = {jn: float(np.rad2deg(q_des_rad[jn])) for jn in q_des_rad.keys()}
    cmd["gripper"] = float(hold_or_target_pose_deg.get("gripper", 0.0))
    return cmd


def move_to_pose_pid(
    m, d, viewer,
    target_pose_deg: dict[str, float],
    duration: float = 2.0,
    realtime: bool = True,
    joint_names=DEFAULT_JOINTS,
    pid: JointPID | None = None,
    perturb: PerturbationModel | None = None,
    plotter: Optional[_PlotterProto] = None,
    # NEW: logger for ROS/Gazebo replay (writes commanded reference as CSV)
    logger: Optional["CommandLogger"] = None,
):
    """
    PID torque control with optional perturbations.
    Interpolates from current pose to target over 'duration' seconds.
    Logs the commanded reference (not measured state) if logger is provided.
    """
    if pid is None:
        pid = build_default_pid(joint_names)
    if perturb is None:
        perturb = build_default_perturbations(joint_names)

    pid.reset()

    # read initial q as "start" for interpolation (radians)
    q0, _ = get_q_qd_dict(m, d, joint_names)

    # convert target degrees -> radians (only the controlled joints)
    qT = {jn: float(np.deg2rad(target_pose_deg[jn])) for jn in joint_names}

    steps = int(max(1, duration / m.opt.timestep))
    t0 = float(d.time)

    # For gripper command interpolation (0..100) we need start value.
    # If caller didn't provide, default 0.
    g0 = float(target_pose_deg.get("gripper", 0.0))
    gT = float(target_pose_deg.get("gripper", 0.0))

    # If the *current* pose dict exists elsewhere you can pass it in;
    # for now we keep gripper constant unless target_pose_deg includes it.
    if "gripper" in target_pose_deg:
        # If you want a real gripper interpolation, you can pass starting gripper explicitly.
        gT = float(target_pose_deg["gripper"])

    for _ in range(steps):
        t = float(d.time)
        s = (t - t0) / max(duration, 1e-9)
        s = float(np.clip(s, 0.0, 1.0))

        # Desired joint pose (radians)
        q_des = lerp_pose(q0, qT, s)

        # Measurement + perturbations
        q, qd = get_q_qd_dict(m, d, joint_names)
        q_meas, qd_meas = perturb.noisy_measurement(q, qd)

        # Control + disturbances
        tau_pid = pid.compute(q_meas, qd_meas, q_des, m.opt.timestep)
        tau_dist = perturb.apply_joint_torques(t=t, dt=m.opt.timestep)
        tau_total = {jn: float(tau_pid[jn] + tau_dist[jn]) for jn in joint_names}

        # NEW: log commanded reference for Gazebo replay
        if logger is not None:
            cmd_deg_0_100 = _cmd_dict_deg_for_logging(q_des, target_pose_deg)
            # If you want gripper to interpolate, do it here:
            cmd_deg_0_100["gripper"] = (1.0 - s) * g0 + s * gT
            logger.log(t=t, cmd_deg_0_100=cmd_deg_0_100)

        apply_joint_torques_qfrc(m, d, joint_names, tau_total)
        step_sim(m, d, viewer, realtime=realtime, plotter=plotter)


def hold_position_pid(
    m, d, viewer,
    hold_pose_deg: dict[str, float],
    duration: float = 2.0,
    realtime: bool = True,
    joint_names=DEFAULT_JOINTS,
    pid: JointPID | None = None,
    perturb: PerturbationModel | None = None,
    plotter: Optional[_PlotterProto] = None,
    # NEW: logger for ROS/Gazebo replay
    logger: Optional["CommandLogger"] = None,
):
    """
    Holds a fixed target pose with PID, injecting optional disturbances.
    Logs the commanded reference each step if logger is provided.
    """
    if pid is None:
        pid = build_default_pid(joint_names)
    if perturb is None:
        perturb = build_default_perturbations(joint_names)

    pid.reset()

    q_des = {jn: float(np.deg2rad(hold_pose_deg[jn])) for jn in joint_names}
    steps = int(max(1, duration / m.opt.timestep))

    for _ in range(steps):
        t = float(d.time)

        q, qd = get_q_qd_dict(m, d, joint_names)
        q_meas, qd_meas = perturb.noisy_measurement(q, qd)

        tau_pid = pid.compute(q_meas, qd_meas, q_des, m.opt.timestep)
        tau_dist = perturb.apply_joint_torques(t=t, dt=m.opt.timestep)
        tau_total = {jn: float(tau_pid[jn] + tau_dist[jn]) for jn in joint_names}

        # NEW: log commanded reference for Gazebo replay
        if logger is not None:
            cmd_deg_0_100 = _cmd_dict_deg_for_logging(q_des, hold_pose_deg)
            logger.log(t=t, cmd_deg_0_100=cmd_deg_0_100)

        apply_joint_torques_qfrc(m, d, joint_names, tau_total)
        step_sim(m, d, viewer, realtime=realtime, plotter=plotter)
