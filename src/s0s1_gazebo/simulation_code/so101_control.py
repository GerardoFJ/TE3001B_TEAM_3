# so101_control.py
from __future__ import annotations
from dataclasses import dataclass
import numpy as np


# ---------------- PID ----------------
@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float
    i_limit: float = 5.0
    tau_limit: float = 10.0


class JointPID:
    """
    Joint-space PID producing torques: tau = Kp*e + Ki*∫e dt + Kd*de/dt
    Uses integral clamp for basic anti-windup.
    """
    def __init__(self, joint_names: list[str], gains: dict[str, PIDGains]):
        self.joint_names = joint_names
        self.gains = gains
        self.i_state = {jn: 0.0 for jn in joint_names}
        self.prev_e = {jn: 0.0 for jn in joint_names}

    def reset(self):
        for jn in self.joint_names:
            self.i_state[jn] = 0.0
            self.prev_e[jn] = 0.0

    def compute(
        self,
        q: dict[str, float],
        qd: dict[str, float],
        q_des: dict[str, float],
        dt: float
    ) -> dict[str, float]:
        out = {}
        dt = max(dt, 1e-9)

        for jn in self.joint_names:
            g = self.gains[jn]
            e = q_des[jn] - q[jn]
            de = (e - self.prev_e[jn]) / dt

            self.i_state[jn] += e * dt
            self.i_state[jn] = float(np.clip(self.i_state[jn], -g.i_limit, g.i_limit))

            tau = g.kp * e + g.ki * self.i_state[jn] + g.kd * de
            tau = float(np.clip(tau, -g.tau_limit, g.tau_limit))

            out[jn] = tau
            self.prev_e[jn] = e

        return out


# ---------------- Perturbations ----------------
@dataclass
class PerturbationConfig:
    sinus_amp: float = 3.75
    sinus_freq_hz: float = 0.6
    noise_std: float = 3.25
    noise_tau: float = 3.3
    impulse_prob_per_s: float = 0.15
    impulse_mag: float = 2.5
    impulse_dur: float = 0.06

    disturb_tau_limit: float = 6.0

    meas_q_std: float = 0.0
    meas_qd_std: float = 0.0

    seed: int = 7


class PerturbationModel:
    """
    Generates:
      - Sinusoidal joint torque disturbance
      - Colored (low-pass filtered) noise torque
      - Occasional random impulses
      - Optional measurement noise (q, qd)
    """
    def __init__(self, joint_names: list[str], cfg: PerturbationConfig):
        self.joint_names = joint_names
        self.cfg = cfg
        self.rng = np.random.default_rng(cfg.seed)
        self.noise_state = {jn: 0.0 for jn in joint_names}
        self.impulse_remaining = {jn: 0.0 for jn in joint_names}
        self.impulse_sign = {jn: 1.0 for jn in joint_names}

    def reset(self):
        for jn in self.joint_names:
            self.noise_state[jn] = 0.0
            self.impulse_remaining[jn] = 0.0
            self.impulse_sign[jn] = 1.0

    def apply_joint_torques(self, t: float, dt: float) -> dict[str, float]:
        cfg = self.cfg
        dt = max(dt, 1e-9)

        # impulse trigger
        if self.rng.random() < cfg.impulse_prob_per_s * dt:
            k = int(self.rng.integers(1, min(3, len(self.joint_names) + 1)))
            chosen = self.rng.choice(self.joint_names, size=k, replace=False)
            for jn in chosen:
                self.impulse_remaining[jn] = cfg.impulse_dur
                self.impulse_sign[jn] = float(self.rng.choice([-1.0, 1.0]))

        torques = {}
        for idx, jn in enumerate(self.joint_names):
            # colored noise
            w = self.rng.normal(0.0, cfg.noise_std)
            if cfg.noise_tau > 1e-6:
                alpha = dt / (cfg.noise_tau + dt)
                self.noise_state[jn] = (1 - alpha) * self.noise_state[jn] + alpha * w
            else:
                self.noise_state[jn] = w

            sinus = cfg.sinus_amp * np.sin(2 * np.pi * cfg.sinus_freq_hz * t + 0.7 * idx)

            imp = 0.0
            if self.impulse_remaining[jn] > 0.0:
                imp = cfg.impulse_mag * self.impulse_sign[jn]
                self.impulse_remaining[jn] -= dt

            raw = float(sinus + self.noise_state[jn] + imp)
            torques[jn] = float(np.clip(raw, -cfg.disturb_tau_limit, cfg.disturb_tau_limit))

        return torques

    def noisy_measurement(
        self,
        q: dict[str, float],
        qd: dict[str, float],
    ) -> tuple[dict[str, float], dict[str, float]]:
        cfg = self.cfg
        if cfg.meas_q_std <= 0 and cfg.meas_qd_std <= 0:
            return q, qd

        qn, qdn = {}, {}
        for jn in self.joint_names:
            qn[jn] = float(q[jn] + self.rng.normal(0.0, cfg.meas_q_std))
            qdn[jn] = float(qd[jn] + self.rng.normal(0.0, cfg.meas_qd_std))
        return qn, qdn


# ---------------- MuJoCo helpers ----------------
def get_q_qd_dict(m, d, joint_names: list[str]) -> tuple[dict[str, float], dict[str, float]]:
    import mujoco
    q, qd = {}, {}
    for jn in joint_names:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid < 0:
            raise ValueError(f"Joint '{jn}' not found in model.")
        qadr = int(m.jnt_qposadr[jid])
        dadr = int(m.jnt_dofadr[jid])
        q[jn] = float(d.qpos[qadr])
        qd[jn] = float(d.qvel[dadr])
    return q, qd


def apply_joint_torques_qfrc(m, d, joint_names: list[str], tau: dict[str, float]):
    """
    Applies torques to joints via d.qfrc_applied at each joint's DOF address.
    """
    import mujoco
    d.qfrc_applied[:] = 0.0
    for jn in joint_names:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid < 0:
            raise ValueError(f"Joint '{jn}' not found in model.")
        dof = int(m.jnt_dofadr[jid])
        d.qfrc_applied[dof] += float(tau[jn])
