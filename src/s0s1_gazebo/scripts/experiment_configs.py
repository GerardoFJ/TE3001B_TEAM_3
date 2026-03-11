# experiment_configs.py
"""
Defines all PID gain configurations for the SO101 PID tuning study.

Controller families (20 total experiments):
  - P   (Ki=0, Kd=0)  — 5 combos varying Kp scale
  - PD  (Ki=0)        — 5 combos varying Kp and Kd scales
  - PI  (Kd=0)        — 5 combos varying Kp and Ki scales
  - PID (all active)  — 5 combos varying all three scales

Philosophy:
  All gains are derived from "base" reference gains by a uniform scale factor
  applied to every joint simultaneously. This isolates the effect of each
  controller term. Per-joint ratios are preserved so the arm's kinematic
  hierarchy (heavier proximal joints need higher gains) is maintained.
  The base gains were designed to produce stable, moderately fast motion
  under the default MuJoCo perturbation model.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

from so101_control import JointPID, PIDGains

# ---------------------------------------------------------------------------
# Base reference gains (reflect joint inertia / torque-limit hierarchy)
# ---------------------------------------------------------------------------
_BASE_KP = {
    "shoulder_pan":  55.0,
    "shoulder_lift": 30.0,
    "elbow_flex":    25.0,
    "wrist_flex":    15.0,
    "wrist_roll":    10.0,
}
_BASE_KI = {
    "shoulder_pan":  0.9,
    "shoulder_lift": 0.7,
    "elbow_flex":    0.5,
    "wrist_flex":    0.3,
    "wrist_roll":    0.1,
}
_BASE_KD = {
    "shoulder_pan":  2.0,
    "shoulder_lift": 1.0,
    "elbow_flex":    0.5,
    "wrist_flex":    0.5,
    "wrist_roll":    0.8,
}
_TAU_LIMIT = {
    "shoulder_pan":  8.0,
    "shoulder_lift": 18.0,
    "elbow_flex":    15.0,
    "wrist_flex":    6.0,
    "wrist_roll":    3.0,
}
_I_LIMIT = {jn: 2.0 for jn in _BASE_KP}

JOINT_NAMES = list(_BASE_KP.keys())


# ---------------------------------------------------------------------------
# Config dataclass
# ---------------------------------------------------------------------------
@dataclass
class ExperimentConfig:
    combo_id: str          # e.g.  "P1", "PD3", "PID5"
    family: str            # "P", "PD", "PI", "PID"
    label: str             # human-readable description
    kp_scale: float
    ki_scale: float
    kd_scale: float

    def build_pid(self) -> JointPID:
        gains: Dict[str, PIDGains] = {}
        for jn in JOINT_NAMES:
            gains[jn] = PIDGains(
                kp=_BASE_KP[jn] * self.kp_scale,
                ki=_BASE_KI[jn] * self.ki_scale,
                kd=_BASE_KD[jn] * self.kd_scale,
                i_limit=_I_LIMIT[jn],
                tau_limit=_TAU_LIMIT[jn],
            )
        return JointPID(JOINT_NAMES, gains)

    def gains_summary(self) -> str:
        """One-line gain summary (shoulder_pan as representative)."""
        jn = "shoulder_pan"
        kp = _BASE_KP[jn] * self.kp_scale
        ki = _BASE_KI[jn] * self.ki_scale
        kd = _BASE_KD[jn] * self.kd_scale
        return f"Kp×{self.kp_scale:.2f} Ki×{self.ki_scale:.2f} Kd×{self.kd_scale:.2f}  (pan: Kp={kp:.1f} Ki={ki:.2f} Kd={kd:.2f})"


# ---------------------------------------------------------------------------
# P family  (Ki = 0, Kd = 0)
# ---------------------------------------------------------------------------
P_CONFIGS: List[ExperimentConfig] = [
    ExperimentConfig("P1", "P", "Very low Kp — compliant, sluggish",       kp_scale=0.10, ki_scale=0.0, kd_scale=0.0),
    ExperimentConfig("P2", "P", "Low Kp — slow, large steady-state error", kp_scale=0.30, ki_scale=0.0, kd_scale=0.0),
    ExperimentConfig("P3", "P", "Medium Kp — moderate stiffness",          kp_scale=0.60, ki_scale=0.0, kd_scale=0.0),
    ExperimentConfig("P4", "P", "High Kp — fast but lightly oscillatory",  kp_scale=1.00, ki_scale=0.0, kd_scale=0.0),
    ExperimentConfig("P5", "P", "Very high Kp — fast, persistent ringing", kp_scale=2.00, ki_scale=0.0, kd_scale=0.0),
]

# ---------------------------------------------------------------------------
# PD family  (Ki = 0)
# ---------------------------------------------------------------------------
PD_CONFIGS: List[ExperimentConfig] = [
    ExperimentConfig("PD1", "PD", "Low Kp + low Kd — slow, well-damped",            kp_scale=0.30, ki_scale=0.0, kd_scale=0.50),
    ExperimentConfig("PD2", "PD", "Medium Kp + low Kd — faster, light damping",     kp_scale=0.60, ki_scale=0.0, kd_scale=0.50),
    ExperimentConfig("PD3", "PD", "Base Kp + base Kd — balanced reference",         kp_scale=1.00, ki_scale=0.0, kd_scale=1.00),
    ExperimentConfig("PD4", "PD", "High Kp + medium Kd — fast, good settling",      kp_scale=1.50, ki_scale=0.0, kd_scale=1.50),
    ExperimentConfig("PD5", "PD", "High Kp + high Kd — aggressive, may chatter",    kp_scale=2.00, ki_scale=0.0, kd_scale=3.00),
]

# ---------------------------------------------------------------------------
# PI family  (Kd = 0)
# ---------------------------------------------------------------------------
PI_CONFIGS: List[ExperimentConfig] = [
    ExperimentConfig("PI1", "PI", "Low Kp + low Ki — slow, bias removal intact",       kp_scale=0.30, ki_scale=0.50, kd_scale=0.0),
    ExperimentConfig("PI2", "PI", "Medium Kp + low Ki — balanced, minimal windup",     kp_scale=0.60, ki_scale=0.50, kd_scale=0.0),
    ExperimentConfig("PI3", "PI", "Base Kp + base Ki — reference configuration",       kp_scale=1.00, ki_scale=1.00, kd_scale=0.0),
    ExperimentConfig("PI4", "PI", "High Kp + high Ki — aggressive, windup risk",       kp_scale=1.50, ki_scale=2.00, kd_scale=0.0),
    ExperimentConfig("PI5", "PI", "Very high Ki — severe windup / instability study",  kp_scale=1.00, ki_scale=4.00, kd_scale=0.0),
]

# ---------------------------------------------------------------------------
# PID family  (all three active)
# ---------------------------------------------------------------------------
PID_CONFIGS: List[ExperimentConfig] = [
    ExperimentConfig("PID1", "PID", "Conservative — low all gains",                     kp_scale=0.30, ki_scale=0.30, kd_scale=0.50),
    ExperimentConfig("PID2", "PID", "Moderate — balanced (no integral windup risk)",    kp_scale=0.80, ki_scale=0.50, kd_scale=1.00),
    ExperimentConfig("PID3", "PID", "Base — default well-tuned reference",              kp_scale=1.00, ki_scale=1.00, kd_scale=1.00),
    ExperimentConfig("PID4", "PID", "Kd-dominant — high damping, low integral",        kp_scale=1.50, ki_scale=0.30, kd_scale=2.50),
    ExperimentConfig("PID5", "PID", "Aggressive full PID — best disturbance rejection", kp_scale=1.50, ki_scale=1.50, kd_scale=2.00),
]

# All configs in a flat list for iteration
ALL_CONFIGS: List[ExperimentConfig] = P_CONFIGS + PD_CONFIGS + PI_CONFIGS + PID_CONFIGS

# Convenience: lookup by ID
CONFIG_BY_ID: Dict[str, ExperimentConfig] = {c.combo_id: c for c in ALL_CONFIGS}
