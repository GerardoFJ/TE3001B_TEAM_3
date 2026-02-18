# pid_tuning_config.py
"""
PID Tuning Configuration System

This module defines systematic gain combinations for P, PD, PI, and PID controller families.
Each family has ≥5 distinct tunings to explore the parameter space.

Tuning Philosophy:
  - Gains are set per-joint to account for inertia differences
  - Torque limits ensure safety and prevent saturation
  - Integral limits prevent windup
  - All combinations target the same motion (0 → target → 0) under disturbance
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List
import numpy as np

from so101_control import PIDGains

DEFAULT_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# Inertia scalars for per-joint tuning (relative to shoulder_lift)
# Larger joints need higher gains due to higher inertia
INERTIA_SCALE = {
    "shoulder_pan": 0.90,   # Horizontal pan, moderate inertia
    "shoulder_lift": 1.00,  # Reference joint
    "elbow_flex": 0.85,     # Lighter than shoulder_lift
    "wrist_flex": 0.60,     # Much lighter (end of chain)
    "wrist_roll": 0.50,     # Lightest (pure rotation)
}

# Torque limits per joint (safety and stability)
TAU_LIMITS = {
    "shoulder_pan": 8.0,
    "shoulder_lift": 18.0,
    "elbow_flex": 15.0,
    "wrist_flex": 6.0,
    "wrist_roll": 3.0,
}


@dataclass
class TuningCombination:
    """
    A single gain configuration for the entire arm.
    
    Attributes:
      combo_id: Unique identifier (e.g., "P-1", "PID-4")
      family: One of "P", "PD", "PI", "PID"
      name: Human-readable description (e.g., "Very High Kp")
      kp_base: Base proportional gain (scaled to each joint)
      ki_base: Base integral gain (scaled to each joint)
      kd_base: Base derivative gain (scaled to each joint)
      i_limit_base: Base integral state limit (prevents windup)
      philosophy: Explanation of the tuning choice
    """
    combo_id: str
    family: str
    name: str
    kp_base: float
    ki_base: float
    kd_base: float
    i_limit_base: float = 2.0
    philosophy: str = ""

    def build_gains(self, joint_names=DEFAULT_JOINTS) -> Dict[str, PIDGains]:
        """
        Scale base gains by joint inertia and return per-joint PIDGains.
        """
        gains = {}
        for joint in joint_names:
            scale = INERTIA_SCALE.get(joint, 1.0)
            gains[joint] = PIDGains(
                kp=self.kp_base * scale,
                ki=self.ki_base * scale,
                kd=self.kd_base * scale,
                i_limit=self.i_limit_base,
                tau_limit=TAU_LIMITS.get(joint, 10.0),
            )
        return gains


# ============================================================================
# PROPORTIONAL ONLY (P Control)
# ============================================================================
P_TUNINGS = [
    TuningCombination(
        combo_id="P-1",
        family="P",
        name="Very Low Kp (0.5×)",
        kp_base=12.5,  # 0.5× baseline
        ki_base=0.0,
        kd_base=0.0,
        philosophy="Minimal stiffness; very slow response. Expect large steady-state error, no overshoot."
    ),
    TuningCombination(
        combo_id="P-2",
        family="P",
        name="Low Kp (1.0×)",
        kp_base=25.0,  # Baseline
        ki_base=0.0,
        kd_base=0.0,
        philosophy="Standard stiffness. Still significant steady-state error under disturbance. Smooth, no ringing."
    ),
    TuningCombination(
        combo_id="P-3",
        family="P",
        name="Medium Kp (1.5×)",
        kp_base=37.5,  # 1.5× baseline
        ki_base=0.0,
        kd_base=0.0,
        philosophy="Increased stiffness reduces error but may produce oscillations. Faster rise time."
    ),
    TuningCombination(
        combo_id="P-4",
        family="P",
        name="High Kp (2.0×)",
        kp_base=50.0,  # 2.0× baseline
        ki_base=0.0,
        kd_base=0.0,
        philosophy="Strong stiffness, fast response. Risk of overshoot and sustained oscillations."
    ),
    TuningCombination(
        combo_id="P-5",
        family="P",
        name="Very High Kp (2.5×)",
        kp_base=62.5,  # 2.5× baseline
        ki_base=0.0,
        kd_base=0.0,
        philosophy="Aggressive control; near instability. Excessive overshoot, high-frequency chatter."
    ),
]

# ============================================================================
# PROPORTIONAL-DERIVATIVE (PD Control)
# ============================================================================
PD_TUNINGS = [
    TuningCombination(
        combo_id="PD-1",
        family="PD",
        name="Low Kp + Low Kd",
        kp_base=20.0,
        ki_base=0.0,
        kd_base=0.5,
        philosophy="Gentle control with light damping. Slow, but stable and smooth."
    ),
    TuningCombination(
        combo_id="PD-2",
        family="PD",
        name="Medium Kp + Light Kd",
        kp_base=30.0,
        ki_base=0.0,
        kd_base=1.0,
        philosophy="Balanced speed and comfort. Derivative prevents overshoot."
    ),
    TuningCombination(
        combo_id="PD-3",
        family="PD",
        name="Medium Kp + Medium Kd",
        kp_base=40.0,
        ki_base=0.0,
        kd_base=2.0,
        philosophy="Increased damping stabilizes faster response. Good transient-response tradeoff."
    ),
    TuningCombination(
        combo_id="PD-4",
        family="PD",
        name="High Kp + High Kd",
        kp_base=50.0,
        ki_base=0.0,
        kd_base=3.0,
        philosophy="Strong stiffness + strong damping → fast, well-damped response. Still has steady-state error."
    ),
    TuningCombination(
        combo_id="PD-5",
        family="PD",
        name="Heavy Kd Dominance",
        kp_base=25.0,
        ki_base=0.0,
        kd_base=5.0,
        philosophy="Derivative action dominates; exaggerated velocity feedback. Over-damped, slow."
    ),
    TuningCombination(
        combo_id="PD-6",
        family="PD",
        name="Very High Kp + Moderate Kd",
        kp_base=60.0,
        ki_base=0.0,
        kd_base=2.0,
        philosophy="Aggressive proportional with moderate derivative damping. Risk: derivative noise amplification."
    ),
]

# ============================================================================
# PROPORTIONAL-INTEGRAL (PI Control)
# ============================================================================
PI_TUNINGS = [
    TuningCombination(
        combo_id="PI-1",
        family="PI",
        name="Low Kp + Low Ki",
        kp_base=20.0,
        ki_base=0.1,
        kd_base=0.0,
        i_limit_base=1.0,
        philosophy="Minimal integral action; slow bias removal. Low windup risk, cautious tuning."
    ),
    TuningCombination(
        combo_id="PI-2",
        family="PI",
        name="Medium Kp + Light Ki",
        kp_base=30.0,
        ki_base=0.3,
        kd_base=0.0,
        i_limit_base=1.5,
        philosophy="Moderate integral builds up slowly. Trades speed for steady-state elimination."
    ),
    TuningCombination(
        combo_id="PI-3",
        family="PI",
        name="Medium Kp + Medium Ki",
        kp_base=35.0,
        ki_base=0.6,
        kd_base=0.0,
        i_limit_base=2.0,
        philosophy="Balanced integral action. Can remove bias under disturbance if tuned carefully."
    ),
    TuningCombination(
        combo_id="PI-4",
        family="PI",
        name="High Kp + High Ki",
        kp_base=45.0,
        ki_base=1.0,
        kd_base=0.0,
        i_limit_base=2.0,
        philosophy="Aggressive integral may cause overshoot and oscillation. Watch for windup."
    ),
    TuningCombination(
        combo_id="PI-5",
        family="PI",
        name="Low Kp + Aggressive Ki",
        kp_base=20.0,
        ki_base=1.5,
        kd_base=0.0,
        i_limit_base=3.0,
        philosophy="Integral dominates; integrator acts like slow proportional. High oscillation risk."
    ),
    TuningCombination(
        combo_id="PI-6",
        family="PI",
        name="Medium Kp + High Ki (Limited)",
        kp_base=32.0,
        ki_base=0.9,
        kd_base=0.0,
        i_limit_base=1.0,
        philosophy="High Ki but tight i_limit control. Balances disturbance rejection vs. stability."
    ),
]

# ============================================================================
# FULL PID CONTROL
# ============================================================================
PID_TUNINGS = [
    TuningCombination(
        combo_id="PID-1",
        family="PID",
        name="Conservative (all low)",
        kp_base=20.0,
        ki_base=0.1,
        kd_base=0.5,
        i_limit_base=1.0,
        philosophy="Safe, smooth. Slow bias removal, light damping. Baseline for adding gain."
    ),
    TuningCombination(
        combo_id="PID-2",
        family="PID",
        name="Balanced (baseline)",
        kp_base=30.0,
        ki_base=0.3,
        kd_base=1.0,
        i_limit_base=1.5,
        philosophy="All terms proportional. Moderate response, reasonable damping, slow integral."
    ),
    TuningCombination(
        combo_id="PID-3",
        family="PID",
        name="Fast Response",
        kp_base=45.0,
        ki_base=0.5,
        kd_base=1.5,
        i_limit_base=2.0,
        philosophy="Emphasizes speed via high Kp. Moderate damping and integral. Watch overshoot."
    ),
    TuningCombination(
        combo_id="PID-4",
        family="PID",
        name="Smooth Damped",
        kp_base=35.0,
        ki_base=0.2,
        kd_base=2.5,
        i_limit_base=1.5,
        philosophy="High derivative dominates transient response. Slow integral bias removal."
    ),
    TuningCombination(
        combo_id="PID-5",
        family="PID",
        name="Aggressive (all high)",
        kp_base=55.0,
        ki_base=0.9,
        kd_base=2.5,
        i_limit_base=2.0,
        philosophy="All gains aggressive. Fast response + damping + bias removal. Near instability."
    ),
    TuningCombination(
        combo_id="PID-6",
        family="PID",
        name="Disturbance-Focused",
        kp_base=40.0,
        ki_base=1.2,
        kd_base=1.0,
        i_limit_base=2.5,
        philosophy="High Ki focuses on disturbance rejection. May oscillate under impulse."
    ),
]

# ============================================================================
# Registry and Helper Functions
# ============================================================================

ALL_TUNINGS = {
    "P": P_TUNINGS,
    "PD": PD_TUNINGS,
    "PI": PI_TUNINGS,
    "PID": PID_TUNINGS,
}


def get_tuning_by_id(combo_id: str) -> TuningCombination | None:
    """Lookup a tuning combination by combo_id (e.g., 'P-1', 'PID-4')."""
    for family_tunings in ALL_TUNINGS.values():
        for tuning in family_tunings:
            if tuning.combo_id == combo_id:
                return tuning
    return None


def get_all_combo_ids(families: List[str] | None = None) -> List[str]:
    """
    Return all combo_ids, optionally filtered by families.
    
    Args:
      families: List of family names (e.g., ['P', 'PD']). If None, return all.
    """
    if families is None:
        families = list(ALL_TUNINGS.keys())
    combo_ids = []
    for fam in families:
        if fam in ALL_TUNINGS:
            combo_ids.extend([t.combo_id for t in ALL_TUNINGS[fam]])
    return combo_ids


def print_tuning_summary():
    """Print a summary table of all tuning combinations."""
    print("=" * 100)
    print("PID TUNING COMBINATIONS SUMMARY")
    print("=" * 100)
    
    for family, tunings in ALL_TUNINGS.items():
        print(f"\n{family} CONTROL (Ki={'0' if family in ['P', 'PD'] else '>0'}, "
              f"Kd={'0' if family in ['P', 'PI'] else '>0'}):")
        print("-" * 100)
        print(f"{'ID':<10} {'Name':<30} {'Kp':<8} {'Ki':<8} {'Kd':<8} {'Philosophy':<40}")
        print("-" * 100)
        for tuning in tunings:
            print(f"{tuning.combo_id:<10} {tuning.name:<30} {tuning.kp_base:<8.1f} "
                  f"{tuning.ki_base:<8.2f} {tuning.kd_base:<8.2f} {tuning.philosophy:<40}")
        print()


if __name__ == "__main__":
    print_tuning_summary()
