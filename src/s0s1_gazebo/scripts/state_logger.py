# state_logger.py
"""
Logs actual and desired joint states during MuJoCo simulation.
Produces CSVs with: t, phase, actual positions (deg), desired positions (deg), error (deg).
"""
from __future__ import annotations

import csv
import os
from dataclasses import dataclass
from typing import Dict, List, Optional

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


@dataclass
class StateLogger:
    """
    Records per-step simulation state to a CSV file.

    Columns: t, phase, <jn>_actual_deg, <jn>_desired_deg, <jn>_error_deg  (per joint)
    """
    path: str
    joint_names: List[str] = None

    def __post_init__(self):
        if self.joint_names is None:
            self.joint_names = list(JOINT_NAMES)

        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        self._f = open(self.path, "w", newline="")
        self._w = csv.writer(self._f)

        header = ["t", "phase"]
        for jn in self.joint_names:
            header += [f"{jn}_actual_deg", f"{jn}_desired_deg", f"{jn}_error_deg"]
        self._w.writerow(header)
        self._f.flush()

    def log(
        self,
        t: float,
        phase: str,
        actual_deg: Dict[str, float],
        desired_deg: Dict[str, float],
    ) -> None:
        row = [float(t), phase]
        for jn in self.joint_names:
            a = float(actual_deg.get(jn, 0.0))
            d = float(desired_deg.get(jn, 0.0))
            row += [a, d, a - d]
        self._w.writerow(row)

    def flush(self) -> None:
        self._f.flush()

    def close(self) -> None:
        try:
            self._f.flush()
        finally:
            self._f.close()
