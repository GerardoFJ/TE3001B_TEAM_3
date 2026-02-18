# so101_ros_bridge.py
from __future__ import annotations
import csv
import os
from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

def gripper_0_100_to_rad(g: float) -> float:
    # Your convention: 0..100 mapped to 0..pi
    return float(g) * np.pi / 100.0

@dataclass
class CommandLogger:
    path: str
    joint_names: List[str] = None

    def __post_init__(self):
        if self.joint_names is None:
            self.joint_names = list(JOINT_ORDER)

        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        self._f = open(self.path, "w", newline="")
        self._w = csv.writer(self._f)

        header = ["t"] + [f"{jn}_cmd" for jn in self.joint_names]
        self._w.writerow(header)
        self._f.flush()

    def log(self, t: float, cmd_deg_0_100: Dict[str, float]) -> None:
        # cmd for revolute joints is in degrees, gripper in 0..100
        row = [float(t)]
        for jn in self.joint_names:
            if jn == "gripper":
                row.append(gripper_0_100_to_rad(cmd_deg_0_100[jn]))
            else:
                row.append(np.deg2rad(cmd_deg_0_100[jn]))
        self._w.writerow(row)

    def close(self):
        try:
            self._f.flush()
        finally:
            self._f.close()
