# pid_tuning_experiment.py
"""
PID Tuning Experiment Runner and Data Logger

This module provides:
  1. ExperimentLogger: Records all data from a single experiment
  2. ExperimentRunner: Executes a complete experiment (trajectory + data collection)
  3. ExperimentDataset: Loads and analyzes results from multiple experiments
"""

from __future__ import annotations

import json
import pickle
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple
import time

import mujoco
import numpy as np
import pandas as pd

from so101_control import JointPID, PIDGains, PerturbationModel, PerturbationConfig, get_q_qd_dict, apply_joint_torques_qfrc
from so101_mujoco_pid_utils import move_to_pose_pid, hold_position_pid, step_sim, DEFAULT_JOINTS, build_default_perturbations
from pid_tuning_config import TuningCombination

# ============================================================================
# Data Structures
# ============================================================================

@dataclass
class TimestepData:
    """Data collected at a single timestep."""
    t: float                           # Simulation time
    
    # Desired state (radians)
    q_des: Dict[str, float]            # Desired joint positions
    
    # Measured state (radians)
    q_meas: Dict[str, float]           # Measured joint positions
    qd_meas: Dict[str, float]          # Measured joint velocities
    
    # Control signals (N⋅m)
    tau_pid: Dict[str, float]          # PID torques
    tau_dist: Dict[str, float]         # Disturbance torques
    tau_total: Dict[str, float]        # Total applied torques
    
    # Error metrics (radians)
    e_pos: Dict[str, float]            # Position error (q_des - q_meas)
    e_vel: Dict[str, float]            # Velocity (derivative of error)
    
    # Phase label
    phase: str                         # "move_to_zero", "hold_zero", "return_start", "hold_start"


@dataclass
class ExperimentMetadata:
    """Metadata describing an experiment."""
    combo_id: str                      # e.g., "P-1", "PID-3"
    family: str                        # "P", "PD", "PI", or "PID"
    name: str                          # Human-readable name
    philosophy: str                    # Tuning rationale
    
    gains: Dict[str, PIDGains]         # Per-joint gains
    perturbation_config: dict          # Serialized PerturbationConfig
    
    timestamp: str                     # ISO 8601 timestamp
    hostname: str                      # Where the experiment ran
    notes: str = ""                    # Optional notes


@dataclass
class ExperimentResult:
    """Complete result from a single experiment."""
    metadata: ExperimentMetadata
    data: List[TimestepData]
    duration: float                    # Total elapsed time
    
    def to_dataframe(self) -> pd.DataFrame:
        """Convert timestep data to a pandas DataFrame for easy analysis."""
        rows = []
        for ts in self.data:
            row = {"t": ts.t, "phase": ts.phase}
            
            # Add position, velocity, error data for each joint
            for joint in ts.q_des.keys():
                row[f"q_des_{joint}"] = ts.q_des[joint]
                row[f"q_meas_{joint}"] = ts.q_meas[joint]
                row[f"qd_meas_{joint}"] = ts.qd_meas[joint]
                row[f"e_pos_{joint}"] = ts.e_pos[joint]
                row[f"e_vel_{joint}"] = ts.e_vel[joint]
                row[f"tau_pid_{joint}"] = ts.tau_pid[joint]
                row[f"tau_dist_{joint}"] = ts.tau_dist[joint]
                row[f"tau_total_{joint}"] = ts.tau_total[joint]
            
            rows.append(row)
        
        return pd.DataFrame(rows)


# ============================================================================
# Experiment Logger
# ============================================================================

class ExperimentLogger:
    """Records all data from a single experiment run."""
    
    def __init__(self, metadata: ExperimentMetadata):
        self.metadata = metadata
        self.data: List[TimestepData] = []
        self.start_time = time.time()
    
    def log_step(self, ts: TimestepData):
        """Record one timestep of data."""
        self.data.append(ts)
    
    def finalize(self) -> ExperimentResult:
        """Finalize the experiment result."""
        duration = time.time() - self.start_time
        return ExperimentResult(
            metadata=self.metadata,
            data=self.data,
            duration=duration,
        )
    
    def save(self, directory: Path | str):
        """Save the finalized result to disk."""
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        
        result = self.finalize()
        
        # Save metadata as JSON
        meta_dict = asdict(result.metadata)
        meta_dict["gains"] = {
            jn: asdict(g) for jn, g in result.metadata.gains.items()
        }
        with open(directory / "metadata.json", "w") as f:
            json.dump(meta_dict, f, indent=2)
        
        # Save timestep data as pickle (preserves structure)
        with open(directory / "data.pkl", "wb") as f:
            pickle.dump(result.data, f)
        
        # Also save as CSV for easy inspection
        df = result.to_dataframe()
        df.to_csv(directory / "data.csv", index=False)
        
        # Save summary statistics
        summary = self._compute_summary(result)
        with open(directory / "summary.json", "w") as f:
            json.dump(summary, f, indent=2)
    
    def _compute_summary(self, result: ExperimentResult) -> dict:
        """Compute summary statistics for quick analysis."""
        df = result.to_dataframe()
        joint_names = list(result.metadata.gains.keys())
        
        summary = {
            "combo_id": result.metadata.combo_id,
            "family": result.metadata.family,
            "duration_s": result.duration,
            "num_samples": len(result.data),
            "joints": {},
        }
        
        for joint in joint_names:
            e_col = f"e_pos_{joint}"
            tau_col = f"tau_total_{joint}"
            
            if e_col in df.columns and tau_col in df.columns:
                summary["joints"][joint] = {
                    "e_pos_mean_rad": float(np.mean(np.abs(df[e_col]))),
                    "e_pos_max_rad": float(np.max(np.abs(df[e_col]))),
                    "e_pos_std_rad": float(np.std(df[e_col])),
                    "tau_mean_Nm": float(np.mean(np.abs(df[tau_col]))),
                    "tau_max_Nm": float(np.max(np.abs(df[tau_col]))),
                    "tau_std_Nm": float(np.std(df[tau_col])),
                }
        
        return summary


# ============================================================================
# Experiment Runner
# ============================================================================

class ExperimentRunner:
    """
    Executes a complete PID tuning experiment:
      1. Move from start pose to zero
      2. Hold at zero
      3. Return to start pose
      4. Hold at start pose
    
    All phases experience strong perturbations.
    """
    
    def __init__(
        self,
        tuning: TuningCombination,
        model_path: str = "model/scene_urdf.xml",
        perturb_config: Optional[PerturbationConfig] = None,
        joint_names: List[str] = DEFAULT_JOINTS,
        viewer: bool = True,
        realtime: bool = False,
    ):
        self.tuning = tuning
        self.model_path = model_path
        self.joint_names = joint_names
        self.viewer = viewer
        self.realtime = realtime
        
        # Load model
        self.m = mujoco.MjModel.from_xml_path(model_path)
        self.d = mujoco.MjData(self.m)
        
        # Build gains from tuning combination
        gains = tuning.build_gains(joint_names)
        self.pid = JointPID(joint_names, gains)
        
        # Perturbations
        if perturb_config is None:
            perturb_config = PerturbationConfig()
        self.perturb = PerturbationModel(joint_names, perturb_config)
        
        # Logger
        import socket
        metadata = ExperimentMetadata(
            combo_id=tuning.combo_id,
            family=tuning.family,
            name=tuning.name,
            philosophy=tuning.philosophy,
            gains=gains,
            perturbation_config=asdict(perturb_config),
            timestamp=pd.Timestamp.now().isoformat(),
            hostname=socket.gethostname(),
        )
        self.logger = ExperimentLogger(metadata)
        
        # Trajectory parameters
        self.start_pose_deg = {jn: float(np.rad2deg(self.d.qpos[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, jn)])) 
                              for jn in joint_names}
        self.zero_pose_deg = {jn: 0.0 for jn in joint_names}
    
    def run(self, move_duration: float = 2.0, hold_duration: float = 2.0, viewer_obj=None) -> ExperimentResult:
        """
        Execute the full trajectory:
          - Move to zero (move_duration seconds)
          - Hold zero (hold_duration seconds)
          - Return to start (move_duration seconds)
          - Hold start (hold_duration seconds)
        
        If viewer_obj is provided, use it; otherwise create a MuJoCo viewer.
        """
        if self.viewer and viewer_obj is None:
            try:
                viewer_obj = mujoco.viewer.launch_passive(self.m, self.d)
            except Exception:
                viewer_obj = None
        
        try:
            self._run_trajectory(move_duration, hold_duration, viewer_obj)
        finally:
            if viewer_obj is not None:
                try:
                    viewer_obj.close()
                except:
                    pass
        
        return self.logger.finalize()
    
    def _run_trajectory(self, move_duration: float, hold_duration: float, viewer_obj):
        """Internal trajectory execution with logging."""
        phases = [
            ("move_to_zero", move_duration, self.zero_pose_deg),
            ("hold_zero", hold_duration, self.zero_pose_deg),
            ("return_start", move_duration, self.start_pose_deg),
            ("hold_start", hold_duration, self.start_pose_deg),
        ]
        
        for phase_name, phase_duration, target_pose in phases:
            self._run_phase(phase_name, phase_duration, target_pose, viewer_obj)
    
    def _run_phase(self, phase_name: str, duration: float, target_pose_deg: Dict[str, float], viewer_obj):
        """Run a single phase (move or hold) and log data."""
        self.pid.reset()
        self.perturb.reset()
        
        q0, _ = get_q_qd_dict(self.m, self.d, self.joint_names)
        qT = {jn: float(np.deg2rad(target_pose_deg[jn])) for jn in self.joint_names}
        
        steps = int(max(1, duration / self.m.opt.timestep))
        t0 = float(self.d.time)
        
        for _ in range(steps):
            t = float(self.d.time)
            s = (t - t0) / max(duration, 1e-9)
            s = float(np.clip(s, 0.0, 1.0))
            
            # Desired trajectory (radians)
            q_des = {jn: (1.0 - s) * q0[jn] + s * qT[jn] for jn in self.joint_names}
            
            # Measured state
            q, qd = get_q_qd_dict(self.m, self.d, self.joint_names)
            q_meas, qd_meas = self.perturb.noisy_measurement(q, qd)
            
            # Compute errors
            e_pos = {jn: q_des[jn] - q_meas[jn] for jn in self.joint_names}
            e_vel = qd_meas  # Velocity error (we don't have desired velocity here)
            
            # Control
            tau_pid = self.pid.compute(q_meas, qd_meas, q_des, self.m.opt.timestep)
            tau_dist = self.perturb.apply_joint_torques(t=t, dt=self.m.opt.timestep)
            tau_total = {jn: float(tau_pid[jn] + tau_dist[jn]) for jn in self.joint_names}
            
            # Log data
            ts = TimestepData(
                t=t,
                q_des=q_des,
                q_meas=q_meas,
                qd_meas=qd_meas,
                tau_pid=tau_pid,
                tau_dist=tau_dist,
                tau_total=tau_total,
                e_pos=e_pos,
                e_vel=e_vel,
                phase=phase_name,
            )
            self.logger.log_step(ts)
            
            # Apply torques and step
            apply_joint_torques_qfrc(self.m, self.d, self.joint_names, tau_total)
            step_sim(self.m, self.d, viewer_obj, realtime=self.realtime)


# ============================================================================
# Experiment Dataset (Multi-Experiment Analysis)
# ============================================================================

class ExperimentDataset:
    """Loads and analyzes results from multiple experiments."""
    
    def __init__(self, results_dir: Path | str = "results/tuning"):
        self.results_dir = Path(results_dir)
        self.results: Dict[str, ExperimentResult] = {}
    
    def load_experiment(self, combo_id: str) -> ExperimentResult | None:
        """Load a single experiment result from disk."""
        exp_dir = self.results_dir / combo_id
        if not exp_dir.exists():
            return None
        
        # Load metadata
        with open(exp_dir / "metadata.json") as f:
            meta_dict = json.load(f)
        
        gains = {
            jn: PIDGains(**g) for jn, g in meta_dict.pop("gains").items()
        }
        meta_dict["gains"] = gains
        metadata = ExperimentMetadata(**meta_dict)
        
        # Load data
        with open(exp_dir / "data.pkl", "rb") as f:
            data = pickle.load(f)
        
        result = ExperimentResult(metadata=metadata, data=data, duration=0.0)
        self.results[combo_id] = result
        return result
    
    def load_all(self):
        """Load all experiments from results directory."""
        for combo_dir in self.results_dir.iterdir():
            if combo_dir.is_dir():
                self.load_experiment(combo_dir.name)
    
    def get_combo_ids(self) -> List[str]:
        """Return list of all loaded combo_ids."""
        return list(self.results.keys())
    
    def get_by_family(self, family: str) -> Dict[str, ExperimentResult]:
        """Get all results for a specific controller family."""
        return {
            combo_id: result
            for combo_id, result in self.results.items()
            if result.metadata.family == family
        }


if __name__ == "__main__":
    print("PID Tuning Experiment Module")
    print("Use ExperimentRunner to run experiments")
    print("Use ExperimentDataset to load and analyze results")
