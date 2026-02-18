# pid_analysis.py
"""
PID Tuning Analysis and Visualization

Provides tools for:
  1. Computing per-joint and aggregate performance metrics
  2. Generating publication-quality plots
  3. Creating qualitative observation tables
  4. Cross-experiment comparisons
"""

from __future__ import annotations

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass

from pid_tuning_experiment import ExperimentResult, ExperimentDataset, TimestepData


# ============================================================================
# Performance Metrics
# ============================================================================

@dataclass
class JointMetrics:
    """Per-joint performance metrics."""
    joint_name: str
    
    # Position error (radians)
    e_pos_mean: float           # Mean absolute error
    e_pos_max: float            # Maximum absolute error
    e_pos_std: float            # Standard deviation
    e_pos_final: float          # Final error
    
    # Velocity
    qd_mean: float              # Mean velocity magnitude
    qd_max: float               # Peak velocity
    
    # Torque statistics
    tau_pid_mean: float         # Mean PID torque magnitude
    tau_pid_max: float          # Peak PID torque
    tau_dist_mean: float        # Mean disturbance torque
    tau_dist_max: float         # Peak disturbance torque
    tau_total_mean: float       # Mean total torque
    tau_total_max: float        # Peak total torque
    
    # Temporal behavior
    rise_time_s: Optional[float]  # Time to reach 90% of target (~3 sec expected)
    settling_time_s: Optional[float]  # Time to settle within 2% band
    overshoot_pct: float        # Peak overshoot (%)
    steady_state_error_rad: float  # Final error
    
    # Damping quality
    oscillation_count: int      # Number of oscillation cycles
    oscillation_freq_hz: Optional[float]  # Dominant frequency


@dataclass
class ExperimentMetrics:
    """Complete metrics for one experiment."""
    combo_id: str
    family: str
    name: str
    philosophy: str
    
    joint_metrics: Dict[str, JointMetrics]
    overall_error_rad: float    # AMS of all joint errors
    overall_effort_Nm: float    # RMS of all torques
    disturbance_rejection: float  # Measure of robustness


def compute_rise_time(time: np.ndarray, error: np.ndarray, target_error_threshold: float = 0.1) -> Optional[float]:
    """
    Compute time to reach 90% of steady-state (or first significant reduction).
    """
    if len(time) < 2:
        return None
    
    abs_err = np.abs(error)
    initial = abs_err[0]
    
    if initial < 0.01:
        return None  # Already near target
    
    threshold = initial * 0.1  # 90% reduction
    indices = np.where(abs_err < threshold)[0]
    
    if len(indices) > 0:
        return float(time[indices[0]])
    return None


def compute_settling_time(time: np.ndarray, error: np.ndarray, band_pct: float = 2.0) -> Optional[float]:
    """
    Compute time for error to settle within ±band_pct of final value.
    """
    if len(time) < 2:
        return None
    
    final_error = np.abs(error[-1])
    band = final_error * (band_pct / 100.0)
    
    # Find last time error exits the band
    in_band = np.abs(error) <= band
    last_outside = np.where(~in_band)[0]
    
    if len(last_outside) > 0:
        return float(time[last_outside[-1]])
    return float(time[0])


def compute_overshoot(q_des: np.ndarray, q_meas: np.ndarray) -> float:
    """Compute maximum overshoot as percentage of target change."""
    if len(q_meas) < 2:
        return 0.0
    
    target_change = np.abs(q_des[-1] - q_des[0])
    if target_change < 1e-6:
        return 0.0
    
    # Find peak in the "wrong" direction
    error = q_des - q_meas
    max_overshoot = np.max(np.abs(error))
    
    return float(100.0 * max_overshoot / target_change)


def compute_oscillation_metrics(error: np.ndarray, time: np.ndarray) -> Tuple[int, Optional[float]]:
    """
    Count oscillation cycles and estimate dominant frequency.
    """
    if len(error) < 3:
        return 0, None
    
    dt = np.mean(np.diff(time))
    if dt < 1e-6:
        return 0, None
    
    # Count zero crossings (simplistic measure of oscillations)
    signs = np.sign(error)
    crossings = np.sum(np.abs(np.diff(signs)) > 0)
    cycles = max(0, crossings // 2)
    
    # Estimate dominant frequency via FFT
    if len(error) > 10:
        from scipy import signal
        freq = np.fft.fftfreq(len(error), dt)
        power = np.abs(np.fft.fft(error))
        peak_idx = np.argmax(power[1 : len(freq) // 2]) + 1
        dominant_freq = float(np.abs(freq[peak_idx])) if peak_idx < len(freq) else None
    else:
        dominant_freq = None
    
    return int(cycles), dominant_freq


def compute_joint_metrics(result: ExperimentResult, joint_name: str) -> JointMetrics:
    """Compute all metrics for a single joint across the experiment."""
    df = result.to_dataframe()
    
    if f"e_pos_{joint_name}" not in df.columns:
        raise ValueError(f"Joint '{joint_name}' not found in result data")
    
    # Extract time series
    time = df["t"].values
    q_des = df[f"q_des_{joint_name}"].values
    q_meas = df[f"q_meas_{joint_name}"].values
    qd = df[f"qd_meas_{joint_name}"].values
    e_pos = df[f"e_pos_{joint_name}"].values
    tau_pid = df[f"tau_pid_{joint_name}"].values
    tau_dist = df[f"tau_dist_{joint_name}"].values
    tau_total = df[f"tau_total_{joint_name}"].values
    
    # Position error metrics
    e_pos_abs = np.abs(e_pos)
    e_pos_mean = float(np.mean(e_pos_abs))
    e_pos_max = float(np.max(e_pos_abs))
    e_pos_std = float(np.std(e_pos))
    e_pos_final = float(e_pos[-1])
    
    # Velocity metrics
    qd_mean = float(np.mean(np.abs(qd)))
    qd_max = float(np.max(np.abs(qd)))
    
    # Torque metrics
    tau_pid_mean = float(np.mean(np.abs(tau_pid)))
    tau_pid_max = float(np.max(np.abs(tau_pid)))
    tau_dist_mean = float(np.mean(np.abs(tau_dist)))
    tau_dist_max = float(np.max(np.abs(tau_dist)))
    tau_total_mean = float(np.mean(np.abs(tau_total)))
    tau_total_max = float(np.max(np.abs(tau_total)))
    
    # Dynamic response
    rise_time = compute_rise_time(time, e_pos)
    settling_time = compute_settling_time(time, e_pos)
    overshoot = compute_overshoot(q_des, q_meas)
    osc_count, osc_freq = compute_oscillation_metrics(e_pos, time)
    
    return JointMetrics(
        joint_name=joint_name,
        e_pos_mean=e_pos_mean,
        e_pos_max=e_pos_max,
        e_pos_std=e_pos_std,
        e_pos_final=e_pos_final,
        qd_mean=qd_mean,
        qd_max=qd_max,
        tau_pid_mean=tau_pid_mean,
        tau_pid_max=tau_pid_max,
        tau_dist_mean=tau_dist_mean,
        tau_dist_max=tau_dist_max,
        tau_total_mean=tau_total_mean,
        tau_total_max=tau_total_max,
        rise_time_s=rise_time,
        settling_time_s=settling_time,
        overshoot_pct=overshoot,
        steady_state_error_rad=e_pos_final,
        oscillation_count=osc_count,
        oscillation_freq_hz=osc_freq,
    )


def compute_experiment_metrics(result: ExperimentResult) -> ExperimentMetrics:
    """Compute all metrics for a full experiment."""
    df = result.to_dataframe()
    joint_names = list(result.metadata.gains.keys())
    
    # Per-joint metrics
    joint_metrics = {}
    errors_all = []
    torques_all = []
    
    for joint in joint_names:
        metrics = compute_joint_metrics(result, joint)
        joint_metrics[joint] = metrics
        errors_all.append(df[f"e_pos_{joint}"].values)
        torques_all.append(df[f"tau_total_{joint}"].values)
    
    # Aggregate error (RMS across all joints)
    all_errors = np.concatenate(errors_all)
    overall_error = float(np.sqrt(np.mean(all_errors ** 2)))
    
    # Aggregate effort (RMS across all joints)
    all_torques = np.concatenate(torques_all)
    overall_effort = float(np.sqrt(np.mean(all_torques ** 2)))
    
    # Disturbance rejection (inverse of response to disturbance)
    # Higher error under disturbance = lower rejection
    disturbance_rejection = 1.0 / (1.0 + overall_error)
    
    return ExperimentMetrics(
        combo_id=result.metadata.combo_id,
        family=result.metadata.family,
        name=result.metadata.name,
        philosophy=result.metadata.philosophy,
        joint_metrics=joint_metrics,
        overall_error_rad=overall_error,
        overall_effort_Nm=overall_effort,
        disturbance_rejection=disturbance_rejection,
    )


# ============================================================================
# Plotting Functions
# ============================================================================

def plot_experiment(result: ExperimentResult, output_dir: Optional[Path | str] = None, save_png: bool = True):
    """
    Generate a comprehensive 3-row subplot figure for each joint:
      Row 1: Desired vs. Measured Position
      Row 2: Error and Velocity
      Row 3: Control Torques
    """
    df = result.to_dataframe()
    joint_names = list(result.metadata.gains.keys())
    n_joints = len(joint_names)
    
    # Create figure with 3 rows × n_joints columns
    fig = plt.figure(figsize=(18, 12))
    gs = gridspec.GridSpec(3, n_joints, figure=fig, hspace=0.35, wspace=0.3)
    
    # Color scheme
    color_des = '#1f77b4'  # Blue
    color_meas = '#ff7f0e'  # Orange
    color_error = '#d62728'  # Red
    color_tau = '#2ca02c'   # Green
    
    for idx, joint in enumerate(joint_names):
        # Get data for this joint
        time = df["t"].values
        q_des = df[f"q_des_{joint}"].values
        q_meas = df[f"q_meas_{joint}"].values
        e_pos = df[f"e_pos_{joint}"].values
        qd = df[f"qd_meas_{joint}"].values
        tau_pid = df[f"tau_pid_{joint}"].values
        tau_dist = df[f"tau_dist_{joint}"].values
        tau_total = df[f"tau_total_{joint}"].values
        
        # Convert to degrees for plotting
        q_des_deg = np.rad2deg(q_des)
        q_meas_deg = np.rad2deg(q_meas)
        e_pos_deg = np.rad2deg(e_pos)
        
        # ROW 0: Position
        ax0 = fig.add_subplot(gs[0, idx])
        ax0.plot(time, q_des_deg, label="Desired", color=color_des, linewidth=2, linestyle='--')
        ax0.plot(time, q_meas_deg, label="Measured", color=color_meas, linewidth=2)
        ax0.set_ylabel("Position (°)")
        ax0.set_title(f"{joint}")
        ax0.legend(loc="best", fontsize=8)
        ax0.grid(True, alpha=0.3)
        ax0.set_xlim([time[0], time[-1]])
        
        # Shade phases
        _shade_phases(ax0, df["phase"].values, time)
        
        # ROW 1: Error and Velocity
        ax1 = fig.add_subplot(gs[1, idx])
        ax1_v = ax1.twinx()
        
        ax1.plot(time, e_pos_deg, label="Position Error", color=color_error, linewidth=2)
        ax1_v.plot(time, qd, label="Velocity", color='purple', linewidth=1.5, alpha=0.7)
        
        ax1.set_ylabel("Error (°)", color=color_error)
        ax1_v.set_ylabel("Velocity (rad/s)", color='purple')
        ax1.tick_params(axis='y', labelcolor=color_error)
        ax1_v.tick_params(axis='y', labelcolor='purple')
        ax1.grid(True, alpha=0.3)
        ax1.set_xlim([time[0], time[-1]])
        ax1.axhline(0, color='black', linestyle='-', linewidth=0.5, alpha=0.5)
        
        # ROW 2: Torques
        ax2 = fig.add_subplot(gs[2, idx])
        ax2.plot(time, tau_pid, label="PID Torque", color=color_tau, linewidth=1.5, alpha=0.8)
        ax2.plot(time, tau_dist, label="Disturbance", color='red', linewidth=0.8, alpha=0.6)
        ax2.plot(time, tau_total, label="Total", color='black', linewidth=2)
        
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Torque (N⋅m)")
        ax2.legend(loc="best", fontsize=8)
        ax2.grid(True, alpha=0.3)
        ax2.set_xlim([time[0], time[-1]])
        ax2.axhline(0, color='black', linestyle='-', linewidth=0.5, alpha=0.5)
    
    # Add overall title
    fig.suptitle(
        f"{result.metadata.combo_id} - {result.metadata.name}\n"
        f"Philosophy: {result.metadata.philosophy}",
        fontsize=14, fontweight='bold'
    )
    
    if output_dir is not None and save_png:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_dir / f"{result.metadata.combo_id}_overview.png", dpi=150, bbox_inches='tight')
    
    return fig


def _shade_phases(ax, phases: np.ndarray, time: np.ndarray):
    """Shade background by phase for clarity."""
    phase_colors = {
        "move_to_zero": "#ffcccc",
        "hold_zero": "#ccffcc",
        "return_start": "#ccccff",
        "hold_start": "#ffffcc",
    }
    
    current_phase = None
    start_idx = 0
    
    for idx, phase in enumerate(phases):
        if phase != current_phase:
            if current_phase is not None:
                t_start = time[start_idx]
                t_end = time[idx - 1] if idx > 0 else time[idx]
                ax.axvspan(t_start, t_end, alpha=0.1, color=phase_colors.get(current_phase, 'gray'))
            current_phase = phase
            start_idx = idx
    
    # Shade final phase
    if current_phase is not None:
        t_start = time[start_idx]
        t_end = time[-1]
        ax.axvspan(t_start, t_end, alpha=0.1, color=phase_colors.get(current_phase, 'gray'))


def plot_family_comparison(results: Dict[str, ExperimentResult], family: str, output_dir: Optional[Path | str] = None):
    """
    Compare all combinations within a controller family side-by-side.
    Shows position and error for each joint, one row per combination.
    """
    joint_names = list(next(iter(results.values())).metadata.gains.keys())
    n_joints = len(joint_names)
    n_combos = len(results)
    
    fig = plt.figure(figsize=(20, 3 * n_combos))
    gs = gridspec.GridSpec(n_combos, n_joints, figure=fig, hspace=0.4, wspace=0.3)
    
    for combo_idx, (combo_id, result) in enumerate(sorted(results.items())):
        df = result.to_dataframe()
        time = df["t"].values
        
        for joint_idx, joint in enumerate(joint_names):
            ax = fig.add_subplot(gs[combo_idx, joint_idx])
            
            q_des = np.rad2deg(df[f"q_des_{joint}"].values)
            q_meas = np.rad2deg(df[f"q_meas_{joint}"].values)
            
            ax.plot(time, q_des, label="Desired", color='blue', linestyle='--', linewidth=1.5)
            ax.plot(time, q_meas, label="Measured", color='orange', linewidth=1.5)
            ax.set_ylabel("Position (°)")
            ax.grid(True, alpha=0.3)
            ax.set_xlim([time[0], time[-1]])
            
            if combo_idx == 0:
                ax.set_title(joint, fontsize=10, fontweight='bold')
            if joint_idx == 0:
                ax.text(-0.5, 0.5, result.metadata.name, rotation=90, 
                       verticalalignment='center', horizontalalignment='right',
                       transform=ax.transAxes, fontsize=9, fontweight='bold')
            
            ax.legend(loc='best', fontsize=7)
    
    fig.suptitle(f"{family} Control Family Comparison", fontsize=14, fontweight='bold')
    
    if output_dir is not None:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_dir / f"{family}_comparison.png", dpi=150, bbox_inches='tight')
    
    return fig


# ============================================================================
# Qualitative Table Generation
# ============================================================================

def generate_qualitative_table(results: Dict[str, ExperimentMetrics], family: str) -> pd.DataFrame:
    """
    Generate a qualitative observation table for a controller family.
    
    Returns a DataFrame with columns:
      - Combo ID
      - Gains (summary)
      - Rise & Speed
      - Overshoot
      - Oscillation
      - Steady-State Error
      - Disturbance Reaction
      - Stability
      - Engineering Notes
    """
    rows = []
    
    for combo_id in sorted(results.keys()):
        metrics = results[combo_id]
        
        # Aggregate metrics across all joints
        rise_times = [m.rise_time_s for m in metrics.joint_metrics.values() if m.rise_time_s is not None]
        overshoots = [m.overshoot_pct for m in metrics.joint_metrics.values()]
        osc_counts = [m.oscillation_count for m in metrics.joint_metrics.values()]
        e_final = [m.steady_state_error_rad for m in metrics.joint_metrics.values()]
        
        # Qualitative assessments
        if rise_times:
            avg_rise = np.mean(rise_times)
            if avg_rise > 3.0:
                rise_desc = "Very slow"
            elif avg_rise > 1.5:
                rise_desc = "Slow"
            elif avg_rise > 0.8:
                rise_desc = "Moderate"
            elif avg_rise > 0.4:
                rise_desc = "Fast"
            else:
                rise_desc = "Very fast"
        else:
            rise_desc = "N/A"
        
        avg_overshoot = np.mean(overshoots) if overshoots else 0.0
        if avg_overshoot < 2:
            overshoot_desc = "None"
        elif avg_overshoot < 10:
            overshoot_desc = "Small"
        elif avg_overshoot < 25:
            overshoot_desc = "Moderate"
        else:
            overshoot_desc = "Large"
        
        avg_osc = np.mean(osc_counts)
        if avg_osc < 1:
            osc_desc = "None"
        elif avg_osc < 2:
            osc_desc = "Light ringing"
        elif avg_osc < 4:
            osc_desc = "Moderate ringing"
        else:
            osc_desc = "Persistent oscillation"
        
        avg_e_final = np.mean(np.abs(e_final))
        if avg_e_final < 0.01:
            sse_desc = "≈ 0 (excellent)"
        elif avg_e_final < 0.05:
            sse_desc = "Small (~0.05 rad)"
        elif avg_e_final < 0.15:
            sse_desc = "Moderate (~0.15 rad)"
        else:
            sse_desc = "Large (>0.15 rad)"
        
        disturbance = f"{metrics.disturbance_rejection:.2f}"
        
        # Stability assessment
        if avg_overshoot > 40 or avg_osc > 5:
            stability = "Marginal"
        elif avg_overshoot > 20 or avg_osc > 3:
            stability = "Questionable"
        else:
            stability = "Stable"
        
        # Engineering notes
        kp_base = list(metrics.joint_metrics.values())[0].steady_state_error_rad  # placeholder
        notes = metrics.philosophy[:60] + "..."
        
        rows.append({
            "Combo ID": combo_id,
            "Name": metrics.name,
            "Rise & Speed": rise_desc,
            "Overshoot": overshoot_desc,
            "Oscillation": osc_desc,
            "Steady-State Error": sse_desc,
            "Disturbance Rejection": disturbance,
            "Stability": stability,
            "Engineering Notes": notes,
        })
    
    return pd.DataFrame(rows)


# ============================================================================
# Export and Reporting
# ============================================================================

def save_analysis_report(dataset: ExperimentDataset, output_dir: Path | str = "results/analysis"):
    """Generate and save a complete analysis report with tables and plots."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Compute metrics for all experiments
    all_metrics = {}
    for combo_id, result in dataset.results.items():
        all_metrics[combo_id] = compute_experiment_metrics(result)
    
    # Group by family
    families = {}
    for combo_id, metrics in all_metrics.items():
        family = metrics.family
        if family not in families:
            families[family] = {}
        families[family][combo_id] = metrics
    
    # Generate tables
    for family, metric_dict in families.items():
        table_df = generate_qualitative_table(metric_dict, family)
        table_df.to_csv(output_dir / f"{family}_qualitative_table.csv", index=False)
        print(f"\n{family} Controller Family:")
        print(table_df.to_string(index=False))
    
    # Generate comparison plots
    for family, result_dict in dataset.get_by_family("P").items() if "P" in families else []:
        try:
            plot_family_comparison(result_dict, family, output_dir)
        except Exception as e:
            print(f"Error plotting {family}: {e}")


if __name__ == "__main__":
    print("PID Analysis Module")
    print("Use compute_experiment_metrics() to analyze a single experiment")
    print("Use generate_qualitative_table() to create comparison tables")
    print("Use plot_experiment() to visualize results")
