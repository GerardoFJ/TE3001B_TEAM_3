#!/usr/bin/env python3
"""
plot_experiments.py — Generate all analysis plots and qualitative tables
                      for the SO101 PID tuning study.

Usage:
    python3 plot_experiments.py [--results-dir results] [--out-dir plots]

Outputs (saved under <out_dir>/):
  <family>_positions.png   — per-family grid of joint position vs time (5 combos)
  <family>_errors.png      — per-family RMS error bar chart
  best_comparison.png      — all four best configs overlaid on shoulder_pan
  summary_table.csv        — machine-readable qualitative metrics table
"""
from __future__ import annotations

import argparse
import os
import sys
import warnings

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D

# Allow running from scripts/ directory
_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from experiment_configs import (
    ALL_CONFIGS, P_CONFIGS, PD_CONFIGS, PI_CONFIGS, PID_CONFIGS,
    ExperimentConfig,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
PHASES = ["move_to_zero", "hold_zero", "return_to_start", "hold_start"]
PHASE_COLORS = {
    "move_to_zero":    "#e0e0e0",
    "hold_zero":       "#c8e6c9",
    "return_to_start": "#fff9c4",
    "hold_start":      "#bbdefb",
}
FAMILY_COLORS = {
    "P":   "#e53935",
    "PD":  "#1e88e5",
    "PI":  "#43a047",
    "PID": "#8e24aa",
}
FAMILIES = [
    ("P",   P_CONFIGS),
    ("PD",  PD_CONFIGS),
    ("PI",  PI_CONFIGS),
    ("PID", PID_CONFIGS),
]


# ---------------------------------------------------------------------------
# Data loading helpers
# ---------------------------------------------------------------------------
def load_state(results_dir: str, combo_id: str) -> pd.DataFrame | None:
    path = os.path.join(results_dir, combo_id, "state.csv")
    if not os.path.exists(path):
        warnings.warn(f"Missing: {path}")
        return None
    return pd.read_csv(path)


def rms(series: pd.Series) -> float:
    return float(np.sqrt(np.mean(series**2)))


def max_abs(series: pd.Series) -> float:
    return float(series.abs().max())


def overshoot_pct(actual: pd.Series, desired: pd.Series) -> float:
    """
    Overshoot as percentage of total travel: max(actual - desired_final) / |travel|.
    Only meaningful during move phases.
    """
    final = float(desired.iloc[-1])
    initial = float(desired.iloc[0])
    travel = abs(final - initial)
    if travel < 1e-3:
        return 0.0
    peak_error = float((actual - final).abs().max())
    return 100.0 * peak_error / travel


# ---------------------------------------------------------------------------
# Per-family position grid plot
# ---------------------------------------------------------------------------
def plot_family_positions(
    family_name: str,
    configs: list[ExperimentConfig],
    results_dir: str,
    out_dir: str,
) -> None:
    """
    5-row grid. Each row = one combo. Columns = joints.
    Shows actual (solid) vs desired (dashed) with phase shading.
    """
    n_combos = len(configs)
    n_joints  = len(JOINT_NAMES)
    fig, axes = plt.subplots(
        n_combos, n_joints,
        figsize=(4 * n_joints, 2.8 * n_combos),
        sharex=False, sharey=False,
    )
    if n_combos == 1:
        axes = axes[np.newaxis, :]

    fig.suptitle(f"Family {family_name} — Joint Positions vs Time", fontsize=14, y=1.01)

    for row, cfg in enumerate(configs):
        df = load_state(results_dir, cfg.combo_id)
        for col, jn in enumerate(JOINT_NAMES):
            ax = axes[row, col]

            if df is None:
                ax.text(0.5, 0.5, "no data", ha="center", va="center",
                        transform=ax.transAxes)
                continue

            t = df["t"].values
            actual  = df[f"{jn}_actual_deg"].values
            desired = df[f"{jn}_desired_deg"].values

            # Phase background shading
            for phase, color in PHASE_COLORS.items():
                mask = df["phase"] == phase
                if mask.any():
                    t_phase = t[mask.values]
                    ax.axvspan(t_phase[0], t_phase[-1], color=color, alpha=0.4, zorder=0)

            ax.plot(t, actual,  lw=1.2, color=FAMILY_COLORS[family_name], label="actual")
            ax.plot(t, desired, lw=0.9, color="black", ls="--", alpha=0.7, label="desired")

            if row == 0:
                ax.set_title(jn.replace("_", "\n"), fontsize=9)
            if col == 0:
                short = cfg.combo_id
                ax.set_ylabel(f"{short}\n(deg)", fontsize=8)
            ax.tick_params(labelsize=7)
            ax.grid(True, lw=0.4, alpha=0.5)

    # Legend
    legend_elements = [
        Line2D([0], [0], color=FAMILY_COLORS[family_name], lw=1.5, label="actual"),
        Line2D([0], [0], color="black", lw=1, ls="--", alpha=0.7, label="desired"),
    ]
    # Phase legend patches
    from matplotlib.patches import Patch
    for phase, color in PHASE_COLORS.items():
        legend_elements.append(Patch(facecolor=color, alpha=0.4,
                                     label=phase.replace("_", " ")))
    fig.legend(handles=legend_elements, loc="lower center",
               ncol=len(legend_elements), fontsize=8,
               bbox_to_anchor=(0.5, -0.02))

    plt.tight_layout()
    path = os.path.join(out_dir, f"{family_name}_positions.png")
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Per-family RMS error bar chart
# ---------------------------------------------------------------------------
def plot_family_errors(
    family_name: str,
    configs: list[ExperimentConfig],
    results_dir: str,
    out_dir: str,
) -> None:
    """
    Bar chart: RMS tracking error per joint per combo.
    Uses only hold phases (steady-state) to separate transient from steady-state error.
    """
    fig, axes = plt.subplots(1, len(JOINT_NAMES),
                             figsize=(3.5 * len(JOINT_NAMES), 4), sharey=False)
    fig.suptitle(f"Family {family_name} — RMS Tracking Error (hold phases)", fontsize=12)

    x = np.arange(len(configs))
    labels = [c.combo_id for c in configs]

    for col, jn in enumerate(JOINT_NAMES):
        ax = axes[col]
        rms_vals = []
        for cfg in configs:
            df = load_state(results_dir, cfg.combo_id)
            if df is None:
                rms_vals.append(0.0)
                continue
            hold_mask = df["phase"].isin(["hold_zero", "hold_start"])
            err_col   = f"{jn}_error_deg"
            if err_col not in df.columns:
                rms_vals.append(0.0)
                continue
            rms_vals.append(rms(df.loc[hold_mask, err_col]))

        bars = ax.bar(x, rms_vals, color=FAMILY_COLORS[family_name], alpha=0.8, edgecolor="white")
        ax.set_xticks(x)
        ax.set_xticklabels(labels, fontsize=8)
        ax.set_title(jn.replace("_", "\n"), fontsize=9)
        ax.set_ylabel("RMS error (deg)", fontsize=8)
        ax.grid(True, axis="y", lw=0.4, alpha=0.5)
        for bar, val in zip(bars, rms_vals):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.02,
                    f"{val:.2f}", ha="center", va="bottom", fontsize=7)

    plt.tight_layout()
    path = os.path.join(out_dir, f"{family_name}_errors.png")
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Best-per-family comparison on a single joint
# ---------------------------------------------------------------------------
def plot_best_comparison(
    results_dir: str,
    out_dir: str,
    best_ids: dict[str, str],
    representative_joint: str = "shoulder_pan",
) -> None:
    """
    Overlays the four best configs (one per family) on one joint.
    """
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.set_title(
        f"Best per family — {representative_joint} (actual position vs time)",
        fontsize=11,
    )

    for family, cid in best_ids.items():
        df = load_state(results_dir, cid)
        if df is None:
            continue
        t = df["t"].values
        actual  = df[f"{representative_joint}_actual_deg"].values
        desired = df[f"{representative_joint}_desired_deg"].values
        ax.plot(t, actual,  lw=1.5, color=FAMILY_COLORS[family], label=f"{family} ({cid}) actual")

    # Desired trajectory from any available run
    for cid in best_ids.values():
        df = load_state(results_dir, cid)
        if df is not None:
            ax.plot(df["t"].values,
                    df[f"{representative_joint}_desired_deg"].values,
                    lw=1.0, ls="--", color="black", alpha=0.5, label="desired")
            break

    # Phase shading
    if df is not None:
        for phase, color in PHASE_COLORS.items():
            mask = df["phase"] == phase
            if mask.any():
                t_phase = df["t"].values[mask.values]
                ax.axvspan(t_phase[0], t_phase[-1], color=color, alpha=0.3, zorder=0)

    ax.set_xlabel("time (s)", fontsize=10)
    ax.set_ylabel("position (deg)", fontsize=10)
    ax.grid(True, lw=0.4, alpha=0.5)
    ax.legend(fontsize=8, loc="upper right")

    plt.tight_layout()
    path = os.path.join(out_dir, "best_comparison.png")
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Quantitative metrics → CSV summary table
# ---------------------------------------------------------------------------
def build_summary_table(results_dir: str, out_dir: str) -> pd.DataFrame:
    """
    Computes per-combo metrics and writes summary_table.csv.
    Metrics (on shoulder_pan as representative joint):
      - rms_move_error_deg    : RMS error during move phases
      - rms_hold_error_deg    : RMS error during hold phases (steady-state)
      - max_overshoot_deg     : peak |actual − desired_final| during moves
      - settling_time_s       : first time |error| < 2 deg and stays < 2 deg for 0.5s
    """
    rows = []
    jn = "shoulder_pan"

    for cfg in ALL_CONFIGS:
        df = load_state(results_dir, cfg.combo_id)
        if df is None:
            continue

        err_col = f"{jn}_error_deg"
        if err_col not in df.columns:
            continue

        # Move RMS
        move_mask = df["phase"].isin(["move_to_zero", "return_to_start"])
        hold_mask = df["phase"].isin(["hold_zero", "hold_start"])

        rms_move = rms(df.loc[move_mask, err_col]) if move_mask.any() else float("nan")
        rms_hold = rms(df.loc[hold_mask, err_col]) if hold_mask.any() else float("nan")

        # Overshoot: max absolute error during move to zero
        mz_mask = df["phase"] == "move_to_zero"
        if mz_mask.any():
            sub = df[mz_mask]
            final_desired = float(sub[f"{jn}_desired_deg"].iloc[-1])
            overshoot = float((sub[f"{jn}_actual_deg"] - final_desired).abs().max())
        else:
            overshoot = float("nan")

        # Settling time: within hold_zero phase, time until |error| < 2 deg
        if hold_mask.any():
            sub  = df[df["phase"] == "hold_zero"].reset_index(drop=True)
            thresh = 2.0  # deg
            t0_hold = float(sub["t"].iloc[0])
            settled = sub[sub[err_col].abs() < thresh]
            settling_time = float(settled["t"].iloc[0] - t0_hold) if len(settled) > 0 else float("nan")
        else:
            settling_time = float("nan")

        rows.append({
            "combo_id":         cfg.combo_id,
            "family":           cfg.family,
            "label":            cfg.label,
            "kp_scale":         cfg.kp_scale,
            "ki_scale":         cfg.ki_scale,
            "kd_scale":         cfg.kd_scale,
            "rms_move_error_deg": round(rms_move, 3),
            "rms_hold_error_deg": round(rms_hold, 3),
            "max_overshoot_deg":  round(overshoot, 3),
            "settling_time_s":    round(settling_time, 3),
        })

    df_out = pd.DataFrame(rows)
    path = os.path.join(out_dir, "summary_table.csv")
    df_out.to_csv(path, index=False)
    print(f"  Saved: {path}")
    return df_out


# ---------------------------------------------------------------------------
# Per-family qualitative table plot  (rendered as a matplotlib table image)
# ---------------------------------------------------------------------------
def _qualitative_row(cfg: ExperimentConfig, df: pd.DataFrame | None) -> list:
    """
    Derive qualitative descriptors from numeric data.
    Kept brief so they fit in a table cell.
    """
    jn = "shoulder_pan"
    if df is None:
        return [cfg.combo_id, cfg.gains_summary(), "—", "—", "—", "—", "—", "—"]

    err_col   = f"{jn}_error_deg"
    move_mask = df["phase"].isin(["move_to_zero", "return_to_start"])
    hold_mask = df["phase"].isin(["hold_zero", "hold_start"])

    rms_move = rms(df.loc[move_mask, err_col]) if move_mask.any() else float("nan")
    rms_hold = rms(df.loc[hold_mask, err_col]) if hold_mask.any() else float("nan")

    # Speed: average error drop rate in first 0.5 s of move_to_zero
    mz = df[df["phase"] == "move_to_zero"].reset_index(drop=True)
    if len(mz) > 10:
        early = mz[mz["t"] - mz["t"].iloc[0] < 0.5]
        speed = "Fast" if len(early) and early[err_col].abs().mean() < 20 else "Slow"
    else:
        speed = "N/A"

    # Overshoot
    if len(mz) > 0:
        final_des = float(mz[f"{jn}_desired_deg"].iloc[-1])
        osh = float((mz[f"{jn}_actual_deg"] - final_des).abs().max())
        overshoot_str = "Large (>{:.0f}°)".format(osh) if osh > 10 else \
                        "Moderate ({:.0f}°)".format(osh) if osh > 3 else "Small (<3°)"
    else:
        overshoot_str = "N/A"

    # Oscillation: std of hold error
    hold_err_std = float(df.loc[hold_mask, err_col].std()) if hold_mask.any() else float("nan")
    osc_str = "High σ={:.1f}°".format(hold_err_std) if hold_err_std > 5 else \
              "Moderate σ={:.1f}°".format(hold_err_std) if hold_err_std > 2 else \
              "Low σ={:.1f}°".format(hold_err_std)

    # Steady-state error
    sse_str = "Large ({:.1f}°)".format(rms_hold) if rms_hold > 10 else \
              "Medium ({:.1f}°)".format(rms_hold) if rms_hold > 3 else \
              "Small ({:.1f}°)".format(rms_hold)

    # Stability: did trajectory diverge?
    max_err = max_abs(df[err_col])
    stable_str = "Diverged" if max_err > 120 else \
                 "Marginal"  if max_err > 60  else "Stable"

    return [
        cfg.combo_id,
        cfg.gains_summary().split("(")[0].strip(),
        speed,
        overshoot_str,
        osc_str,
        sse_str,
        stable_str,
        cfg.label,
    ]


def plot_qualitative_table(
    family_name: str,
    configs: list[ExperimentConfig],
    results_dir: str,
    out_dir: str,
) -> None:
    col_headers = ["ID", "Gains (scale)", "Speed", "Overshoot",
                   "Oscillation", "Steady-State Error", "Stability", "Notes"]

    table_data = []
    for cfg in configs:
        df = load_state(results_dir, cfg.combo_id)
        table_data.append(_qualitative_row(cfg, df))

    fig, ax = plt.subplots(figsize=(22, 1.2 * len(configs) + 1.5))
    ax.axis("off")
    ax.set_title(f"Qualitative Analysis Table — Family {family_name}", fontsize=12, pad=10)

    tbl = ax.table(
        cellText=table_data,
        colLabels=col_headers,
        cellLoc="left",
        loc="center",
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8)
    tbl.scale(1, 1.8)

    # Style header
    for j in range(len(col_headers)):
        tbl[(0, j)].set_facecolor("#37474f")
        tbl[(0, j)].set_text_props(color="white", fontweight="bold")

    # Alternate row shading
    for i in range(1, len(configs) + 1):
        for j in range(len(col_headers)):
            tbl[(i, j)].set_facecolor("#f5f5f5" if i % 2 == 0 else "white")

    plt.tight_layout()
    path = os.path.join(out_dir, f"{family_name}_table.png")
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Shoulder-pan zoom plot: actual vs desired for all combos in a family
# (single-axis overlay — good for comparing all 5 at once)
# ---------------------------------------------------------------------------
def plot_family_overlay(
    family_name: str,
    configs: list[ExperimentConfig],
    results_dir: str,
    out_dir: str,
    joint: str = "shoulder_pan",
) -> None:
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.set_title(f"Family {family_name} — {joint} actual position (all combos)", fontsize=11)

    cmap = matplotlib.colormaps.get_cmap("tab10").resampled(len(configs))
    for i, cfg in enumerate(configs):
        df = load_state(results_dir, cfg.combo_id)
        if df is None:
            continue
        t = df["t"].values
        ax.plot(t, df[f"{joint}_actual_deg"].values,
                lw=1.3, color=cmap(i), label=f"{cfg.combo_id}")

    # Desired from first available
    for cfg in configs:
        df = load_state(results_dir, cfg.combo_id)
        if df is not None:
            ax.plot(df["t"].values, df[f"{joint}_desired_deg"].values,
                    lw=1.5, ls="--", color="black", alpha=0.6, label="desired")
            for phase, color in PHASE_COLORS.items():
                mask = df["phase"] == phase
                if mask.any():
                    t_ph = df["t"].values[mask.values]
                    ax.axvspan(t_ph[0], t_ph[-1], color=color, alpha=0.25, zorder=0)
            break

    ax.set_xlabel("time (s)", fontsize=10)
    ax.set_ylabel("position (deg)", fontsize=10)
    ax.grid(True, lw=0.4, alpha=0.5)
    ax.legend(fontsize=8, loc="upper right", ncol=2)
    plt.tight_layout()
    path = os.path.join(out_dir, f"{family_name}_overlay_{joint}.png")
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(description="Plot SO101 PID experiment results")
    parser.add_argument("--results-dir", default="results",
                        help="Directory containing per-combo CSV folders")
    parser.add_argument("--out-dir",     default="plots",
                        help="Directory to write PNG plots")
    parser.add_argument(
        "--best",
        nargs=4,
        metavar=("P_ID", "PD_ID", "PI_ID", "PID_ID"),
        default=["P4", "PD3", "PI3", "PID5"],
        help="Best combo ID for each family (for comparison plot)",
    )
    args = parser.parse_args()

    results = os.path.abspath(args.results_dir)
    out_dir  = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    best_ids = {
        "P":   args.best[0],
        "PD":  args.best[1],
        "PI":  args.best[2],
        "PID": args.best[3],
    }

    print(f"Results: {results}")
    print(f"Plots  : {out_dir}\n")

    for family_name, configs in FAMILIES:
        print(f"--- Family {family_name} ---")
        plot_family_positions(family_name, configs, results, out_dir)
        plot_family_errors(family_name, configs, results, out_dir)
        plot_qualitative_table(family_name, configs, results, out_dir)
        plot_family_overlay(family_name, configs, results, out_dir)

    print("\n--- Cross-family comparison ---")
    plot_best_comparison(results, out_dir, best_ids)

    print("\n--- Summary table ---")
    df_summary = build_summary_table(results, out_dir)
    print(df_summary.to_string(index=False))


if __name__ == "__main__":
    main()
