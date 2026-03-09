#!/usr/bin/env python3
"""
TE300XB-5 — Analysis and Plotting Script
Generates all required plots from Section 10 of the handout.

Run from any directory:
    python3 ~/dev_ws/src/xarm_ros2/xarm_perturbations/analysis/plot_trials.py
"""
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
RESULTS = os.path.expanduser(
    '~/dev_ws/src/xarm_ros2/xarm_perturbations/results')
PLOTS = os.path.expanduser(
    '~/dev_ws/src/xarm_ros2/xarm_perturbations/analysis/plots')
os.makedirs(PLOTS, exist_ok=True)

TRIALS = {
    'ctc_nopert':   'trial_ctc_nopert',
    'pid_nopert':   'trial_pdpid_nopert',
    'ctc_pert':     'trial_ctc_pert',
    'pid_pert':     'trial_pdpid_pert',
}
LABELS = {
    'ctc_nopert': 'CTC  (sin perturb.)',
    'pid_nopert': 'PID  (sin perturb.)',
    'ctc_pert':   'CTC  (con perturb.)',
    'pid_pert':   'PID  (con perturb.)',
}
COLORS = {
    'ctc_nopert': '#1f77b4',
    'pid_nopert': '#ff7f0e',
    'ctc_pert':   '#2ca02c',
    'pid_pert':   '#d62728',
}

EPS_THRESH = 0.005   # 5 mm waypoint success threshold


# ---------------------------------------------------------------------------
# Load CSVs  (pick newest file per trial folder)
# ---------------------------------------------------------------------------
def load_trial(folder_name: str) -> pd.DataFrame:
    folder = os.path.join(RESULTS, folder_name)
    csvs = sorted([f for f in os.listdir(folder) if f.endswith('.csv')])
    if not csvs:
        raise FileNotFoundError(f'No CSV in {folder}')
    path = os.path.join(folder, csvs[-1])   # newest
    print(f'  Loading {path}')
    return pd.read_csv(path)


print('Loading trial data...')
data = {k: load_trial(v) for k, v in TRIALS.items()}


# ---------------------------------------------------------------------------
# Helper — detect dwell windows from perturb_active or constant p_des
# ---------------------------------------------------------------------------
def dwell_mask(df: pd.DataFrame) -> np.ndarray:
    """True where p_des is constant (dwell period)."""
    p = df[['p_des_x', 'p_des_y', 'p_des_z']].values
    diff = np.linalg.norm(np.diff(p, axis=0), axis=1)
    mask = np.concatenate([[False], diff < 1e-6])
    return mask


def shade_dwells(ax, t, mask, alpha=0.15, color='grey'):
    in_dwell = False
    t0 = None
    for i, (ti, m) in enumerate(zip(t, mask)):
        if m and not in_dwell:
            t0 = ti
            in_dwell = True
        elif not m and in_dwell:
            ax.axvspan(t0, ti, alpha=alpha, color=color)
            in_dwell = False
    if in_dwell:
        ax.axvspan(t0, t[-1], alpha=alpha, color=color)


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------
def compute_metrics(df: pd.DataFrame) -> dict:
    q     = df[[f'q{i}'     for i in range(1, 7)]].values
    q_des = df[[f'q_des{i}' for i in range(1, 7)]].values
    e_j   = q - q_des

    p     = df[['p_x',     'p_y',     'p_z']].values
    p_des = df[['p_des_x', 'p_des_y', 'p_des_z']].values
    e_ee  = np.linalg.norm(p - p_des, axis=1)

    dm = dwell_mask(df)

    # Waypoint success: e_EE < EPS during last 50% of each dwell window
    success, total = 0, 0
    in_dwell, seg = False, []
    for i, m in enumerate(dm):
        if m and not in_dwell:
            seg = [i]
            in_dwell = True
        elif m and in_dwell:
            seg.append(i)
        elif not m and in_dwell:
            half = seg[len(seg)//2:]
            total += 1
            if np.all(e_ee[half] < EPS_THRESH):
                success += 1
            in_dwell = False

    return {
        'joint_rmse':     np.sqrt(np.mean(e_j**2, axis=0)),
        'joint_max':      np.max(np.abs(e_j), axis=0),
        'ee_rmse':        float(np.sqrt(np.mean(e_ee**2))),
        'ee_max':         float(np.max(e_ee)),
        'wp_success':     100.0 * success / total if total else 0.0,
        'wp_total':       total,
        'e_ee':           e_ee,
        'e_j':            e_j,
    }


print('Computing metrics...')
metrics = {k: compute_metrics(v) for k, v in data.items()}


# ===========================================================================
# PLOT 1 — Joint Tracking  (one figure per trial, 6 subplots)
# ===========================================================================
def plot_joint_tracking(key: str):
    df  = data[key]
    t   = df['time_rel'].values
    dm  = dwell_mask(df)
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle(f'Joint Tracking — {LABELS[key]}', fontsize=13)
    for j in range(6):
        ax = axes[j // 2, j % 2]
        ax.plot(t, df[f'q{j+1}'].values,     label='Measured',  lw=1.2)
        ax.plot(t, df[f'q_des{j+1}'].values, label='Desired',
                lw=1.2, ls='--', color='orange')
        shade_dwells(ax, t, dm)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(f'$q_{j+1}$ [rad]')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.4)
    plt.tight_layout()
    out = os.path.join(PLOTS, f'joint_tracking_{key}.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f'  Saved {out}')


# ===========================================================================
# PLOT 2 — Task-Space XYZ tracking  (one figure per trial, 3 subplots)
# ===========================================================================
def plot_taskspace(key: str):
    df  = data[key]
    t   = df['time_rel'].values
    dm  = dwell_mask(df)
    fig, axes = plt.subplots(3, 1, figsize=(12, 9))
    fig.suptitle(f'Task-Space Tracking — {LABELS[key]}', fontsize=13)
    for i, ax_label in enumerate(['X', 'Y', 'Z']):
        ax = axes[i]
        ax.plot(t, df[f'p_{ax_label.lower()}'].values,
                label='Measured', lw=1.2)
        ax.plot(t, df[f'p_des_{ax_label.lower()}'].values,
                label='Desired', lw=1.2, ls='--', color='orange')
        shade_dwells(ax, t, dm)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(f'{ax_label} [m]')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.4)
    plt.tight_layout()
    out = os.path.join(PLOTS, f'taskspace_xyz_{key}.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f'  Saved {out}')


# ===========================================================================
# PLOT 3 — Path projections XY / XZ / YZ  (one figure per trial)
# ===========================================================================
def plot_3d_path(key: str):
    df  = data[key]
    dm  = dwell_mask(df)

    # Waypoint positions
    wp_x, wp_y, wp_z = [], [], []
    prev = False
    for i, m in enumerate(dm):
        if m and not prev:
            wp_x.append(df['p_des_x'].iloc[i])
            wp_y.append(df['p_des_y'].iloc[i])
            wp_z.append(df['p_des_z'].iloc[i])
        prev = m

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle(f'Path Projections — {LABELS[key]}', fontsize=13)

    projections = [('p_x', 'p_y', 'X [m]', 'Y [m]', wp_x, wp_y),
                   ('p_x', 'p_z', 'X [m]', 'Z [m]', wp_x, wp_z),
                   ('p_y', 'p_z', 'Y [m]', 'Z [m]', wp_y, wp_z)]

    for ax, (cx, cy, xl, yl, wpx, wpy) in zip(axes, projections):
        ax.plot(df[cx], df[cy], lw=1.0, color=COLORS[key], label='Actual')
        ax.plot(df[cx.replace('p_', 'p_des_')],
                df[cy.replace('p_', 'p_des_')],
                lw=1.0, ls='--', color='orange', label='Desired')
        ax.scatter(wpx, wpy, s=50, c='red', zorder=5, label='Waypoints')
        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.4)

    plt.tight_layout()
    out = os.path.join(PLOTS, f'path_3d_{key}.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f'  Saved {out}')


# ===========================================================================
# PLOT 4 — EE Error norm over time  (one figure per trial)
# ===========================================================================
def plot_ee_error(key: str):
    df  = data[key]
    t   = df['time_rel'].values
    dm  = dwell_mask(df)
    e   = metrics[key]['e_ee']
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, e * 1000, lw=1.2, color=COLORS[key], label='$e_{EE}$')
    ax.axhline(EPS_THRESH * 1000, ls='--', color='red',
               label=f'Threshold {EPS_THRESH*1000:.0f} mm')
    shade_dwells(ax, t, dm)
    if df['perturb_active'].any():
        pa = df['perturb_active'].values.astype(bool)
        ax.fill_between(t, 0, ax.get_ylim()[1] if ax.get_ylim()[1] > 0 else 50,
                        where=pa, alpha=0.15, color='purple',
                        label='Perturbation active')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('$e_{EE}$ [mm]')
    ax.set_title(f'End-Effector Error — {LABELS[key]}')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.4)
    plt.tight_layout()
    out = os.path.join(PLOTS, f'ee_error_{key}.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f'  Saved {out}')


# ===========================================================================
# PLOT 5 — Phase Portraits  e_j vs ė_j  (one figure per trial, 6 joints)
# ===========================================================================
def plot_phase_portraits(key: str):
    df    = data[key]
    e_j   = metrics[key]['e_j']
    q_d   = df[[f'qd{i}'     for i in range(1, 7)]].values
    qd_des= df[[f'qd_des{i}' for i in range(1, 7)]].values
    e_dot = q_d - qd_des

    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    fig.suptitle(f'Phase Portraits — {LABELS[key]}', fontsize=13)
    for j in range(6):
        ax = axes[j // 3, j % 3]
        ax.plot(e_j[:, j], e_dot[:, j], lw=0.6, alpha=0.7,
                color=COLORS[key])
        ax.scatter(0, 0, s=80, c='red', zorder=5, label='Equilibrium')
        ax.set_xlabel(f'$e_{j+1}$ [rad]')
        ax.set_ylabel(f'$\\dot{{e}}_{j+1}$ [rad/s]')
        ax.set_title(f'Joint {j+1}')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.4)
    plt.tight_layout()
    out = os.path.join(PLOTS, f'phase_portraits_{key}.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f'  Saved {out}')


# ===========================================================================
# PLOT 6 — Comparison: CTC vs PID  (no-pert and with-pert)
# ===========================================================================
def plot_comparison(condition: str, k1: str, k2: str):
    """Overlay EE error and task-space Z for two controllers."""
    df1, df2 = data[k1], data[k2]
    t1 = df1['time_rel'].values
    t2 = df2['time_rel'].values
    e1 = metrics[k1]['e_ee'] * 1000
    e2 = metrics[k2]['e_ee'] * 1000

    fig, axes = plt.subplots(2, 1, figsize=(13, 8))
    fig.suptitle(f'CTC vs PID — {condition}', fontsize=13)

    # EE error
    ax = axes[0]
    ax.plot(t1, e1, lw=1.2, color=COLORS[k1], label=LABELS[k1])
    ax.plot(t2, e2, lw=1.2, color=COLORS[k2], label=LABELS[k2])
    ax.axhline(EPS_THRESH * 1000, ls='--', color='red',
               label=f'Threshold {EPS_THRESH*1000:.0f} mm')
    shade_dwells(ax, t1, dwell_mask(df1))
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('$e_{EE}$ [mm]')
    ax.set_title('End-Effector Error Norm')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.4)

    # Z tracking
    ax = axes[1]
    ax.plot(t1, df1['p_z'].values,     lw=1.2, color=COLORS[k1],
            label=f'Z actual — {LABELS[k1]}')
    ax.plot(t2, df2['p_z'].values,     lw=1.2, color=COLORS[k2],
            label=f'Z actual — {LABELS[k2]}')
    ax.plot(t1, df1['p_des_z'].values, lw=1.0, ls='--', color='grey',
            label='Z desired')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Z [m]')
    ax.set_title('Z-Axis Tracking')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.4)

    plt.tight_layout()
    out = os.path.join(PLOTS, f'comparison_{condition}.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f'  Saved {out}')


# ===========================================================================
# PLOT 7 — Summary Metrics Table
# ===========================================================================
def plot_metrics_table():
    rows = []
    for k in ['ctc_nopert', 'pid_nopert', 'ctc_pert', 'pid_pert']:
        m = metrics[k]
        rows.append([
            LABELS[k],
            f"{np.mean(m['joint_rmse'])*1000:.2f}",
            f"{np.mean(m['joint_max'])*1000:.2f}",
            f"{m['ee_rmse']*1000:.2f}",
            f"{m['ee_max']*1000:.2f}",
            f"{m['wp_success']:.1f}",
        ])

    col_labels = ['Trial', 'Joint RMSE avg\n[mrad]', 'Joint Max avg\n[mrad]',
                  'EE RMSE\n[mm]', 'EE Max\n[mm]', 'WP Success\n[%]']

    fig, ax = plt.subplots(figsize=(13, 3))
    ax.axis('off')
    tbl = ax.table(cellText=rows, colLabels=col_labels,
                   loc='center', cellLoc='center')
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10)
    tbl.scale(1, 2.2)
    # Header color
    for j in range(len(col_labels)):
        tbl[0, j].set_facecolor('#4a4a8a')
        tbl[0, j].set_text_props(color='white', fontweight='bold')
    for i, k in enumerate(['ctc_nopert', 'pid_nopert', 'ctc_pert', 'pid_pert']):
        tbl[i+1, 0].set_facecolor(COLORS[k])
        tbl[i+1, 0].set_text_props(color='white', fontweight='bold')
    ax.set_title('Summary Comparison Table — All Trials', pad=14, fontsize=12)
    plt.tight_layout()
    out = os.path.join(PLOTS, 'summary_table.png')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  Saved {out}')

    # Also print to terminal
    print('\n' + '='*70)
    print(f'{"Trial":<25} {"Joint RMSE":>12} {"EE RMSE":>10} {"EE Max":>10} {"WP%":>8}')
    print('-'*70)
    for k in ['ctc_nopert', 'pid_nopert', 'ctc_pert', 'pid_pert']:
        m = metrics[k]
        print(f'{LABELS[k]:<25} '
              f'{np.mean(m["joint_rmse"])*1000:>10.2f}mr '
              f'{m["ee_rmse"]*1000:>8.2f}mm '
              f'{m["ee_max"]*1000:>8.2f}mm '
              f'{m["wp_success"]:>6.1f}%')
    print('='*70)


# ===========================================================================
# Generate all plots
# ===========================================================================
print('\nGenerating plots...')
for key in TRIALS:
    print(f'\n--- {LABELS[key]} ---')
    plot_joint_tracking(key)
    plot_taskspace(key)
    plot_3d_path(key)
    plot_ee_error(key)
    plot_phase_portraits(key)

print('\n--- Comparison plots ---')
plot_comparison('sin_perturbaciones', 'ctc_nopert', 'pid_nopert')
plot_comparison('con_perturbaciones', 'ctc_pert',   'pid_pert')

print('\n--- Summary table ---')
plot_metrics_table()

print(f'\nDone. All plots saved to:\n  {PLOTS}')
