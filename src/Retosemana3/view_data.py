"""
TE3001B — Visualizador de Datos NPZ (Maestro + Esclavo)
Carga y grafica los archivos master_data.npz y slave_data.npz.

Uso:
    python3 view_data.py                        # busca ambos archivos automáticamente
    python3 view_data.py --master master_data.npz --slave slave_data.npz
"""

import argparse
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches

# ─── Colores (mismo estilo que master/slave_robot.py) ────────────────────────
BG_FIG   = '#0a0a1a'
BG_AX    = '#0d1117'
GRID_C   = '#1e2530'
C = ['#00BFFF', '#FF6B6B', '#69FF47', '#FFD700', '#FF69B4', '#00FFD0',
     '#FF8C00', '#BF5FFF']


def style_ax(ax, title='', xlabel='', ylabel=''):
    ax.set_facecolor(BG_AX)
    ax.tick_params(colors='#aaa')
    ax.xaxis.label.set_color('#aaa')
    ax.yaxis.label.set_color('#aaa')
    ax.title.set_color('white')
    for spine in ax.spines.values():
        spine.set_edgecolor('#333')
    ax.grid(True, color=GRID_C, linestyle='--', alpha=0.5)
    if title:
        ax.set_title(title, fontsize=10)
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)


# ─── Carga de archivos ────────────────────────────────────────────────────────
def load_npz(path, label):
    if path is None or not os.path.isfile(path):
        print(f"[AVISO] No se encontró {label}: {path}")
        return None
    data = np.load(path)
    print(f"[OK] {label} cargado: {path}")
    print(f"     Claves: {list(data.keys())}")
    return data


# ─── Panel MAESTRO ────────────────────────────────────────────────────────────
def plot_master(fig, gs, row, data):
    """
    Dibuja 4 subplots del maestro en una fila de GridSpec:
      1. Trayectoria EF (x vs y)
      2. Ángulos articulares q1, q2, q3
      3. Torques τ1, τ2, τ3
      4. Fuerzas hápticas Fe_x, Fe_y
    """
    t   = data['t']
    q   = data['q']      # (N, 3)
    tau = data['tau']    # (N, 3)
    Fe  = data['Fe']     # (N, 2)
    x   = data['x']     # (N, 2)

    ax_traj = fig.add_subplot(gs[row, 0])
    ax_q    = fig.add_subplot(gs[row, 1])
    ax_tau  = fig.add_subplot(gs[row, 2])
    ax_Fe   = fig.add_subplot(gs[row, 3])

    # 1. Trayectoria EF ──────────────────────────────────────────────────────
    style_ax(ax_traj, 'Trayectoria EF — Maestro', 'x [m]', 'y [m]')
    ax_traj.plot(x[:, 0], x[:, 1], color=C[0], linewidth=1.5, label='EF')
    ax_traj.plot(x[0, 0], x[0, 1], 'o', color=C[2], markersize=8, label='Inicio')
    ax_traj.plot(x[-1, 0], x[-1, 1], 's', color=C[1], markersize=8, label='Fin')
    # Agujero destino
    hole = (0.55, 0.10)
    ax_traj.add_patch(mpatches.Circle(hole, 0.025, color='#FFD700', alpha=0.4))
    ax_traj.plot(*hole, 'x', color='white', markersize=8, markeredgewidth=2)
    ax_traj.text(hole[0]+0.03, hole[1]+0.03, 'HOLE', color='#FFD700', fontsize=8)
    ax_traj.legend(fontsize=8, facecolor='#1a1a2e', labelcolor='white')
    ax_traj.set_aspect('equal', adjustable='datalim')

    # 2. Ángulos articulares ──────────────────────────────────────────────────
    style_ax(ax_q, 'Ángulos Articulares', 'Tiempo [s]', 'q [rad]')
    ls = ['solid', 'dashed', 'dotted']
    for i in range(3):
        ax_q.plot(t, q[:, i], color=C[i], linewidth=1.5,
                  linestyle=ls[i], label=f'q{i+1}')
    ax_q.legend(fontsize=8, facecolor='#1a1a2e', labelcolor='white')

    # 3. Torques articulares ──────────────────────────────────────────────────
    style_ax(ax_tau, 'Torques Articulares', 'Tiempo [s]', 'τ [Nm]')
    for i in range(3):
        ax_tau.plot(t, tau[:, i], color=C[i], linewidth=1.5, label=f'τ{i+1}')
    ax_tau.axhline(y=0, color='#444', linewidth=0.8)
    ax_tau.legend(fontsize=8, facecolor='#1a1a2e', labelcolor='white')

    # 4. Fuerzas hápticas ─────────────────────────────────────────────────────
    style_ax(ax_Fe, 'Fuerzas Hápticas Reflejadas', 'Tiempo [s]', 'F [N]')
    ax_Fe.plot(t, Fe[:, 0], color=C[4], linewidth=1.8, label='Fₓ')
    ax_Fe.plot(t, Fe[:, 1], color=C[5], linewidth=1.8, label='F_y')
    ax_Fe.axhline(y=0, color='#444', linewidth=0.8)
    ax_Fe.axhline(y=2,  color='#FF4444', linewidth=1.0, linestyle='--',
                  alpha=0.7, label='Umbral 2 N')
    ax_Fe.axhline(y=-2, color='#FF4444', linewidth=1.0, linestyle='--', alpha=0.7)
    ax_Fe.legend(fontsize=8, facecolor='#1a1a2e', labelcolor='white')


# ─── Panel ESCLAVO ────────────────────────────────────────────────────────────
def plot_slave(fig, gs, row, data):
    """
    Dibuja 4 subplots del esclavo en una fila de GridSpec:
      1. Trayectoria EF (x vs y) con agujero
      2. Fuerzas de contacto Fc_x, Fc_y
      3. Torques τ1, τ2, τ3
      4. Error cartesiano |ex|, |ey| en mm
    """
    t   = data['t']
    tau = data['tau']   # (N, 3)
    Fc  = data['Fc']    # (N, 2)
    x   = data['x']    # (N, 2)
    ex  = data['ex']   # (N, 2)

    ax_traj = fig.add_subplot(gs[row, 0])
    ax_Fc   = fig.add_subplot(gs[row, 1])
    ax_tau  = fig.add_subplot(gs[row, 2])
    ax_err  = fig.add_subplot(gs[row, 3])

    # 1. Trayectoria EF ──────────────────────────────────────────────────────
    style_ax(ax_traj, 'Trayectoria EF — Esclavo', 'x [m]', 'y [m]')
    ax_traj.plot(x[:, 0], x[:, 1], color=C[1], linewidth=1.5, label='EF')
    ax_traj.plot(x[0, 0], x[0, 1], 'o', color=C[2], markersize=8, label='Inicio')
    ax_traj.plot(x[-1, 0], x[-1, 1], 's', color=C[0], markersize=8, label='Fin')
    # Agujero
    hole = (0.55, 0.10)
    hole_rect = mpatches.Rectangle((hole[0]-0.04, hole[1]-0.005), 0.08, 0.04,
                               color='#2a2a4a', zorder=1)
    ax_traj.add_patch(hole_rect)
    hole_open = mpatches.Rectangle((hole[0]-0.009, hole[1]-0.005), 0.018, 0.025,
                               color=BG_AX, zorder=2)
    ax_traj.add_patch(hole_open)
    ax_traj.plot(*hole, 'x', color='#FFD700', markersize=10,
                 markeredgewidth=2, zorder=3)
    ax_traj.text(hole[0]+0.02, hole[1]+0.015, 'HOLE', color='#FFD700', fontsize=8)
    ax_traj.legend(fontsize=8, facecolor='#1a1a2e', labelcolor='white')
    ax_traj.set_aspect('equal', adjustable='datalim')

    # 2. Fuerzas de contacto ──────────────────────────────────────────────────
    style_ax(ax_Fc, 'Fuerzas de Contacto', 'Tiempo [s]', 'F [N]')
    ax_Fc.plot(t, Fc[:, 0], color=C[4], linewidth=1.8, label='Fₓ contacto')
    ax_Fc.plot(t, Fc[:, 1], color=C[5], linewidth=1.8, label='F_y contacto')
    ax_Fc.axhline(y=0, color='#444', linewidth=0.8)
    ax_Fc.axhline(y=2,  color='#FF4444', linewidth=1.2, linestyle='--', alpha=0.7)
    ax_Fc.axhline(y=-2, color='#FF4444', linewidth=1.2, linestyle='--', alpha=0.7,
                  label='Umbral ±2 N')
    ax_Fc.legend(fontsize=8, facecolor='#1a1a2e', labelcolor='white')

    # 3. Torques articulares ──────────────────────────────────────────────────
    style_ax(ax_tau, 'Torques Articulares', 'Tiempo [s]', 'τ [Nm]')
    for i in range(3):
        ax_tau.plot(t, tau[:, i], color=C[i], linewidth=1.5, label=f'τ{i+1}')
    ax_tau.axhline(y= 20, color='#FF4444', linewidth=0.8, linestyle='--', alpha=0.5)
    ax_tau.axhline(y=-20, color='#FF4444', linewidth=0.8, linestyle='--', alpha=0.5)
    ax_tau.axhline(y=0, color='#444', linewidth=0.8)
    ax_tau.legend(fontsize=8, facecolor='#1a1a2e', labelcolor='white')

    # 4. Error cartesiano ─────────────────────────────────────────────────────
    style_ax(ax_err, 'Error Cartesiano', 'Tiempo [s]', 'Error [mm]')
    ax_err.plot(t, np.abs(ex[:, 0]) * 1000, color=C[1], linewidth=1.8, label='|eₓ|')
    ax_err.plot(t, np.abs(ex[:, 1]) * 1000, color=C[2], linewidth=1.8, label='|e_y|')
    ax_err.axhline(y=1.0, color='#888', linewidth=1.0, linestyle=':', label='Meta 1 mm')
    ax_err.legend(fontsize=8, facecolor='#1a1a2e', labelcolor='white')


# ─── Estadísticas resumidas ───────────────────────────────────────────────────
def print_stats(data, label):
    print(f"\n{'='*55}")
    print(f"  {label}")
    print(f"{'='*55}")
    for key in data.keys():
        arr = data[key]
        if arr.ndim == 1:
            print(f"  {key:6s}  min={arr.min():.4f}  max={arr.max():.4f}"
                  f"  mean={arr.mean():.4f}  std={arr.std():.4f}")
        else:
            for col in range(arr.shape[1]):
                col_data = arr[:, col]
                print(f"  {key}[{col}]  min={col_data.min():.4f}"
                      f"  max={col_data.max():.4f}"
                      f"  mean={col_data.mean():.4f}"
                      f"  std={col_data.std():.4f}")


# ─── MAIN ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='Visualizador TE3001B — Datos Maestro/Esclavo NPZ')
    parser.add_argument('--master', default='master_data.npz',
                        help='Ruta al archivo master_data.npz')
    parser.add_argument('--slave',  default='slave_data.npz',
                        help='Ruta al archivo slave_data.npz')
    args = parser.parse_args()

    master = load_npz(args.master, 'MAESTRO')
    slave  = load_npz(args.slave,  'ESCLAVO')

    if master is None and slave is None:
        print("\nError: no se encontró ningún archivo de datos.")
        sys.exit(1)

    if master:
        print_stats(master, 'MAESTRO — Estadísticas')
    if slave:
        print_stats(slave,  'ESCLAVO — Estadísticas')

    # ── Configurar figura ──────────────────────────────────────────────────
    n_rows = (1 if master else 0) + (1 if slave else 0)
    fig = plt.figure(figsize=(18, 5 * n_rows + 1), facecolor=BG_FIG)

    titles = []
    if master:
        titles.append('MAESTRO — Computed Torque + IK + Feedback Háptico')
    if master and slave:
        titles.append('ESCLAVO — Control de Impedancia + Peg-in-Hole')
    elif slave:
        titles.append('ESCLAVO — Control de Impedancia + Peg-in-Hole')

    fig.suptitle('TE3001B — Análisis Post-Sesión: ' + ' | '.join(titles),
                 color='white', fontsize=13, fontweight='bold', y=0.99)

    gs = gridspec.GridSpec(n_rows, 4, figure=fig,
                           hspace=0.45, wspace=0.35,
                           left=0.06, right=0.97,
                           top=0.93, bottom=0.08)

    row = 0
    if master:
        # Row label
        fig.text(0.005, 1 - (row + 0.5) / n_rows, 'MAESTRO',
                 color='#00BFFF', fontsize=10, fontweight='bold',
                 va='center', rotation='vertical')
        plot_master(fig, gs, row, master)
        row += 1

    if slave:
        fig.text(0.005, 1 - (row + 0.5) / n_rows, 'ESCLAVO',
                 color='#FF6B6B', fontsize=10, fontweight='bold',
                 va='center', rotation='vertical')
        plot_slave(fig, gs, row, slave)

    plt.show()


if __name__ == '__main__':
    main()
