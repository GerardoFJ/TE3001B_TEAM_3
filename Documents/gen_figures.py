#!/usr/bin/env python3
"""
Generate all figures for ReporteFinal_Team4 from the papubag ROS2 bag.

Data available in the bag:
  - /master/joint_states        : real motion in Isaac Sim
  - /slave/joint_states         : slave tracking master in Isaac Sim
  - /teleop/operator_force_direct : ESP32 master sensor (joystick + spring)
  - /teleop/slave_force_direct   : ESP32 slave sensor (zeros - not active)
  - /master/servo_server/delta_twist_cmds : admittance velocity output
  - /teleop/master_pose          : master EE pose (FK)
  - /teleop/master_velocity      : master EE velocity

Note: slave force estimate = 0 because Isaac Sim does not provide
joint torques in the effort field; the physical operator sensor IS active.
"""
import os, sys
import sqlite3
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from scipy.ndimage import uniform_filter1d

BAG    = '/home/danielh/sch_ws/TE3001B_TEAM_3/papubag/papubag_0.db3'
OUTDIR = '/home/danielh/sch_ws/TE3001B_TEAM_3/Documents/figures'
os.makedirs(OUTDIR, exist_ok=True)

# ── rclpy deserialization ─────────────────────────────────────────────────────
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ── Matplotlib style ──────────────────────────────────────────────────────────
plt.rcParams.update({
    'figure.dpi': 150,
    'font.size': 9,
    'axes.titlesize': 10,
    'axes.labelsize': 9,
    'legend.fontsize': 8,
    'lines.linewidth': 1.2,
    'axes.grid': True,
    'grid.alpha': 0.30,
    'grid.linestyle': '--',
    'axes.spines.top': False,
    'axes.spines.right': False,
})
C = ['#0044aa', '#cc1e2d', '#228b22', '#d45f00',
     '#7b2d8b', '#007b8a', '#8b6914', '#444444']

# ── SQLite helpers ────────────────────────────────────────────────────────────
db  = sqlite3.connect(BAG)
cur = db.cursor()

def get_tid(name):
    cur.execute('SELECT id FROM topics WHERE name=?', (name,))
    r = cur.fetchone()
    return r[0] if r else None

def read(name, msg_type):
    tid = get_tid(name)
    if tid is None:
        return []
    cur.execute('SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,))
    MT = get_message(msg_type)
    out = []
    for ts, data in cur.fetchall():
        try:
            out.append((ts * 1e-9, deserialize_message(bytes(data), MT)))
        except Exception:
            pass
    return out

# ── Load topics ───────────────────────────────────────────────────────────────
print('Loading topics...')
master_js = read('/master/joint_states',               'sensor_msgs/msg/JointState')
slave_js  = read('/slave/joint_states',                'sensor_msgs/msg/JointState')
op_force  = read('/teleop/operator_force_direct',      'geometry_msgs/msg/WrenchStamped')
sl_force  = read('/teleop/slave_force_direct',         'geometry_msgs/msg/WrenchStamped')
master_tw = read('/master/servo_server/delta_twist_cmds','geometry_msgs/msg/TwistStamped')
slave_tw  = read('/slave/servo_server/delta_twist_cmds', 'geometry_msgs/msg/TwistStamped')
master_p  = read('/teleop/master_pose',                'geometry_msgs/msg/PoseStamped')
master_v  = read('/teleop/master_velocity',            'geometry_msgs/msg/TwistStamped')
for name, data in [('master_js', master_js), ('slave_js', slave_js),
                   ('op_force', op_force), ('sl_force', sl_force),
                   ('master_tw', master_tw), ('slave_tw', slave_tw),
                   ('master_p', master_p), ('master_v', master_v)]:
    print(f'  {name}: {len(data)} msgs')

# ── Time alignment (t=0 at first master joint_state) ─────────────────────────
t0 = master_js[0][0]
def rel(t): return t - t0

# ── Extract arrays ────────────────────────────────────────────────────────────
def js_arrays(lst):
    ts, qs = [], []
    for t, msg in lst:
        p = list(msg.position)
        if len(p) >= 6:
            ts.append(rel(t)); qs.append(np.degrees(p[:6]))
    return np.array(ts), np.array(qs)

def wrench_arrays(lst):
    ts, fx, fy, fz = [], [], [], []
    for t, msg in lst:
        ts.append(rel(t))
        fx.append(msg.wrench.force.x)
        fy.append(msg.wrench.force.y)
        fz.append(msg.wrench.force.z)
    return np.array(ts), np.array(fx), np.array(fy), np.array(fz)

def twist_arrays(lst):
    ts, vx, vy, vz = [], [], [], []
    for t, msg in lst:
        ts.append(rel(t))
        vx.append(msg.twist.linear.x)
        vy.append(msg.twist.linear.y)
        vz.append(msg.twist.linear.z)
    return np.array(ts), np.array(vx), np.array(vy), np.array(vz)

def pose_arrays(lst):
    ts, x, y, z = [], [], [], []
    for t, msg in lst:
        ts.append(rel(t))
        x.append(msg.pose.position.x)
        y.append(msg.pose.position.y)
        z.append(msg.pose.position.z)
    return np.array(ts), np.array(x), np.array(y), np.array(z)

print('Extracting arrays...')
t_m,  q_m   = js_arrays(master_js)
t_s,  q_s   = js_arrays(slave_js)
t_op, fx_op, fy_op, fz_op = wrench_arrays(op_force)
t_sl, fx_sl, fy_sl, fz_sl = wrench_arrays(sl_force)
t_mt, vx_mt, vy_mt, vz_mt = twist_arrays(master_tw)
t_st, vx_st, vy_st, vz_st = twist_arrays(slave_tw)
t_mp, px_m, py_m, pz_m    = pose_arrays(master_p)
t_mv, vx_mv, vy_mv, vz_mv = twist_arrays(master_v)

# Duration
T_MAX = max(t_m[-1] if len(t_m) else 0,
            t_s[-1] if len(t_s) else 0,
            t_op[-1] if len(t_op) else 0)
print(f'  Total duration: {T_MAX:.1f} s')

# ── Detect "contact" events from Fz of operator sensor (spring compressed) ────
# Fz > threshold means spring was compressed (user pushed down)
FZ_CONTACT_TH = 2.0  # N
fmag_op = np.sqrt(fx_op**2 + fy_op**2 + fz_op**2)
contact_events = []
in_contact = False
t_start = 0
for t, fz, fm in zip(t_op, fz_op, fmag_op):
    if fm > FZ_CONTACT_TH and not in_contact:
        in_contact = True; t_start = t
    elif fm <= FZ_CONTACT_TH and in_contact:
        in_contact = False
        if t - t_start > 0.3:   # min 300 ms
            contact_events.append((t_start, t))
if in_contact:
    contact_events.append((t_start, t_op[-1]))
print(f'  Contact events detected: {len(contact_events)}')

def shade_contacts(ax, events=contact_events, alpha=0.13, color='red'):
    for i, (ts, te) in enumerate(events):
        lbl = 'Fuerza aplicada' if i == 0 else ''
        ax.axvspan(ts, te, alpha=alpha, color=color, label=lbl)

# ── Interpolate slave to master timestamps ────────────────────────────────────
def interp_js(t_ref, t_src, q_src):
    out = np.zeros((len(t_ref), 6))
    for j in range(6):
        out[:, j] = np.interp(t_ref, t_src, q_src[:, j])
    return out

q_s_i = interp_js(t_m, t_s, q_s)
e_q   = q_m - q_s_i   # tracking error [degrees]

# ─────────────────────────────────────────────────────────────────────────────
# FIG E2: Joint tracking error (6 joints)
# ─────────────────────────────────────────────────────────────────────────────
print('Generating fig_E2_joint_tracking_error...')
jnames = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']
rms_vals = [np.sqrt(np.mean(e_q[:,j]**2)) for j in range(6)]
max_vals = [np.max(np.abs(e_q[:,j])) for j in range(6)]

fig, axes = plt.subplots(6, 1, figsize=(8, 10), sharex=True)
fig.suptitle('Error de Seguimiento Articular  '
             r'$e_i(t) = q_i^{master}(t) - q_i^{slave}(t)$',
             fontsize=10, fontweight='bold')
for j in range(6):
    ax = axes[j]
    ax.plot(t_m, e_q[:, j], color=C[j], lw=0.8)
    ax.axhline(0, color='k', lw=0.5, ls=':')
    ax.set_ylabel(f'$e_{j+1}$ (°)', fontsize=8)
    shade_contacts(ax)
    ax.set_title(f'{jnames[j]}  —  RMS={rms_vals[j]:.2f}°  Máx={max_vals[j]:.2f}°',
                 fontsize=8.5, loc='right')
    ylim = max(max_vals[j]*1.6, 0.5)
    ax.set_ylim(-ylim, ylim)
axes[-1].set_xlabel('Tiempo (s)')
red_p = mpatches.Patch(color='red', alpha=0.3, label='Fuerza operador activa')
fig.legend(handles=[red_p], loc='lower right', fontsize=8)
plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig(f'{OUTDIR}/fig_E2_joint_tracking_error.pdf', bbox_inches='tight')
plt.close()
print(f'  Joint tracking RMS (deg): {[f"{r:.3f}" for r in rms_vals]}')
print(f'  Joint tracking Max (deg): {[f"{m:.3f}" for m in max_vals]}')
print(f'  Mean RMS: {np.mean(rms_vals):.3f}°')

# ─────────────────────────────────────────────────────────────────────────────
# FIG E2b: Master vs Slave positions (overlay)
# ─────────────────────────────────────────────────────────────────────────────
print('Generating fig_E2b_joint_positions...')
fig, axes = plt.subplots(3, 2, figsize=(10, 7), sharex=True)
fig.suptitle('Posición Articular — Maestro (Isaac Sim) vs Esclavo',
             fontsize=10, fontweight='bold')
for j in range(6):
    ax = axes[j // 2, j % 2]
    ax.plot(t_m, q_m[:,j],  color=C[0], lw=0.9, label='Maestro', alpha=0.9)
    ax.plot(t_s, q_s[:,j],  color=C[1], lw=0.9, label='Esclavo', ls='--', alpha=0.9)
    ax.set_title(jnames[j], fontsize=9)
    ax.set_ylabel('Ángulo (°)')
    shade_contacts(ax)
    if j == 0: ax.legend(fontsize=8)
for ax in axes[-1, :]: ax.set_xlabel('Tiempo (s)')
plt.tight_layout()
plt.savefig(f'{OUTDIR}/fig_E2b_joint_positions.pdf', bbox_inches='tight')
plt.close()

# ─────────────────────────────────────────────────────────────────────────────
# FIG A2: Message arrival inter-arrival times (latency)
# ─────────────────────────────────────────────────────────────────────────────
print('Generating fig_A2_latency_histogram...')
tid_mjs   = get_tid('/master/joint_states')
tid_sjs   = get_tid('/slave/joint_states')
tid_op    = get_tid('/teleop/operator_force_direct')
tid_mtw   = get_tid('/master/servo_server/delta_twist_cmds')

def get_iats(tid):
    cur.execute('SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,))
    ts = np.array([r[0]*1e-6 for r in cur.fetchall()])   # ms
    iat = np.diff(ts)
    return iat[(iat > 0.1) & (iat < 100)]

iat_mjs  = get_iats(tid_mjs)
iat_sjs  = get_iats(tid_sjs)
iat_op   = get_iats(tid_op)
iat_mtw  = get_iats(tid_mtw)

datasets = [
    (iat_mjs,  '/master/joint\_states\n(250 Hz target)', C[0]),
    (iat_sjs,  '/slave/joint\_states\n(250 Hz target)',  C[1]),
    (iat_op,   '/teleop/operator\_force\_direct\n(50 Hz target)',  C[2]),
    (iat_mtw,  '/master/servo/delta\_twist\_cmds\n(50 Hz target)', C[3]),
]
fig, axes = plt.subplots(2, 2, figsize=(10, 6))
fig.suptitle('Distribución del Periodo de Llegada de Mensajes ROS 2 (DDS/LAN)',
             fontsize=10, fontweight='bold')
for ax, (iat, lbl, col) in zip(axes.flat, datasets):
    ax.hist(iat, bins=60, color=col, alpha=0.75, edgecolor='none')
    mn, sd = iat.mean(), iat.std()
    p99 = np.percentile(iat, 99)
    ax.axvline(mn,  color='k',      lw=1.5, ls='--', label=f'μ={mn:.2f} ms')
    ax.axvline(p99, color='orange', lw=1.5, ls=':',  label=f'P99={p99:.2f} ms')
    ax.set_title(lbl, fontsize=8.5)
    ax.set_xlabel('Periodo (ms)'); ax.set_ylabel('Frecuencia')
    ax.legend(fontsize=7.5)
    ax.text(0.98, 0.85, f'σ={sd:.2f} ms', transform=ax.transAxes,
            ha='right', fontsize=8)
plt.tight_layout()
plt.savefig(f'{OUTDIR}/fig_A2_latency_histogram.pdf', bbox_inches='tight')
plt.close()
print(f'  master/joint_states: μ={iat_mjs.mean():.2f} ms  P99={np.percentile(iat_mjs,99):.2f} ms')
print(f'  slave/joint_states:  μ={iat_sjs.mean():.2f} ms  P99={np.percentile(iat_sjs,99):.2f} ms')
print(f'  operator_force:      μ={iat_op.mean():.2f} ms  P99={np.percentile(iat_op,99):.2f} ms')
print(f'  master twist cmds:   μ={iat_mtw.mean():.2f} ms  P99={np.percentile(iat_mtw,99):.2f} ms')

# ─────────────────────────────────────────────────────────────────────────────
# FIG F1: Operator force (ESP32) — all 3 axes
# ─────────────────────────────────────────────────────────────────────────────
print('Generating fig_F1_operator_force...')
fmag = np.sqrt(fx_op**2 + fy_op**2 + fz_op**2)

fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
ax = axes[0]
ax.plot(t_op, fx_op, color=C[0], lw=0.9, label='$F_x$ (joystick X)')
ax.plot(t_op, fy_op, color=C[1], lw=0.9, label='$F_y$ (joystick Y)')
ax.axhline( 3.0, color=C[0], lw=0.7, ls=':', alpha=0.5)
ax.axhline(-3.0, color=C[0], lw=0.7, ls=':', alpha=0.5)
shade_contacts(ax)
ax.set_title('Sensor ESP32 Maestro — Fuerzas del Joystick ($k=1000$ N/m, $\\pm3$ N)')
ax.set_ylabel('Fuerza (N)'); ax.legend(ncol=2, fontsize=8)
ax.set_ylim(-4.5, 4.5)

ax = axes[1]
ax.plot(t_op, fz_op,  color=C[2], lw=0.9, label='$F_z$ (resorte-ultrasónico)')
ax.plot(t_op, fmag,   color='k',   lw=0.7, ls=':', alpha=0.6, label='$\\|F\\|$')
ax.axhline(FZ_CONTACT_TH, color='orange', lw=1.0, ls='--',
           label=f'Umbral $F_{{th}}={FZ_CONTACT_TH}$ N')
shade_contacts(ax)
ax.set_title(f'Fuerza eje Z (resorte): $F_z = k \\cdot \\delta$,  $F_z^{{max}}={fz_op.max():.1f}$ N')
ax.set_ylabel('Fuerza (N)'); ax.set_xlabel('Tiempo (s)')
ax.legend(ncol=3, fontsize=8)
ax.set_ylim(-2, fz_op.max()*1.15)

fig.suptitle('Fuerza del Operador — Sensor Físico ESP32 (/teleop/operator_force_direct)',
             fontsize=10, fontweight='bold')
plt.tight_layout()
plt.savefig(f'{OUTDIR}/fig_F1_operator_force.pdf', bbox_inches='tight')
plt.close()
print(f'  Fz max = {fz_op.max():.2f} N  |  Fx range = [{fx_op.min():.2f},{fx_op.max():.2f}]')

# ─────────────────────────────────────────────────────────────────────────────
# FIG: Admittance velocity output (master servo commands)
# ─────────────────────────────────────────────────────────────────────────────
print('Generating fig_admittance_velocity...')
vmag_mt = np.sqrt(vx_mt**2 + vy_mt**2 + vz_mt**2)
vmag_st = np.sqrt(vx_st**2 + vy_st**2 + vz_st**2)

fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
ax = axes[0]
ax.plot(t_mt, vx_mt*1000, color=C[0], lw=0.9, label='$v_x$ (mm/s)')
ax.plot(t_mt, vy_mt*1000, color=C[1], lw=0.9, label='$v_y$ (mm/s)')
ax.plot(t_mt, vz_mt*1000, color=C[2], lw=0.9, label='$v_z$ (mm/s)')
ax.axhline( 100, color='k', lw=0.7, ls=':', alpha=0.4, label='$\\pm v_{max}=100$ mm/s')
ax.axhline(-100, color='k', lw=0.7, ls=':', alpha=0.4)
shade_contacts(ax)
ax.set_title('Velocidad Cartesiana — Maestro  (MasterAdmittanceNode → MoveIt Servo)')
ax.set_ylabel('Velocidad (mm/s)'); ax.legend(ncol=4, fontsize=8)

ax = axes[1]
ax.plot(t_st, vx_st*1000, color=C[0], lw=0.9, label='$v_x$ (mm/s)')
ax.plot(t_st, vy_st*1000, color=C[1], lw=0.9, label='$v_y$ (mm/s)')
ax.plot(t_st, vz_st*1000, color=C[2], lw=0.9, label='$v_z$ (mm/s)')
ax.axhline( 100, color='k', lw=0.7, ls=':', alpha=0.4)
ax.axhline(-100, color='k', lw=0.7, ls=':', alpha=0.4)
shade_contacts(ax)
ax.set_title('Velocidad Cartesiana — Esclavo  (SlaveImpedanceNode → MoveIt Servo)')
ax.set_ylabel('Velocidad (mm/s)'); ax.set_xlabel('Tiempo (s)')
ax.legend(ncol=3, fontsize=8)

fig.suptitle('Comandos de Velocidad a MoveIt Servo (TwistStamped @ 50 Hz)',
             fontsize=10, fontweight='bold')
plt.tight_layout()
plt.savefig(f'{OUTDIR}/fig_admittance_velocity.pdf', bbox_inches='tight')
plt.close()

# ─────────────────────────────────────────────────────────────────────────────
# FIG: Master EE pose trajectory (x, y, z)
# ─────────────────────────────────────────────────────────────────────────────
print('Generating fig_EE_pose...')
fig, axes = plt.subplots(2, 1, figsize=(9, 5), sharex=True)

ax = axes[0]
ax.plot(t_mp, px_m*1000, color=C[0], lw=0.9, label='$p_x$')
ax.plot(t_mp, py_m*1000, color=C[1], lw=0.9, label='$p_y$')
ax.plot(t_mp, pz_m*1000, color=C[2], lw=0.9, label='$p_z$')
shade_contacts(ax)
ax.set_title('Posición del Efector Final — Maestro  (/teleop/master\\_pose, FK)')
ax.set_ylabel('Posición (mm)'); ax.legend(ncol=3, fontsize=8)

ax = axes[1]
ax.plot(t_mv, vx_mv*1000, color=C[0], lw=0.9, label='$v_x$')
ax.plot(t_mv, vy_mv*1000, color=C[1], lw=0.9, label='$v_y$')
ax.plot(t_mv, vz_mv*1000, color=C[2], lw=0.9, label='$v_z$')
ax.axhline(100,  color='k', lw=0.7, ls=':', alpha=0.4, label='$\\pm v_{max}$')
ax.axhline(-100, color='k', lw=0.7, ls=':', alpha=0.4)
shade_contacts(ax)
ax.set_title('Velocidad EE publicada por Admitancia  (/teleop/master\\_velocity)')
ax.set_ylabel('Velocidad (mm/s)'); ax.set_xlabel('Tiempo (s)')
ax.legend(ncol=4, fontsize=8)

fig.suptitle('Trayectoria y Velocidad del Efector Final — Maestro',
             fontsize=10, fontweight='bold')
plt.tight_layout()
plt.savefig(f'{OUTDIR}/fig_EE_pose.pdf', bbox_inches='tight')
plt.close()

# ─────────────────────────────────────────────────────────────────────────────
# FIG E3a: Force + admittance response (central figure of the report)
# ─────────────────────────────────────────────────────────────────────────────
print('Generating fig_E3a_admittance_response...')
fig = plt.figure(figsize=(10, 8))
gs  = fig.add_gridspec(3, 1, hspace=0.45)

ax0 = fig.add_subplot(gs[0])
ax0.plot(t_op, fx_op, color=C[0], lw=0.9, label='$F_x^{op}$ joystick')
ax0.plot(t_op, fy_op, color=C[1], lw=0.9, label='$F_y^{op}$ joystick')
ax0.plot(t_op, fz_op, color=C[2], lw=1.1, label='$F_z^{op}$ resorte')
ax0.axhline(FZ_CONTACT_TH,  color='orange', lw=1, ls='--', label=f'$F_{{th}}$={FZ_CONTACT_TH} N')
shade_contacts(ax0)
ax0.set_ylabel('Fuerza (N)')
ax0.set_title('(a) Fuerza del Operador — Sensor ESP32  '
              '[$k=1000$ N/m,  $\\pm$3 N joystick,  50 Hz]')
ax0.legend(ncol=4, fontsize=7.5); ax0.set_ylim(-4.5, max(fz_op.max()*1.15, 5))

ax1 = fig.add_subplot(gs[1], sharex=ax0)
ax1.plot(t_mt, vx_mt*1000, color=C[0], lw=0.9, label='$v_x^{cmd}$')
ax1.plot(t_mt, vy_mt*1000, color=C[1], lw=0.9, label='$v_y^{cmd}$')
ax1.plot(t_mt, vz_mt*1000, color=C[2], lw=0.9, label='$v_z^{cmd}$')
ax1.axhline( 100, color='k', lw=0.7, ls=':', alpha=0.4, label='$\\pm v_{max}$')
ax1.axhline(-100, color='k', lw=0.7, ls=':', alpha=0.4)
shade_contacts(ax1)
ax1.set_ylabel('mm/s')
ax1.set_title('(b) Velocidad Cartesiana del Maestro — Salida de Admitancia  '
              '[$M_d$=2 kg, $B_d$=10 N·s/m]')
ax1.legend(ncol=4, fontsize=7.5)

ax2 = fig.add_subplot(gs[2], sharex=ax0)
ax2.plot(t_m, q_m[:, 1], color=C[0], lw=0.9, label='Maestro $q_2$')
ax2.plot(t_s, q_s[:, 1], color=C[1], lw=0.9, label='Esclavo $q_2$', ls='--')
ax2.plot(t_m, q_m[:, 4], color=C[2], lw=0.9, label='Maestro $q_5$')
ax2.plot(t_s, q_s[:, 4], color=C[3], lw=0.9, label='Esclavo $q_5$', ls='--')
shade_contacts(ax2)
ax2.set_ylabel('Ángulo (°)')
ax2.set_title('(c) Seguimiento Articular — Joints más activos ($q_2, q_5$)')
ax2.set_xlabel('Tiempo (s)'); ax2.legend(ncol=4, fontsize=7.5)

red_p = mpatches.Patch(color='red', alpha=0.25, label='Fuerza operador activa ($\\|F\\| > 2$ N)')
fig.legend(handles=[red_p], loc='lower right', fontsize=8)
fig.suptitle('Respuesta del Sistema de Teleoperación Bilateral\n'
             'Fuerza Operador → Admitancia → Seguimiento Articular',
             fontsize=10, fontweight='bold')
plt.savefig(f'{OUTDIR}/fig_E3a_admittance_response.pdf', bbox_inches='tight')
plt.close()
print(f'  Saved fig_E3a_admittance_response.pdf')

# ─────────────────────────────────────────────────────────────────────────────
# FIG E3b: Scatter force vs velocity (validates admittance model)
# ─────────────────────────────────────────────────────────────────────────────
print('Generating fig_E3b_admittance_validation...')
# Interpolate operator force to twist timestamps
if len(t_mt) > 0 and len(t_op) > 0:
    fx_at_tw = np.interp(t_mt, t_op, fx_op)
    fy_at_tw = np.interp(t_mt, t_op, fy_op)
    fz_at_tw = np.interp(t_mt, t_op, fz_op)

    fig, axes = plt.subplots(1, 3, figsize=(11, 4))
    pairs = [
        (fx_at_tw, vx_mt*1000, '$F_x^{op}$ (N)', '$v_x^{cmd}$ (mm/s)', 'X-axis'),
        (fy_at_tw, vy_mt*1000, '$F_y^{op}$ (N)', '$v_y^{cmd}$ (mm/s)', 'Y-axis'),
        (fz_at_tw, vz_mt*1000, '$F_z^{op}$ (N)', '$v_z^{cmd}$ (mm/s)', 'Z-axis'),
    ]
    for ax, (fx, vy, xl, yl, title) in zip(axes, pairs):
        ax.scatter(fx, vy, s=4, alpha=0.3, color=C[0])
        valid = np.isfinite(fx) & np.isfinite(vy)
        if valid.sum() > 20:
            try:
                c = np.polyfit(fx[valid], vy[valid], 1)
                xlim = np.array([fx[valid].min(), fx[valid].max()])
                ax.plot(xlim, np.polyval(c, xlim), 'r-', lw=1.5,
                        label=f'Pendiente={c[0]:.1f} mm/s/N')
                ss_r = np.sum((vy[valid] - np.polyval(c, fx[valid]))**2)
                ss_t = np.sum((vy[valid] - vy[valid].mean())**2)
                r2 = 1 - ss_r/ss_t if ss_t > 0 else 0
                ax.text(0.05, 0.88, f'$R^2={r2:.3f}$', transform=ax.transAxes, fontsize=9)
            except Exception:
                pass
        ax.set_xlabel(xl); ax.set_ylabel(yl)
        ax.set_title(f'Eje {title}'); ax.legend(fontsize=8)

    fig.suptitle('Validación del Modelo de Admitancia: '
                 '$\\mathbf{v}_{cmd} \\propto \\mathbf{F}_{op}$\n'
                 '(pendiente $\\approx 1/B_d = 0.1$ m/s/N = 100 mm/s/N teórico)',
                 fontsize=9, fontweight='bold')
    plt.tight_layout()
    plt.savefig(f'{OUTDIR}/fig_E3b_admittance_validation.pdf', bbox_inches='tight')
    plt.close()
    print('  Saved fig_E3b_admittance_validation.pdf')

# ─────────────────────────────────────────────────────────────────────────────
# Summary printout for the report
# ─────────────────────────────────────────────────────────────────────────────
print()
print('='*65)
print('MÉTRICAS FINALES PARA EL REPORTE (ReporteFinal_Team4.tex)')
print('='*65)
print(f'Duración experimento:         {T_MAX:.1f} s')
print(f'Mensajes /master/joint_states: {len(master_js)}')
print(f'Mensajes /slave/joint_states:  {len(slave_js)}')
print(f'Mensajes sensor ESP32 maestro: {len(op_force)}')
print(f'Eventos fuerza activa (>2 N):  {len(contact_events)}')
for i, (ts, te) in enumerate(contact_events):
    print(f'  Evento {i+1}: t={ts:.1f}–{te:.1f} s  ({te-ts:.1f} s)')
print()
print('Error de seguimiento articular:')
for j in range(6):
    print(f'  Joint {j+1}: RMS={rms_vals[j]:.3f}°  Max={max_vals[j]:.3f}°')
print(f'  Promedio RMS:  {np.mean(rms_vals):.3f}°')
print(f'  Promedio Max:  {np.mean(max_vals):.3f}°')
print()
print('Fuerza del operador (ESP32 maestro):')
print(f'  Fx: [{fx_op.min():.2f}, {fx_op.max():.2f}] N')
print(f'  Fy: [{fy_op.min():.2f}, {fy_op.max():.2f}] N')
print(f'  Fz: [{fz_op.min():.2f}, {fz_op.max():.2f}] N  (resorte, máx comprensión)')
print()
print('Latencia de mensajes (inter-arrival):')
print(f'  /master/joint_states:       μ={iat_mjs.mean():.2f} ms  σ={iat_mjs.std():.2f} ms  P99={np.percentile(iat_mjs,99):.2f} ms')
print(f'  /slave/joint_states:        μ={iat_sjs.mean():.2f} ms  σ={iat_sjs.std():.2f} ms  P99={np.percentile(iat_sjs,99):.2f} ms')
print(f'  /teleop/operator_force:     μ={iat_op.mean():.2f} ms  σ={iat_op.std():.2f} ms  P99={np.percentile(iat_op,99):.2f} ms')
print(f'  /master/servo/twist_cmds:   μ={iat_mtw.mean():.2f} ms  σ={iat_mtw.std():.2f} ms  P99={np.percentile(iat_mtw,99):.2f} ms')
print()
print('Figuras generadas en:', OUTDIR)
figs = sorted(os.listdir(OUTDIR))
for f in figs:
    print(f'  {f}')
print('='*65)
