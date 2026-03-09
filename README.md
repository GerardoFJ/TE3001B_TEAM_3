# TE3001B_TEAM_3
TE3001B Team 3 Repository

---

## xarm_perturbationsV2

Paquete ROS 2 para el reto **Robotics Control Challenge 4.1**: evaluación y comparación de dos controladores en espacio articular aplicados al brazo xArm Lite 6, con y sin perturbaciones externas.

### Descripción general

El paquete implementa una cadena completa de control y análisis:

```
IK Reference Generator  →  Joint-Space Controller  →  Perturbation Injector  →  MoveIt Servo  →  xArm Lite 6
```

### Nodos principales

| Nodo | Ejecutable | Descripción |
|------|-----------|-------------|
| `ik_reference_generator` | `ik_reference_generator` | Genera referencias articulares (q, q̇, q̈) y cartesianas (p_des) a partir de cinemática inversa diferencial con control de nulo. Publica trayectorias punto a punto con fases de dwell. |
| `joint_space_controller` | `joint_space_controller` | Controlador dual en espacio articular. Soporta dos modos: **PID** (con anti-windup integral) y **CTC** (Control por Torque Computado, basado en modelo dinámico con matriz de inercia, torques gravitacionales y fricción). Publica velocidades cartesianas vía Jacobiano a MoveIt Servo. |
| `perturbation_injector` | `perturbation_injector` | Inyecta perturbaciones sobre la señal de control. Modos disponibles: **off** (sin perturbación), **sine** (disturbancia senoidal) y **gaussian** (ruido gaussiano de media cero). Puede operar standalone o en modo relay (suma la perturbación a la salida del controlador). |

### Controladores implementados

**PID en espacio articular**
- Ley de control: `τ = -(Kp·e + Kd·ė + Ki·∫e)`
- Anti-windup con saturación integral
- Ganancias por defecto: `Kp = diag(15,15,15,10,8,5)`, `Kd = diag(5,5,5,4,3,2)`, `Ki = diag(2,2,2,1,1,0.5)`

**CTC (Computed Torque Control)**
- Linealización por realimentación: `v = q̈_des - Kp·e - Kd·ė`
- Ley de torque: `τ = M(q)·v + G(q) + F(q̇)`
- Ganancias por defecto: `Kp = diag(30,30,30,20,15,10)`, `Kd = diag(10,10,10,8,6,4)`

### Escenarios de prueba (trials)

| Trial | Controlador | Perturbación |
|-------|-------------|--------------|
| `trial_ctc_nopert` | CTC | Sin perturbación |
| `trial_pdpid_nopert` | PID | Sin perturbación |
| `trial_ctc_pert` | CTC | Gaussiana (eje X, σ=0.01 m/s) |
| `trial_pdpid_pert` | PID | Gaussiana (eje X, σ=0.01 m/s) |

Los CSVs de cada prueba se guardan en:
```
src/xarm_perturbationsV2/xarm_perturbations/results/<trial_name>/
```

### Launch

```bash
# CTC + perturbación gaussiana (configuración por defecto del launch)
ros2 launch xarm_perturbations xarm_perturbations.launch.py

# PID sin perturbación (manual)
ros2 run xarm_perturbations joint_space_controller --ros-args \
  -p controller_type:=pid \
  -p output_topic:=/servo_server/delta_twist_cmds \
  -p trial_name:=trial_pdpid_nopert
```

### Safety features

- Saturación de torque: ±10 N·m por articulación
- Saturación de velocidad cartesiana: 0.10 m/s por eje
- Emergency stop si el error articular supera 0.8 rad (con grace period de 8 s al inicio)

---

## Análisis y Reporte

Las gráficas y el reporte de este trabajo se encuentran en:

```
src/xarm_perturbationsV2/xarm_perturbations/analysis/
├── ReporteTeam4.pdf
├── plot_trials.py                  # Script que genera todas las gráficas
└── plots/
    ├── joint_tracking_ctc_nopert.png     # Seguimiento articular — CTC sin pert.
    ├── joint_tracking_pid_nopert.png     # Seguimiento articular — PID sin pert.
    ├── joint_tracking_ctc_pert.png       # Seguimiento articular — CTC con pert.
    ├── joint_tracking_pid_pert.png       # Seguimiento articular — PID con pert.
    ├── taskspace_xyz_ctc_nopert.png      # Seguimiento XYZ — CTC sin pert.
    ├── taskspace_xyz_pid_nopert.png      # Seguimiento XYZ — PID sin pert.
    ├── taskspace_xyz_ctc_pert.png        # Seguimiento XYZ — CTC con pert.
    ├── taskspace_xyz_pid_pert.png        # Seguimiento XYZ — PID con pert.
    ├── path_3d_ctc_nopert.png            # Proyecciones de trayectoria — CTC sin pert.
    ├── path_3d_pid_nopert.png            # Proyecciones de trayectoria — PID sin pert.
    ├── path_3d_ctc_pert.png              # Proyecciones de trayectoria — CTC con pert.
    ├── path_3d_pid_pert.png              # Proyecciones de trayectoria — PID con pert.
    ├── ee_error_ctc_nopert.png           # Error de efector final — CTC sin pert.
    ├── ee_error_pid_nopert.png           # Error de efector final — PID sin pert.
    ├── ee_error_ctc_pert.png             # Error de efector final — CTC con pert.
    ├── ee_error_pid_pert.png             # Error de efector final — PID con pert.
    ├── phase_portraits_ctc_nopert.png    # Retratos de fase — CTC sin pert.
    ├── phase_portraits_pid_nopert.png    # Retratos de fase — PID sin pert.
    ├── phase_portraits_ctc_pert.png      # Retratos de fase — CTC con pert.
    ├── phase_portraits_pid_pert.png      # Retratos de fase — PID con pert.
    ├── comparison_sin_perturbaciones.png # CTC vs PID sin perturbación
    ├── comparison_con_perturbaciones.png # CTC vs PID con perturbación
    └── summary_table.png                 # Tabla resumen de métricas (RMSE, Max, WP%)
```

### Métricas reportadas

- **Joint RMSE / Max** — Error cuadrático medio y error máximo por articulación [mrad]
- **EE RMSE / Max** — Error del efector final en norma Euclidiana [mm]
- **Waypoint Success (%)** — Porcentaje de waypoints alcanzados con error < 5 mm en el último 50% del dwell

Para regenerar las gráficas:

```bash
python3 src/xarm_perturbationsV2/xarm_perturbations/analysis/plot_trials.py
```
