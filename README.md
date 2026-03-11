# TE3001B — Team 3

**Fundamentacion de Robotica** — Tecnologico de Monterrey, 2026

ROS 2 (Humble) workspace for xArm Lite 6 control, teleoperation, and embedded motor control.

## Repository Structure

```
TE3001B_TEAM_3/
├── src/                          # ROS 2 packages
│   ├── Challenges/               # Course challenges
│   │   ├── custom_interfaces/    # SetProcessBool.srv definition
│   │   ├── main_pkg/             # Basic publisher/subscriber (C++)
│   │   └── motor_control/        # DC motor PID simulation
│   ├── lite6_move/               # MoveIt Cartesian path planner for Lite6
│   ├── s0s1_gazebo/              # SO101 gripper MuJoCo simulation + URDF
│   ├── xarm_perturbations/       # CTC vs PID joint-space control under perturbations
│   ├── xarm_teleops/             # Master-slave teleoperation with force feedback
│   ├── uros_ws/                  # Micro-ROS workspace (ESP32 motor control)
│   └── Retosemana3/              # Week 3 challenge — teleoperation data analysis
├── MiniReto_PIDMotor/            # PlatformIO firmware — ESP32 motor PWM via micro-ROS
└── README.md
```

## Packages

### Challenges (`src/Challenges/`)

| Package | Type | Description |
|---------|------|-------------|
| `custom_interfaces` | ament_cmake | `SetProcessBool.srv` service definition |
| `main_pkg` | ament_cmake | `sender` and `process` nodes — basic ROS 2 messaging |
| `motor_control` | ament_python | DC motor simulation with PID controller (`dc_motor`, `set_point`, `controller`) |

### xArm Lite 6 Packages

| Package | Type | Description |
|---------|------|-------------|
| `lite6_move` | ament_cmake | MoveIt Cartesian path planning for Lite 6 |
| `xarm_perturbations` | ament_python | Joint-space CTC vs PID controllers with perturbation analysis |
| `xarm_teleops` | ament_python | Two-computer master-slave teleoperation with haptic force feedback |

### Simulation

| Package | Type | Description |
|---------|------|-------------|
| `s0s1_gazebo` | ament_cmake | SO101 gripper MuJoCo simulation with ROS 2 bridge and PID control |

### Embedded / Micro-ROS

| Package | Type | Description |
|---------|------|-------------|
| `uros_ws` | micro-ROS | ESP32 motor control via micro-ROS agent |
| `MiniReto_PIDMotor` | PlatformIO | ESP32 firmware for PWM motor control (micro-ROS subscriber) |

## Build

ROS 2 runs inside Docker. Build inside the container:

```bash
cd ~/dev_ws
colcon build
source install/setup.bash
```

For PlatformIO firmware (`MiniReto_PIDMotor/`):

```bash
pio run
pio run --target upload
```

## xarm_perturbationsV2 — CTC vs PID Control Challenge

Joint-space control comparison on xArm Lite 6 with and without external perturbations.

**Pipeline:**
```
IK Reference Generator → Joint-Space Controller → Perturbation Injector → MoveIt Servo → xArm Lite 6
```

**Nodes:**
- `ik_reference_generator` — IK-based joint reference trajectories (q, dq, ddq)
- `joint_space_controller` — PID or CTC (Computed Torque Control) in joint space
- `perturbation_injector` — Sine or Gaussian disturbance injection

**Controllers:**
- **PID:** `tau = -(Kp*e + Kd*de + Ki*int_e)` with anti-windup
- **CTC:** `tau = M(q)*v + G(q) + F(dq)` with feedback linearization

**Trials:**

| Trial | Controller | Perturbation |
|-------|------------|--------------|
| `trial_ctc_nopert` | CTC | None |
| `trial_pdpid_nopert` | PID | None |
| `trial_ctc_pert` | CTC | Gaussian (x-axis, sigma=0.01 m/s) |
| `trial_pdpid_pert` | PID | Gaussian (x-axis, sigma=0.01 m/s) |

**Usage:**

```bash
# Terminal 0: Launch MoveIt Servo
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.175

# Terminal 1: Reference generator
ros2 run xarm_perturbations ik_reference_generator --ros-args \
  -p wz:=2.5 -p lam:=0.015 -p k_task:=14.0 -p k_null:=1.5 \
  -p dwell_sec:=1.5 -p segment_sec:=2.0 -p control_rate_hz:=200.0

# Terminal 2: Controller (change controller_type and trial_name per trial)
ros2 run xarm_perturbations joint_space_controller --ros-args \
  -p controller_type:=ctc \
  -p output_topic:=/servo_server/delta_twist_cmds \
  -p trial_name:=trial_ctc_nopert

# Terminal 3 (perturbation trials only):
ros2 run xarm_perturbations perturbation_injector --ros-args \
  -p input_topic:=/controller_output \
  -p output_topic:=/servo_server/delta_twist_cmds \
  -p enabled:=true -p mode:=gaussian \
  -p gauss_std_linear:=0.01 -p gauss_axis:=x
```

Results and analysis: `src/xarm_perturbations/analysis/`

## xarm_teleops — Teleoperation with Force Feedback

Master-slave teleoperation between two xArm Lite 6 robots over UDP.

```
PC Master                           PC Slave
┌──────────────────┐   UDP 5005   ┌──────────────────┐
│ master_control   │ ───────────► │ slave_control    │
│                  │ ◄──────────  │                  │
│  q_master →      │   UDP 5006  │  → ForceEstimator│
│  ← F_ext, tau    │  (F_ext, τ) │  ← tau_meas      │
└────────┬─────────┘             └────────┬─────────┘
         │ Ethernet                       │ Ethernet
   xArm Lite 6 (Master)           xArm Lite 6 (Slave)
```

**Force estimation:** `F_ext = (J^T)^+ * (tau_meas - tau_gravity)`

```bash
# Slave PC (first):
python3 src/slave_control.py

# Master PC:
python3 src/master_control.py
```

## Safety

- Torque saturation: +/-10 Nm per joint
- Cartesian velocity saturation: 0.10 m/s per axis
- Emergency stop on joint error > 0.8 rad (8 s grace period at startup)

## License

See [LICENSE](LICENSE).
