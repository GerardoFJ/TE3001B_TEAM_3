# TE3001B — Team 4

**Fundamentacion de Robotica** — Tecnológico de Monterrey, 2026

ROS 2 (Humble) workspace for bilateral teleoperation with force sensing between two xArm Lite 6 robots, using MoveIt Servo and micro-ROS ESP32 physical force sensors.

---

## Delivery Checklist

| Item | Location |
|------|----------|
| `reporte.pdf` | `Documents/ReporteFinal_Team4.pdf` |
| `reporte.tex` | `Documents/ReporteFinal_Team4.tex` |
| `master_control.py` | `src/xarm_bilateral_teleop/xarm_bilateral_teleop/master_node.py` |
| `slave_control.py` | `src/xarm_bilateral_teleop/xarm_bilateral_teleop/slave_node.py` |
| `force_estimator.py` | `src/xarm_bilateral_teleop/xarm_bilateral_teleop/force_estimator.py` |
| `net_test.py` | `src/xarm_teleops/src/net_test.py` |
| `data/` (bag file) | `papubag/papubag_0.db3` |
| `figures/` | `Documents/figures/` |
| `video_demo.mp4` | *(pending — record 60–120 s demo)* |
| `README.md` | `README.md` (this file) |

---

## Repository Structure

```
TE3001B_TEAM_3/
├── Documents/
│   ├── ReporteFinal_Team4.tex   # IEEE LaTeX report (source)
│   ├── ReporteFinal_Team4.pdf   # Compiled report
│   ├── gen_figures.py           # Script to regenerate all figures from bag
│   └── figures/                 # Generated PDF figures (9 files)
├── src/
│   ├── xarm_bilateral_teleop/   # Main bilateral teleop package (ROS 2)
│   │   ├── xarm_bilateral_teleop/
│   │   │   ├── master_node.py       # Admittance controller (master arm)
│   │   │   ├── slave_node.py        # Impedance follower (slave arm)
│   │   │   ├── force_estimator.py   # Sensorless force estimation (DLS + LPF)
│   │   │   ├── force_sensor_node.py # ROS 2 node wrapping ForceEstimator
│   │   │   ├── kinematics.py        # FK, geometric Jacobian, gravity torques
│   │   │   └── keyboard_teleop.py   # Manual keyboard override
│   │   ├── launch/
│   │   │   ├── bilateral_teleop.launch.py        # Real robots (both arms)
│   │   │   ├── bilateral_teleop_isaac.launch.py  # Isaac Sim backend
│   │   │   ├── master.launch.py                  # Master arm only
│   │   │   └── slave.launch.py                   # Slave arm only
│   │   └── config/
│   │       └── teleop_params.yaml   # All tuning parameters
│   ├── xarm_teleops/            # Legacy UDP teleoperation (reference)
│   │   └── src/
│   │       ├── master_control.py
│   │       ├── slave_control.py
│   │       ├── force_estimator.py
│   │       └── net_test.py
│   ├── xarm_perturbations/      # CTC vs PID joint-space control
│   ├── lite6_move/              # MoveIt Cartesian path planner
│   ├── s0s1_gazebo/             # SO101 gripper MuJoCo simulation
│   └── Challenges/              # Course challenges (interfaces, motor PID)
├── microros_force_sensor/       # ESP32 firmware — master force sensor
│   ├── src/main.cpp             #   HC-SR04 + joystick → /teleop/operator_force_direct
│   └── include/config.h         #   k=1000 N/m, ±3 N, port 8888
├── microros_slave_sensor/       # ESP32 firmware — slave force sensor
│   ├── src/main.cpp             #   HC-SR04 + joystick → /teleop/slave_force_direct
│   └── include/config.h         #   k=1500 N/m, ±6 N, port 8889
├── papubag/
│   └── papubag_0.db3            # ROS 2 bag — recorded experiment (37.6 s)
└── MiniReto_PIDMotor/           # PlatformIO firmware — ESP32 motor PWM
```

---

## Reproducing the Experiments

### Prerequisites

- ROS 2 Humble + MoveIt 2 + MoveIt Servo
- `xarm_ros2` package (UFACTORY): `https://github.com/xArm-Developer/xarm_ros2`
- Python 3.10, packages: `numpy`, `scipy`
- Two xArm Lite 6 robots on LAN (or Isaac Sim)
- Two ESP32 boards flashed with the firmware below

### 1. Build the workspace

```bash
cd ~/sch_ws/TE3001B_TEAM_3
colcon build --packages-select xarm_bilateral_teleop
source install/setup.bash
```

### 2. Flash the ESP32 force sensors (PlatformIO)

```bash
# Master sensor (port 8888, k=1000 N/m)
cd microros_force_sensor
pio run --target upload

# Slave sensor (port 8889, k=1500 N/m)
cd ../microros_slave_sensor
pio run --target upload
```

Both ESP32s connect to WiFi SSID `Team4` and publish `WrenchStamped` at 50 Hz
to a micro-ROS agent running on `192.168.1.53`.

### 3. Start micro-ROS agents

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888 &   # master sensor
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8889 &   # slave sensor
```

### 4. Launch bilateral teleoperation

**With real robots:**
```bash
ros2 launch xarm_bilateral_teleop bilateral_teleop.launch.py \
    master_ip:=192.168.1.175 slave_ip:=192.168.1.226
```

**With Isaac Sim (no physical robots):**
```bash
ros2 launch xarm_bilateral_teleop bilateral_teleop_isaac.launch.py
```

### 5. Tune parameters at runtime

All gains and thresholds are in `src/xarm_bilateral_teleop/config/teleop_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `Md` | `[2, 2, 2]` kg | Admittance virtual mass |
| `Bd` | `[10, 10, 10]` N·s/m | Admittance damping |
| `alpha` | `0.3` | Slave-to-master force reflection ratio |
| `Kp` | `[5, 5, 5]` 1/s | Slave impedance proportional gain |
| `max_cart_speed` | `0.10` m/s | Cartesian velocity clamp |
| `singularity_threshold` | `50.0` | κ(J) limit → halt motion |
| `force_deadband_n` | `1.5` N | Admittance deadband |

ESP32 sensor parameters (edit `include/config.h` and reflash):

| Parameter | Master | Slave |
|-----------|--------|-------|
| `SPRING_K` | 1000 N/m | 1500 N/m |
| `JOY_MAX_FORCE_N` | 3.0 N | 6.0 N |
| `AGENT_PORT` | 8888 | 8889 |

### 6. Regenerate report figures from bag

```bash
source /opt/ros/humble/setup.bash
python3 Documents/gen_figures.py
# Figures saved to Documents/figures/
```

### 7. Network latency test

```bash
python3 src/xarm_teleops/src/net_test.py
```

---

## Key Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/master/joint_states` | JointState | 250 Hz | Master arm state |
| `/slave/joint_states` | JointState | 250 Hz | Slave arm state |
| `/teleop/operator_force_direct` | WrenchStamped | 50 Hz | ESP32 master sensor |
| `/teleop/slave_force_direct` | WrenchStamped | 50 Hz | ESP32 slave sensor |
| `/teleop/master_pose` | PoseStamped | 50 Hz | Master EE pose (FK) |
| `/teleop/master_velocity` | TwistStamped | 50 Hz | Master EE velocity |
| `/master/servo_server/delta_twist_cmds` | TwistStamped | 50 Hz | Master MoveIt Servo input |
| `/slave/servo_server/delta_twist_cmds` | TwistStamped | 50 Hz | Slave MoveIt Servo input |

---

## Experimental Results (bag: `papubag/papubag_0.db3`)

| Metric | Value |
|--------|-------|
| Duration | 37.6 s |
| Joint tracking RMS (mean) | 0.702° |
| Joint tracking max (mean) | 2.522° |
| Force events detected (‖F‖ > 2 N) | 7 |
| Max operator force Fz | 38.5 N |
| `/master/joint_states` inter-arrival | μ=7.12 ms, P99=42.70 ms |
| `/slave/joint_states` inter-arrival | μ=7.05 ms, P99=39.66 ms |

---

## Safety

- Cartesian velocity clamp: ±0.10 m/s per axis
- Force clamping before admittance: ±20 N
- Singularity detection: κ(J) > 50 → publish zero twist
- Slave timeout: master pose stale > 0.5 s → publish zero twist

---

## License

See [LICENSE](LICENSE).
