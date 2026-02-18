# SO101 PID Tuning Framework - Quick Start Guide

Welcome to the **SO101 PID Tuning & Cross-Simulator Validation** project!

This framework provides a systematic, data-driven approach to tuning PID controllers on the SO101 robot arm in MuJoCo, with support for cross-simulator validation in Gazebo.

## What's Included

### Core Modules
- **`pid_tuning_config.py`**: 23 preconfigured gain combinations (P, PD, PI, PID families)
- **`pid_tuning_experiment.py`**: Experiment runner and data collection system
- **`pid_analysis.py`**: Metrics computation and visualization
- **`run_pid_tuning_experiments.py`**: Main executable for running all experiments

### Existing Integration  
- **`so101_control.py`**: PID controller + perturbation model (from earlier work)
- **`so101_mujoco_pid_utils.py`**: MuJoCo integration helpers
- **`so101_mujoco_utils.py`, `so101_mujoco_utils2.py`**: Utility functions

### Documentation
- **`README.md`**: Complete project guide
- **`docs/RUNNING_EXPERIMENTS.md`**: How to execute experiments
- **`docs/INTERPRETING_RESULTS.md`**: How to read plots and tables
- **`docs/CONTROLLER_THEORY.md`**: PID control physics & mathematics

## 5-Minute Start

### 1. Check Installation
```bash
python -c "import mujoco, numpy, pandas, matplotlib; print('OK')"
ls model/scene_urdf.xml
```

### 2. Run One Experiment
```bash
python run_pid_tuning_experiments.py --combos P-1 --skip-viewer
```

Expected output:
```
============================================================
Running experiment: P-1
...
Experiment completed in 45.2s
Overall error: 0.2847 rad
Overall effort: 1.3256 N⋅m
```

### 3. See Results
```bash
ls results/tuning/P-1/
# metadata.json, data.csv, summary.json, data.pkl

cat results/tuning/P-1/summary.json | python -m json.tool
```

### 4. Run All & Analyze
```bash
# Run all 23 experiments (30 minutes)
python run_pid_tuning_experiments.py --skip-viewer

# Generate plots & tables
python run_pid_tuning_experiments.py --analyze-only

# View results
ls results/analysis/
# P_qualitative_table.csv, P-1_overview.png, P_comparison.png, ...
```

## Project Structure

```
so101-pid-tuning/
├── README.md                              ← START HERE
├── requirements.txt                       ← Install dependencies
│
├── Core Framework:
├── pid_tuning_config.py                   (23 gain combinations)
├── pid_tuning_experiment.py               (experiment runner)
├── pid_analysis.py                        (analysis & plotting)
├── run_pid_tuning_experiments.py          (main executable) ★
│
├── Integration (Existing):
├── so101_control.py                       (PID + perturbation model)
├── so101_mujoco_pid_utils.py              (MuJoCo helpers)
├── so101_mujoco_utils.py
├── so101_mujoco_utils2.py
├── so101_ros_bridge.py
│
├── Documentation:
├── docs/
│   ├── RUNNING_EXPERIMENTS.md         ← How to run
│   ├── INTERPRETING_RESULTS.md        ← How to read results
│   └── CONTROLLER_THEORY.md           ← PID physics
│
├── Data & Results:
├── model/                                 (MuJoCo XML and assets)
├── results/
│   ├── tuning/                            (raw experiment data)
│   │   ├── P-1/  (metadata.json, data.csv, data.pkl, summary.json)
│   │   ├── P-2/
│   │   └── ...
│   └── analysis/                          (plots and tables)
│       ├── P_qualitative_table.csv
│       ├── P-1_overview.png
│       └── ...
│
├── Configuration:
├── pyproject.toml
├── .gitignore
└── uv.lock
```

## Key Commands

```bash
# Show all 23 gain combinations
python run_pid_tuning_experiments.py --show-config

# Run specific families
python run_pid_tuning_experiments.py --families P PD --skip-viewer

# Run specific combos
python run_pid_tuning_experiments.py --combos P-1 PD-1 PID-2 --skip-viewer

# Run with viewer (interactive, slower)
python run_pid_tuning_experiments.py --combos P-1

# Analyze existing results only
python run_pid_tuning_experiments.py --analyze-only

# Help
python run_pid_tuning_experiments.py --help
```

## What Gets Generated

### Per Experiment
- **metadata.json**: Configuration, gains, perturbations
- **data.csv**: Timestep data (q_des, q_meas, e_pos, tau_pid, tau_dist, etc.)
- **summary.json**: Quick statistics (mean/max errors, torques)

### Analysis
- **Plots**: 3-row subplots for each joint (position, error, torques)
- **Family Comparisons**: Side-by-side all combos in a family
- **Qualitative Tables**: CSV with observations (rise time, overshoot, stability, etc.)

## Next Steps

1. **Read** `README.md` for full overview
2. **Run** a test experiment (e.g., `P-1`) to verify setup
3. **Study** `docs/CONTROLLER_THEORY.md` to understand PID mechanics
4. **Run** all experiments: `python run_pid_tuning_experiments.py --skip-viewer`
5. **Analyze** results in `results/analysis/`
6. **Write** your report with tables and plots
7. **Push** to GitHub with complete results

## Key Insights

### P Control
- ✓ Simple, robust (no integrators to cause windup)
- ✗ Cannot eliminate steady-state error
- ✗ Cannot damp oscillations (marginal stability at high Kp)

### PD Control  
- ✓ Fast response + smooth (derivative damping)
- ✓ No steady-state error removal, but adequate for many tasks
- ✓ Excellent disturbance rejection with adequate damping
- ✗ Still has residual error under constant load

### PI Control
- ✓ Eliminates steady-state error (integral removes bias)
- ✗ Slower transient response (integral takes time to build up)
- ✗ Risk of oscillation and windup if Ki too high

### PID Control
- ✓ Combines speed (Kp), damping (Kd), and zero error (Ki)
- ✓ Best all-around performance if tuned well
- ✗ More complex; requires anti-windup logic
- ✗ More tuning parameters to dial in

**Best for SO101 arm**: **PID-2 (balanced) or PD-3 (smooth)** depending on your priority (zero error vs. robustness).

## File Guide: Where to Look

| I want to... | Go to... |
|--------------|----------|
| **Run experiments** | `run_pid_tuning_experiments.py` (main executable) |
| **Understand configurations** | `pid_tuning_config.py` + `README.md` Table 2 |
| **See how PID works** | `so101_control.py` Class `JointPID` |
| **Extend framework** | `pid_tuning_experiment.py` Class `ExperimentRunner` |
| **Add custom metrics** | `pid_analysis.py` Function `compute_joint_metrics()` |
| **Read step-by-step guide** | `docs/RUNNING_EXPERIMENTS.md` |
| **Interpret my results** | `docs/INTERPRETING_RESULTS.md` |
| **Understand control theory** | `docs/CONTROLLER_THEORY.md` |

## Support & Troubleshooting

**"Module not found"**:
```bash
pip install -r requirements.txt
```

**"Joint not found"**:
```bash
grep "name=\"shoulder_pan\"" model/scene_urdf.xml
```

**"Viewer crashes (GLX error)"**:
```bash
python run_pid_tuning_experiments.py --skip-viewer
```

**"Results not saving"**:
```bash
mkdir -p results/tuning results/analysis
chmod 755 results/tuning results/analysis
```

**"Want to run faster"**:
```bash
python run_pid_tuning_experiments.py --skip-viewer --families P  # Just P family
```

## Grading Checklist

- [ ] Run at least 5 combinations per family (P, PD, PI, PID) → 20 points min
- [ ] Generate readable plots (3-row subplots per combo) → 15 points
- [ ] Create qualitative tables (technical observations per family) → 20 points
- [ ] Explain Kp effect (stiffness, speed, error) → 5 points
- [ ] Explain Kd effect (damping, settling) → 5 points
- [ ] Explain Ki effect (steady-state removal, windup) → 5 points
- [ ] Justify best configuration with data → 10 points
- [ ] Gazebo replay (if required) → 10 points
- [ ] Cross-simulator comparison (if required) → 10 points
- [ ] GitHub repo with README → Prerequisite; 0/110 without it

**Total**: 100 points

---

**Questions?** See `docs/` or review inline code comments in control modules.

**Ready?** Start with: `python run_pid_tuning_experiments.py --show-config`
