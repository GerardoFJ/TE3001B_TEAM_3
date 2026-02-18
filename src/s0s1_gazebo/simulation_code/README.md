# SO101 PID Tuning & Cross-Simulator Validation

A systematic framework for tuning and validating PID controllers on the SO101 6-DOF robot arm in MuJoCo simulation with cross-simulator validation in Gazebo.

## Overview

This project implements a rigorous engineering investigation of four PID controller families:

- **P (Proportional)**: 5 gain combinations exploring stiffness effects
- **PD (Proportional-Derivative)**: 6 combinations emphasizing speed vs. damping
- **PI (Proportional-Integral)**: 6 combinations studying steady-state elimination & windup
- **PID (Full Control)**: 6 combinations balancing all three actions

Each experiment subjects the robot to **strong perturbations** (sinusoidal + noise + impulses) and records:
- Joint positions and velocities
- Control torques (PID + disturbance + total)
- Position and velocity errors
- Trajectory phases (move → hold → return → hold)

## Project Structure

```
.
├── pid_tuning_config.py          # Experiment configuration (23 gain combinations)
├── pid_tuning_experiment.py       # Experiment runner & data logging
├── pid_analysis.py                # Metrics computation & visualization
├── run_pid_tuning_experiments.py  # Main experiment executor
│
├── so101_control.py               # Core PID controller, perturbation model
├── so101_mujoco_pid_utils.py      # MuJoCo integration helpers
├── so101_mujoco_utils.py          # Utility functions
├── so101_mujoco_utils2.py         # Realtime plotter (Dash/Plotly)
├── so101_ros_bridge.py            # ROS/Gazebo command logging
│
├── model/                         # MuJoCo XML & assets
│   ├── scene_urdf.xml
│   ├── robot_from_urdf.xml
│   └── assets/
│
├── results/
│   ├── tuning/                    # Raw experiment data (combo_id/*)
│   │   ├── P-1/
│   │   │   ├── metadata.json      # Configuration & gains
│   │   │   ├── data.csv           # Timestep data (CSV)
│   │   │   ├── data.pkl           # Timestep data (pickle)
│   │   │   └── summary.json       # Quick statistics
│   │   └── ... (other combos)
│   └── analysis/                  # Analysis output
│       ├── P_qualitative_table.csv
│       ├── PD_qualitative_table.csv
│       ├── PI_qualitative_table.csv
│       ├── PID_qualitative_table.csv
│       ├── P-1_overview.png        # Per-combo plots
│       └── ... (other plots)
│
├── pyproject.toml
├── README.md                      # This file
└── docs/                          # Additional documentation
    ├── RUNNING_EXPERIMENTS.md
    ├── INTERPRETING_RESULTS.md
    └── CONTROLLER_THEORY.md
```

## Installation & Setup

### Prerequisites

- Python ≥ 3.10
- MuJoCo ≥ 3.1.0
- Standard scientific stack: numpy, scipy, pandas, matplotlib

### Installation

```bash
# Clone or navigate to workspace
cd /home/danielh/workspace/src/simulation_code

# Install dependencies (using uv or pip)
uv sync
# OR
pip install -r requirements.txt

# Verify model path is correct
ls model/scene_urdf.xml  # Should exist
```

## Quick Start

### 1. Run All Experiments

```bash
# Full suite: P, PD, PI, PID (20+ experiments)
python run_pid_tuning_experiments.py

# Without viewer (headless, faster)
python run_pid_tuning_experiments.py --skip-viewer

# At realtime simulation speed (slower)
python run_pid_tuning_experiments.py --realtime
```

### 2. Run Specific Families

```bash
# Only P and PD families
python run_pid_tuning_experiments.py --families P PD

# Only PID family
python run_pid_tuning_experiments.py --families PID
```

### 3. Run Specific Combinations

```bash
# Only P-1, P-2, PD-1
python run_pid_tuning_experiments.py --combos P-1 P-2 PD-1
```

### 4. View Configuration

```bash
python run_pid_tuning_experiments.py --show-config
```

### 5. Analyze Only (No New Experiments)

```bash
# Load existing results and regenerate analysis
python run_pid_tuning_experiments.py --analyze-only
```

## Experiment Structure

Each experiment follows this trajectory:

```
Time    0s          2s           4s           6s           8s       
Phase   move_to     hold_zero    return_start hold_start
        zero        (2s)         (2s)         (2s)
        (2s)
        
        q(0°)       q(0°)        q(start°)    q(start°)
```

### Perturbations (Active Throughout)

All experiments apply the same perturbation profile:
- **Sinusoidal**: 0.8 N⋅m amplitude, 0.5 Hz (slow fatigue-like disturbance)
- **Colored Noise**: 0.25 N⋅m std, 0.25s time constant (band-limited randomness)
- **Impulses**: 12% probability/sec, 2.0 N⋅m magnitude, 50ms duration (sudden hits)

This forces controllers to demonstrate **disturbance rejection** alongside tracking accuracy.

## Tuning Combinations

### P Family (Proportional Only)

| ID  | Name              | Kp    | Philosophy                                              |
|-----|-------------------|-------|--------------------------------------------------------|
| P-1 | Very Low Kp       | 12.5  | Minimal stiffness; very slow response, large error     |
| P-2 | Low Kp (baseline) | 25.0  | Standard stiffness; significant steady-state error     |
| P-3 | Medium Kp         | 37.5  | Increased stiffness; faster but may oscillate          |
| P-4 | High Kp           | 50.0  | Strong stiffness, fast; risk of overshoot              |
| P-5 | Very High Kp      | 62.5  | Aggressive; near instability, high-frequency chatter   |

### PD Family (With Derivative Damping)

| ID  | Name                      | Kp  | Kd   | Philosophy                                              |
|-----|---------------------------|-----|------|--------------------------------------------------------|
| PD-1| Low Kp + Low Kd          | 20  | 0.5  | Gentle, stable; slow response                          |
| PD-2| Medium Kp + Light Kd     | 30  | 1.0  | Balanced speed and comfort                             |
| PD-3| Medium Kp + Medium Kd    | 40  | 2.0  | Increased damping stabilizes faster response           |
| PD-4| High Kp + High Kd        | 50  | 3.0  | Fast, well-damped; still has steady-state error        |
| PD-5| Heavy Kd Dominance       | 25  | 5.0  | Derivative dominates; over-damped, slow                |
| PD-6| Very High Kp + Moderate Kd| 60 | 2.0  | Aggressive Kp with moderate damping; noise risk         |

### PI Family (With Integral Bias Removal)

| ID  | Name                    | Kp | Ki  | Philosophy                                              |
|-----|-------------------------|----|----|--------------------------------------------------------|
| PI-1| Low Kp + Low Ki        | 20 | 0.1 | Minimal integral; slow bias removal, low windup risk    |
| PI-2| Medium Kp + Light Ki   | 30 | 0.3 | Moderate integral; trades speed for steady-state       |
| PI-3| Medium Kp + Medium Ki  | 35 | 0.6 | Balanced integral action                               |
| PI-4| High Kp + High Ki      | 45 | 1.0 | Aggressive integral; risk of overshoot & oscillation    |
| PI-5| Low Kp + Aggressive Ki | 20 | 1.5 | Integrator dominates; high oscillation risk             |
| PI-6| Medium Kp + High Ki (Limited)| 32 | 0.9 | High Ki but limited state; balances rejection vs. stability|

### PID Family (Full Control)

| ID  | Name                    | Kp | Ki  | Kd  | Philosophy                                              |
|-----|-------------------------|----|----|-----|-------------------------------------------------------| 
| PID-1| Conservative (all low) | 20 | 0.1 | 0.5 | Safe, smooth; slow bias removal, light damping         |
| PID-2| Balanced (baseline)    | 30 | 0.3 | 1.0 | All terms proportional; moderate response              |
| PID-3| Fast Response          | 45 | 0.5 | 1.5 | Emphasizes speed; watch for overshoot                  |
| PID-4| Smooth Damped          | 35 | 0.2 | 2.5 | High derivative dominates transient response           |
| PID-5| Aggressive (all high)  | 55 | 0.9 | 2.5 | Fast + damped + bias removal; near instability          |
| PID-6| Disturbance-Focused    | 40 | 1.2 | 1.0 | High Ki focuses on disturbance rejection                |

**Note**: All gains are scaled per-joint by inertia (shoulder > elbow > wrist) and applied torque limits per joint.

## Output Interpretation

### Data Files

For each experiment (`results/tuning/<combo_id>/`):

- **metadata.json**: Experiment configuration, gains, perturbation settings
- **data.csv**: Timestep data (t, q_des, q_meas, qd_meas, errors, torques, phase)
- **data.pkl**: Binary pickle of timestep objects (for Python analysis)
- **summary.json**: Aggregate statistics (mean/max errors, torques by joint)

### Plot Files

(`results/analysis/`)

For each experiment:
- **`<combo_id>_overview.png`**: 3-row subplot for each joint:
  - Row 1: Desired vs. Measured Position
  - Row 2: Position Error vs. Velocity (dual axis)
  - Row 3: PID Torque, Disturbance, Total Torque

For each family:
- **`<family>_comparison.png`**: All combinations for that family side-by-side

### Qualitative Tables

CSV files with per-family observations:

| Column | Meaning | Examples |
|--------|---------|----------|
| Combo ID | Experiment ID | P-1, PID-4 |
| Name | Human name | "Medium Kp + High Kd" |
| Rise & Speed | Time to reach 90% | Slow, Moderate, Fast |
| Overshoot | Peak excess beyond target | None, Small, Large |
| Oscillation | Ringing/cycling behavior | None, Light ringing, Persistent |
| Steady-State Error | Final position error | ≈0, Small (~0.05 rad), Large (>0.15 rad) |
| Disturbance Rejection | Robustness metric (0–1) | 0.85, 0.72 |
| Stability | Risk assessment | Stable, Questionable, Marginal |
| Engineering Notes | Summary logic | "Derivative improves damping" |

## Expected Observations

### P Control

- **Low Kp**: Sluggish, large steady-state error, no overshoot
- **Medium Kp**: Acceptable rise time, growing overshoot as Kp increases
- **High Kp**: Fast but unstable, persistent oscillations, may saturate

**Insight**: Stiffness (Kp) directly proportional to rise time and error sensitivity.

### PD Control

- **Light Kd**: Derivative barely visible
- **Moderate Kd**: Well-damped response, smooth settlement
- **Heavy Kd**: Over-damped, sluggish despite high Kp; noise amplification risk

**Insight**: Derivative (Kd) term provides damping that converts oscillation energy into heat.

### PI Control

- **Low Ki**: Still has steady-state error (integral action too weak)
- **Moderate Ki**: Integrator gradually removes error; slow but thorough
- **High Ki**: Can cause overshoot/oscillation (lag in integral response)
- **Tight i_limit**: Prevents windup under disturbance

**Insight**: Integral (Ki) term only removes **constant** bias; useless during transients; dangerous if not limited.

### PID Control

- **Balanced PID**: Combines speed (Kp), damping (Kd), and steady-state removal (Ki)
- **Best for disturbance rejection**: High Ki + Kd manages both noise rejection and smoothness
- **Limitations**: Cannot exceed maximum torque limits; no magic tuning for all conditions

**Insight**: Disturbance rejection requires high gains (→ stiffness) but risks instability; derivative damping is the key stabilizer.

## Analysis Functions

All analysis is encapsulated in `pid_analysis.py`:

```python
from pid_analysis import (
    compute_experiment_metrics,      # Single-experiment analysis
    compute_joint_metrics,           # Per-joint statistics
    generate_qualitative_table,      # Build comparison table
    plot_experiment,                 # Visualize one combo
    plot_family_comparison,          # Side-by-side all combos in family
)

from pid_tuning_experiment import ExperimentDataset

# Load results
dataset = ExperimentDataset("results/tuning")
dataset.load_all()

# Analyze a specific result
result = dataset.results["P-1"]
metrics = compute_experiment_metrics(result)
print(f"Error: {metrics.overall_error_rad:.4f} rad")
print(f"Effort: {metrics.overall_effort_Nm:.4f} N⋅m")

# Visualize
plot_experiment(result, output_dir="results/analysis")
```

## Extending This Framework

### Adding Custom Gain Combinations

Edit `pid_tuning_config.py`:

```python
P_TUNINGS.append(
    TuningCombination(
        combo_id="P-6",
        family="P",
        name="Custom Test",
        kp_base=72.5,
        ki_base=0.0,
        kd_base=0.0,
        philosophy="My custom tuning strategy..."
    )
)
```

### Modifying Perturbations

Edit `DEFAULT_PERTURB_CONFIG` in `run_pid_tuning_experiments.py`:

```python
DEFAULT_PERTURB_CONFIG = PerturbationConfig(
    sinus_amp=1.5,          # Increase sinusoidal disturbance
    impulse_prob_per_s=0.2, # More frequent impulses
    # ... other fields
)
```

### Custom Analysis Metrics

Extend `pid_analysis.py` with new metric classes:

```python
@dataclass
class CustomMetric:
    # Your custom metrics here
    pass

def compute_custom_metrics(result: ExperimentResult) -> CustomMetric:
    # Your analysis logic
    pass
```

## Grading Rubric (100 points)

| Category | Points | Evaluated |
|----------|--------|-----------|
| Experimental Completeness | 20 | ≥5 combinations per family, all phases |
| Plot Quality | 15 | Readable, labeled, informative |
| Qualitative Tables | 20 | Precise, technical, data-driven |
| Understanding of Kp | 5 | Relation to stiffness/speed |
| Understanding of Kd | 5 | Relation to damping/settling |
| Understanding of Ki | 5 | Relation to steady-state/windup |
| Best Configuration | 10 | Justified selection with data |
| Gazebo Validation | 10 | Command replay, comparison analysis |
| Simulator Comparison | 10 | Insight into differences |

## Common Issues & Troubleshooting

### "Joint not found" error

Check that model XML has correct joint names:
```bash
grep "name=\"shoulder_pan\"" model/scene_urdf.xml
```

### Viewer crashes with GLX error

Use headless mode:
```bash
python run_pid_tuning_experiments.py --skip-viewer
```

### Results not saved

Check that `results/tuning/` directory exists and is writable:
```bash
mkdir -p results/tuning
chmod 755 results/tuning
```

### Analysis plots not generating

Ensure matplotlib backend is configured (check for Agg):
```python
import matplotlib
matplotlib.use('Agg')  # Use before any plotting
```

## References

- MuJoCo documentation: https://mujoco.readthedocs.io/
- PID control theory: K.J. Åström & R.M. Murray, "Feedback Systems" (Chapter 11)
- Perturbation & robustness: S. Skogestad, "Simple analytic rules for model reduction and PID tuning"

## License

[To be specified by your institution]

## Contact & Support

For questions about the framework:
- Check `docs/` directory for detailed guides
- Review example output in `results/analysis/`
- Consult inline code comments in control modules

---

**Last Updated**: February 2026  
**Author**: SO101 Control Engineering Team
