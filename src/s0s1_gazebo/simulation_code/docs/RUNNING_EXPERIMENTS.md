# Running PID Tuning Experiments

## Overview

This guide walks through running the complete PID tuning suite for the SO101 robot arm.

## Step-by-Step: First Experiment

### 1. Verify Installation

```bash
cd /home/danielh/workspace/src/simulation_code

# Check dependencies
python -c "import mujoco, numpy, pandas, matplotlib; print('OK')"

# Check model exists
ls model/scene_urdf.xml
```

### 2. View Configuration

```bash
python run_pid_tuning_experiments.py --show-config
```

Output shows all 23 tuning combinations across P, PD, PI, PID families.

### 3. Run a Single Combination

```bash
# Run P-1 (Very Low Kp) only
python run_pid_tuning_experiments.py --combos P-1 --skip-viewer

# This will:
#   1. Load the model
#   2. Configure PID with P-1 gains
#   3. Execute 8-second trajectory (move → hold → return → hold)
#   4. Apply perturbations throughout
#   5. Save results to results/tuning/P-1/
```

Expected output:
```
============================================================
Running experiment: P-1
============================================================
Family: P
Name: Very Low Kp (0.5×)
Philosophy: Minimal stiffness; very slow response...
Executing trajectory (move_dur=2.0s, hold_dur=2.0s)...
Experiment completed in 45.2s
Saving to results/tuning/P-1/...
Overall error: 0.2847 rad
Overall effort: 1.3256 N⋅m
Disturbance rejection: 0.778
```

### 4. Inspect Results

```bash
# View raw data
ls results/tuning/P-1/
# Output:
#   metadata.json    (gains, configuration)
#   data.csv         (timestep data)
#   summary.json     (summary statistics)
#   data.pkl         (pickle for Python)

# Quick summary statistics
cat results/tuning/P-1/summary.json | python -m json.tool
```

### 5. Run a Full Family

```bash
# Run all P combinations (P-1 through P-5)
python run_pid_tuning_experiments.py --families P --skip-viewer

# This runs 5 experiments sequentially; total ~4 minutes
```

### 6. Generate Analysis

After experiments complete:

```bash
# Regenerate plots and tables
python run_pid_tuning_experiments.py --analyze-only

# This produces:
#   results/analysis/P_qualitative_table.csv
#   results/analysis/P-1_overview.png
#   results/analysis/P-2_overview.png
#   ...
#   results/analysis/P_comparison.png
```

## Running All Experiments

### Full Suite

```bash
# All 23 experiments with viewer (interactive, slow)
python run_pid_tuning_experiments.py

# All 23 experiments, headless (no viewer, fast)
python run_pid_tuning_experiments.py --skip-viewer

# Expected runtime: ~30 minutes (headless)
```

### Selective Runs

```bash
# Only PD and PID families (12 experiments)
python run_pid_tuning_experiments.py --families PD PID --skip-viewer

# Only PI family (6 experiments)
python run_pid_tuning_experiments.py --families PI --skip-viewer

# Specific combos: P-2, PD-1, PID-3
python run_pid_tuning_experiments.py --combos P-2 PD-1 PID-3 --skip-viewer
```

## Understanding the Output

### Data CSV Structure

`results/tuning/<combo_id>/data.csv` has columns:

```
t,phase,
q_des_shoulder_pan,q_meas_shoulder_pan,qd_meas_shoulder_pan,e_pos_shoulder_pan,tau_pid_shoulder_pan,tau_dist_shoulder_pan,tau_total_shoulder_pan,
... (repeated for each joint)
```

For each row (timestep):
- `t`: Simulation time (seconds)
- `phase`: One of move_to_zero, hold_zero, return_start, hold_start
- `q_des_<joint>`: Desired position (radians)
- `q_meas_<joint>`: Measured position (radians)
- `qd_meas_<joint>`: Measured velocity (rad/s)
- `e_pos_<joint>`: Position error = q_des - q_meas (radians)
- `tau_pid_<joint>`: PID control torque (N⋅m)
- `tau_dist_<joint>`: Disturbance torque (N⋅m)
- `tau_total_<joint>`: Total applied torque (N⋅m)

### Metadata JSON

`results/tuning/<combo_id>/metadata.json`:

```json
{
  "combo_id": "P-1",
  "family": "P",
  "name": "Very Low Kp (0.5×)",
  "philosophy": "Minimal stiffness; very slow response...",
  "timestamp": "2025-02-17T14:32:00",
  "hostname": "your-machine",
  "gains": {
    "shoulder_pan": {"kp": 12.5, "ki": 0.0, "kd": 0.0, ...},
    "shoulder_lift": {"kp": 25.0, "ki": 0.0, "kd": 0.0, ...},
    ...
  },
  "perturbation_config": {
    "sinus_amp": 0.8,
    "sinus_freq_hz": 0.5,
    "noise_std": 0.25,
    ...
  }
}
```

### Summary JSON

`results/tuning/<combo_id>/summary.json`:

```json
{
  "combo_id": "P-1",
  "family": "P",
  "duration_s": 8.02,
  "num_samples": 1605,
  "joints": {
    "shoulder_pan": {
      "e_pos_mean_rad": 0.0847,
      "e_pos_max_rad": 0.2345,
      "e_pos_std_rad": 0.0632,
      "tau_mean_Nm": 1.234,
      "tau_max_Nm": 4.567,
      ...
    }
  }
}
```

## Recovering Failed Experiments

If an experiment crashes:

```bash
# The partially-saved result will remain invalid.
# Remove it and re-run:
rm -rf results/tuning/P-1/
python run_pid_tuning_experiments.py --combos P-1 --skip-viewer
```

Or resume from checkpoint:

```bash
# Re-run all; existing combos are automatically skipped:
python run_pid_tuning_experiments.py --families P --skip-viewer
# Only new ones run; already-completed P-1, P-2 are skipped
```

## Distributed / Parallel Execution

For large-scale studies, run combinations on multiple machines:

```bash
# Machine 1: Run P and PD
machine1$ python run_pid_tuning_experiments.py --families P PD --skip-viewer

# Machine 2: Run PI and PID
machine2$ python run_pid_tuning_experiments.py --families PI PID --skip-viewer

# Collect results (e.g., via shared network drive)
# Then run analysis on combined results
python run_pid_tuning_experiments.py --analyze-only
```

## Performance Tips

### Speedup

1. **Headless mode**: Remove viewer overhead
   ```bash
   python run_pid_tuning_experiments.py --skip-viewer
   ```

2. **Run a subset**: Test P family first before PID
   ```bash
   python run_pid_tuning_experiments.py --families P --skip-viewer
   ```

3. **Sequential vs. parallel**: Currently sequential (easy recovery if one fails)
   - Multi-processing requires careful handling of MuJoCo state

### Slowdown (but more insight)

1. **With viewer**: Interactive observation of robot motion
   ```bash
   python run_pid_tuning_experiments.py --combos P-1 --families P
   ```

2. **Realtime simulation**: Matches wall-clock time (better video recording)
   ```bash
   python run_pid_tuning_experiments.py --combos P-1 --realtime --skip-viewer
   ```

## Customization

### Change Perturbation Strength

Edit `run_pid_tuning_experiments.py`, function `DEFAULT_PERTURB_CONFIG`:

```python
# Make disturbances more severe:
DEFAULT_PERTURB_CONFIG = PerturbationConfig(
    sinus_amp=1.5,            # Increase from 0.8
    impulse_prob_per_s=0.20,  # More frequent
    impulse_mag=3.0,          # Stronger impulses
)

# Now re-run experiments
python run_pid_tuning_experiments.py --skip-viewer
```

### Change Trajectory Timing

Edit `MOVE_DURATION` and `HOLD_DURATION`:

```python
MOVE_DURATION = 3.0      # Slower movement (default: 2.0)
HOLD_DURATION = 3.0      # Longer holds   (default: 2.0)
```

### Change Model

```bash
# Use custom model XML
python run_pid_tuning_experiments.py --model path/to/custom_model.xml --skip-viewer
```

## Next Steps

1. **Analyze Results**: See `INTERPRETING_RESULTS.md`
2. **Understand Theory**: See `CONTROLLER_THEORY.md`
3. **Generate Report**: Extract plots and tables from `results/analysis/`
4. **Cross-Simulator**: Export best config to Gazebo (future step)
