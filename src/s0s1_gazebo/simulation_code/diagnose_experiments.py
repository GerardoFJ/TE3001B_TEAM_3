#!/usr/bin/env python3
# diagnose_experiments.py
"""
Diagnostic script to verify that experiments are producing different results
with different gain combinations.
"""

from pathlib import Path
import pandas as pd
import json

results_dir = Path("results/tuning")

print("=" * 80)
print("PID TUNING EXPERIMENT DIAGNOSTICS")
print("=" * 80)

# List all experiments
exp_dirs = sorted([d for d in results_dir.iterdir() if d.is_dir()])
print(f"\nFound {len(exp_dirs)} experiments:")
for exp_dir in exp_dirs:
    print(f"  - {exp_dir.name}")

# Load and compare each experiment
experiments = {}
for exp_dir in exp_dirs:
    meta_file = exp_dir / "metadata.json"
    data_file = exp_dir / "data.csv"
    
    if meta_file.exists() and data_file.exists():
        with open(meta_file) as f:
            metadata = json.load(f)
        
        df = pd.read_csv(data_file)
        
        # Extract key metrics
        shoulder_pan_error = df["e_pos_shoulder_pan"].abs().mean()
        shoulder_pan_error_max = df["e_pos_shoulder_pan"].abs().max()
        shoulder_pan_tau = df["tau_total_shoulder_pan"].abs().mean()
        
        experiments[exp_dir.name] = {
            "combo_id": metadata.get("combo_id"),
            "family": metadata.get("family"),
            "name": metadata.get("name"),
            "kp_base": metadata["gains"]["shoulder_pan"]["kp"],
            "ki_base": metadata["gains"]["shoulder_pan"]["ki"],
            "kd_base": metadata["gains"]["shoulder_pan"]["kd"],
            "e_pos_mean": shoulder_pan_error,
            "e_pos_max": shoulder_pan_error_max,
            "tau_mean": shoulder_pan_tau,
            "num_samples": len(df),
        }

print("\n" + "=" * 80)
print("EXPERIMENT COMPARISON - shoulder_pan joint")
print("=" * 80)
print(f"{'ID':<10} {'Kp':<8} {'Ki':<8} {'Kd':<8} {'E_pos (mean)':<15} {'E_pos (max)':<15} {'Tau (mean)':<12}")
print("-" * 80)

for combo_id in sorted(experiments.keys()):
    exp = experiments[combo_id]
    print(f"{combo_id:<10} {exp['kp_base']:<8.1f} {exp['ki_base']:<8.2f} {exp['kd_base']:<8.2f} "
          f"{exp['e_pos_mean']:<15.6f} {exp['e_pos_max']:<15.6f} {exp['tau_mean']:<12.4f}")

# Check for duplicates
print("\n" + "=" * 80)
print("CHECKING FOR DUPLICATE RESULTS")
print("=" * 80)

e_pos_values = [exp["e_pos_mean"] for exp in experiments.values()]
if len(set([f"{v:.8f}" for v in e_pos_values])) == 1:
    print("⚠️  WARNING: All experiments have IDENTICAL position errors!")
    print("   This suggests all experiments are using the same gains or producing identical results.")
    print("   Possible causes:")
    print("   1. All experiments ran with the same model state (not reset between runs)")
    print("   2. Gains are not being applied correctly")
    print("   3. The robot doesn't move (torque limits too low)")
else:
    print("✓ Position errors are DIFFERENT across experiments (good sign)")
    print(f"   Range: {min(e_pos_values):.6f} to {max(e_pos_values):.6f} rad")

tau_values = [exp["tau_mean"] for exp in experiments.values()]
if len(set([f"{v:.4f}" for v in tau_values])) == 1:
    print("\n⚠️  WARNING: All experiments have IDENTICAL torque magnitudes!")
    print("   This suggests gains are not changing between experiments.")
else:
    print("\n✓ Torque magnitudes are DIFFERENT across experiments (good sign)")
    print(f"   Range: {min(tau_values):.4f} to {max(tau_values):.4f} N·m")

print("\n" + "=" * 80)
print("SAMPLE DATA CHECK")
print("=" * 80)

# Check one experiment
if exp_dirs:
    sample_exp = exp_dirs[0]
    print(f"\nSample experiment: {sample_exp.name}")
    df = pd.read_csv(sample_exp / "data.csv")
    print(f"Columns: {list(df.columns[:10])}")
    print(f"Rows: {len(df)}")
    print(f"\nFirst 5 rows (shoulder_pan):")
    print(df[["t", "phase", "q_des_shoulder_pan", "q_meas_shoulder_pan", "e_pos_shoulder_pan", 
              "tau_pid_shoulder_pan", "tau_dist_shoulder_pan", "tau_total_shoulder_pan"]].head())

print("\n" + "=" * 80)
