#!/usr/bin/env python3
# run_pid_tuning_experiments.py
"""
Systematic PID Tuning Experiment Runner

This script executes all configured experiments (P, PD, PI, PID families with multiple gain combinations).
Results are saved to results/ directory with complete data, plots, and analysis tables.

Usage:
    python run_pid_tuning_experiments.py --help
    python run_pid_tuning_experiments.py --families P PD  # Run P and PD only
    python run_pid_tuning_experiments.py --combos P-1 P-2  # Run specific combinations
    python run_pid_tuning_experiments.py --skip-viewer  # Run without MuJoCo viewer
    python run_pid_tuning_experiments.py --analyze-only  # Load and analyze existing results
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Optional
import time

from so101_control import PerturbationConfig
from pid_tuning_config import get_all_combo_ids, get_tuning_by_id, print_tuning_summary
from pid_tuning_experiment import ExperimentRunner, ExperimentDataset
from pid_analysis import compute_experiment_metrics, generate_qualitative_table, plot_experiment, plot_family_comparison


# ============================================================================
# Configuration
# ============================================================================

DEFAULT_RESULTS_DIR = Path("results/tuning")
DEFAULT_MODEL_PATH = "model/scene_urdf.xml"

# Experiment timing
MOVE_DURATION = 2.0      # Seconds for move_to_zero and return_start phases
HOLD_DURATION = 2.0      # Seconds for hold_zero and hold_start phases

# Perturbation settings (strong, consistent across all experiments)
DEFAULT_PERTURB_CONFIG = PerturbationConfig(
    sinus_amp=0.8,
    sinus_freq_hz=0.5,
    noise_std=0.25,
    noise_tau=0.25,
    impulse_prob_per_s=0.12,
    impulse_mag=2.0,
    impulse_dur=0.05,
    meas_q_std=0.0,
    meas_qd_std=0.0,
    seed=7,
)


# ============================================================================
# Experiment Execution
# ============================================================================

def run_single_experiment(
    combo_id: str,
    results_dir: Path,
    model_path: str,
    viewer: bool = True,
    realtime: bool = False,
) -> bool:
    """
    Run a single experiment and save results.
    
    Returns:
      True if successful, False otherwise.
    """
    print(f"\n{'='*60}")
    print(f"Running experiment: {combo_id}")
    print(f"{'='*60}")
    
    # Get tuning configuration
    tuning = get_tuning_by_id(combo_id)
    if tuning is None:
        print(f"ERROR: Combination '{combo_id}' not found!")
        return False
    
    print(f"Family: {tuning.family}")
    print(f"Name: {tuning.name}")
    print(f"Philosophy: {tuning.philosophy}")
    
    try:
        # Create experiment runner
        runner = ExperimentRunner(
            tuning=tuning,
            model_path=model_path,
            perturb_config=DEFAULT_PERTURB_CONFIG,
            viewer=viewer,
            realtime=realtime,
        )
        
        # Run the full trajectory
        print(f"Executing trajectory (move_dur={MOVE_DURATION}s, hold_dur={HOLD_DURATION}s)...")
        start = time.time()
        result = runner.run(move_duration=MOVE_DURATION, hold_duration=HOLD_DURATION)
        elapsed = time.time() - start
        
        print(f"Experiment completed in {elapsed:.1f}s")
        
        # Save results
        exp_dir = results_dir / combo_id
        print(f"Saving to {exp_dir}...")
        runner.logger.save(exp_dir)
        
        # Compute metrics for quick feedback
        metrics = compute_experiment_metrics(result)
        print(f"Overall error: {metrics.overall_error_rad:.4f} rad")
        print(f"Overall effort: {metrics.overall_effort_Nm:.4f} N⋅m")
        print(f"Disturbance rejection: {metrics.disturbance_rejection:.3f}")
        
        return True
        
    except Exception as e:
        print(f"ERROR during experiment: {e}")
        import traceback
        traceback.print_exc()
        return False


def run_experiments_batch(
    combo_ids: List[str],
    results_dir: Path,
    model_path: str,
    viewer: bool = True,
    realtime: bool = False,
    verbose: bool = True,
) -> dict:
    """
    Run a batch of experiments and return summary statistics.
    
    Returns:
      Dictionary with success counts and timings.
    """
    results_dir.mkdir(parents=True, exist_ok=True)
    
    stats = {
        "total": len(combo_ids),
        "successful": 0,
        "failed": 0,
        "skipped": 0,
        "total_time_s": 0.0,
        "failed_ids": [],
    }
    
    start_time = time.time()
    
    for idx, combo_id in enumerate(combo_ids):
        exp_dir = results_dir / combo_id
        
        # Skip if already completed
        if (exp_dir / "metadata.json").exists():
            print(f"\n[{idx+1}/{len(combo_ids)}] Skipping {combo_id} (already exists)")
            stats["skipped"] += 1
            continue
        
        print(f"\n[{idx+1}/{len(combo_ids)}]", end="")
        
        success = run_single_experiment(
            combo_id=combo_id,
            results_dir=results_dir,
            model_path=model_path,
            viewer=viewer,
            realtime=realtime,
        )
        
        if success:
            stats["successful"] += 1
        else:
            stats["failed"] += 1
            stats["failed_ids"].append(combo_id)
    
    stats["total_time_s"] = time.time() - start_time
    
    return stats


# ============================================================================
# Analysis and Reporting
# ============================================================================

def analyze_results(results_dir: Path, output_dir: Optional[Path] = None):
    """
    Load all experiment results and generate analysis report.
    """
    if output_dir is None:
        output_dir = results_dir.parent / "analysis"
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"\nLoading experiment results from {results_dir}...")
    dataset = ExperimentDataset(results_dir)
    dataset.load_all()
    
    loaded_combos = dataset.get_combo_ids()
    print(f"Loaded {len(loaded_combos)} experiments: {', '.join(loaded_combos)}")
    
    # Compute metrics
    print(f"\nComputing metrics...")
    all_metrics = {}
    for combo_id, result in dataset.results.items():
        all_metrics[combo_id] = compute_experiment_metrics(result)
    
    # Generate plots for each experiment
    print(f"Generating plots...")
    for combo_id, result in dataset.results.items():
        try:
            plot_experiment(result, output_dir=output_dir)
        except Exception as e:
            print(f"  Warning: Could not plot {combo_id}: {e}")
    
    # Group by family
    families = {}
    for combo_id, metrics in all_metrics.items():
        family = metrics.family
        if family not in families:
            families[family] = {}
        families[family][combo_id] = metrics
    
    # Generate qualitative tables
    print(f"Generating qualitative tables...")
    for family in sorted(families.keys()):
        print(f"\n{family} Controller Family:")
        print("-" * 100)
        
        table_df = generate_qualitative_table(families[family], family)
        table_df.to_csv(output_dir / f"{family}_qualitative_table.csv", index=False)
        print(table_df.to_string(index=False))
    
    # Generate comparison plots per family
    print(f"\nGenerating family comparison plots...")
    family_results = {}
    for combo_id, result in dataset.results.items():
        family = result.metadata.family
        if family not in family_results:
            family_results[family] = {}
        family_results[family][combo_id] = result
    
    for family in sorted(family_results.keys()):
        try:
            plot_family_comparison(family_results[family], family, output_dir=output_dir)
            print(f"  OK: {family} comparison plot saved")
        except Exception as e:
            print(f"  Warning: Could not plot {family} comparison: {e}")
    
    print(f"\n{'='*80}")
    print(f"Analysis complete! Results saved to {output_dir}")
    print(f"{'='*80}")


# ============================================================================
# CLI
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Systematic PID tuning experiment runner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python run_pid_tuning_experiments.py              # Run all experiments
  python run_pid_tuning_experiments.py --families P PD  # Run P and PD only
  python run_pid_tuning_experiments.py --combos P-1 P-2 P-3  # Run specific combos
  python run_pid_tuning_experiments.py --skip-viewer  # Run without viewer
  python run_pid_tuning_experiments.py --analyze-only  # Analyze existing results
"""
    )
    
    parser.add_argument(
        "--families",
        nargs="+",
        default=None,
        choices=["P", "PD", "PI", "PID"],
        help="Which controller families to run (default: all)",
    )
    
    parser.add_argument(
        "--combos",
        nargs="+",
        default=None,
        help="Specific combo IDs to run (e.g., P-1 P-2 PID-3)",
    )
    
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=DEFAULT_RESULTS_DIR,
        help="Directory to save experiment results",
    )
    
    parser.add_argument(
        "--model",
        type=str,
        default=DEFAULT_MODEL_PATH,
        help="Path to MuJoCo model XML",
    )
    
    parser.add_argument(
        "--skip-viewer",
        action="store_true",
        help="Run without MuJoCo viewer (headless mode)",
    )
    
    parser.add_argument(
        "--realtime",
        action="store_true",
        help="Run at simulated realtime speed (slower)",
    )
    
    parser.add_argument(
        "--analyze-only",
        action="store_true",
        help="Skip experiments, only analyze existing results",
    )
    
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for analysis output (default: results/analysis)",
    )
    
    parser.add_argument(
        "--show-config",
        action="store_true",
        help="Print tuning configuration summary and exit",
    )
    
    args = parser.parse_args()
    
    # Show configuration if requested
    if args.show_config:
        print_tuning_summary()
        return 0
    
    # Determine which combos to run
    if args.combos is not None:
        combo_ids = args.combos
    elif args.families is not None:
        combo_ids = get_all_combo_ids(args.families)
    else:
        combo_ids = get_all_combo_ids()  # All
    
    print(f"{'='*80}")
    print(f"SO101 PID Tuning Experiment Suite")
    print(f"{'='*80}")
    print(f"Combinations to run: {len(combo_ids)}")
    print(f"  {', '.join(combo_ids)}")
    
    # Run experiments
    if not args.analyze_only:
        print(f"\nStarting experiments...")
        
        stats = run_experiments_batch(
            combo_ids=combo_ids,
            results_dir=args.results_dir,
            model_path=args.model,
            viewer=not args.skip_viewer,
            realtime=args.realtime,
        )
        
        print(f"\n{'='*80}")
        print(f"Experiment Summary:")
        print(f"  Total: {stats['total']}")
        print(f"  Successful: {stats['successful']}")
        print(f"  Failed: {stats['failed']}")
        print(f"  Skipped: {stats['skipped']}")
        print(f"  Time: {stats['total_time_s']/60:.1f} minutes")
        if stats["failed_ids"]:
            print(f"  Failed IDs: {', '.join(stats['failed_ids'])}")
        print(f"{'='*80}")
    
    # Analyze results
    analyze_results(
        results_dir=args.results_dir,
        output_dir=args.output_dir,
    )
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
