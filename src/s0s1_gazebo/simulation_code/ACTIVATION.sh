#!/bin/bash
# Quick activation script for PID tuning experiments

source venv/bin/activate

# Show available commands
echo "Virtual environment activated!"
echo ""
echo "Quick commands:"
echo "  python run_pid_tuning_experiments.py --show-config           # View all 23 combinations"
echo "  python run_pid_tuning_experiments.py --families P --skip-viewer  # Run all P (5 exp)"
echo "  python run_pid_tuning_experiments.py --skip-viewer           # Run all 23 experiments"
echo "  python run_pid_tuning_experiments.py --analyze-only          # Analyze existing results"
echo ""
