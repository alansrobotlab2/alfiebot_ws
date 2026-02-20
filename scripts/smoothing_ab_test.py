#!/usr/bin/env python3
"""A/B test smoothing configs across multiple episodes.

Runs groot_replay_eval.py with different pipeline configs serially,
then compares key metrics in a summary table. Averages across episodes
to account for model stochasticity (flow matching denoising variance).

Usage:
    python scripts/smoothing_ab_test.py --host 192.168.50.201
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

BASE_CMD = [
    sys.executable,
    "src/alfie_gr00t/alfie_gr00t/scripts/groot_replay_eval.py",
    "--config", "src/alfie_gr00t/config/groot_client.yaml",
    "--dataset-path", "data/alfiebot.CanDoChallenge",
    "--continuous-inference", "true",
    "--smoothing-strategy", "exp_decay",
    "--no-plots",
]

# Configs to test: (name, extra_args)
CONFIGS = [
    ("baseline_a09", {
        "interpolation_method": "linear",
        "smoothing_method": "none",
        "joint_smoothing_alpha": "0.9",
    }),
    ("spline_a09", {
        "interpolation_method": "cubic_spline",
        "spline_window": "6",
        "smoothing_method": "none",
        "joint_smoothing_alpha": "0.9",
    }),
    ("spline_savgol_a09", {
        "interpolation_method": "cubic_spline",
        "spline_window": "6",
        "smoothing_method": "savgol",
        "savgol_window": "5",
        "savgol_polyorder": "2",
        "joint_smoothing_alpha": "0.9",
    }),
    ("spline_savgol_a07", {
        "interpolation_method": "cubic_spline",
        "spline_window": "6",
        "smoothing_method": "savgol",
        "savgol_window": "5",
        "savgol_polyorder": "2",
        "joint_smoothing_alpha": "0.7",
    }),
    ("spline_savgol_a05", {
        "interpolation_method": "cubic_spline",
        "spline_window": "6",
        "smoothing_method": "savgol",
        "savgol_window": "5",
        "savgol_polyorder": "2",
        "joint_smoothing_alpha": "0.5",
    }),
]

EPISODES = [0, 1, 2, 3, 4]


def run_config(name, extra_args, episodes, host, output_base):
    """Run one config across all episodes."""
    output_dir = Path(output_base) / name
    episode_str = ",".join(str(e) for e in episodes)

    cmd = list(BASE_CMD) + [
        "--episode-indices", episode_str,
        "--host", host,
        "--output-dir", str(output_dir),
    ]
    for key, val in extra_args.items():
        cmd.extend([f"--{key.replace('_', '-')}", val])

    print(f"\n{'='*60}")
    print(f"  RUNNING: {name}")
    print(f"  Episodes: {episodes}")
    print(f"  Output: {output_dir}")
    print(f"{'='*60}\n")

    result = subprocess.run(cmd, capture_output=False, text=True)
    if result.returncode != 0:
        print(f"  WARNING: {name} exited with code {result.returncode}")
        return None

    summary_path = output_dir / "summary.json"
    if not summary_path.exists():
        print(f"  WARNING: No summary.json for {name}")
        return None

    with open(summary_path) as f:
        return json.load(f)


def extract_metrics(summary):
    """Extract key metrics from a summary dict."""
    agg = summary.get("aggregate", {})
    episodes = summary.get("episodes", {})

    # Per-group smoothness (average across episodes)
    head_proc_ratios = []
    head_raw_ratios = []
    rarm_proc_ratios = []
    rarm_raw_ratios = []
    proc_hf = []
    raw_hf = []

    for ep_data in episodes.values():
        sm = ep_data.get("smoothness", {})
        pg = sm.get("per_group", {})

        if "head" in pg:
            head_proc_ratios.append(pg["head"]["processed_jerk_ratio"])
            head_raw_ratios.append(pg["head"]["raw_jerk_ratio"])
        if "right_arm" in pg:
            rarm_proc_ratios.append(pg["right_arm"]["processed_jerk_ratio"])
            rarm_raw_ratios.append(pg["right_arm"]["raw_jerk_ratio"])

        proc_hf.append(sm.get("processed_hf_power_ratio", 0))
        raw_hf.append(sm.get("raw_hf_power_ratio", 0))

    return {
        "raw_mse": agg.get("raw_mse_mean", 0),
        "proc_mse": agg.get("processed_mse_mean", 0),
        "raw_mae": agg.get("raw_mae_mean", 0),
        "proc_mae": agg.get("processed_mae_mean", 0),
        "raw_jerk": agg.get("raw_jerk_rms_mean", 0),
        "proc_jerk": agg.get("processed_jerk_rms_mean", 0),
        "gt_jerk": agg.get("gt_jerk_rms_mean", 0),
        "raw_jerk_ratio": agg.get("raw_jerk_ratio_mean", 0),
        "proc_jerk_ratio": agg.get("processed_jerk_ratio_mean", 0),
        "raw_hf": agg.get("raw_hf_power_ratio_mean", 0),
        "proc_hf": agg.get("processed_hf_power_ratio_mean", 0),
        "head_proc_ratio": float(np.mean(head_proc_ratios)) if head_proc_ratios else 0,
        "head_raw_ratio": float(np.mean(head_raw_ratios)) if head_raw_ratios else 0,
        "rarm_proc_ratio": float(np.mean(rarm_proc_ratios)) if rarm_proc_ratios else 0,
        "rarm_raw_ratio": float(np.mean(rarm_raw_ratios)) if rarm_raw_ratios else 0,
        "proc_hf_mean": float(np.mean(proc_hf)) if proc_hf else 0,
        "raw_hf_mean": float(np.mean(raw_hf)) if raw_hf else 0,
        "pipeline_delta": agg.get("pipeline_delta_mean", 0),
    }


def print_comparison(results):
    """Print comparison table."""
    print("\n")
    print("=" * 120)
    print("  SMOOTHING A/B TEST — MULTI-EPISODE COMPARISON")
    print("=" * 120)

    # Header
    names = list(results.keys())
    header = f"  {'Metric':<25}"
    for name in names:
        header += f"  {name:>18}"
    print(header)
    print(f"  {'-'*25}" + f"  {'-'*18}" * len(names))

    rows = [
        ("Proc MAE", "proc_mae", ".6f"),
        ("Proc MSE", "proc_mse", ".6f"),
        ("Raw Jerk RMS", "raw_jerk", ".4f"),
        ("Proc Jerk RMS", "proc_jerk", ".4f"),
        ("Proc Jerk Ratio (xGT)", "proc_jerk_ratio", ".3f"),
        ("Head Proc Ratio (xGT)", "head_proc_ratio", ".3f"),
        ("R.Arm Proc Ratio (xGT)", "rarm_proc_ratio", ".3f"),
        ("Proc HF Power %", "proc_hf_mean", ".1%"),
        ("Raw HF Power %", "raw_hf_mean", ".1%"),
        ("Pipeline Delta", "pipeline_delta", ".6f"),
    ]

    for label, key, fmt in rows:
        line = f"  {label:<25}"
        vals = [results[n][key] for n in names]
        best = min(vals)
        for v in vals:
            marker = " *" if v == best and len(set(f"{x:{fmt}}" for x in vals)) > 1 else "  "
            line += f"  {v:>{16}{fmt}}{marker}"
        print(line)

    print(f"  {'-'*25}" + f"  {'-'*18}" * len(names))
    print("  * = best in row")
    print("=" * 120)

    # Interpretation
    print("\n  KEY:")
    print("    Jerk Ratio <1.0 = smoother than GT, >1.0 = jerkier than GT")
    print("    HF Power % = fraction of energy above 3Hz (lower = less oscillation)")
    print("    Pipeline Delta = mean |processed - raw| (how much the pipeline modifies)")
    print()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.50.201")
    parser.add_argument("--output-base", default="/tmp/groot_smoothing_ab")
    parser.add_argument("--episodes", default="0,1,2,3,4",
                        help="Comma-separated episode indices")
    args = parser.parse_args()

    episodes = [int(x) for x in args.episodes.split(",")]
    output_base = Path(args.output_base)
    output_base.mkdir(parents=True, exist_ok=True)

    all_results = {}
    for name, extra_args in CONFIGS:
        summary = run_config(name, extra_args, episodes, args.host, output_base)
        if summary:
            all_results[name] = extract_metrics(summary)

    if all_results:
        print_comparison(all_results)

        # Save raw results
        with open(output_base / "comparison.json", "w") as f:
            json.dump(all_results, f, indent=2)
        print(f"  Raw results saved to: {output_base}/comparison.json")
    else:
        print("  No successful runs to compare!")


if __name__ == "__main__":
    main()
