#!/usr/bin/env python3
"""Systematic parameter sweep for smoothness optimization.

Runs groot_replay_eval.py with varying configurations, captures metrics,
and outputs a summary table for SMOOTHIT.md.

Usage:
    python3 smoothing_sweep.py --server-host 192.168.50.201
    python3 smoothing_sweep.py --server-host 192.168.50.201 --phases A,B
    python3 smoothing_sweep.py --server-host 192.168.50.201 --episodes 0,1,2
"""

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

# Paths relative to workspace root
WORKSPACE = Path(__file__).resolve().parents[4]  # alfiebot_ws/
REPLAY_EVAL = WORKSPACE / 'src/alfie_gr00t/alfie_gr00t/scripts/groot_replay_eval.py'
CONFIG_YAML = WORKSPACE / 'src/alfie_gr00t/config/groot_client.yaml'
DATASET_PATH = WORKSPACE / 'data/alfiebot.CanDoChallenge'
RESULTS_DIR = WORKSPACE / 'smoothing_sweep_results'

# Focus joints for per-joint analysis
FOCUS_JOINTS = ['cmd_vel_lx', 'right_shoulder_pitch', 'right_elbow_pitch', 'head_yaw']

# Body part groups matching groot_replay_eval.py
BODY_PART_GROUPS = [
    'base', 'back', 'left_arm', 'left_hand', 'right_arm', 'right_hand', 'head',
]


def define_experiments():
    """Define all experiment configurations.

    Each config is a dict:
        name: short identifier
        phase: A/B/C/D/E
        desc: one-line description
        overrides: dict of CLI flag -> value (flag names without --)
        rtc: bool (add --rtc flag)
    """
    experiments = []

    # =========================================================================
    # Phase A — Baselines
    # =========================================================================
    experiments.append({
        'name': 'A1_bare_minimum',
        'phase': 'A',
        'desc': 'No smoothing (NVIDIA SO100-like)',
        'overrides': {
            'continuous-inference': 'false',
            'rate-limit-enabled': 'false',
            'smoothing-method': 'none',
            'deadband-left-arm': 0.0,
            'deadband-right-arm': 0.0,
            'deadband-left-gripper': 0.0,
            'deadband-right-gripper': 0.0,
            'deadband-head': 0.0,
            'joint-smoothing-alpha': 1.0,
        },
        'rtc': True,
    })
    experiments.append({
        'name': 'A2_rate_limit_only',
        'phase': 'A',
        'desc': 'Rate limiting only',
        'overrides': {
            'continuous-inference': 'false',
            'smoothing-method': 'none',
            'deadband-left-arm': 0.0,
            'deadband-right-arm': 0.0,
            'deadband-left-gripper': 0.0,
            'deadband-right-gripper': 0.0,
            'deadband-head': 0.0,
            'joint-smoothing-alpha': 1.0,
        },
        'rtc': True,
    })
    experiments.append({
        'name': 'A3_ratelimit_deadband',
        'phase': 'A',
        'desc': 'Rate limit + deadband (YAML values)',
        'overrides': {
            'continuous-inference': 'false',
            'smoothing-method': 'none',
            'joint-smoothing-alpha': 1.0,
        },
        'rtc': True,
    })
    experiments.append({
        'name': 'A4_classic_full',
        'phase': 'A',
        'desc': 'Classic mode, full YAML settings',
        'overrides': {
            'continuous-inference': 'false',
        },
        'rtc': True,
    })

    # =========================================================================
    # Phase B — Temporal Ensembling Strategy
    # =========================================================================
    experiments.append({
        'name': 'B1_ensemble_latest',
        'phase': 'B',
        'desc': 'Continuous, latest only (no ensembling)',
        'overrides': {
            'smoothing-strategy': 'latest',
        },
        'rtc': True,
    })
    experiments.append({
        'name': 'B2_exp_decay_0.01',
        'phase': 'B',
        'desc': 'exp_decay m=0.01 (current default)',
        'overrides': {
            'smoothing-strategy': 'exp_decay',
            'smoothing-decay-m': 0.01,
        },
        'rtc': True,
    })
    experiments.append({
        'name': 'B3_exp_decay_0.001',
        'phase': 'B',
        'desc': 'exp_decay m=0.001 (more averaging)',
        'overrides': {
            'smoothing-strategy': 'exp_decay',
            'smoothing-decay-m': 0.001,
        },
        'rtc': True,
    })
    experiments.append({
        'name': 'B4_exp_decay_0.1',
        'phase': 'B',
        'desc': 'exp_decay m=0.1 (strong recency)',
        'overrides': {
            'smoothing-strategy': 'exp_decay',
            'smoothing-decay-m': 0.1,
        },
        'rtc': True,
    })
    experiments.append({
        'name': 'B5_ensemble_uniform',
        'phase': 'B',
        'desc': 'Uniform weighting',
        'overrides': {
            'smoothing-strategy': 'uniform',
        },
        'rtc': True,
    })

    # =========================================================================
    # Phase C — EMA Alpha (with exp_decay m=0.01)
    # =========================================================================
    experiments.append({
        'name': 'C1_alpha_1.0',
        'phase': 'C',
        'desc': 'EMA off (alpha=1.0)',
        'overrides': {
            'joint-smoothing-alpha': 1.0,
        },
        'rtc': True,
    })
    experiments.append({
        'name': 'C2_alpha_0.7',
        'phase': 'C',
        'desc': 'Light EMA (alpha=0.7)',
        'overrides': {
            'joint-smoothing-alpha': 0.7,
        },
        'rtc': True,
    })
    experiments.append({
        'name': 'C3_alpha_0.5',
        'phase': 'C',
        'desc': 'Current EMA (alpha=0.5)',
        'overrides': {
            'joint-smoothing-alpha': 0.5,
        },
        'rtc': True,
    })
    experiments.append({
        'name': 'C4_alpha_0.3',
        'phase': 'C',
        'desc': 'Heavy EMA (alpha=0.3)',
        'overrides': {
            'joint-smoothing-alpha': 0.3,
        },
        'rtc': True,
    })

    # =========================================================================
    # Phase D — Post-Ensemble Filters
    # =========================================================================
    experiments.append({
        'name': 'D1_no_filter',
        'phase': 'D',
        'desc': 'No post-filter (ensemble+EMA only)',
        'overrides': {
            'smoothing-method': 'none',
        },
        'rtc': True,
    })
    experiments.append({
        'name': 'D2_savgol_w3',
        'phase': 'D',
        'desc': 'SavGol window=3',
        'overrides': {
            'smoothing-method': 'savgol',
            'savgol-window': 3,
        },
        'rtc': True,
    })
    experiments.append({
        'name': 'D3_savgol_w5',
        'phase': 'D',
        'desc': 'SavGol window=5 (current)',
        'overrides': {
            'smoothing-method': 'savgol',
            'savgol-window': 5,
        },
        'rtc': True,
    })
    experiments.append({
        'name': 'D4_savgol_w7',
        'phase': 'D',
        'desc': 'SavGol window=7',
        'overrides': {
            'smoothing-method': 'savgol',
            'savgol-window': 7,
        },
        'rtc': True,
    })
    experiments.append({
        'name': 'D5_butter_7hz',
        'phase': 'D',
        'desc': 'Butterworth 7Hz cutoff',
        'overrides': {
            'smoothing-method': 'butterworth',
            'butterworth-cutoff-hz': 7.0,
        },
        'rtc': True,
    })
    experiments.append({
        'name': 'D6_butter_5hz',
        'phase': 'D',
        'desc': 'Butterworth 5Hz cutoff',
        'overrides': {
            'smoothing-method': 'butterworth',
            'butterworth-cutoff-hz': 5.0,
        },
        'rtc': True,
    })
    experiments.append({
        'name': 'D7_butter_3hz',
        'phase': 'D',
        'desc': 'Butterworth 3Hz cutoff',
        'overrides': {
            'smoothing-method': 'butterworth',
            'butterworth-cutoff-hz': 3.0,
        },
        'rtc': True,
    })

    # =========================================================================
    # Phase E — RTC comparison + combined best
    # Will be populated dynamically after phases A-D
    # =========================================================================

    return experiments


def build_command(
    exp: dict,
    episodes: str,
    server_host: str,
    server_port: int,
    output_dir: Path,
) -> list[str]:
    """Build the subprocess command for a single experiment."""
    cmd = [
        sys.executable, str(REPLAY_EVAL),
        '--config', str(CONFIG_YAML),
        '--dataset-path', str(DATASET_PATH),
        '--episode-indices', episodes,
        '--host', server_host,
        '--port', str(server_port),
        '--output-dir', str(output_dir / exp['name']),
        '--no-plots',
    ]
    if exp.get('rtc', False):
        cmd.append('--rtc')

    for key, val in exp.get('overrides', {}).items():
        cmd.append(f'--{key}')
        cmd.append(str(val))

    return cmd


def parse_metrics(stdout: str) -> dict:
    """Parse key metrics from groot_replay_eval stdout."""
    result = {
        'mse_overall': None,
        'jerk_ratio_overall': None,
        'hf_power_overall': None,
        'inference_count': None,
        'per_group': {},
        'per_joint_mse': {},
    }

    # Overall MSE — look for PROCESSED MSE line
    m = re.search(r'PROCESSED\s+MSE:\s+([\d.]+)', stdout)
    if m:
        result['mse_overall'] = float(m.group(1))

    # Aggregate MSE (multi-episode)
    m = re.search(r'Accuracy \(MSE\):\s+([\d.]+)', stdout)
    if m:
        result['mse_overall'] = float(m.group(1))

    # Inference count
    m = re.search(r'Inferences:\s+(\d+)', stdout)
    if m:
        result['inference_count'] = int(m.group(1))

    # Overall jerk ratio (processed)
    # Pattern: "Processed    0.1234     1.23x    ..."
    m = re.search(r'Processed\s+[\d.]+\s+([\d.]+)x', stdout)
    if m:
        result['jerk_ratio_overall'] = float(m.group(1))

    # Aggregate jerk ratio
    m = re.search(r'Processed:\s+[\d.]+\s+\(([\d.]+)x GT\)', stdout)
    if m:
        result['jerk_ratio_overall'] = float(m.group(1))

    # Overall HF power (processed)
    # Pattern after jerk line: "Processed    0.1234     1.23x     0.001234     12.3%"
    m = re.search(r'Processed\s+[\d.]+\s+[\d.]+x\s+[\d.]+\s+([\d.]+)%', stdout)
    if m:
        result['hf_power_overall'] = float(m.group(1))

    # Per-group smoothness
    # Pattern: "right_arm     0.0100     0.0200     0.0150      2.00x      1.50x     1.5%     1.2%"
    group_pattern = re.compile(
        r'^\s*(\w+)\s+'        # group name
        r'([\d.]+)\s+'         # gt jerk
        r'([\d.]+)\s+'         # raw jerk
        r'([\d.]+)\s+'         # proc jerk
        r'([\d.]+)x\s+'       # raw ratio
        r'([\d.]+)x\s+'       # proc ratio
        r'([\d.]+)%\s+'       # raw HF%
        r'([\d.]+)%',         # proc HF%
        re.MULTILINE,
    )
    for m in group_pattern.finditer(stdout):
        name = m.group(1)
        result['per_group'][name] = {
            'gt_jerk': float(m.group(2)),
            'raw_jerk': float(m.group(3)),
            'proc_jerk': float(m.group(4)),
            'raw_jerk_ratio': float(m.group(5)),
            'proc_jerk_ratio': float(m.group(6)),
            'raw_hf_pct': float(m.group(7)),
            'proc_hf_pct': float(m.group(8)),
        }

    # Per-group MSE
    # Pattern: "right_arm     0.001234     0.001234  ..."
    mse_pattern = re.compile(
        r'^\s*(\w+)\s+'        # group name
        r'([\d.]+)\s+'         # raw MSE
        r'([\d.]+)\s+'         # proc MSE
        r'([\d.]+)\s+'         # raw MAE
        r'([\d.]+)\s+'         # proc MAE
        r'([\d.]+)',           # delta
        re.MULTILINE,
    )
    for m in mse_pattern.finditer(stdout):
        name = m.group(1)
        if name in BODY_PART_GROUPS:
            result['per_joint_mse'][name] = {
                'raw_mse': float(m.group(2)),
                'proc_mse': float(m.group(3)),
            }

    return result


def run_experiment(
    exp: dict,
    episodes: str,
    server_host: str,
    server_port: int,
    output_dir: Path,
    timeout: int = 600,
) -> dict:
    """Run a single experiment and return parsed metrics."""
    cmd = build_command(exp, episodes, server_host, server_port, output_dir)

    print(f"\n{'='*70}")
    print(f"  Running: {exp['name']} — {exp['desc']}")
    print(f"  Command: {' '.join(cmd[-10:])}")
    print(f"{'='*70}\n")

    t0 = time.time()
    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            cwd=str(WORKSPACE),
        )
        elapsed = time.time() - t0

        # Save raw output
        out_path = output_dir / exp['name']
        out_path.mkdir(parents=True, exist_ok=True)
        (out_path / 'stdout.txt').write_text(proc.stdout)
        (out_path / 'stderr.txt').write_text(proc.stderr)

        if proc.returncode != 0:
            print(f"  ERROR: returncode={proc.returncode}")
            print(f"  stderr (last 500 chars): {proc.stderr[-500:]}")
            return {
                'name': exp['name'],
                'phase': exp['phase'],
                'desc': exp['desc'],
                'status': 'error',
                'elapsed_s': elapsed,
                'error': proc.stderr[-500:],
            }

        metrics = parse_metrics(proc.stdout)
        metrics['name'] = exp['name']
        metrics['phase'] = exp['phase']
        metrics['desc'] = exp['desc']
        metrics['status'] = 'ok'
        metrics['elapsed_s'] = elapsed

        # Print summary
        jerk = metrics.get('jerk_ratio_overall', '?')
        mse = metrics.get('mse_overall', '?')
        print(f"  Result: MSE={mse}, Jerk={jerk}x, Time={elapsed:.0f}s")

        # Print focus group jerk ratios
        for group in ['base', 'right_arm', 'head']:
            if group in metrics.get('per_group', {}):
                g = metrics['per_group'][group]
                print(f"    {group}: jerk={g['proc_jerk_ratio']:.2f}x, HF={g['proc_hf_pct']:.1f}%")

        return metrics

    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        print(f"  TIMEOUT after {timeout}s")
        return {
            'name': exp['name'],
            'phase': exp['phase'],
            'desc': exp['desc'],
            'status': 'timeout',
            'elapsed_s': elapsed,
        }
    except Exception as e:
        elapsed = time.time() - t0
        print(f"  EXCEPTION: {e}")
        return {
            'name': exp['name'],
            'phase': exp['phase'],
            'desc': exp['desc'],
            'status': 'exception',
            'elapsed_s': elapsed,
            'error': str(e),
        }


def format_markdown_table(results: list[dict]) -> str:
    """Format results as a markdown table for SMOOTHIT.md."""
    lines = []

    # Summary table
    lines.append('| # | Config | MSE | Jerk | Base Jerk | R.Arm Jerk | Head Jerk | Base HF% | R.Arm HF% | Head HF% |')
    lines.append('|---|--------|-----|------|-----------|------------|-----------|----------|-----------|----------|')

    for r in results:
        if r.get('status') != 'ok':
            lines.append(f"| {r['name']} | {r['desc']} | ERROR | — | — | — | — | — | — | — |")
            continue

        mse = r.get('mse_overall', 0)
        jerk = r.get('jerk_ratio_overall', 0)

        base_j = r.get('per_group', {}).get('base', {}).get('proc_jerk_ratio', 0)
        rarm_j = r.get('per_group', {}).get('right_arm', {}).get('proc_jerk_ratio', 0)
        head_j = r.get('per_group', {}).get('head', {}).get('proc_jerk_ratio', 0)

        base_hf = r.get('per_group', {}).get('base', {}).get('proc_hf_pct', 0)
        rarm_hf = r.get('per_group', {}).get('right_arm', {}).get('proc_hf_pct', 0)
        head_hf = r.get('per_group', {}).get('head', {}).get('proc_hf_pct', 0)

        lines.append(
            f"| {r['name']} | {r['desc']} | {mse:.4f} | {jerk:.2f}x | "
            f"{base_j:.2f}x | {rarm_j:.2f}x | {head_j:.2f}x | "
            f"{base_hf:.1f}% | {rarm_hf:.1f}% | {head_hf:.1f}% |"
        )

    return '\n'.join(lines)


def find_best_config(results: list[dict], metric: str = 'jerk_ratio_overall') -> dict:
    """Find the config with the lowest value for the given metric."""
    valid = [r for r in results if r.get('status') == 'ok' and r.get(metric) is not None]
    if not valid:
        return {}
    return min(valid, key=lambda r: r[metric])


def main():
    parser = argparse.ArgumentParser(description='Smoothing parameter sweep')
    parser.add_argument('--server-host', type=str, default='192.168.50.201')
    parser.add_argument('--server-port', type=int, default=5555)
    parser.add_argument('--episodes', type=str, default='0,1,2',
                        help='Comma-separated episode indices')
    parser.add_argument('--phases', type=str, default='A,B,C,D',
                        help='Comma-separated phases to run (A,B,C,D,E)')
    parser.add_argument('--output-dir', type=str, default=str(RESULTS_DIR))
    parser.add_argument('--timeout', type=int, default=600,
                        help='Per-experiment timeout in seconds')
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    phases = set(args.phases.upper().split(','))
    experiments = define_experiments()

    # Filter to requested phases
    experiments = [e for e in experiments if e['phase'] in phases]

    print(f"\n{'#'*70}")
    print(f"  SMOOTHING PARAMETER SWEEP")
    print(f"  Phases: {sorted(phases)}")
    print(f"  Experiments: {len(experiments)}")
    print(f"  Episodes: {args.episodes}")
    print(f"  Server: {args.server_host}:{args.server_port}")
    print(f"  Output: {output_dir}")
    print(f"{'#'*70}\n")

    all_results = []
    for i, exp in enumerate(experiments):
        print(f"\n[{i+1}/{len(experiments)}]", end='')
        result = run_experiment(
            exp,
            episodes=args.episodes,
            server_host=args.server_host,
            server_port=args.server_port,
            output_dir=output_dir,
            timeout=args.timeout,
        )
        all_results.append(result)

        # Save intermediate results after each run
        with open(output_dir / 'results.json', 'w') as f:
            json.dump(all_results, f, indent=2, default=str)

    # Generate markdown summary
    md_table = format_markdown_table(all_results)
    (output_dir / 'summary.md').write_text(md_table)

    # Print summary
    print(f"\n\n{'#'*70}")
    print(f"  SWEEP COMPLETE — {len(all_results)} experiments")
    print(f"{'#'*70}\n")
    print(md_table)

    # Find best configs
    ok_results = [r for r in all_results if r.get('status') == 'ok']
    if ok_results:
        best_jerk = find_best_config(ok_results, 'jerk_ratio_overall')
        print(f"\n  Best overall jerk: {best_jerk.get('name')} ({best_jerk.get('jerk_ratio_overall', '?')}x)")

        # Best per-group
        for group in ['base', 'right_arm', 'head']:
            valid = [r for r in ok_results if group in r.get('per_group', {})]
            if valid:
                best = min(valid, key=lambda r: r['per_group'][group].get('proc_jerk_ratio', 999))
                print(f"  Best {group} jerk: {best['name']} ({best['per_group'][group]['proc_jerk_ratio']:.2f}x)")

    print(f"\n  Results: {output_dir / 'results.json'}")
    print(f"  Summary: {output_dir / 'summary.md'}")


if __name__ == '__main__':
    main()
