#!/usr/bin/env python3
"""Compare live GR00T runs (e.g. the RTC robot A/B) on raw policy output.

Each run is the csv_log_path base the client was launched with, e.g.
/tmp/ab/B_rtc4_t1.csv. The client writes three files from it:

  <base>.csv               100 Hz: action/smoothed/state per tick (hold-and-wait)
  <base>_actions.csv       one row per action step: the RAW chunk action
                           executed at that step, before EMA/rate limiting
  <base>_chunks.jsonl      every chunk as returned by the server, with RTT

Metrics on the raw executed sequence, defined as in Isaac-GR00T
scripts/deployment/rtc_smoothness_eval.py so live numbers line up with the
offline ones:
  seam jump   mean |a(t) - a(t-1)| (averaged over joints) where a new chunk takes over
  step size   the same away from seams; seam ratio = seam / step (>1 = visible seams)
  jerk        RMS of the second difference of the executed actions
plus jerk / reference jerk, where the reference is the demonstrations' own
action sequence (--dataset), RTT from the chunk log, late-chunk % (RTT above the latency_skip
window), hold-and-wait %, and
right-arm tracking error.

Usage (Isaac-GR00T venv or any python with pandas + pyarrow):
    python rtc_ab_analysis.py \\
        A=/tmp/ab/A_off4_t1.csv,/tmp/ab/A_off4_t2.csv \\
        B=/tmp/ab/B_rtc4_t1.csv C=/tmp/ab/C_rtc2_t1.csv \\
        --dataset ~/alfiebot_ws/data/CleanTheRoom/lerobot
A label's runs are pooled (seams/steps summed over runs).
"""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np
import pandas as pd

JOINTS = [
    'cmd_vel_lx', 'cmd_vel_ly', 'cmd_vel_lz', 'cmd_vel_ax', 'cmd_vel_ay', 'cmd_vel_az',
    'back_joint',
    'left_shoulder_yaw', 'left_shoulder_pitch', 'left_elbow_pitch', 'left_wrist_pitch',
    'left_wrist_roll', 'left_gripper',
    'right_shoulder_yaw', 'right_shoulder_pitch', 'right_elbow_pitch', 'right_wrist_pitch',
    'right_wrist_roll', 'right_gripper',
    'head_yaw', 'head_pitch', 'head_roll',
]
POS = slice(6, 22)      # position joints (back, arms, grippers, head)
R_ARM = slice(13, 18)


def _base(path: str) -> Path:
    p = Path(path)
    return p.with_suffix('') if p.suffix == '.csv' else p


def load_run(base: Path):
    acts = pd.read_csv(f'{base}_actions.csv')
    # The client also writes a row when a chunk ARRIVES mid-step (same
    # abs_idx, new chunk_id); keep one row per executed action step.
    keep = acts['action_idx'].ne(acts['action_idx'].shift())
    acts = acts[keep].reset_index(drop=True)
    a = acts[[f'action_{j}' for j in JOINTS]].to_numpy(float)
    idx = acts['action_idx'].to_numpy()
    seams = np.flatnonzero(idx[1:] < idx[:-1]) + 1  # first step of a new chunk
    out = {'executed': a, 'seams': seams,
           'state': acts[[f'state_{j}' for j in JOINTS]].to_numpy(float),
           'effective_skip': acts['effective_skip'].value_counts().to_dict()}
    chunks = Path(f'{base}_chunks.jsonl')
    out['rtt'] = []
    out['rtc_frac'] = None
    if chunks.exists():
        recs = [json.loads(line) for line in chunks.open() if line.strip()]
        out['rtt'] = [r['rtt_ms'] for r in recs[1:]]  # first includes warmup
        out['rtc_frac'] = float(np.mean([r['rtc_prefix'] for r in recs[1:]])) if len(recs) > 1 else None
        out['horizon'] = len(recs[0]['actions']) if recs else None
    dbg = Path(f'{base}.csv')
    out['hold_pct'] = None
    if dbg.exists() and out.get('horizon'):
        d = pd.read_csv(dbg, usecols=['action_idx'])
        out['hold_pct'] = float((d['action_idx'] == out['horizon'] - 1).mean() * 100)
    return out


def metrics(executed, seams, dims=slice(None)):
    x = executed[:, dims]
    deltas = np.abs(np.diff(x, axis=0)).mean(axis=1)
    seam_idx = seams - 1
    seam_idx = seam_idx[(seam_idx >= 0) & (seam_idx < len(deltas))]
    mask = np.ones(len(deltas), bool)
    mask[seam_idx] = False
    second = np.diff(x, n=2, axis=0)
    seam = float(deltas[seam_idx].mean()) if len(seam_idx) else float('nan')
    step = float(deltas[mask].mean())
    return {'seam_jump': seam, 'step_size': step, 'seam_ratio': seam / step,
            'jerk_rms': float(np.sqrt((second ** 2).mean())), 'n_seams': int(len(seam_idx)),
            'n_steps': int(len(x))}


def reference_jerk(dataset: str, dims):
    """RMS second difference of the demonstrations' action sequences."""
    files = sorted(glob.glob(str(Path(dataset).expanduser() / 'data' / '*' / '*.parquet')))
    seconds = []
    for f in files:
        act = np.stack(pd.read_parquet(f, columns=['action'])['action'].to_numpy())
        seconds.append(np.diff(act[:, dims], n=2, axis=0))
    s = np.concatenate(seconds)
    return float(np.sqrt((s ** 2).mean())), len(files)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='LABEL=base[,base...] (base = csv_log_path)')
    ap.add_argument('--dataset', default='', help='LeRobot dataset for the reference jerk')
    ap.add_argument('--out', default='', help='write the summary JSON here')
    ap.add_argument('--skip-ms', type=float, default=2 * 1000 / 15,
                    help='latency_skip window in ms; RTT above it = late chunk (default: 2 steps at 15 FPS)')
    args = ap.parse_args()

    ref = {}
    if args.dataset:
        for name, dims in (('all', slice(None)), ('pos', POS)):
            ref[name], n_eps = reference_jerk(args.dataset, dims)
        print(f'reference jerk (demonstrations, {n_eps} episodes): all {ref["all"]:.5f}  pos {ref["pos"]:.5f}')

    summary = {}
    for spec in args.runs:
        label, paths = spec.split('=', 1)
        runs = [load_run(_base(p)) for p in paths.split(',')]
        res = {}
        for name, dims in (('all', slice(None)), ('pos', POS)):
            per = [metrics(r['executed'], r['seams'], dims) for r in runs]
            w = np.array([p['n_seams'] for p in per], float)
            ws = np.array([p['n_steps'] for p in per], float)
            agg = {
                'seam_jump': float(np.average([p['seam_jump'] for p in per], weights=w)) if w.sum() else float('nan'),
                'step_size': float(np.average([p['step_size'] for p in per], weights=ws)),
                'jerk_rms': float(np.sqrt(np.average([p['jerk_rms'] ** 2 for p in per], weights=ws))),
                'n_seams': int(w.sum()), 'n_steps': int(ws.sum()),
            }
            agg['seam_ratio'] = agg['seam_jump'] / agg['step_size']
            if name in ref:
                agg['jerk_vs_ref'] = agg['jerk_rms'] / ref[name]
            res[name] = agg
        rtt = np.concatenate([r['rtt'] for r in runs]) if any(len(r['rtt']) for r in runs) else np.array([])
        res['rtt_ms'] = ({'median': float(np.median(rtt)), 'p95': float(np.percentile(rtt, 95)),
                          'max': float(rtt.max())} if len(rtt) else None)
        # A chunk is late when its RTT exceeds the latency_skip window: its
        # action[skip] then starts after the time it was planned for.
        res['late_pct'] = float((rtt > args.skip_ms).mean() * 100) if len(rtt) else None
        res['rtc_prefix_frac'] = [r['rtc_frac'] for r in runs]
        res['hold_pct'] = [r['hold_pct'] for r in runs]
        res['effective_skip'] = [r['effective_skip'] for r in runs]
        track = np.concatenate([np.abs(r['executed'][:, R_ARM] - r['state'][:, R_ARM]).mean(1) for r in runs])
        res['r_arm_track_err_median'] = float(np.median(track))
        summary[label] = res

    hdr = f'{"run":8s} {"seam":>8s} {"step":>8s} {"ratio":>6s} {"jerk":>8s} {"vs ref":>7s} {"seam pos":>9s} {"jerk pos":>9s} {"RTT med/p95":>12s} {"late%":>6s} {"hold%":>6s} {"seams":>6s}'
    print(hdr)
    for label, r in summary.items():
        a, p = r['all'], r['pos']
        rtt = f'{r["rtt_ms"]["median"]:.0f}/{r["rtt_ms"]["p95"]:.0f}' if r['rtt_ms'] else '-'
        holds = [h for h in r['hold_pct'] if h is not None]
        print(f'{label:8s} {a["seam_jump"]:8.5f} {a["step_size"]:8.5f} {a["seam_ratio"]:6.2f} '
              f'{a["jerk_rms"]:8.5f} {a.get("jerk_vs_ref", float("nan")):7.2f} {p["seam_jump"]:9.5f} '
              f'{p["jerk_rms"]:9.5f} {rtt:>12s} {r["late_pct"] if r["late_pct"] is not None else float("nan"):6.1f} {np.mean(holds) if holds else float("nan"):6.1f} {a["n_seams"]:6d}')
    print('rtc prefix sent (fraction of chunks):', {k: v['rtc_prefix_frac'] for k, v in summary.items()})
    if args.out:
        Path(args.out).write_text(json.dumps({'reference': ref, 'runs': summary}, indent=1))
        print('wrote', args.out)


if __name__ == '__main__':
    main()
