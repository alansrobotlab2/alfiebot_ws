"""Validate the inference server's native N1.7 RTC path against a checkpoint.

Runs in the Isaac-GR00T environment (no ROS, no robot):

    cd ~/Isaac-GR00T && source .venv/bin/activate && source scripts/activate_orin.sh
    python ~/alfiebot_ws/src/alfie_gr00t/alfie_gr00t/scripts/rtc_validate.py \
        --checkpoint checkpoints/GR00T-N1.7-3B --dataset demo_data/droid_sample \
        --embodiment oxe_droid_relative_eef_relative_joint

With an Alfie checkpoint: --checkpoint alfie-gr00t/checkpoint-N \
    --dataset ~/alfiebot_ws/data/<dataset>/lerobot --embodiment new_embodiment

Step 0 -> chunk A. Pretend `--executed` steps were executed; observe there;
commit the next `--overlap` actions of A as the prefix with `--frozen` frozen
steps. Compare the RTC chunk against a plain chunk for the same observation:
frozen rows must reproduce the prefix (PASS), the ramped rows should sit
closer to the prefix than the plain chunk does.
"""
import argparse
import importlib.util
import json
import os
import time
from copy import deepcopy
import numpy as np
import torch

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--checkpoint', default=os.path.expanduser('~/Isaac-GR00T/checkpoints/GR00T-N1.7-3B'))
ap.add_argument('--dataset', default=os.path.expanduser('~/Isaac-GR00T/demo_data/droid_sample'))
ap.add_argument('--embodiment', default='oxe_droid_relative_eef_relative_joint')
ap.add_argument('--episode', type=int, default=0)
ap.add_argument('--executed', type=int, default=4, help='steps executed before the second observation')
ap.add_argument('--overlap', type=int, default=6)
ap.add_argument('--frozen', type=int, default=2)
ap.add_argument('--denoising-steps', type=int, default=4)
ap.add_argument('--out', default='/tmp/rtc_validate.json')
args = ap.parse_args()

SERVER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'groot_inference_server.py')
spec = importlib.util.spec_from_file_location('alfie_server', SERVER)
srv = importlib.util.module_from_spec(spec); spec.loader.exec_module(srv)

from gr00t.data.dataset.lerobot_episode_loader import LeRobotEpisodeLoader
from gr00t.data.dataset.sharded_single_step_dataset import extract_step_data
from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.data.utils import parse_observation_gr00t
from gr00t.policy.gr00t_policy import Gr00tPolicy

tag = EmbodimentTag(args.embodiment)
t0 = time.time()
policy = Gr00tPolicy(embodiment_tag=tag, model_path=args.checkpoint, device='cuda')
policy.model.action_head.num_inference_timesteps = args.denoising_steps
print(f'model loaded in {time.time()-t0:.0f}s', flush=True)
modality = policy.get_modality_config()
action_keys = modality['action'].modality_keys
loader = LeRobotEpisodeLoader(args.dataset, modality_configs=modality)
traj = loader[args.episode]
obs_cfg = deepcopy(modality); obs_cfg.pop('action')

def obs_at(step):
    dp = extract_step_data(traj, step, obs_cfg, tag)
    obs = {f'state.{k}': v for k, v in dp.states.items()}
    obs.update({f'video.{k}': np.array(v) for k, v in dp.images.items()})
    for lk in modality['language'].modality_keys:
        obs[lk] = dp.text
    return parse_observation_gr00t(obs, modality)

def flat(d):
    return np.concatenate([np.asarray(d[k][0]) for k in action_keys], axis=-1)  # (H, D)

wrapper = srv.JpegPolicyWrapper(policy, language_key=modality['language'].modality_keys[0], rtc_enabled=True)

ci0, st0 = policy.prepare_inputs(obs_at(0))
torch.manual_seed(0); A_dict = policy.run_inference(ci0, st0); A = flat(A_dict)
H, D = A.shape
print(f'chunk A: horizon={H} dim={D}', flush=True)

executed, overlap, frozen = args.executed, args.overlap, args.frozen
prefix = A[executed:executed + overlap]
groups, start = {}, 0
for k in action_keys:
    dim = np.asarray(A_dict[k][0]).shape[-1]
    groups[k] = prefix[:, start:start + dim]; start += dim

ci4, st4 = policy.prepare_inputs(obs_at(executed))
torch.manual_seed(1); t = time.time(); B_plain = flat(policy.run_inference(ci4, st4)); t_plain = time.time() - t
seed, options = wrapper._rtc_seed_from_groups(groups, st4[0], frozen)
print('options', options, 'seed nonzero rows', int((np.abs(seed.cpu().numpy()[0]).sum(-1) > 0).sum()), flush=True)
torch.manual_seed(1); t = time.time(); B_rtc = flat(wrapper._run_action_head_rtc(ci4, st4, (seed, options))); t_rtc = time.time() - t
# second call to time without warmup effects
torch.manual_seed(2); t = time.time(); B_rtc2 = flat(wrapper._run_action_head_rtc(ci4, st4, (seed, options))); t_rtc2 = time.time() - t

def per_row(X):
    return np.abs(X[:overlap] - prefix).max(axis=-1)
res = {
    'H': int(H), 'D': int(D), 'options': options,
    'frozen_max_abs_err_rtc': per_row(B_rtc)[:frozen].max().item(),
    'frozen_max_abs_err_plain': per_row(B_plain)[:frozen].max().item(),
    'row_err_rtc': per_row(B_rtc).round(5).tolist(),
    'row_err_plain': per_row(B_plain).round(5).tolist(),
    'post_overlap_err_rtc_vs_plain': float(np.abs(B_rtc[overlap:] - B_plain[overlap:]).mean()),
    'rtc_seed_repeatability_frozen': float(np.abs(B_rtc2[:frozen] - B_rtc[:frozen]).max()),
    'ms_plain': round(t_plain * 1000, 1), 'ms_rtc': round(t_rtc * 1000, 1), 'ms_rtc_2nd': round(t_rtc2 * 1000, 1),
    'prefix_scale': float(np.abs(prefix).mean()),
}
print(json.dumps(res, indent=1))
json.dump(res, open(args.out, 'w'), indent=1)
print('wrote', args.out)
ok = res['frozen_max_abs_err_rtc'] < 1e-3 * max(1.0, res['prefix_scale']) * 10 and all(
    r <= p + 1e-6 for r, p in zip(res['row_err_rtc'][:frozen], res['row_err_plain'][:frozen]))
print('RESULT:', 'PASS' if ok else 'CHECK', flush=True)
