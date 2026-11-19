# Data Collection Guide

This guide covers Phase 2 of the GR00T integration: collecting demonstration data for training.

## Overview

We use ROS2 bag recording to capture all sensor data and robot actions during teleoperation. The recorded data will later be converted to GR00T training format.

## Recorded Topics

The data recorder captures all sensor observations and robot actions:

### Observation Space (Input to GR00T)

**Camera Feeds (4 stereo streams - compressed):**
- `/alfie/stereo_camera/left_wide/image_raw/compressed` + `camera_info`
- `/alfie/stereo_camera/left_center/image_raw/compressed` + `camera_info`
- `/alfie/stereo_camera/right_center/image_raw/compressed` + `camera_info`
- `/alfie/stereo_camera/right_wide/image_raw/compressed` + `camera_info`

**Robot Proprioception:**
- `/alfie/robotlowstate` - Low-level robot state (joints, grippers, base)

### Action Space (Output from GR00T)

There is no separate command topic. `/alfie/robotlowstate` carries both the
measured state and the last command for every DOF (servo `target_location`,
back `command_position`, base `command_cmd_vel`), and `rosbag_to_groot.py`
derives the 22D action vector from those command fields.

**Metadata:**
- `/alfie/recording/metadata` - Annotations and labels (dataset name, start/stop)

> **Note:** The GR00T policy uses camera images + `/alfie/robotlowstate` as observations. The additional topics (`camera_info`, `tf`, `robot_description`, `rosout`) are recorded for dataset analysis and debugging.

## Data Layout

Recordings are grouped by **recording session**, not by task: a session often
mixes tasks, demos and corrections. Each session is one folder under
`~/alfie_data/raw/`:

```
~/alfie_data/raw/
└── 2026-11-20_s02_hw2/         YYYY-MM-DD_sNN_hwN: date, session of that day, hardware version
    ├── session.json            {session, hw, created, closed, ...}
    ├── demonstrations/         raw MCAP bags from the recorder
    │   └── demo_20261120_143022/
    │       ├── demo_20261120_143022_0.mcap.zstd
    │       └── episode.json    facts known only at record time
    ├── lerobot/                GR00T LeRobot v2.1 output of rosbag_to_groot.py
    │   ├── meta/               info.json, modality.json, tasks.jsonl, episodes.jsonl
    │   ├── data/chunk-000/
    │   └── videos/chunk-000/
    └── registry/               copy of the task registry, written when the session is closed
```

- The hardware version comes from `config/hardware.yaml` (`current`).
- A session is **open** while you record into it and review it. **Closing** it
  (episode manager) makes it final: the recorder, the converter and the episode
  manager refuse to change a closed session, episode deletion included;
  anything recorded later goes into a new session. Closing writes the episode
  and frame counts into `session.json` and removes any stats files. Only closed
  sessions are synced and trained on.
- Training never reads a session directly. `scripts/build_views.py` turns the
  closed sessions into a **build** under `~/alfie_data/views/` (see "Building a
  Training Set"), and `scripts/sync_sessions.py` copies closed sessions to the
  training machine.

The recorder, converter and episode manager derive their paths from
`alfie_gr00t/dataset_paths.py`. `--data-root` / `ALFIE_DATA_ROOT` moves the
tree.

No data lives in the workspace and no folder is named for a challenge. The 218
recordings made before sessions existed (the `CleanTheRoom` dataset that
checkpoint-40000 was trained on) were copied into the sessions of their three
recording days (`2026-11-10_s01_hw2` to `2026-11-12_s01_hw2`) by
`scripts/backfill_sessions.py`; each episode carries `legacy_dataset` and
`legacy_episode_index`, its number in that dataset. The diagnostic scripts take
any session's `lerobot/` folder as `--dataset-path`.

### Task registry

`config/tasks.yaml` has one entry per task ID and is the only list of tasks:
the episode manager's task picker, the converter, the inference client and the
training-side export all read it. `config/objects.yaml` (object names) and
`config/hardware.yaml` (hardware versions) sit beside it. Each task has

- `text`: the string the deployed policy is trained on, and what recording
  writes to `meta/tasks.jsonl` today;
- `instruction`: the wording from the v6 training run on, generated from the
  structured fields (`verb`, `object`, `destination`) by the templates at the
  top of the file. The client and the datasets switch to it together;
- `family`, `status`, `start_state`, `includes`, `end_state`.

To add a task, add its entry with `status: collecting`, rebuild
(`colcon build --packages-select alfie_gr00t`) and restart the episode manager
and recorder.

### What is stored with each episode

Every row of a session's `meta/episodes.jsonl` carries, besides the LeRobot
fields:

| field | meaning |
|---|---|
| `episode_uid` | `<session>/<episode index>`, e.g. `2026-11-20_s02_hw2/000007` |
| `task_id`, `family`, `verb`, `object`, `destination`, `start_state`, `end_state` | the task and its structured fields, from the registry |
| `source` | `demo`, `correction`, `scripted_recovery` or `autonomous_success` |
| `round` | correction round (0 for demos) |
| `policy_ckpt`, `run_id`, `instruction_sent`, `takeover_s` | corrections: the checkpoint that was driving, the run it was taken from, the string the policy was running on, seconds into the run |
| `failure_mode` | corrections: why the operator took over (set in review) |
| `operator`, `objects`, `surface`, `hw`, `recorded_at` | who, what, where, on which hardware, when |
| `quality` | `ok`, `marginal` or `bad` (set in review) |
| `notes` | free text (set in review) |
| `close_frame` | first frame of the grasp: the task's `close` condition in `tasks.yaml` (for `pick_up_soda_can`, right gripper stalled on the can for 2 s with the back still down). Null for a miss and for tasks without one |
| `done_frame` | frame where the task's `done` condition is confirmed, as the client would end the task (for `pick_up_soda_can` the end of the lift, about 10 s after the grasp). Null when it never holds |
| `untrimmed_length` | frames the episode had before a cut after the grasp (null = not cut; no task cuts today) |
| `alfiebot_commit` | commit of `~/alfiebot_ws` when the episode was recorded |
| `bag` | the source bag folder |

The recorder writes the record-time part into `episode.json` in the bag folder;
the converter copies it into the row and adds the registry fields. In the two
older datasets `source` holds the bag name and corrections are marked
`origin: takeover`; the tools read both forms.

## Quick Start

### 1. Build the package

```bash
cd ~/alfiebot_ws
colcon build --packages-select alfie_gr00t
source install/setup.bash
```

### 2. Start the data recorder

The recorder is started by `alfie_bringup.py` (`launch_recorder:=true` is the
default). To run it on its own:

**Terminal 1: Launch data recorder**
```bash
ros2 launch alfie_gr00t data_collection.launch.py
```

It records into today's open session and starts one (`..._s01_hw2`, then
`s02`, ...) on the first recording of the day, or after the last one was closed.
`dataset:=<session>` pins a folder instead.

**Set the recording context** in the Foxglove Episode Manager panel ("Next
recording"): the **source**, the **correction round**, the **operator** and the
surface. It applies to every episode until you change it and survives restarts.
The **task** is optional there: leave it "(not set)" to record whatever you
like in any order and tag the episodes after converting, or set it to have the
next episodes labelled as they are recorded. Without the panel:

```bash
ros2 topic pub --once /alfie/recording/set_context std_msgs/msg/String \
  '{data: "{\"task_id\": \"pick_up_soda_can\", \"source\": \"demo\", \"operator\": \"alan\"}"}'
ros2 topic echo --once /alfie/recording/context     # what the recorder will write
```

An episode recorded with no task set is converted as **unlabelled** and tagged
in review.

### 3. Start/Stop recording

**Terminal 2: Start recording**
```bash
ros2 topic pub --once /alfie/recording/start std_msgs/msg/Bool '{data: true}'
```

Now teleoperate Alfie to perform the can pickup task.

**When done, stop recording:**
```bash
ros2 topic pub --once /alfie/recording/stop std_msgs/msg/Bool '{data: true}'
```

### 4. Check recording status

```bash
ros2 topic echo /alfie/recording/status      # RECORDING:<dataset>/demo_... or STOPPED
```

## Recording Workflow

### Full Demo Recording Session

1. **Setup**: Position a soda can on the floor in view of cameras
2. **Start recorder**: Launch data_collection node
3. **Start recording**: Publish to `/recording/start`
4. **Teleoperate**: Drive to can, align, reach, grasp, lift
5. **Stop recording**: Publish to `/recording/stop`
6. **Repeat**: Move can to new position and record again

### Best Practices

**Diversity is key!** Collect demonstrations with:
- ✅ Various can positions (left, right, center, far, close)
- ✅ Different can orientations (standing, lying down)
- ✅ Multiple lighting conditions (bright, normal, dim)
- ✅ Different starting robot poses
- ✅ Various approach angles
- ✅ Different can types/brands (Coke, Pepsi, etc.)

**Quality over quantity:**
- 500+ successful demonstrations minimum
- Include ~100 partial successes (dropped can, grasp slippage)
- Avoid recordings with:
  - Camera occlusions
  - Network lag/stuttering
  - E-stop triggers
  - Hardware failures

## Takeover Episodes (correcting a GR00T run)

Record a correction from where the policy is about to fail (HG-DAgger style).
The episode starts mid-task and holds only frames where the operator is driving.

1. Start the data recorder (bringup does) and set the **correction round** and
   **operator** in the episode manager. Put on the headset.
2. Start the run with the agent ("pick up the can") or `ab_trial.py`. Once
   `groot_client` reports ACTIVE, the VR node goes into **observe**. It releases
   every subsystem so GR00T drives, and passthrough turns on.
3. When the run is about to fail, **click the left stick**:
   - The robot freezes at its commanded pose, gripper included, and the base stops.
   - The latched `/alfie/groot/takeover` event (`{stamp_ns, task, run_id,
     policy_ckpt, takeover_s}`) stops
     `groot_client` without disabling the servos, and cancels the agent's task
     or clean-the-room mission.
   - Recording starts and passthrough turns off.
4. Wait for the recording indicator, then correct the task:
   - Grip moves an arm.
   - Each gripper stays where it was until its trigger reaches the held value.
   - The head follows headset motion from the frozen pose.
   - Sticks drive the base, A/B move the back.
5. **Right stick click** ends the episode. VR then stays in normal teleop.

Set `groot_takeover:=false` on the VR node to turn this off. Without it, the
headset takes the head and base from any GR00T run.

Conversion is the same `rosbag_to_groot.py` run. A takeover becomes an episode
with:
- `source: correction`, whatever source the recording context says
- the run's task (the registry task whose `text` the policy was running on)
- `run_id` (one per policy activation, so every correction taken from the same
  run shares it and can be kept in the same split), `policy_ckpt` (the
  checkpoint the server reports), `instruction_sent` and `takeover_s`
- `round` from the recording context
- only the operator's part: the frozen lead-in is trimmed to 0.5 s before the
  first motion, and the full ending is kept
- nothing at all if fewer than 16 frames remain (reported as failed; shorter
  than one action chunk is no use to training)

The episode manager marks corrections `C<round>`; tag why you took over
("Why taken over") while reviewing.

Training with takeovers:
- They stay in the same raw session as the demos recorded with them.
  `build_views.py` puts them in their own view (`<family>__correction` by
  default), weighted with `--ratio`.
- Once a task has 5 corrections rated ok, the build holds some out
  (`heldout/`): does the model predict the correction from a bad spot?
- Set the **correction round** before recording against a new checkpoint; each
  row also keeps `policy_ckpt` and `run_id`.
- Retrain from the base model on demos and takeovers together, rather than
  continuing from the last checkpoint on new data alone.

## Reviewing Episodes

The episode manager is a ROS node (started by bringup) that owns the sessions on
disk. Use it from the **Episode Manager panel in Foxglove**
(`alfie_foxglove_extension`):

- **Session**: pick a session to review. **New session** starts the next one of
  today and points the recorder at it; **Record into this session** points the
  recorder at an open session you are looking at.
- **Next recording**: the recording context (task, source, correction round,
  operator, surface) and where the next episode goes.
- **Convert** runs `rosbag_to_groot.py` on all the new bags and streams its
  output. Bags recorded with a task keep it; the others become unlabelled
  episodes. Nothing is guessed.
- **Review** plays the four cameras in sync: <kbd>Space</kbd> play/pause,
  <kbd>←</kbd>/<kbd>→</kbd> one frame, <kbd>Shift</kbd> for one second, playback
  speed 0.25×–4×.
- **Tag** with the task picker: click **Unlabelled** above the episode list,
  then type a task's number (`number` in `tasks.yaml`) or a few words from its
  ID or description and press <kbd>Enter</kbd>. The box clears and keeps the
  focus, and the next episode is selected, so a batch goes `1` <kbd>Enter</kbd>
  `2` <kbd>Enter</kbd> ... <kbd>T</kbd> puts the cursor in the picker,
  <kbd>↑</kbd>/<kbd>↓</kbd> choose among several matches, <kbd>Esc</kbd> leaves
  it. Tagging rewrites the episode's row (task ID, structured fields, grasp and
  done frames) and its parquet `task_index`. The chip beside each episode is
  the task's number. <kbd>J</kbd>/<kbd>K</kbd> walk the list.
- **Quality** (`ok` / `marginal` / `bad`), **Why taken over** (corrections) and
  **Notes** are saved to the episode's row.
- **Delete** removes the source bag, the parquet and videos, and the metadata row.
  Other episodes keep their indices.
- **Close session…** when the review is done; it is unavailable while any
  episode is still unlabelled. It saves a copy of the task
  registry, the git commits of the alfiebot workspace and Isaac-GR00T, and the
  data schema into the session, then freezes it. After that nothing can be
  relabelled, deleted or converted, and the recorder moves on to a new session.
  Close before syncing to the training machine.

The node also serves a plain web page at `http://alfiebot.local:7880/` (startup
logs every URL that works). It plays, relabels by task index and deletes, but
has no session, recording-context or quality controls.

```bash
ros2 run alfie_gr00t episode_manager        # opens the newest session
```

The panel also shows what training does with an episode: `h` marks one the
newest training build on this machine holds out, and an episode of a closed
session can be left out of future builds with a reason ("Exclude from
training"). Sessions of an earlier day that were never closed are marked in the
session list, because only closed sessions are trained on.

**Convert refuses to run while episodes have no bag** in `demonstrations/` (e.g.
the bags were moved off the robot) unless you confirm: the converter treats a
missing bag as a deleted demo and would drop those episodes.

| parameter | default | meaning |
|---|---|---|
| `dataset` | `$ALFIE_DATASET`, else the newest session | session opened at startup; others are picked in the UI |
| `data_root` | `$ALFIE_DATA_ROOT` or `~/alfie_data/raw` | folder of sessions |
| `tasks_yaml` | installed `config/tasks.yaml` | task registry |
| `workspace_dir` / `isaac_groot_dir` | `~/alfiebot_ws` / `~/Isaac-GR00T` | git checkouts whose commits are saved when a session is closed |
| `port` / `host` | `7880` / `0.0.0.0` | HTTP bind (no auth: keep it on a trusted network) |
| `python` | `~/Isaac-GR00T/.venv/bin/python` | interpreter for the converter and parquet edits; needs pyarrow, pandas, zstandard |
| `num_threads` / `use_gpu` | `8` / `false` | passed to `rosbag_to_groot.py` |
| `recorder_status_topic` | `/alfie/recording/status` | data_recorder state, shown in the top bar |

The node itself runs on system python with no extra packages (React and htm are
vendored under `alfie_gr00t/episode_manager/web/`, no build step).

## Data Storage

**Default location:** `~/alfie_data/raw/<session>/demonstrations/`

Each recording creates a directory:
```
<session>/demonstrations/
├── demo_20261120_143022/
│   ├── metadata.yaml
│   ├── episode.json
│   └── demo_20261120_143022_0.mcap.zstd
└── demo_20261120_143156/
    └── ...
```

**Storage format:** MCAP with zstd compression
- Efficient for large camera streams
- ~500MB-1GB per 2-minute demonstration
- Compatible with ROS2 bag tools

## Playback and Verification

### Play back a recording

```bash
ros2 bag play ~/alfie_data/raw/<session>/demonstrations/demo_20250129_143022
```

### Inspect bag info

```bash
ros2 bag info ~/alfie_data/raw/<session>/demonstrations/demo_20250129_143022
```

### Extract images for visualization

```bash
# Install tool if needed (rosbag2 metapackage: ros2 bag CLI + storage plugins)
sudo apt install ros-jazzy-rosbag2

# Extract images from a specific topic
ros2 bag play demo_20250129_143022 --topics /oak/rgb/image_raw
```

## Troubleshooting

### Recording doesn't start

**Check topics are publishing:**
```bash
ros2 topic list
ros2 topic hz /alfie/stereo_camera/left_wide/image_raw/compressed
ros2 topic hz /alfie/joint_states
ros2 topic hz /alfie/robotlowstate
```

**Check recorder node:**
```bash
ros2 node info /alfie/data_recorder
```

### High disk usage

**Enable compression** (already default):
- MCAP format with zstd compression
- Reduces size by ~3-5x vs raw

**Reduce camera resolution** (if needed):
- Edit camera driver config to lower resolution
- 640x480 usually sufficient for training

**Shorter recordings:**
- Keep demos <3 minutes
- Break long tasks into subtasks

### Timestamp synchronization issues

ROS2 bag automatically timestamps messages. If you see drift:
- Check system time is synchronized (NTP)
- Verify camera drivers publish correct timestamps
- Use `message_filters` for explicit sync (advanced)

## Converting to GR00T Training Format

Once you have recorded demonstrations, convert them to the GR00T-flavoured
LeRobot v2 format (Isaac-GR00T `getting_started/data_preparation.md`) using
`rosbag_to_groot.py`. This produces parquet files (state/action data), MP4
videos (camera observations), and the meta files required for training. On a
brand-new dataset it also creates `meta/info.json`, `meta/modality.json` and
`meta/tasks.jsonl`.

### Check available data

Run with `--report` to see a summary of recorded demos and any existing converted output:

```bash
python3 src/alfie_gr00t/scripts/rosbag_to_groot.py --dataset 2026-11-20_s02_hw2 --report
```

### Convert all demos

The episode manager's Convert button runs this for you. By hand (ROS sourced,
Isaac-GR00T venv python):

```bash
python3 src/alfie_gr00t/scripts/rosbag_to_groot.py \
    --dataset 2026-11-20_s02_hw2 --num-threads 8
```

Each bag is labelled with the task it was recorded for (`episode.json`), and
`meta/tasks.jsonl` gets that task's `text` from the registry. A bag recorded
without a task becomes an unlabelled episode (`task_index` -1 in its row and
parquet until it is tagged); `--task-id pick_up_soda_can` labels such bags
instead. Episodes shorter than 16 frames are not converted.

### Where a pick ends

A pick includes the lift: it ends with the item held in the `carry` pose
(`tasks.yaml` poses), back all the way up, held still for 1-2 s. Every task
that starts with the item in hand starts from that pose, so a pick from any
surface can be followed by any of them. Each task's `includes` line lists the
motions its episodes must contain.

The converter can cut episodes a fixed time after the grasp (`close.keep_s` on
a task; `--trim-bags` cuts the source bag at the same point and moves the
original to `<session>/demonstrations_untrimmed/`). No task uses it: a policy
trained on picks cut before the lift never raises the back, and nothing else
demonstrates that motion.

### GPU-accelerated video encoding

On the Jetson, use `--use-gpu` to encode videos with NVENC instead of CPU libx264:

```bash
python3 src/alfie_gr00t/scripts/rosbag_to_groot.py \
    --dataset 2026-11-20_s02_hw2 --num-threads 8 --use-gpu
```

## Building a Training Set

Training reads a **build**, never a session. A build is made from every closed
session by `scripts/build_views.py` (rules in `alfie_gr00t/training_views.py`;
needs pyarrow, so run it with the Isaac-GR00T venv python):

```bash
~/Isaac-GR00T/.venv/bin/python scripts/build_views.py --dry-run   # show the split
~/Isaac-GR00T/.venv/bin/python scripts/build_views.py
```

```
~/alfie_data/
├── raw/                          closed sessions (the source)
├── catalog/episodes.parquet      one row per episode of every closed session, rebuilt each run
└── views/
    ├── 2026-11-20_b01/           one build: never changed after it is written
    │   ├── manifest.json         which episode went where; code and registry commits; held-out rule
    │   ├── mix.json              sampling share of each train view
    │   ├── pick_place__demo/lerobot/         train views: named from the episodes' family and source
    │   ├── pick_place__correction/lerobot/
    │   └── heldout/lerobot/      never trained on
    └── latest -> 2026-11-20_b01
```

- **Views.** Each is a normal GR00T LeRobot dataset: episodes renumbered from 0,
  `task_index` renumbered against the view's own `tasks.jsonl`, videos
  hard-linked to the session's. Every row keeps its session fields and its
  `episode_uid`. Training is split into views by task family and source
  unless told otherwise, so `pick_place__correction` is simply the corrections
  of the `pick_place` family and a new family or source needs no code change.
  `--group-by task_id,source` or `--group-by family,source,round` splits on any
  other episode fields, to weight a single task or the latest correction round
  on its own.
- **Left out.** Episodes rated `bad`, and those in `raw/exclusions.jsonl`: an
  episode found bad after its session was closed is excluded there with a
  reason (episode manager, "Exclude from training"; it can be taken back),
  because a closed session is not edited. `sync_sessions.py` merges the two
  machines' lists. `marginal` episodes are trained on (`--exclude-marginal`
  drops them) and never held out.
- **Held out.** Chosen at each build: within every task x source group, the
  episodes rated `ok` with the lowest hash of their `episode_uid`, 10% rounded
  up (`--heldout-fraction`), at least one, none from a group of fewer than 5.
  Everything the previous build held out stays held out, so a later checkpoint
  never trains on an earlier one's test episodes (`--previous none` starts
  over, on purpose).
- **Weights.** `mix.json` gives each train view the share of sampled frames.
  The default is its share of frames, which is what pooling the folders would
  do; `--ratio pick_place__correction=0.12` fixes one and the rest split what
  is left.
- **Wording.** `--wording text` (default) labels episodes with the string the
  deployed policy hears; `--wording instruction` is the v6 wording. The robot
  client must change with it.
- **Stats.** None are written. The GR00T trainer generates `stats.json` and
  `relative_stats.json` in each view on first use and merges them across views;
  `LeRobotEpisodeLoader` on its own (evals) needs `gr00t/data/stats.py` run on
  the view first.

New builds are made on the training machine, which is where the held-out set
is chosen. A checkpoint is kept with the `manifest.json` of its build; on the
robot the same build is written again from the local sessions for engine
calibration (train views) and evaluation (`heldout/`):

```bash
~/Isaac-GR00T/.venv/bin/python scripts/build_views.py --from-manifest <checkpoint>/manifest.json
```

It refuses to write anything if an episode of the build is missing locally.

### Copy sessions to the training machine

```bash
python3 scripts/sync_sessions.py user@workstation:alfie_data/raw            # compare only
python3 scripts/sync_sessions.py user@workstation:alfie_data/raw --push     # copy
```

Only closed sessions go. Run it on the workstation with `--pull` and the
robot's address if the robot cannot log in there. `--no-bags` leaves the raw
bags behind (training does not read them, but the robot then holds the only
copy).

### Fine-tune (GR00T N1.7)

`alfiebot_config.py` sets the action horizon and representation (RELATIVE
arms and head; ABSOLUTE base velocity, back and grippers). The image settings
match the base model's processor (short edge 256, 0.95 crop) so the 352x256
camera frames are used at native aspect. `build_views.py` prints the
`--dataset-path` / `--dataset-mix-ratios` arguments of a build
(`--dataset-mix-ratios` is a flag of the Isaac-GR00T fork):

```bash
python gr00t/experiment/launch_finetune.py \
    --base-model-path nvidia/GR00T-N1.7-3B \
    --dataset-path ~/alfie_data/views/latest/pick_place__demo/lerobot \
    --embodiment-tag NEW_EMBODIMENT \
    --modality-config-path ~/alfiebot_ws/src/alfie_gr00t/alfie_gr00t/alfiebot_config.py \
    --shortest-image-edge 256 --crop-fraction 0.95 \
    --color-jitter-params brightness 0.3 contrast 0.4 saturation 0.5 hue 0.08 \
    --global-batch-size 32 --max-steps 10000 --save-steps 2000 \
    --output-dir ~/Isaac-GR00T/alfie-gr00t
```

After training, `action_chunk_size` in `groot_client.yaml` must equal the
horizon.

### All options

Run with no arguments for a full summary of available options and examples:

```bash
python3 src/alfie_gr00t/scripts/rosbag_to_groot.py
```

### Output structure

After conversion, `<dataset>/lerobot/` will contain:

```
lerobot/
├── data/
│   └── chunk-000/
│       ├── episode_000000.parquet
│       ├── episode_000001.parquet
│       └── ...
├── videos/
│   └── chunk-000/
│       ├── observation.images.left_wide/
│       │   ├── episode_000000.mp4
│       │   └── ...
│       ├── observation.images.right_wide/
│       ├── observation.images.left_center/
│       └── observation.images.right_center/
└── meta/
    ├── info.json
    ├── modality.json
    ├── episodes.jsonl
    └── tasks.jsonl
```

## Next Steps

After a recording sitting:

1. **Convert and tag** - Convert the new bags and tag them in the episode manager
2. **Review** - Rate episodes ok / marginal / bad; delete the unusable ones
3. **Close the session** - Only closed sessions are trained on
4. **Sync** - `sync_sessions.py` copies closed sessions to the training machine
5. **Build and train** - `build_views.py` there, then fine-tune from the build

## Advanced Options

### Custom data root or bag directory

```bash
# Whole tree on another drive: <data_root>/<dataset>/demonstrations
ros2 launch alfie_gr00t data_collection.launch.py \
    data_root:=/mnt/ssd/alfie_data dataset:=2026-11-20_s02_hw2

# Explicit bag directory, ignores the dataset layout
ros2 launch alfie_gr00t data_collection.launch.py \
    output_dir:=/path/to/custom/dir
```

### Auto-start recording

```bash
ros2 launch alfie_gr00t data_collection.launch.py \
    auto_start:=true
```

### Longer max duration

```bash
ros2 launch alfie_gr00t data_collection.launch.py \
    max_duration:=600.0  # 10 minutes
```

### Record specific topics only

Edit [data_recorder.py](alfie_gr00t/nodes/data_recorder.py:38) and modify `self.topics_to_record`.

## Tips for Efficient Data Collection

**Session planning:**
- Record in 2-hour blocks
- Take breaks to avoid operator fatigue
- 20-30 demos per session is realistic

**Setup optimization:**
- Pre-position multiple cans
- Quick reset between demos
- Use consistent starting pose

**Team approach:**
- One person teleoperates
- Another manages recording start/stop
- Third person annotates previous recordings

**Target: 500 demos in ~2 weeks**
- 50 demos/day × 10 days
- ~2-3 hours of recording per day
- ~3-4 hours of annotation per day

---

**Questions?** See main [README.md](README.md) or open an issue.
