#!/usr/bin/env python3
"""Gradio-based episode manager for the alfiebot CanDoChallenge dataset."""

import json
import shutil
import subprocess
from pathlib import Path
from datetime import datetime

import gradio as gr
import pyarrow as pa
import pyarrow.parquet as pq

DATASET_DIR = Path(__file__).resolve().parents[3] / "data" / "alfiebot.CanDoChallenge"
META_DIR = DATASET_DIR / "meta"
VIDEO_DIR = DATASET_DIR / "videos" / "chunk-000"
DATA_DIR = DATASET_DIR / "data" / "chunk-000"

CAMERAS = ["left_wide", "right_wide", "left_center", "right_center"]

ROSBAG_TO_GROOT_SCRIPT = Path(__file__).resolve().parent / "rosbag_to_groot.py"
DEMOS_DIR = str(Path("~/alfiebot_ws/data/demonstrations").expanduser())


def load_episodes():
    """Load episodes.jsonl as a dict keyed by episode_index."""
    episodes = {}
    with open(META_DIR / "episodes.jsonl") as f:
        for line in f:
            line = line.strip()
            if line:
                ep = json.loads(line)
                episodes[ep["episode_index"]] = ep
    return episodes


def load_tasks():
    tasks = {}
    with open(META_DIR / "tasks.jsonl") as f:
        for line in f:
            line = line.strip()
            if line:
                t = json.loads(line)
                tasks[t["task_index"]] = t["task"]
    return tasks


def format_episode_name(ep):
    return f"episode_{ep['episode_index']:06d}"


def format_source_date(source):
    try:
        parts = source.split("_")
        date_str = parts[1]
        time_str = parts[2]
        dt = datetime(
            int(date_str[:4]), int(date_str[4:6]), int(date_str[6:8]),
            int(time_str[:2]), int(time_str[2:4]), int(time_str[4:6]),
        )
        return dt.strftime("%Y-%m-%d %H:%M:%S")
    except (IndexError, ValueError):
        return source


def get_video_path(idx, cam):
    vp = VIDEO_DIR / f"observation.images.{cam}" / f"episode_{idx:06d}.mp4"
    return str(vp) if vp.exists() else None


def get_episode_details(ep_idx, episodes, tasks):
    if ep_idx is None or ep_idx not in episodes:
        return "Select an episode to view details."

    idx = ep_idx
    ep = episodes[idx]

    task_name = tasks.get(ep["task_index"], "unknown")
    duration = ep["duration"]
    mins = int(duration // 60)
    secs = duration % 60

    parquet_path = DATA_DIR / f"episode_{idx:06d}.parquet"

    episode_name = f"episode_{idx:06d}"
    details = f"""## {episode_name}

| Field | Value |
|-------|-------|
| **Episode Index** | {ep['episode_index']} |
| **Task** | {task_name} |
| **Task Index** | {ep['task_index']} |
| **Frames** | {ep['length']} |
| **Duration** | {mins}m {secs:.1f}s ({duration:.2f}s) |
| **FPS** | 15 |
| **Source** | {ep['source']} |
| **Recorded** | {format_source_date(ep['source'])} |

### Files
| File | Exists |
|------|--------|
| **Parquet** | {'yes' if parquet_path.exists() else 'MISSING'} |
"""
    for cam in CAMERAS:
        exists = 'yes' if get_video_path(idx, cam) else 'MISSING'
        details += f"| **Video ({cam})** | {exists} |\n"

    return details


def update_episode_task(episode_index: int, new_task_index: int, episodes: dict) -> str:
    """Update task_index in episodes.jsonl and the corresponding parquet file.

    Returns a log string with update status and verification results.
    """
    log_lines = []
    old_task_index = episodes[episode_index]["task_index"]
    ep_name = f"episode_{episode_index:06d}"
    log_lines.append(f"Updating {ep_name}: task_index {old_task_index} -> {new_task_index}")

    # Update episodes.jsonl
    episodes[episode_index]["task_index"] = new_task_index
    with open(META_DIR / "episodes.jsonl", "w") as f:
        for ep in sorted(episodes.values(), key=lambda e: e["episode_index"]):
            f.write(json.dumps(ep) + "\n")
    log_lines.append("episodes.jsonl written.")

    # Verify episodes.jsonl
    with open(META_DIR / "episodes.jsonl") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            ep = json.loads(line)
            if ep["episode_index"] == episode_index:
                verified = ep["task_index"]
                ok = "OK" if verified == new_task_index else "MISMATCH"
                log_lines.append(f"episodes.jsonl verify: task_index={verified} [{ok}]")
                break

    # Update parquet file
    parquet_path = DATA_DIR / f"{ep_name}.parquet"
    if parquet_path.exists():
        table = pq.read_table(parquet_path)
        df = table.to_pandas()
        df["task_index"] = new_task_index
        pq.write_table(pa.Table.from_pandas(df), parquet_path)
        log_lines.append(f"{ep_name}.parquet written.")

        # Verify parquet
        verify_table = pq.read_table(parquet_path)
        verify_val = verify_table.to_pandas()["task_index"].iloc[0]
        ok = "OK" if verify_val == new_task_index else "MISMATCH"
        log_lines.append(f"{ep_name}.parquet verify: task_index={verify_val} [{ok}]")
    else:
        log_lines.append(f"{ep_name}.parquet not found, skipped.")

    return "\n".join(log_lines)


def run_rosbag_to_groot():
    """Run rosbag_to_groot.py to convert any new demonstrations."""
    cmd = [
        "python3", str(ROSBAG_TO_GROOT_SCRIPT),
        "--demos-dir", DEMOS_DIR,
        "--output-dir", str(DATASET_DIR),
        "--task-index", "0",
        "--start-episode", "0",
        "--num-threads", "8",
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    output = result.stdout
    if result.returncode != 0:
        output += f"\nSTDERR:\n{result.stderr}"
    return result.returncode, output


VIDEO_IDS = ["vid-lw", "vid-rw", "vid-lc", "vid-rc"]

_GET_VIDEOS_JS = """
    function getVideos() {
        const ids = %s;
        const videos = [];
        for (const id of ids) {
            const el = document.getElementById(id);
            if (el) {
                const v = el.querySelector('video');
                if (v) videos.push(v);
            }
        }
        return videos;
    }
""" % str(VIDEO_IDS)

PLAY_JS = "() => { %s getVideos().forEach(v => v.play()); }" % _GET_VIDEOS_JS
PAUSE_JS = "() => { %s getVideos().forEach(v => v.pause()); }" % _GET_VIDEOS_JS
STOP_JS = "() => { %s getVideos().forEach(v => { v.pause(); v.currentTime = 0; }); }" % _GET_VIDEOS_JS

SEEK_JS = """
(val) => {
    %s
    const pct = val / 100;
    getVideos().forEach(v => {
        if (v.duration) v.currentTime = pct * v.duration;
    });
}
""" % _GET_VIDEOS_JS

# JS to attach a native 'input' listener so seeking happens while dragging
SLIDER_LIVE_JS = """
() => {
    %s
    const el = document.getElementById('playback-slider');
    if (!el) return;
    const range = el.querySelector('input[type=range]');
    if (!range || range.dataset.liveSeek) return;
    range.dataset.liveSeek = '1';
    range.addEventListener('input', () => {
        const pct = parseFloat(range.value) / 100;
        getVideos().forEach(v => {
            if (v.duration) v.currentTime = pct * v.duration;
        });
    });
}
""" % _GET_VIDEOS_JS

# CSS to auto-size episode list to viewport
CSS = """
#episode-list .table-wrap {
    max-height: calc(100vh - 150px) !important;
    overflow: auto !important;
}
#vid-lw video, #vid-rw video, #vid-lc video, #vid-rc video {
    pointer-events: none;
}
#vid-lw .controls, #vid-rw .controls, #vid-lc .controls, #vid-rc .controls,
#vid-lw video::-webkit-media-controls, #vid-rw video::-webkit-media-controls,
#vid-lc video::-webkit-media-controls, #vid-rc video::-webkit-media-controls {
    display: none !important;
}
"""


def build_ui():
    episodes = load_episodes()
    tasks = load_tasks()
    episode_names = sorted([format_episode_name(ep) for ep in episodes.values()], reverse=True)

    with gr.Blocks(title="Alfiebot Episode Manager", fill_height=True) as app:
        gr.Markdown("# Alfiebot Episode Manager")
        status = gr.Markdown(f"**Dataset:** {DATASET_DIR}  \n**Episodes:** {len(episodes)} | **Tasks:** {len(tasks)}")

        with gr.Row():
            # Left column: episode list (1/3)
            with gr.Column(scale=1):
                refresh_btn = gr.Button("Refresh", size="sm")
                episode_list = gr.Dataframe(
                    value=[[name] for name in episode_names],
                    headers=["Episode"],
                    label=f"Episodes ({len(episode_names)})",
                    interactive=False,
                    elem_id="episode-list",
                )

            # Right column: videos + controls + details (3/4)
            # Right column: videos + controls + details (2/3)
            with gr.Column(scale=2):
                # Video grid 2x2
                with gr.Group():
                    with gr.Row():
                        vid_lw = gr.Video(label="Left Wide", interactive=False,
                                          autoplay=True, height=240,
                                          elem_id="vid-lw")
                        vid_rw = gr.Video(label="Right Wide", interactive=False,
                                          autoplay=True, height=240,
                                          elem_id="vid-rw")
                    with gr.Row():
                        vid_lc = gr.Video(label="Left Center", interactive=False,
                                          autoplay=True, height=240,
                                          elem_id="vid-lc")
                        vid_rc = gr.Video(label="Right Center", interactive=False,
                                          autoplay=True, height=240,
                                          elem_id="vid-rc")

                # Playback controls
                with gr.Row():
                    play_btn = gr.Button("Play", size="sm")
                    pause_btn = gr.Button("Pause", size="sm")
                    stop_btn = gr.Button("Stop", size="sm")

                slider = gr.Slider(0, 100, value=0, step=0.5,
                                   label="Playback Position (%)")

                # Episode Editor
                gr.Markdown("### Episode Editor")
                with gr.Row():
                    task_choices = [f"{idx}: {desc}" for idx, desc in sorted(tasks.items())]
                    task_dropdown = gr.Dropdown(
                        choices=task_choices,
                        show_label=False,
                        interactive=True,
                    )
                    update_task_btn = gr.Button("Update Task", size="sm")
                    delete_episode_btn = gr.Button(
                        "Delete Episode", size="sm", variant="stop",
                    )

                # Episode details
                details = gr.Markdown("Select an episode to view details.")

        selected_idx = gr.State(value=None)

        # Wire up episode selection
        videos = [vid_lw, vid_rw, vid_lc, vid_rc]

        def do_reload(log_text=""):
            """Reload metadata from disk and select top episode.

            Returns tuple matching refresh_outputs.
            """
            nonlocal episodes, tasks, episode_names
            episodes = load_episodes()
            tasks = load_tasks()
            episode_names = sorted([format_episode_name(ep) for ep in episodes.values()], reverse=True)

            status_text = (
                f"**Dataset:** {DATASET_DIR}  \n"
                f"**Episodes:** {len(episodes)} | **Tasks:** {len(tasks)}"
            )
            if log_text:
                status_text += f"\n\n### Last Output\n```\n{log_text}\n```"

            # Auto-select top episode
            if episode_names:
                top_name = episode_names[0]
                top_idx = int(top_name.split("_")[1])
                video_paths = [get_video_path(top_idx, cam) for cam in CAMERAS]
                detail_text = get_episode_details(top_idx, episodes, tasks)
                ep = episodes[top_idx]
                task_value = f"{ep['task_index']}: {tasks.get(ep['task_index'], 'unknown')}"
            else:
                top_idx = None
                video_paths = [None, None, None, None]
                detail_text = "No episodes."
                task_value = None

            return (
                [[name] for name in episode_names],
                status_text,
                *video_paths,
                detail_text,
                task_value,
                top_idx,
            )

        refresh_outputs = [episode_list, status, *videos, details, task_dropdown, selected_idx]

        def on_refresh():
            """Run rosbag_to_groot conversion, then reload UI."""
            gr.Info("Running rosbag_to_groot conversion...")
            try:
                returncode, output = run_rosbag_to_groot()
                if returncode == 0:
                    gr.Info("Conversion complete.")
                else:
                    gr.Warning(f"Conversion finished with errors (exit code {returncode})")
            except subprocess.TimeoutExpired:
                gr.Warning("Conversion timed out after 10 minutes")
                output = "TIMED OUT"
            except Exception as e:
                gr.Warning(f"Conversion failed: {e}")
                output = str(e)
            return do_reload(log_text=output)

        refresh_btn.click(
            fn=on_refresh,
            outputs=refresh_outputs,
        )

        def on_select(evt: gr.SelectData):
            name = episode_names[evt.index[0]]
            idx = int(name.split("_")[1])
            video_paths = [get_video_path(idx, cam) for cam in CAMERAS]
            detail_text = get_episode_details(idx, episodes, tasks)
            ep = episodes[idx]
            task_idx = ep["task_index"]
            task_value = f"{task_idx}: {tasks.get(task_idx, 'unknown')}"
            return *video_paths, detail_text, task_value, idx

        episode_list.select(
            fn=on_select,
            outputs=[*videos, details, task_dropdown, selected_idx],
        )

        def on_update_task(task_selection, ep_idx):
            if ep_idx is None or not task_selection:
                return "No episode selected or no task chosen."
            new_task_index = int(task_selection.split(":")[0])
            log = update_episode_task(ep_idx, new_task_index, episodes)
            detail_text = get_episode_details(ep_idx, episodes, tasks)
            detail_text += f"\n\n### Update Log\n```\n{log}\n```"
            return detail_text

        update_task_btn.click(
            fn=on_update_task,
            inputs=[task_dropdown, selected_idx],
            outputs=[details],
        )

        def on_delete(ep_idx):
            if ep_idx is None:
                gr.Warning("No episode selected.")
                return (
                    [[name] for name in episode_names],
                    f"**Dataset:** {DATASET_DIR}  \n**Episodes:** {len(episodes)} | **Tasks:** {len(tasks)}",
                    None, None, None, None,
                    "No episode selected.", None, None,
                )
            ep = episodes.get(ep_idx)
            if ep is None:
                gr.Warning(f"Episode index {ep_idx} not found.")
                return do_reload()
            source = ep["source"]
            source_dir = Path(DEMOS_DIR) / source
            log_parts = []
            if source_dir.exists():
                shutil.rmtree(source_dir)
                log_parts.append(f"Deleted {source_dir}")
                gr.Info(f"Deleted source: {source}")
            else:
                log_parts.append(f"Source dir not found: {source_dir}")
                gr.Warning(f"Source dir not found: {source}")

            # Run rosbag_to_groot to regenerate dataset without deleted episode
            gr.Info("Running rosbag_to_groot conversion...")
            try:
                returncode, output = run_rosbag_to_groot()
                log_parts.append(output)
                if returncode == 0:
                    gr.Info("Conversion complete.")
                else:
                    gr.Warning(f"Conversion finished with errors (exit code {returncode})")
            except subprocess.TimeoutExpired:
                gr.Warning("Conversion timed out after 10 minutes")
                log_parts.append("TIMED OUT")
            except Exception as e:
                gr.Warning(f"Conversion failed: {e}")
                log_parts.append(str(e))

            return do_reload(log_text="\n".join(log_parts))

        delete_episode_btn.click(
            fn=on_delete,
            inputs=[selected_idx],
            outputs=refresh_outputs,
        )

        # Wire up playback controls via JS
        play_btn.click(None, js=PLAY_JS)
        pause_btn.click(None, js=PAUSE_JS)
        stop_btn.click(None, js=STOP_JS)
        slider.release(None, inputs=[slider], js=SEEK_JS)

    return app


def get_local_ip():
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"


if __name__ == "__main__":
    app = build_ui()
    ip = get_local_ip()
    print(f"\n  Access from other devices: http://{ip}:7860\n")
    app.launch(server_name="0.0.0.0", server_port=7860, share=False, css=CSS,
               allowed_paths=[str(DATASET_DIR)])
