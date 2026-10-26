// REST client and shared helpers for the episode manager panel.
//
// The backend is the episode manager node in alfie_gr00t
// (alfie_gr00t/episode_manager/http_server.py). This panel is a second front
// end for it; the node's own web UI keeps working alongside.

export type Task = { index: number; task: string };

export type Episode = {
  index: number;
  task_index: number | null;
  length: number | null;
  duration: number | null;
  source: string;
  recorded: string | null;
  has_source: boolean;
  files: Record<string, boolean>;
};

export type DatasetSummary = {
  name: string;
  path: string;
  demos_path: string;
  converted: boolean;
  fps: number | null;
  tasks: Task[];
  episodes: Episode[];
  total_frames: number;
  pending: string[];
  orphans: number;
  stats_stale: boolean;
};

export type DatasetInfo = {
  data_root: string;
  datasets: string[];
  dataset: DatasetSummary;
  python: string;
  busy: boolean;
  recorder: string;
};

export type Status = { dataset?: string; busy: boolean; recorder: string; pending: number };

export type Job = {
  state: "idle" | "running" | "ok" | "failed" | "cancelled";
  dataset?: string;
  returncode?: number | null;
  started?: number;
  elapsed?: number;
  log: string[];
};

export type Result = {
  ok: boolean;
  error?: string;
  log?: string[];
  needs_force?: boolean;
  orphans?: number;
};

export type Api = {
  dataset: () => Promise<DatasetInfo>;
  status: () => Promise<Status>;
  job: () => Promise<Job>;
  selectDataset: (name: string) => Promise<Result>;
  setTask: (episode: number, task: number) => Promise<Result>;
  remove: (episode: number) => Promise<Result>;
  convert: (task: number, force: boolean, taskName: string) => Promise<Result>;
  cancel: () => Promise<Result>;
  videoUrl: (episode: number, camera: string, rev: number) => string;
};

/** A client for the episode manager at `base` (e.g. https://alfiebot.local:8082/episodes). */
export function makeApi(base: string): Api {
  const root = base.replace(/\/+$/, "");

  async function get<T>(path: string): Promise<T> {
    const res = await fetch(root + path, { cache: "no-store" });
    if (!res.ok) {
      throw new Error(`${res.status} ${res.statusText}`);
    }
    return (await res.json()) as T;
  }

  async function post(path: string, body?: unknown): Promise<Result> {
    try {
      const res = await fetch(root + path, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body ?? {}),
      });
      return (await res.json()) as Result;
    } catch (err) {
      return { ok: false, error: String(err) };
    }
  }

  return {
    dataset: async () => await get<DatasetInfo>("/api/dataset"),
    status: async () => await get<Status>("/api/status"),
    job: async () => await get<Job>("/api/job"),
    selectDataset: async (name: string) => await post("/api/select_dataset", { name }),
    setTask: async (episode: number, task: number) =>
      await post("/api/set_task", { episode, task }),
    remove: async (episode: number) => await post("/api/delete", { episode }),
    convert: async (task: number, force: boolean, taskName: string) =>
      await post("/api/convert", { task, force, task_name: taskName }),
    cancel: async () => await post("/api/cancel"),
    videoUrl: (episode: number, camera: string, rev: number) =>
      `${root}/api/video?episode=${episode}&camera=${camera}&rev=${rev}`,
  };
}

export const CAMERAS = [
  { key: "left_wide", label: "Left wide" },
  { key: "right_wide", label: "Right wide" },
  { key: "left_center", label: "Left center" },
  { key: "right_center", label: "Right center" },
] as const;

// --- Formatting -------------------------------------------------------------

export const episodeName = (index: number): string => `episode_${String(index).padStart(6, "0")}`;

export function fmtDuration(seconds: number | null | undefined): string {
  if (seconds == undefined || Number.isNaN(seconds)) {
    return "--";
  }
  const m = Math.floor(seconds / 60);
  const s = seconds - m * 60;
  return m !== 0 ? `${m}m ${s.toFixed(1).padStart(4, "0")}s` : `${s.toFixed(1)}s`;
}

// Tasks get a stable hue by index so a label reads at a glance in the list.
const TASK_TONES = 8;
export const taskTone = (index: number | null | undefined): string =>
  index == undefined ? "task-none" : `task-${index % TASK_TONES}`;

export const filesOk = (episode: Episode): boolean => Object.values(episode.files).every(Boolean);
