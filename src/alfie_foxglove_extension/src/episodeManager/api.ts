// REST client and shared helpers for the episode manager panel.
//
// The backend is the episode manager node in alfie_gr00t
// (alfie_gr00t/episode_manager/http_server.py). This panel is a second front
// end for it; the node's own web UI keeps working alongside.

/**
 * A task of the open dataset. `index` is its row in meta/tasks.jsonl, or null
 * for a registry task the dataset has no episode of yet. `id` is the task
 * registry ID, or null for a string that is not in the registry.
 */
export type Task = {
  index: number | null;
  task: string;
  id: string | null;
  /** The registry's short number for the task, for picking it by number. */
  number: number | null;
  instruction: string | null;
  description: string | null;
};

export type Episode = {
  index: number;
  task_index: number | null;
  length: number | null;
  duration: number | null;
  /** Source bag folder. */
  bag: string;
  /** demo | correction | scripted_recovery | autonomous_success */
  source: string;
  recorded: string | null;
  has_source: boolean;
  files: Record<string, boolean>;
  episode_uid: string | null;
  task_id: string | null;
  family: string | null;
  round: number | null;
  policy_ckpt: string | null;
  run_id: string | null;
  instruction_sent: string | null;
  takeover_s: number | null;
  failure_mode: string | null;
  objects: string[] | null;
  surface: string | null;
  hw: string | null;
  operator: string | null;
  quality: string;
  notes: string | null;
  /** First frame of the grasp (gripper closed on the object, before the lift). */
  close_frame: number | null;
  /** Frame the task's done condition is confirmed. */
  done_frame: number | null;
  legacy_dataset?: string | null;
  legacy_episode_index: number | null;
};

/** Review fields that can be set on an episode. */
export type EpisodeFields = { quality?: string; failure_mode?: string | null; notes?: string };

export type RegistryTask = {
  id: string;
  number: number | null;
  task: string;
  instruction: string | null;
  description: string | null;
  family: string | null;
  status: string;
  recordable: boolean;
};

export type Registry = {
  source: string | null;
  tasks: RegistryTask[];
  hw: string | null;
  sources: string[];
  qualities: string[];
  failure_modes: string[];
};

/** What the recorder writes with the next episodes (data_recorder owns it). */
export type RecordingContext = {
  task_id: string;
  source: string;
  round: number;
  operator: string;
  objects: string[];
  surface: string;
  /** Session the recorder writes into; "" = today's, made on the first recording. */
  session: string;
  hw: string;
};

export type DatasetSummary = {
  name: string;
  path: string;
  demos_path: string;
  converted: boolean;
  /** A recording session (YYYY-MM-DD_sNN_hwN), as opposed to an older dataset. */
  is_session: boolean;
  /** When the session was closed; closed sessions cannot be edited. */
  closed: string | null;
  hw: string | null;
  fps: number | null;
  /** Ordered by task ID; strings that are not registry tasks follow. */
  tasks: Task[];
  /** Episodes converted without a task, waiting to be tagged. */
  unlabelled: number;
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
  registry: Registry;
  /** null until the data recorder has been seen. */
  context: RecordingContext | null;
  python: string;
  busy: boolean;
  recorder: string;
};

export type Status = {
  dataset?: string;
  closed?: boolean;
  context?: RecordingContext | null;
  busy: boolean;
  recorder: string;
  pending: number;
};

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
  pending?: number;
  session?: string;
};

export type Api = {
  dataset: () => Promise<DatasetInfo>;
  status: () => Promise<Status>;
  job: () => Promise<Job>;
  selectDataset: (name: string) => Promise<Result>;
  /** Tag an episode: by registry ID when the task has one, else by tasks.jsonl index. */
  setTask: (episode: number, task: Task) => Promise<Result>;
  setFields: (episode: number, fields: EpisodeFields) => Promise<Result>;
  setContext: (update: Partial<RecordingContext>) => Promise<Result>;
  newSession: () => Promise<Result>;
  closeSession: (force: boolean) => Promise<Result>;
  remove: (episode: number) => Promise<Result>;
  /** Convert the new bags. One recorded without a task becomes an unlabelled episode. */
  convert: (force: boolean) => Promise<Result>;
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
    setTask: async (episode: number, task: Task) =>
      await post(
        "/api/set_task",
        task.id != undefined ? { episode, task_id: task.id } : { episode, task: task.index },
      ),
    setFields: async (episode: number, fields: EpisodeFields) =>
      await post("/api/set_fields", { episode, fields }),
    setContext: async (update: Partial<RecordingContext>) => await post("/api/context", update),
    newSession: async () => await post("/api/new_session"),
    closeSession: async (force: boolean) => await post("/api/close_session", { force }),
    remove: async (episode: number) => await post("/api/delete", { episode }),
    convert: async (force: boolean) => await post("/api/convert", { force, task: null }),
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

/** Stable key for a task in pickers: its registry ID, else its tasks.jsonl index. */
export const taskKey = (task: Task): string => task.id ?? `#${task.index ?? ""}`;

/** Which episodes the list shows: all, one task (its tasks.jsonl index), or the untagged ones. */
export type TaskFilter = number | "unlabelled" | undefined;

/**
 * Tasks matching what was typed in the task picker, best first. Every word
 * must match: digits match the start of the task's number, anything else is
 * looked for in its ID, instruction, training string and description.
 */
export function searchTasks(tasks: Task[], query: string): Task[] {
  const words = query.trim().toLowerCase().split(/\s+/).filter(Boolean);
  if (words.length === 0) {
    return tasks;
  }
  const scored: { task: Task; exact: boolean }[] = [];
  for (const task of tasks) {
    const number = task.number != undefined ? String(task.number) : "";
    const text = [task.id, task.instruction, task.task, task.description]
      .filter(Boolean)
      .join(" ")
      .toLowerCase();
    const idWords = (task.id ?? "").replace(/_/g, " ");
    const hit = (w: string) =>
      (/^\d+$/.test(w) && number.startsWith(w)) || text.includes(w) || idWords.includes(w);
    if (words.every(hit)) {
      scored.push({ task, exact: words.length === 1 && words[0] === number });
    }
  }
  // "1" picks task 1, not 10-19: an exact number goes first; otherwise keep the ID order.
  return [...scored.filter((s) => s.exact), ...scored.filter((s) => !s.exact)].map((s) => s.task);
}

export const SOURCE_LABELS: Record<string, string> = {
  demo: "demo",
  correction: "correction",
  scripted_recovery: "scripted recovery",
  autonomous_success: "autonomous success",
};
