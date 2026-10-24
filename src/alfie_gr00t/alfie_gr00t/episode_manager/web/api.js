// Browser-side plumbing: React bindings, the JSON API, polling, formatting.

const { React, ReactDOM, htm } = window;
export const html = htm.bind(React.createElement);
export { React, ReactDOM };
export const { useState, useEffect, useRef, useCallback, useMemo } = React;

// --- REST -------------------------------------------------------------------

async function get(path) {
  const res = await fetch(path, { cache: 'no-store' });
  return res.json();
}

async function post(path, body) {
  try {
    const res = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {}),
    });
    return await res.json();
  } catch (err) {
    return { ok: false, error: String(err) };
  }
}

export const api = {
  dataset: () => get('/api/dataset'),
  status: () => get('/api/status'),
  job: () => get('/api/job'),
  selectDataset: (name) => post('/api/select_dataset', { name }),
  setTask: (episode, task) => post('/api/set_task', { episode, task }),
  remove: (episode) => post('/api/delete', { episode }),
  convert: (task, force, taskName) => post('/api/convert', { task, force, task_name: taskName }),
  cancel: () => post('/api/cancel'),
};

export const CAMERAS = [
  { key: 'left_wide', label: 'Left wide' },
  { key: 'right_wide', label: 'Right wide' },
  { key: 'left_center', label: 'Left center' },
  { key: 'right_center', label: 'Right center' },
];

export const videoUrl = (episode, camera, rev) =>
  `/api/video?episode=${episode}&camera=${camera}&rev=${rev}`;

// --- Polling ----------------------------------------------------------------

/** Call `fn` every `periodMs` while `active`; the latest `fn` is always used. */
export function useInterval(fn, periodMs, active = true) {
  const saved = useRef(fn);
  saved.current = fn;
  useEffect(() => {
    if (!active) return undefined;
    const id = setInterval(() => saved.current(), periodMs);
    return () => clearInterval(id);
  }, [periodMs, active]);
}

/** A boolean persisted per browser; storage can be unavailable, so never trust it. */
export function useStoredFlag(key, initial) {
  const [value, setValue] = useState(() => {
    try {
      const raw = window.localStorage.getItem(key);
      return raw === null ? initial : raw === '1';
    } catch (err) {
      return initial;
    }
  });
  const update = useCallback((next) => {
    setValue(next);
    try { window.localStorage.setItem(key, next ? '1' : '0'); } catch (err) { /* ignore */ }
  }, [key]);
  return [value, update];
}

// --- Formatting -------------------------------------------------------------

export const episodeName = (index) => `episode_${String(index).padStart(6, '0')}`;

export function fmtDuration(seconds) {
  if (seconds === undefined || seconds === null || Number.isNaN(seconds)) return '--';
  const m = Math.floor(seconds / 60);
  const s = seconds - m * 60;
  return m ? `${m}m ${s.toFixed(1).padStart(4, '0')}s` : `${s.toFixed(1)}s`;
}

// Tasks get a stable hue by index so a label reads at a glance in the list.
const TASK_TONES = 8;
export const taskTone = (index) =>
  index === undefined || index === null ? 'task-none' : `task-${index % TASK_TONES}`;

export const filesOk = (episode) => Object.values(episode.files).every(Boolean);
