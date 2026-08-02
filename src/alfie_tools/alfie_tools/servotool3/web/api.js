// Browser-side plumbing: React bindings, the JSON API, and the SSE stream.

const { React, ReactDOM, htm } = window;
export const html = htm.bind(React.createElement);
export { React, ReactDOM };
export const { useState, useEffect, useRef, useCallback, useMemo } = React;

// --- REST -------------------------------------------------------------------

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
  config: () => fetch('/api/config').then((r) => r.json()),
  take: (subsystem) => post('/api/control', { subsystem, action: 'take' }),
  release: (subsystem) => post('/api/control', { subsystem, action: 'release' }),
  releaseAll: () => post('/api/control', { action: 'release_all' }),
  heartbeat: () => post('/api/heartbeat'),
  torqueOff: (subsystem) => post('/api/torque_off', { subsystem }),
  estop: (action) => post('/api/estop', { action }),
  calibrateBack: () => post('/api/calibrate_back'),
  command: (payload) => post('/api/command', payload),
};

// --- Command coalescing -----------------------------------------------------
// Slider drags fire far faster than there is any point posting. Updates are
// merged per subsystem (and per joint) and flushed on a fixed interval, so a
// drag costs a steady trickle of requests instead of one per pixel.

const FLUSH_MS = 50;
const pending = new Map();
let flushTimer = null;

function flush() {
  flushTimer = null;
  for (const [subsystem, payload] of pending) {
    const joints = payload.joints ? Array.from(payload.joints.values()) : undefined;
    api.command({ ...payload, subsystem, joints });
  }
  pending.clear();
}

function schedule() {
  if (flushTimer === null) flushTimer = setTimeout(flush, FLUSH_MS);
}

/** Queue a joint update. `patch` holds only the fields that changed. */
export function sendJoint(subsystem, index, patch) {
  const entry = pending.get(subsystem) || { joints: new Map() };
  if (!entry.joints) entry.joints = new Map();
  entry.joints.set(index, { ...(entry.joints.get(index) || { index }), ...patch });
  pending.set(subsystem, entry);
  schedule();
}

/** Queue a non-joint update (eyes, back). */
export function sendPatch(subsystem, patch) {
  pending.set(subsystem, { ...(pending.get(subsystem) || {}), ...patch });
  schedule();
}

// --- Live state -------------------------------------------------------------

/**
 * Subscribe to the SSE state stream.
 * Returns [snapshot, connected]; EventSource reconnects on its own.
 */
export function useSnapshot() {
  const [snapshot, setSnapshot] = useState(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const source = new EventSource('/api/stream');
    source.onopen = () => setConnected(true);
    source.onerror = () => setConnected(false);
    source.onmessage = (event) => {
      setConnected(true);
      try {
        setSnapshot(JSON.parse(event.data));
      } catch (err) {
        /* a truncated frame is not worth tearing the stream down for */
      }
    };
    return () => source.close();
  }, []);

  return [snapshot, connected];
}

/**
 * Hold the operator deadman open while anything is under tool control.
 * The server releases every subsystem if these stop arriving.
 */
export function useHeartbeat(active, periodMs) {
  useEffect(() => {
    if (!active) return undefined;
    api.heartbeat();
    const id = setInterval(api.heartbeat, periodMs);
    return () => clearInterval(id);
  }, [active, periodMs]);
}

// --- Formatting -------------------------------------------------------------

export const deg = (rad) => (rad * 180) / Math.PI;
export const fmt = (value, digits = 2) =>
  value === undefined || value === null || Number.isNaN(value)
    ? '--'
    : Number(value).toFixed(digits);
