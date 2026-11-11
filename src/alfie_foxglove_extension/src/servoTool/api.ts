// REST + SSE client and shared helpers for the servo tool panel.
//
// The backend is the servotool node in alfie_tools
// (alfie_tools/servotool/server/http_server.py). This panel is a second front
// end for it; the node's own web UI keeps working alongside. Safety lives on
// the node, not here: control ownership, the operator deadman, and clamping to
// URDF limits and max_speed all happen server-side.

import { useEffect, useState } from "react";

// --- /api/config -------------------------------------------------------------

export type JointInfo = {
  key: string;
  subsystem: string;
  index: number;
  label: string;
  urdf_joint: string;
  bus_ids: number[];
  lower: number;
  upper: number;
};

export type BusServo = {
  bus_id: number;
  label: string;
  joint_index: number | null;
  mirrored: boolean;
  reported: boolean;
};

export type RegisterKind = "eeprom" | "sram" | "feedback";

export type Register = {
  field: string;
  address: number;
  label: string;
  group: string;
  kind: RegisterKind;
  unit: string;
  scale: number | null;
  scaled_unit: string | null;
  sign_bit: number | null;
  note: string | null;
  writable: boolean;
  vmin: number;
  vmax: number;
};

export type BackLimits = {
  min_position: number;
  max_position: number;
  max_velocity: number;
  max_acceleration: number;
};

export type Limits = {
  max_speed: number;
  max_accel: number;
  max_torque: number;
  eye_pwm_max: number;
  back: BackLimits;
};

export type Config = {
  source: string;
  prefix: string;
  forward_rate_hz: number;
  control_timeout: number;
  joints: JointInfo[];
  bus_servos: Record<string, BusServo[]>;
  register_groups: { key: string; label: string }[];
  registers: Register[];
  memory_poll_hz: number;
  memory_writable: boolean;
  limits: Limits;
};

// --- /api/stream snapshot ----------------------------------------------------

export type ModuleState = {
  online: boolean;
  hz: number;
  age: number | null;
  board_serial?: string;
  side?: string;
};

/** Joint feedback. Everything but key/online is absent while the module is offline. */
export type JointState = {
  key: string;
  online: boolean;
  enabled?: boolean;
  target_location?: number;
  target_speed?: number;
  target_acceleration?: number;
  target_torque?: number;
  current_location?: number;
  current_speed?: number;
  current_load?: number;
  current_temperature?: number;
  current_voltage?: number;
  current_current?: number;
  servo_status?: number;
  faults?: string[];
  critical?: boolean;
  is_moving?: boolean;
};

export type BackState = {
  online: boolean;
  board_temp?: number;
  limit_switch?: boolean;
  command_position?: number;
  current_position?: number;
  current_velocity?: number;
  is_moving?: boolean;
  is_calibrated?: boolean;
  calibration_status?: number;
  error_code?: number;
  fault_latched?: boolean;
  stall_count?: number;
  stall_position?: number;
  pwm_output?: number;
};

/** What servotool commands for one joint while it holds control. */
export type JointTarget = {
  enabled: boolean;
  target_location: number;
  target_speed: number;
  target_acceleration: number;
  target_torque: number;
};

export type BackTarget = { position: number; velocity: number; acceleration: number };

export type Control = {
  owned: boolean;
  joints: JointTarget[];
  eye_pwm: number[];
  back: BackTarget;
};

export type Memory = {
  subsystem: string;
  bus_id: number;
  label: string;
  joint_index: number | null;
  mirrored: boolean;
  reported: boolean;
  values: Record<string, number> | null;
  age: number | null;
  error: string | null;
  locked: boolean | null;
  faults: string[];
};

export type Snapshot = {
  time: number;
  estop: { known: boolean; engaged: boolean };
  modules: Record<string, ModuleState>;
  joints: JointState[];
  eyes: { online: boolean; state: number[] };
  back: BackState;
  control: Record<string, Control>;
  memory: Memory;
  heartbeat_age: number | null;
};

// --- REST --------------------------------------------------------------------

export type Result = { ok: boolean; error?: string; message?: string };

/** The fields of one joint a command may change. */
export type JointPatch = Partial<JointTarget>;

export type Api = {
  base: string;
  config: () => Promise<Config>;
  select: (subsystem: string, busId: number) => Promise<Result>;
  writeRegister: (subsystem: string, busId: number, address: number, value: number) => Promise<Result>;
  setLock: (subsystem: string, busId: number, locked: boolean) => Promise<Result>;
  take: (subsystem: string) => Promise<Result>;
  release: (subsystem: string) => Promise<Result>;
  releaseAll: () => Promise<Result>;
  heartbeat: () => Promise<Result>;
  torqueOff: (subsystem: string) => Promise<Result>;
  estop: (action: "engage" | "reset") => Promise<Result>;
  calibrateBack: () => Promise<Result>;
  /** Queue a joint update; merged per joint and flushed every FLUSH_MS. */
  sendJoint: (subsystem: string, index: number, patch: JointPatch) => void;
  /** Queue a non-joint update (eyes, back). */
  sendPatch: (subsystem: string, patch: Record<string, unknown>) => void;
  /** Drop queued commands that have not gone out yet. */
  dispose: () => void;
};

// Slider drags fire far faster than there is any point posting. Updates are
// merged per subsystem (and per joint) and flushed on a fixed interval, so a
// drag costs a steady trickle of requests instead of one per pixel.
const FLUSH_MS = 50;

type Pending = { joints?: Map<number, JointPatch & { index: number }>; [key: string]: unknown };

/** A client for the servotool node at `base` (e.g. http://alfiebot.local:7870). */
export function makeApi(base: string): Api {
  const root = base.replace(/\/+$/, "");

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

  const pending = new Map<string, Pending>();
  let flushTimer: ReturnType<typeof setTimeout> | undefined;

  function flush() {
    flushTimer = undefined;
    for (const [subsystem, entry] of pending) {
      const { joints, ...rest } = entry;
      void post("/api/command", {
        ...rest,
        subsystem,
        joints: joints ? Array.from(joints.values()) : undefined,
      });
    }
    pending.clear();
  }

  function schedule() {
    flushTimer ??= setTimeout(flush, FLUSH_MS);
  }

  return {
    base: root,
    config: async () => {
      const res = await fetch(`${root}/api/config`, { cache: "no-store" });
      if (!res.ok) {
        throw new Error(`${res.status} ${res.statusText}`);
      }
      return (await res.json()) as Config;
    },
    select: async (subsystem, busId) => await post("/api/select", { subsystem, bus_id: busId }),
    writeRegister: async (subsystem, busId, address, value) =>
      await post("/api/register", { subsystem, bus_id: busId, address, value }),
    setLock: async (subsystem, busId, locked) =>
      await post("/api/lock", { subsystem, bus_id: busId, locked }),
    take: async (subsystem) => await post("/api/control", { subsystem, action: "take" }),
    release: async (subsystem) => await post("/api/control", { subsystem, action: "release" }),
    releaseAll: async () => await post("/api/control", { action: "release_all" }),
    heartbeat: async () => await post("/api/heartbeat"),
    torqueOff: async (subsystem) => await post("/api/torque_off", { subsystem }),
    estop: async (action) => await post("/api/estop", { action }),
    calibrateBack: async () => await post("/api/calibrate_back"),
    sendJoint: (subsystem, index, patch) => {
      const entry = pending.get(subsystem) ?? {};
      const joints = entry.joints ?? new Map();
      joints.set(index, { ...(joints.get(index) ?? { index }), ...patch });
      pending.set(subsystem, { ...entry, joints });
      schedule();
    },
    sendPatch: (subsystem, patch) => {
      pending.set(subsystem, { ...(pending.get(subsystem) ?? {}), ...patch });
      schedule();
    },
    dispose: () => {
      if (flushTimer != undefined) {
        clearTimeout(flushTimer);
        flushTimer = undefined;
      }
      pending.clear();
    },
  };
}

// --- Live state --------------------------------------------------------------

/**
 * Subscribe to the node's SSE state stream. EventSource reconnects on its own;
 * `connected` drops while it does.
 */
export function useSnapshot(api: Api): { snapshot: Snapshot | undefined; connected: boolean } {
  const [snapshot, setSnapshot] = useState<Snapshot | undefined>();
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    setSnapshot(undefined);
    setConnected(false);
    const source = new EventSource(`${api.base}/api/stream`);
    source.onopen = () => {
      setConnected(true);
    };
    source.onerror = () => {
      setConnected(false);
    };
    source.onmessage = (event: MessageEvent<string>) => {
      setConnected(true);
      try {
        setSnapshot(JSON.parse(event.data) as Snapshot);
      } catch {
        // a truncated frame is not worth tearing the stream down for
      }
    };
    return () => {
      source.close();
    };
  }, [api]);

  return { snapshot, connected };
}

/**
 * Hold the operator deadman open while anything is under tool control. The
 * node releases every subsystem if these stop arriving.
 */
export function useHeartbeat(api: Api, active: boolean, periodMs: number): void {
  useEffect(() => {
    if (!active) {
      return undefined;
    }
    void api.heartbeat();
    const id = setInterval(() => void api.heartbeat(), periodMs);
    return () => {
      clearInterval(id);
    };
  }, [api, active, periodMs]);
}

// --- Formatting --------------------------------------------------------------

export const deg = (rad: number): number => (rad * 180) / Math.PI;
export const RAD_PER_DEG = Math.PI / 180;

export function fmt(value: number | null | undefined, digits = 2): string {
  return value == undefined || Number.isNaN(value) ? "--" : value.toFixed(digits);
}

export const SERVO_SUBSYSTEMS = ["left_arm", "right_arm", "head"] as const;

export const TARGET_LABELS: Record<string, string> = {
  left_arm: "Left arm",
  right_arm: "Right arm",
  head: "Head",
};
