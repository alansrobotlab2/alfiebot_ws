// Servo tool UI, a port of the servotool node's web UI (alfie_tools/servotool/web).
//
// The page shows ONE physical servo at a time, picked with the target + servo
// dropdowns: angle control down the left, that servo's register map on the
// right. Register access is addressed per physical servo (7 per arm), while
// commands are addressed per logical joint (6 per arm) - the selected servo's
// `joint_index` bridges the two.
//
// State model: the SSE snapshot is the truth for everything measured and for
// whether a subsystem is under tool control. What the operator is *asking for*
// lives in a local draft, seeded from the server the moment control is taken.
// Without that split every slider would fight the 10 Hz snapshot echo.

import { ReactElement, useCallback, useEffect, useMemo, useState } from "react";
import { createPortal } from "react-dom";

import { BackPanel, EyesPanel } from "./EyesBack";
import { MemoryPanel } from "./MemoryPanel";
import {
  Api,
  BackTarget,
  Config,
  JointInfo,
  JointPatch,
  JointState,
  JointTarget,
  Result,
  SERVO_SUBSYSTEMS,
  Snapshot,
  useHeartbeat,
  useSnapshot,
} from "./api";
import { AngleColumn, LiveStrip, ServoSelector } from "./servo";
import { Chip } from "./ui";

const HEARTBEAT_MS = 400;
const CONFIG_RETRY_MS = 3000;

type Draft = {
  joints: Record<string, JointTarget[]>;
  eyes?: number[];
  back?: BackTarget;
};

const EMPTY_DRAFT: Draft = { joints: {} };

const isServoSubsystem = (name: string): boolean =>
  (SERVO_SUBSYSTEMS as readonly string[]).includes(name);

/** Seed a draft for newly-held subsystems; drop drafts for released ones. */
function syncDraft(prev: Draft, snapshot: Snapshot): Draft {
  let next = prev;
  const edit = () => {
    if (next === prev) {
      next = { ...prev, joints: { ...prev.joints } };
    }
    return next;
  };
  for (const [name, state] of Object.entries(snapshot.control)) {
    if (isServoSubsystem(name)) {
      const has = next.joints[name] != undefined;
      if (state.owned && !has) {
        edit().joints[name] = state.joints.map((j) => ({ ...j }));
      } else if (!state.owned && has) {
        delete edit().joints[name];
      }
    } else if (name === "eyes") {
      if (state.owned && next.eyes == undefined) {
        edit().eyes = [...state.eye_pwm];
      } else if (!state.owned && next.eyes != undefined) {
        edit().eyes = undefined;
      }
    } else if (name === "back") {
      if (state.owned && next.back == undefined) {
        edit().back = { ...state.back };
      } else if (!state.owned && next.back != undefined) {
        edit().back = undefined;
      }
    }
  }
  return next;
}

function TopBar({
  connected,
  snapshot,
  config,
  serverUrl,
  held,
  onEstop,
  onReset,
  onReleaseAll,
}: {
  connected: boolean;
  snapshot: Snapshot;
  config: Config;
  serverUrl: string;
  held: string[];
  onEstop: () => void;
  onReset: () => void;
  onReleaseAll: () => void;
}): ReactElement {
  const { estop } = snapshot;
  return (
    <>
      <header className="topbar">
        <div className="brand">
          <h1>
            Alfie <span>Servo Tool</span>
          </h1>
          <p className="mono" title={serverUrl}>
            {serverUrl}
          </p>
        </div>

        <div className="topbar-status">
          <Chip tone={connected ? "ok" : "danger"}>{connected ? "streaming" : "disconnected"}</Chip>
          <Chip
            tone={estop.known ? "muted" : "warn"}
            title="command_mux publishes the latched e-stop state"
          >
            mux {estop.known ? "up" : "not seen"}
          </Chip>
          <Chip tone="muted">source: {config.source}</Chip>
          {held.length > 0 ? (
            <Chip tone="accent">holding {held.join(", ")}</Chip>
          ) : (
            <Chip tone="muted">observing</Chip>
          )}
        </div>

        <div className="topbar-actions">
          {held.length > 0 && (
            <button type="button" className="btn btn-ghost" onClick={onReleaseAll}>
              release all
            </button>
          )}
          {estop.engaged ? (
            <button type="button" className="btn btn-ok" onClick={onReset}>
              reset e-stop
            </button>
          ) : (
            <button type="button" className="btn btn-estop" onClick={onEstop}>
              E-STOP
            </button>
          )}
        </div>
      </header>

      {estop.engaged && (
        <div className="banner banner-danger">
          E-stop latched. The mux is ignoring every command source until it is reset.
        </div>
      )}
    </>
  );
}

export function ServoTool({
  api,
  serverUrl,
  overlay,
}: {
  api: Api;
  serverUrl: string;
  /** Lives outside the scroller, so the toast stays put while the panel scrolls. */
  overlay: HTMLElement;
}): ReactElement {
  const [config, setConfig] = useState<Config | undefined>();
  const [configError, setConfigError] = useState<string | undefined>();
  const { snapshot, connected } = useSnapshot(api);
  const [draft, setDraft] = useState<Draft>(EMPTY_DRAFT);
  const [toast, setToast] = useState<string | undefined>();

  // The node may not be running yet (it is a bench tool, not part of bringup),
  // so keep asking for the config until it answers.
  useEffect(() => {
    let cancelled = false;
    let timer: ReturnType<typeof setTimeout> | undefined;
    const load = () => {
      api
        .config()
        .then((c) => {
          if (!cancelled) {
            setConfig(c);
            setConfigError(undefined);
          }
        })
        .catch((err: unknown) => {
          if (!cancelled) {
            setConfigError(String(err));
            timer = setTimeout(load, CONFIG_RETRY_MS);
          }
        });
    };
    load();
    return () => {
      cancelled = true;
      clearTimeout(timer);
      api.dispose();
    };
  }, [api]);

  useEffect(() => {
    if (toast == undefined) {
      return undefined;
    }
    const id = setTimeout(() => {
      setToast(undefined);
    }, 4000);
    return () => {
      clearTimeout(id);
    };
  }, [toast]);

  const held = useMemo(
    () =>
      Object.entries(snapshot?.control ?? {})
        .filter(([, c]) => c.owned)
        .map(([name]) => name),
    [snapshot],
  );

  useHeartbeat(api, held.length > 0, HEARTBEAT_MS);

  // Seed a draft when control is taken; drop it when control goes away (which
  // includes the node's deadman release, so a dropped connection leaves no
  // stale sliders behind).
  useEffect(() => {
    if (snapshot) {
      setDraft((prev) => syncDraft(prev, snapshot));
    }
  }, [snapshot]);

  const jointStates = useMemo(() => {
    const map = new Map<string, JointState>();
    for (const joint of snapshot?.joints ?? []) {
      map.set(joint.key, joint);
    }
    return map;
  }, [snapshot]);

  const jointsByKey = useMemo(() => {
    const map = new Map<string, JointInfo>();
    for (const info of config?.joints ?? []) {
      map.set(info.key, info);
    }
    return map;
  }, [config]);

  // --- actions ---------------------------------------------------------------

  const report = useCallback(async (promise: Promise<Result>, what: string): Promise<Result> => {
    const res = await promise;
    if (!res.ok) {
      setToast(`${what}: ${res.error ?? "failed"}`);
    } else if (res.message != undefined && res.message !== "") {
      setToast(res.message);
    }
    return res;
  }, []);

  const onJointChange = (subsystem: string, index: number, patch: JointPatch) => {
    setDraft((prev) => {
      const joints = prev.joints[subsystem];
      const current = joints?.[index];
      if (!joints || !current) {
        return prev;
      }
      const next = joints.slice();
      next[index] = { ...current, ...patch };
      return { ...prev, joints: { ...prev.joints, [subsystem]: next } };
    });
    api.sendJoint(subsystem, index, patch);
  };

  const onEyesChange = (value: number[]) => {
    setDraft((prev) => ({ ...prev, eyes: value }));
    api.sendPatch("eyes", { eye_pwm: value });
  };

  const onBackChange = (patch: Partial<BackTarget>) => {
    if (!draft.back) {
      return;
    }
    const merged = { ...draft.back, ...patch };
    setDraft((prev) => ({ ...prev, back: merged }));
    api.sendPatch("back", { back: merged });
  };

  const onTorqueOff = (subsystem: string) => {
    setDraft((prev) => {
      const joints = prev.joints[subsystem];
      if (!joints) {
        return prev;
      }
      return {
        ...prev,
        joints: { ...prev.joints, [subsystem]: joints.map((j) => ({ ...j, enabled: false })) },
      };
    });
    void report(api.torqueOff(subsystem), "torque off");
  };

  if (!config || !snapshot) {
    return (
      <div className="boot">
        <div className="boot-card">
          <p>
            Connecting to servotool at <code>{serverUrl}</code>…
          </p>
          {configError != undefined && (
            <>
              <p className="hint">
                No answer ({configError}). servotool is a bench tool and is not started by
                bringup; run <code>ros2 run alfie_tools servotool</code> on the robot. Retrying.
              </p>
              <p className="hint">
                From the Foxglove web app the panel goes through the nginx proxy (
                <code>https://&lt;robot&gt;:8082/servotool</code>); accept the robot&apos;s
                certificate once at <code>https://&lt;robot&gt;:8082/health</code>. If the node
                logs &quot;refusing cross-origin requests&quot;, add that origin to its{" "}
                <code>allowed_origins</code> parameter.
              </p>
            </>
          )}
        </div>
      </div>
    );
  }

  const estopped = snapshot.estop.engaged;
  const { modules, memory, control } = snapshot;
  const subsystem = memory.subsystem;

  const servo = config.bus_servos[subsystem]?.find((s) => s.bus_id === memory.bus_id);
  const jointIndex = servo?.joint_index ?? undefined;
  const joint = jointIndex == undefined ? undefined : jointsByKey.get(`${subsystem}.${jointIndex}`);
  const feedback = joint ? jointStates.get(joint.key) : undefined;
  const owned = control[subsystem]?.owned === true;
  // The firmware refuses EEPROM writes while the servo holds torque. Read it
  // from the register map when we have one (it covers the derived servo too),
  // and fall back to joint feedback.
  const torqueSwitch = memory.values?.torqueswitch;
  const torqueOn = torqueSwitch != undefined ? torqueSwitch !== 0 : feedback?.enabled === true;

  return (
    <div className="app">
      <TopBar
        connected={connected}
        snapshot={snapshot}
        config={config}
        serverUrl={serverUrl}
        held={held}
        onEstop={() => void report(api.estop("engage"), "e-stop")}
        onReset={() => void report(api.estop("reset"), "e-stop reset")}
        onReleaseAll={() => void report(api.releaseAll(), "release all")}
      />

      {!connected && (
        <div className="banner banner-warn">
          Lost the state stream - retrying. Held subsystems are released after{" "}
          {config.control_timeout}s without a heartbeat.
        </div>
      )}

      <ServoSelector
        targets={SERVO_SUBSYSTEMS}
        busServos={config.bus_servos}
        subsystem={subsystem}
        busId={memory.bus_id}
        module={modules[subsystem]}
        onSelect={(target, busId) => void report(api.select(target, busId), "select servo")}
      />

      <main className="servo-page">
        <AngleColumn
          servo={servo}
          joint={joint}
          target={
            owned && jointIndex != undefined ? draft.joints[subsystem]?.[jointIndex] : undefined
          }
          feedback={feedback}
          owned={owned}
          limits={config.limits}
          estopped={estopped}
          moduleOnline={modules[subsystem]?.online === true}
          onTake={() => void report(api.take(subsystem), `take ${subsystem}`)}
          onRelease={() => void report(api.release(subsystem), `release ${subsystem}`)}
          onTorqueOff={() => {
            onTorqueOff(subsystem);
          }}
          onChange={(patch) => {
            if (jointIndex != undefined) {
              onJointChange(subsystem, jointIndex, patch);
            }
          }}
        />

        <div className="servo-detail">
          <LiveStrip joint={joint} feedback={feedback} servo={servo} />
          <MemoryPanel
            memory={memory}
            groups={config.register_groups}
            registers={config.registers}
            writable={config.memory_writable}
            torqueOn={torqueOn}
            onWrite={async (register, value) => {
              await report(
                api.writeRegister(subsystem, memory.bus_id, register.address, value),
                register.label,
              );
            }}
            onLock={async (locked) => {
              await report(
                api.setLock(subsystem, memory.bus_id, locked),
                locked ? "lock" : "unlock",
              );
            }}
          />
        </div>
      </main>

      <section className="aux">
        <EyesPanel
          eyes={snapshot.eyes}
          draft={draft.eyes}
          owned={control.eyes?.owned === true}
          max={config.limits.eye_pwm_max}
          estopped={estopped}
          onTake={() => void report(api.take("eyes"), "take eyes")}
          onRelease={() => void report(api.release("eyes"), "release eyes")}
          onChange={onEyesChange}
        />
        <BackPanel
          back={snapshot.back}
          draft={draft.back}
          owned={control.back?.owned === true}
          limits={config.limits.back}
          estopped={estopped}
          onTake={() => void report(api.take("back"), "take back")}
          onRelease={() => void report(api.release("back"), "release back")}
          onChange={onBackChange}
          onCalibrate={() => void report(api.calibrateBack(), "calibrate back")}
        />
      </section>

      <footer className="foot">
        Commands go to{" "}
        <code>
          {config.prefix}cmd/&lt;subsystem&gt;/{config.source}
        </code>{" "}
        at {config.forward_rate_hz} Hz through command_mux; registers are read from the module
        firmware at {config.memory_poll_hz} Hz.
      </footer>

      {toast != undefined && createPortal(<div className="toast">{toast}</div>, overlay)}
    </div>
  );
}
