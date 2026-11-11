// The servo page: target + servo pickers, the angle control for the selected
// servo's driving joint, and its live feedback strip.

import { ReactElement } from "react";

import {
  BusServo,
  JointInfo,
  JointPatch,
  JointState,
  JointTarget,
  Limits,
  ModuleState,
  RAD_PER_DEG,
  TARGET_LABELS,
  deg,
  fmt,
} from "./api";
import { BiBar, Chip, ControlButton, NumberField, Stat, Toggle, Tone } from "./ui";

// --- selector ----------------------------------------------------------------

export function ServoSelector({
  targets,
  busServos,
  subsystem,
  busId,
  module,
  onSelect,
}: {
  targets: readonly string[];
  busServos: Record<string, BusServo[]>;
  subsystem: string;
  busId: number;
  module: ModuleState | undefined;
  onSelect: (subsystem: string, busId: number) => void;
}): ReactElement {
  const servos = busServos[subsystem] ?? [];
  const online = module?.online === true;

  return (
    <div className="selector">
      <label className="field">
        <span>Target</span>
        <select
          value={subsystem}
          onChange={(e) => {
            onSelect(e.target.value, 1);
          }}
        >
          {targets.map((t) => (
            <option key={t} value={t}>
              {TARGET_LABELS[t] ?? t}
            </option>
          ))}
        </select>
      </label>

      <label className="field">
        <span>Servo</span>
        <select
          value={String(busId)}
          onChange={(e) => {
            onSelect(subsystem, parseInt(e.target.value, 10));
          }}
        >
          {servos.map((s) => (
            <option key={s.bus_id} value={String(s.bus_id)}>
              bus {s.bus_id} — {s.label}
            </option>
          ))}
        </select>
      </label>

      <div className="selector-status">
        <Chip tone={online ? "ok" : "muted"}>
          {online ? `${fmt(module.hz, 0)} Hz` : "module offline"}
        </Chip>
        {module?.side != undefined && (
          <Chip
            tone={module.side === "unknown" ? "danger" : "muted"}
            title={`board ${module.board_serial ?? "?"}`}
          >
            side: {module.side}
          </Chip>
        )}
      </div>
    </div>
  );
}

// --- angle column ------------------------------------------------------------

export function AngleColumn({
  servo,
  joint,
  target,
  feedback,
  owned,
  limits,
  estopped,
  moduleOnline,
  onTake,
  onRelease,
  onChange,
  onTorqueOff,
}: {
  servo: BusServo | undefined;
  joint: JointInfo | undefined;
  target: JointTarget | undefined;
  feedback: JointState | undefined;
  owned: boolean;
  limits: Limits;
  estopped: boolean;
  moduleOnline: boolean;
  onTake: () => void;
  onRelease: () => void;
  onChange: (patch: JointPatch) => void;
  onTorqueOff: () => void;
}): ReactElement {
  // No joint drives this servo (shouldn't happen today, but the table allows it).
  if (!joint) {
    return (
      <aside className="angle-col">
        <p className="hint">
          This servo has no logical joint mapped, so it cannot be commanded from here.
        </p>
      </aside>
    );
  }

  const commanding = owned && target != undefined;
  const measured = feedback?.online === true ? feedback.current_location : undefined;
  const value = commanding ? target.target_location : (measured ?? 0);
  const span = joint.upper - joint.lower;
  const pct = (v: number) =>
    span > 0 ? Math.min(100, Math.max(0, ((v - joint.lower) / span) * 100)) : 0;
  const mirrored = servo?.mirrored === true;

  return (
    <aside className="angle-col">
      <div className="angle-head">
        <div className="angle-now">
          <span className="angle-now-value">
            {measured == undefined ? "--" : fmt(deg(measured), 1)}
          </span>
          <span className="angle-now-unit">°</span>
        </div>
        <div className="angle-sub">
          {measured == undefined ? "no joint feedback" : `${fmt(measured, 3)} rad`}
        </div>
        <div className="angle-flags">
          {measured != undefined && feedback?.enabled === true && <Chip tone="ok">torque</Chip>}
          {measured != undefined && feedback?.is_moving === true && (
            <Chip tone="accent">moving</Chip>
          )}
          {mirrored && (
            <Chip tone="warn" title="driven as the negated mirror of its pair">
              mirrored
            </Chip>
          )}
        </div>
      </div>

      <div className="vslider">
        <span className="vslider-limit">{fmt(deg(joint.upper), 0)}°</span>
        <div className="vslider-track">
          <input
            type="range"
            min={joint.lower}
            max={joint.upper}
            step={0.002}
            value={value}
            disabled={!commanding}
            onChange={(e) => {
              onChange({ target_location: parseFloat(e.target.value) });
            }}
          />
          {measured != undefined && (
            <span
              className="vslider-feedback"
              style={{ bottom: `${pct(measured)}%` }}
              title={`measured ${fmt(measured, 3)} rad`}
            />
          )}
        </div>
        <span className="vslider-limit">{fmt(deg(joint.lower), 0)}°</span>
      </div>

      <div className="angle-entry">
        <NumberField
          className="angle-input"
          label="target"
          unit="rad"
          value={value}
          digits={3}
          min={joint.lower}
          max={joint.upper}
          disabled={!commanding}
          onChange={(v) => {
            onChange({ target_location: v });
          }}
        />
        <NumberField
          className="angle-input"
          label="target"
          unit="°"
          value={deg(value)}
          digits={1}
          min={deg(joint.lower)}
          max={deg(joint.upper)}
          disabled={!commanding}
          onChange={(v) => {
            onChange({ target_location: v * RAD_PER_DEG });
          }}
        />
        <button
          type="button"
          className="btn btn-ghost"
          disabled={!commanding}
          title="command 0 rad (servo centre, 2048 counts)"
          onClick={() => {
            onChange({ target_location: 0 });
          }}
        >
          0 rad
        </button>
      </div>

      <div className="angle-actions">
        <Toggle
          checked={commanding ? target.enabled : feedback?.enabled === true}
          disabled={!commanding}
          on="torque on"
          off="torque off"
          onChange={(enabled) => {
            onChange({ enabled });
          }}
        />
        {owned && (
          <button type="button" className="btn btn-warn" onClick={onTorqueOff}>
            torque off all
          </button>
        )}
        <ControlButton
          owned={owned}
          disabled={!moduleOnline || estopped}
          title={estopped ? "reset the e-stop first" : undefined}
          onTake={onTake}
          onRelease={onRelease}
        />
      </div>

      {mirrored && (
        <p className="hint">
          Bus {servo.bus_id} is the derived half of the shoulder-pitch pair. Commanding it moves
          the joint, which drives both servos; the firmware mirrors the angle for this one.
        </p>
      )}

      <details className="angle-limits">
        <summary>motion limits</summary>
        <div className="angle-limits-body">
          <NumberField
            label="speed"
            unit="rad/s"
            min={0}
            max={limits.max_speed}
            value={commanding ? target.target_speed : 0}
            disabled={!commanding}
            onChange={(v) => {
              onChange({ target_speed: v });
            }}
          />
          <NumberField
            label="accel"
            unit="rad/s²"
            min={0}
            max={limits.max_accel}
            value={commanding ? target.target_acceleration : 0}
            disabled={!commanding}
            onChange={(v) => {
              onChange({ target_acceleration: v });
            }}
          />
          <NumberField
            label="torque limit"
            unit="0-1000"
            min={0}
            max={limits.max_torque}
            value={commanding ? target.target_torque : 0}
            disabled={!commanding}
            onChange={(v) => {
              onChange({ target_torque: v });
            }}
          />
          <p className="hint">
            speed 0 means <em>unlimited</em> to the servo; torque limit 0 means it cannot hold at
            all.
          </p>
        </div>
      </details>
    </aside>
  );
}

// --- live feedback -----------------------------------------------------------
//
// Deliberately separate from the register map: this arrives whether or not the
// firmware serves registers, at 25x the rate, and it is per LOGICAL JOINT. For
// the derived shoulder-pitch servo there is no entry at all - the firmware does
// not report it - and saying so beats showing its partner's numbers.

const WARN_TEMP_C = 55;
const CRITICAL_TEMP_C = 65; // matches master_low_status' auto-estop threshold

function tempTone(temp: number | undefined): Tone | undefined {
  if (temp == undefined) {
    return undefined;
  }
  if (temp >= CRITICAL_TEMP_C) {
    return "danger";
  }
  return temp >= WARN_TEMP_C ? "warn" : undefined;
}

export function LiveStrip({
  joint,
  feedback,
  servo,
}: {
  joint: JointInfo | undefined;
  feedback: JointState | undefined;
  servo: BusServo | undefined;
}): ReactElement {
  if (servo?.reported === false) {
    return (
      <section className="live-strip">
        <header>
          <h2>Live feedback</h2>
          <Chip tone="warn">not reported</Chip>
        </header>
        <p className="hint">
          The firmware does not publish this servo&apos;s feedback - it is the derived half of the
          coupled pair, and {joint ? joint.label : "the joint"} carries the primary&apos;s numbers.
          Its real position, load and temperature are only visible in the register map below.
        </p>
      </section>
    );
  }

  const f = feedback?.online === true ? feedback : undefined;
  const load = f?.current_load ?? 0;

  return (
    <section className="live-strip">
      <header>
        <h2>Live feedback</h2>
        <Chip tone={f ? "ok" : "muted"}>{f ? "state topic · 50 Hz" : "no feedback"}</Chip>
        {f?.faults?.map((fault) => (
          <Chip key={fault} tone="danger">
            {fault}
          </Chip>
        ))}
      </header>

      <div className="live-stats">
        <Stat
          label="position"
          value={f?.current_location == undefined ? "--" : fmt(deg(f.current_location), 1)}
          unit="°"
        />
        <Stat
          label="target"
          value={f?.target_location == undefined ? "--" : fmt(deg(f.target_location), 1)}
          unit="°"
        />
        <Stat label="speed" value={fmt(f?.current_speed, 2)} unit=" rad/s" />
        <Stat label="load" value={fmt(f?.current_load, 1)} unit="%" />
        <Stat
          label="temp"
          value={f?.current_temperature ?? "--"}
          unit="°C"
          tone={tempTone(f?.current_temperature)}
        />
        <Stat label="volts" value={fmt(f?.current_voltage, 1)} unit=" V" />
        <Stat label="current" value={fmt(f?.current_current, 0)} unit=" mA" />
        <Stat label="torque lim" value={fmt(f?.target_torque, 0)} />
      </div>

      <BiBar value={load} max={100} tone={Math.abs(load) > 60 ? "warn" : "accent"} />
    </section>
  );
}
