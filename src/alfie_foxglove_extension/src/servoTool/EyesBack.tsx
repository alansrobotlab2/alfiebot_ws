// Eyes and back: not bus servos, but they share the command_mux and they are
// the other things you reach for during bring-up.

import { ReactElement } from "react";

import { BackLimits, BackState, BackTarget, fmt } from "./api";
import { Chip, ControlButton, NumberField, Panel, SliderRow, Stat } from "./ui";

// --- eyes --------------------------------------------------------------------

const EYE_PRESETS: [string, number][] = [
  ["off", 0],
  ["dim", 200],
  ["half", 2048],
  ["full", 4095],
];

export function EyesPanel({
  eyes,
  draft,
  owned,
  max,
  estopped,
  onTake,
  onRelease,
  onChange,
}: {
  eyes: { online: boolean; state: number[] };
  draft: number[] | undefined;
  owned: boolean;
  max: number;
  estopped: boolean;
  onTake: () => void;
  onRelease: () => void;
  onChange: (pwm: number[]) => void;
}): ReactElement {
  const measured = eyes.state;
  const value = owned && draft ? draft : measured;
  const left = value[0] ?? 0;
  const right = value[1] ?? 0;

  return (
    <Panel
      title="Eyes"
      owned={owned}
      subtitle={
        <Chip tone={eyes.online ? "ok" : "muted"}>
          {eyes.online ? "head module online" : "no head state"}
        </Chip>
      }
      actions={
        <ControlButton
          owned={owned}
          disabled={!eyes.online || estopped}
          onTake={onTake}
          onRelease={onRelease}
        />
      }
    >
      <div className="eyes-body">
        <SliderRow
          label="left"
          value={left}
          min={0}
          max={max}
          step={1}
          digits={0}
          disabled={!owned}
          onChange={(v) => {
            onChange([Math.round(v), right]);
          }}
        />
        <SliderRow
          label="right"
          value={right}
          min={0}
          max={max}
          step={1}
          digits={0}
          disabled={!owned}
          onChange={(v) => {
            onChange([left, Math.round(v)]);
          }}
        />
        <div className="preset-row">
          {EYE_PRESETS.map(([name, pwm]) => (
            <button
              key={name}
              type="button"
              className="btn btn-ghost"
              disabled={!owned}
              onClick={() => {
                onChange([pwm, pwm]);
              }}
            >
              {name}
            </button>
          ))}
        </div>
        <p className="hint">
          Raw PWM duty 0-{max} at 100 Hz. Measured now: {measured[0] ?? "--"} /{" "}
          {measured[1] ?? "--"}.
        </p>
      </div>
    </Panel>
  );
}

// --- back --------------------------------------------------------------------

// Mirrors the ERROR_* constants in alfie_msgs/msg/BackState.msg.
const ERROR_NAMES = [
  "none",
  "invalid command",
  "motor fault",
  "encoder fault",
  "low battery",
  "overtemperature",
  "communication",
  "motor stall",
  "driver fault",
  "motor blocked",
  "load exceeded",
  "motor runaway",
];
const CALIBRATION_NAMES = ["idle", "lowering", "raising", "complete", "error"];

export function BackPanel({
  back,
  draft,
  owned,
  limits,
  estopped,
  onTake,
  onRelease,
  onChange,
  onCalibrate,
}: {
  back: BackState;
  draft: BackTarget | undefined;
  owned: boolean;
  limits: BackLimits;
  estopped: boolean;
  onTake: () => void;
  onRelease: () => void;
  onChange: (patch: Partial<BackTarget>) => void;
  onCalibrate: () => void;
}): ReactElement {
  const online = back.online;
  // A stale module still reports its last values; show none rather than old ones.
  const live: BackState = online ? back : { online: false };
  const target = owned ? draft : undefined;
  const position = target ? target.position : (live.current_position ?? 0);
  const errorCode = live.error_code ?? 0;
  const stalls = live.stall_count ?? 0;
  const calibration = live.calibration_status ?? 0;

  return (
    <Panel
      title="Back / spine"
      owned={owned}
      subtitle={
        <>
          <Chip tone={online ? "ok" : "muted"}>{online ? "online" : "no state"}</Chip>
          {online && back.is_calibrated === false && <Chip tone="warn">uncalibrated</Chip>}
          {online && errorCode !== 0 && (
            <Chip tone="danger">
              {ERROR_NAMES[errorCode] ?? `error ${errorCode}`}
              {back.fault_latched === true ? " (latched)" : ""}
            </Chip>
          )}
          {online && calibration !== 0 && (
            <Chip tone="accent">{CALIBRATION_NAMES[calibration] ?? calibration}</Chip>
          )}
        </>
      }
      actions={
        <>
          <button
            type="button"
            className="btn btn-ghost"
            disabled={!online || estopped}
            onClick={onCalibrate}
          >
            calibrate
          </button>
          <ControlButton
            owned={owned}
            disabled={!online || estopped}
            onTake={onTake}
            onRelease={onRelease}
          />
        </>
      }
    >
      <div className="back-body">
        <div className="joint-telemetry">
          <Stat label="position" value={fmt(live.current_position, 3)} unit=" m" />
          <Stat label="commanded" value={fmt(live.command_position, 3)} unit=" m" />
          <Stat label="velocity" value={fmt(live.current_velocity, 3)} unit=" m/s" />
          <Stat label="pwm" value={live.pwm_output ?? "--"} />
          <Stat label="board" value={live.board_temp ?? "--"} unit="°C" />
          <Stat
            label="stalls"
            value={live.stall_count ?? "--"}
            tone={stalls > 0 ? "warn" : undefined}
          />
        </div>

        <SliderRow
          label="height"
          value={position}
          feedback={live.current_position}
          min={limits.min_position}
          max={limits.max_position}
          step={0.001}
          unit=" m"
          digits={3}
          disabled={!owned}
          onChange={(v) => {
            onChange({ position: v });
          }}
        />

        <div className="back-motion">
          <NumberField
            label="velocity"
            unit="m/s"
            min={0}
            max={limits.max_velocity}
            value={target ? target.velocity : 0}
            disabled={!owned}
            onChange={(v) => {
              onChange({ velocity: v });
            }}
          />
          <NumberField
            label="accel"
            unit="m/s²"
            min={0}
            max={limits.max_acceleration}
            value={target ? target.acceleration : 0}
            disabled={!owned}
            onChange={(v) => {
              onChange({ acceleration: v });
            }}
          />
        </div>

        {online && stalls > 0 && (
          <p className="hint">
            last stall at {fmt(live.stall_position, 3)} m. Two stalls at the same position means
            something is physically in the way - send the back <em>down</em> to release it.
          </p>
        )}
      </div>
    </Panel>
  );
}
