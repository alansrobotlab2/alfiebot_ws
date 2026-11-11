// Small shared presentational pieces.

import { ReactElement, ReactNode, useEffect, useState } from "react";

import { fmt } from "./api";

export type Tone = "muted" | "ok" | "warn" | "danger" | "accent";

export function Chip({
  tone = "muted",
  title,
  children,
}: {
  tone?: Tone;
  title?: string;
  children: ReactNode;
}): ReactElement {
  return (
    <span className={`chip chip-${tone}`} title={title}>
      {children}
    </span>
  );
}

export function Stat({
  label,
  value,
  unit,
  tone,
}: {
  label: string;
  value: ReactNode;
  unit?: string;
  tone?: Tone;
}): ReactElement {
  return (
    <div className={`stat${tone ? ` stat-${tone}` : ""}`}>
      <span className="stat-label">{label}</span>
      <span className="stat-value">
        {value}
        {unit ? <i>{unit}</i> : undefined}
      </span>
    </div>
  );
}

export function Toggle({
  checked,
  disabled,
  on = "ON",
  off = "OFF",
  onChange,
}: {
  checked: boolean;
  disabled?: boolean;
  on?: string;
  off?: string;
  onChange: (checked: boolean) => void;
}): ReactElement {
  return (
    <button
      type="button"
      className={`toggle${checked ? " toggle-on" : ""}`}
      disabled={disabled}
      aria-pressed={checked}
      onClick={() => {
        onChange(!checked);
      }}
    >
      <span className="toggle-track">
        <span className="toggle-knob" />
      </span>
      <span className="toggle-text">{checked ? on : off}</span>
    </button>
  );
}

/**
 * Slider + numeric readout. `feedback` (optional) draws a second marker for the
 * measured value, so target-vs-actual error is visible without reading numbers.
 */
export function SliderRow({
  label,
  value,
  min,
  max,
  step,
  unit,
  digits = 3,
  feedback,
  disabled,
  onChange,
}: {
  label: string;
  value: number;
  min: number;
  max: number;
  step: number;
  unit?: string;
  digits?: number;
  feedback?: number;
  disabled?: boolean;
  onChange: (value: number) => void;
}): ReactElement {
  const span = max - min;
  const pct = (v: number) => (span > 0 ? Math.min(100, Math.max(0, ((v - min) / span) * 100)) : 0);
  return (
    <div className={`slider-row${disabled === true ? " is-disabled" : ""}`}>
      <span className="slider-label">{label}</span>
      <div className="slider-track-wrap">
        <input
          type="range"
          min={min}
          max={max}
          step={step}
          value={value}
          disabled={disabled}
          onChange={(e) => {
            onChange(parseFloat(e.target.value));
          }}
        />
        {feedback == undefined ? undefined : (
          <span
            className="slider-feedback"
            style={{ left: `${pct(feedback)}%` }}
            title={`measured ${fmt(feedback, digits)}${unit ?? ""}`}
          />
        )}
      </div>
      <output className="slider-value">
        {fmt(value, digits)}
        <i>{unit ?? ""}</i>
      </output>
    </div>
  );
}

/**
 * Number input that only pushes a value up when it parses, on Enter or blur, so
 * a half-typed "-0." is never committed as 0 and swings a joint.
 */
export function NumberField({
  label,
  value,
  min,
  max,
  unit,
  digits,
  disabled,
  className = "number-field",
  onChange,
}: {
  label: string;
  value: number;
  min?: number;
  max?: number;
  unit?: string;
  digits?: number;
  disabled?: boolean;
  className?: string;
  onChange: (value: number) => void;
}): ReactElement {
  const shown = digits == undefined ? String(value) : value.toFixed(digits);
  const [text, setText] = useState(shown);
  const [editing, setEditing] = useState(false);

  useEffect(() => {
    if (!editing) {
      setText(shown);
    }
  }, [shown, editing]);

  const commit = () => {
    setEditing(false);
    const parsed = parseFloat(text);
    if (Number.isNaN(parsed)) {
      setText(shown);
      return;
    }
    let clamped = parsed;
    if (min != undefined) {
      clamped = Math.max(min, clamped);
    }
    if (max != undefined) {
      clamped = Math.min(max, clamped);
    }
    onChange(clamped);
  };

  return (
    <label className={`${className}${disabled === true ? " is-disabled" : ""}`}>
      <span>
        {label}
        {unit ? <i> {unit}</i> : undefined}
      </span>
      <input
        type="text"
        inputMode="decimal"
        value={text}
        disabled={disabled}
        title={min != undefined && max != undefined ? `${min}..${max}` : undefined}
        onFocus={() => {
          setEditing(true);
        }}
        onChange={(e) => {
          setText(e.target.value);
        }}
        onBlur={commit}
        onKeyDown={(e) => {
          if (e.key === "Enter") {
            e.currentTarget.blur();
          } else if (e.key === "Escape") {
            setEditing(false);
            setText(shown);
          }
        }}
      />
    </label>
  );
}

/** Signed bar for values that swing either side of zero (load, speed). */
export function BiBar({
  value,
  max,
  tone = "accent",
}: {
  value: number;
  max: number;
  tone?: "accent" | "warn";
}): ReactElement {
  const ratio = Math.max(-1, Math.min(1, max > 0 ? value / max : 0));
  const width = Math.abs(ratio) * 50;
  const left = ratio >= 0 ? 50 : 50 - width;
  return (
    <div className="bibar">
      <span className={`bibar-fill bibar-${tone}`} style={{ left: `${left}%`, width: `${width}%` }} />
      <span className="bibar-zero" />
    </div>
  );
}

export function Panel({
  title,
  subtitle,
  actions,
  owned,
  children,
}: {
  title: string;
  subtitle?: ReactNode;
  actions?: ReactNode;
  owned?: boolean;
  children: ReactNode;
}): ReactElement {
  return (
    <section className={`panel${owned === true ? " panel-owned" : ""}`}>
      <header className="panel-head">
        <div>
          <h2>{title}</h2>
          {subtitle != undefined && <div className="panel-sub">{subtitle}</div>}
        </div>
        <div className="panel-actions">{actions}</div>
      </header>
      {children}
    </section>
  );
}

/** Take / release button shared by every commandable subsystem. */
export function ControlButton({
  owned,
  disabled,
  title,
  onTake,
  onRelease,
}: {
  owned: boolean;
  disabled: boolean;
  title?: string;
  onTake: () => void;
  onRelease: () => void;
}): ReactElement {
  return (
    <button
      type="button"
      className={`btn ${owned ? "btn-active" : "btn-primary"}`}
      disabled={disabled}
      title={title}
      onClick={owned ? onRelease : onTake}
    >
      {owned ? "release control" : "take control"}
    </button>
  );
}
