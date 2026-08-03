// Left-hand column: the angle control for the selected servo's driving joint.

import { html, useEffect, useState, deg, fmt } from '../api.js';
import { Chip, NumberField, Toggle } from './ui.js';

const RAD_PER_DEG = Math.PI / 180;

/** Text input that only pushes a value up when it parses, so a half-typed
 *  "-0." doesn't get committed as 0 and swing the joint. */
function AngleInput({ label, unit, value, digits, disabled, onCommit }) {
  const [text, setText] = useState('');
  const [editing, setEditing] = useState(false);

  useEffect(() => {
    if (!editing) setText(Number(value).toFixed(digits));
  }, [value, editing, digits]);

  const commit = () => {
    setEditing(false);
    const parsed = parseFloat(text);
    if (!Number.isNaN(parsed)) onCommit(parsed);
  };

  return html`
    <label class=${`angle-input${disabled ? ' is-disabled' : ''}`}>
      <span>${label}<i>${unit}</i></span>
      <input
        type="text"
        inputMode="decimal"
        value=${text}
        disabled=${disabled}
        onFocus=${() => setEditing(true)}
        onInput=${(e) => setText(e.target.value)}
        onBlur=${commit}
        onKeyDown=${(e) => { if (e.key === 'Enter') e.target.blur(); }} />
    </label>`;
}

export function AngleColumn({
  servo, joint, target, feedback, owned, limits, estopped, moduleOnline,
  onTake, onRelease, onChange, onTorqueOff,
}) {
  const commanding = owned && !!target && !!joint;
  const live = feedback && feedback.online;

  // No joint drives this servo (shouldn't happen today, but the table allows it).
  if (!joint) {
    return html`
      <aside class="angle-col">
        <p class="hint">This servo has no logical joint mapped, so it cannot be
        commanded from here.</p>
      </aside>`;
  }

  const value = commanding ? target.target_location : (live ? feedback.current_location : 0);
  const span = joint.upper - joint.lower;
  const pct = (v) => (span > 0
    ? Math.min(100, Math.max(0, ((v - joint.lower) / span) * 100))
    : 0);

  return html`
    <aside class="angle-col">
      <div class="angle-head">
        <div class="angle-now">
          <span class="angle-now-value">${live ? fmt(deg(feedback.current_location), 1) : '--'}</span>
          <span class="angle-now-unit">°</span>
        </div>
        <div class="angle-sub">
          ${live ? `${fmt(feedback.current_location, 3)} rad` : 'no joint feedback'}
        </div>
        <div class="angle-flags">
          ${live && feedback.enabled ? html`<${Chip} tone="ok">torque<//>` : null}
          ${live && feedback.is_moving ? html`<${Chip} tone="accent">moving<//>` : null}
          ${servo.mirrored ? html`<${Chip} tone="warn" title="driven as the negated mirror of its pair">mirrored<//>` : null}
        </div>
      </div>

      <div class="vslider">
        <span class="vslider-limit">${fmt(deg(joint.upper), 0)}°</span>
        <div class="vslider-track">
          <input
            type="range"
            min=${joint.lower} max=${joint.upper} step=${0.002}
            value=${value}
            disabled=${!commanding}
            onInput=${(e) => onChange({ target_location: parseFloat(e.target.value) })} />
          ${live
            ? html`<span class="vslider-feedback"
                         style=${{ bottom: `${pct(feedback.current_location)}%` }}
                         title=${`measured ${fmt(feedback.current_location, 3)} rad`}></span>`
            : null}
        </div>
        <span class="vslider-limit">${fmt(deg(joint.lower), 0)}°</span>
      </div>

      <div class="angle-entry">
        <${AngleInput} label="target" unit="rad" value=${value} digits=${3}
                       disabled=${!commanding}
                       onCommit=${(v) => onChange({ target_location: v })} />
        <${AngleInput} label="target" unit="°" value=${deg(value)} digits=${1}
                       disabled=${!commanding}
                       onCommit=${(v) => onChange({ target_location: v * RAD_PER_DEG })} />
        <button type="button" class="btn btn-ghost" disabled=${!commanding}
                title="command 0 rad (servo centre, 2048 counts)"
                onClick=${() => onChange({ target_location: 0 })}>0 rad</button>
      </div>

      <div class="angle-actions">
        <${Toggle}
          checked=${commanding ? target.enabled : (live && feedback.enabled)}
          disabled=${!commanding}
          on="torque on" off="torque off"
          onChange=${(v) => onChange({ enabled: v })} />
        ${owned
          ? html`<button type="button" class="btn btn-warn" onClick=${onTorqueOff}>
                   torque off all
                 </button>`
          : null}
        <button
          type="button"
          class=${`btn ${owned ? 'btn-active' : 'btn-primary'}`}
          disabled=${!moduleOnline || estopped}
          title=${estopped ? 'reset the e-stop first' : ''}
          onClick=${owned ? onRelease : onTake}>
          ${owned ? 'release control' : 'take control'}
        </button>
      </div>

      ${servo.mirrored
        ? html`<p class="hint">
                 Bus ${servo.bus_id} is the derived half of the shoulder-pitch pair.
                 Commanding it moves the joint, which drives both servos; the
                 firmware mirrors the angle for this one.
               </p>`
        : null}

      <details class="angle-limits">
        <summary>motion limits</summary>
        <div class="angle-limits-body">
          <${NumberField} label="speed" unit="rad/s" min=${0} max=${limits.max_speed}
            step=${0.1} value=${commanding ? target.target_speed : 0}
            disabled=${!commanding}
            onChange=${(v) => onChange({ target_speed: v })} />
          <${NumberField} label="accel" unit="rad/s²" min=${0} max=${limits.max_accel}
            step=${0.5} value=${commanding ? target.target_acceleration : 0}
            disabled=${!commanding}
            onChange=${(v) => onChange({ target_acceleration: v })} />
          <${NumberField} label="torque limit" unit="0-1000" min=${0} max=${limits.max_torque}
            step=${10} value=${commanding ? target.target_torque : 0}
            disabled=${!commanding}
            onChange=${(v) => onChange({ target_torque: v })} />
          <p class="hint">
            speed 0 means <em>unlimited</em> to the servo; torque limit 0 means it
            cannot hold at all.
          </p>
        </div>
      </details>
    </aside>`;
}
