// One logical joint: telemetry on the left, command controls on the right.

import { html, useState, deg, fmt } from '../api.js';
import { BiBar, Chip, NumberField, SliderRow, Stat, Toggle } from './ui.js';

const WARN_TEMP_C = 55;
const CRITICAL_TEMP_C = 65;   // matches master_low_status' auto-estop threshold

function tempTone(temp) {
  if (temp >= CRITICAL_TEMP_C) return 'danger';
  if (temp >= WARN_TEMP_C) return 'warn';
  return null;
}

export function JointRow({ info, state, target, owned, limits, onChange }) {
  const [expanded, setExpanded] = useState(false);
  const live = state && state.online;
  const commanding = owned && !!target;

  const position = commanding ? target.target_location : (live ? state.current_location : 0);
  const busLabel = info.bus_ids.length > 1
    ? `bus ${info.bus_ids.join('+')} (coupled pair)`
    : `bus ${info.bus_ids[0]}`;

  return html`
    <article class=${`joint${live ? '' : ' is-offline'}${state && state.critical ? ' is-fault' : ''}`}>
      <div class="joint-head">
        <div class="joint-id">
          <h3>${info.label}</h3>
          <span class="joint-meta" title=${info.urdf_joint}>${busLabel}</span>
        </div>
        <div class="joint-flags">
          ${!live ? html`<${Chip} tone="muted">offline<//>` : null}
          ${live && state.enabled ? html`<${Chip} tone="ok">torque<//>` : null}
          ${live && state.is_moving ? html`<${Chip} tone="accent">moving<//>` : null}
          ${(state && state.faults ? state.faults : []).map(
            (f) => html`<${Chip} key=${f} tone="danger">${f}<//>`)}
        </div>
      </div>

      <div class="joint-telemetry">
        <${Stat} label="position" value=${live ? fmt(deg(state.current_location), 1) : '--'} unit="°" />
        <${Stat} label="target" value=${live ? fmt(deg(state.target_location), 1) : '--'} unit="°" />
        <${Stat} label="speed" value=${live ? fmt(state.current_speed, 2) : '--'} unit=" rad/s" />
        <${Stat} label="load" value=${live ? fmt(state.current_load, 1) : '--'} unit="%" />
        <${Stat} label="temp" value=${live ? state.current_temperature : '--'} unit="°C"
                 tone=${live ? tempTone(state.current_temperature) : null} />
        <${Stat} label="volts" value=${live ? fmt(state.current_voltage, 1) : '--'} unit=" V" />
        <${Stat} label="current" value=${live ? fmt(state.current_current, 0) : '--'} unit=" mA" />
      </div>

      <div class="joint-load-bar">
        <${BiBar} value=${live ? state.current_load : 0} max=${100}
                  tone=${live && Math.abs(state.current_load) > 60 ? 'warn' : 'accent'} />
      </div>

      <div class="joint-controls">
        <${Toggle}
          checked=${commanding ? target.enabled : (live && state.enabled)}
          disabled=${!commanding}
          on="torque on" off="torque off"
          onChange=${(v) => onChange({ enabled: v })} />

        <${SliderRow}
          label="angle"
          value=${position}
          feedback=${live ? state.current_location : null}
          min=${info.lower} max=${info.upper} step=${0.002}
          unit=" rad" digits=${3}
          disabled=${!commanding}
          onChange=${(v) => onChange({ target_location: v })} />

        <button type="button" class="link-button" onClick=${() => setExpanded(!expanded)}>
          ${expanded ? 'hide motion limits' : 'motion limits'}
        </button>
      </div>

      ${expanded
        ? html`
          <div class="joint-advanced">
            <${NumberField} label="speed" unit="rad/s" min=${0} max=${limits.max_speed} step=${0.1}
              value=${commanding ? target.target_speed : (live ? state.target_speed : 0)}
              disabled=${!commanding}
              onChange=${(v) => onChange({ target_speed: v })} />
            <${NumberField} label="accel" unit="rad/s²" min=${0} max=${limits.max_accel} step=${0.5}
              value=${commanding ? target.target_acceleration : (live ? state.target_acceleration : 0)}
              disabled=${!commanding}
              onChange=${(v) => onChange({ target_acceleration: v })} />
            <${NumberField} label="torque limit" unit="0-1000" min=${0} max=${limits.max_torque} step=${10}
              value=${commanding ? target.target_torque : (live ? state.target_torque : 0)}
              disabled=${!commanding}
              onChange=${(v) => onChange({ target_torque: v })} />
            <p class="hint">
              speed 0 means <em>unlimited</em> to the servo, and torque limit 0 means the
              joint cannot hold at all.
            </p>
          </div>`
        : null}
    </article>`;
}
