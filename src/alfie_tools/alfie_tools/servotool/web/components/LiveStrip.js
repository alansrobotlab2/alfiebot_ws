// Live joint feedback, from the module's 50 Hz state topic.
//
// Deliberately separate from the register map: this arrives whether or not the
// firmware serves registers, at 25x the rate, and it is per LOGICAL JOINT. For
// the derived shoulder-pitch servo there is no entry at all - the firmware does
// not report it - and saying so is better than showing its partner's numbers
// under its name.

import { html, deg, fmt } from '../api.js';
import { BiBar, Chip, Stat } from './ui.js';

const WARN_TEMP_C = 55;
const CRITICAL_TEMP_C = 65;   // matches master_low_status' auto-estop threshold

function tempTone(temp) {
  if (temp >= CRITICAL_TEMP_C) return 'danger';
  if (temp >= WARN_TEMP_C) return 'warn';
  return null;
}

export function LiveStrip({ joint, feedback, servo }) {
  const live = feedback && feedback.online;

  if (servo && servo.reported === false) {
    return html`
      <section class="live-strip">
        <header>
          <h2>Live feedback</h2>
          <${Chip} tone="warn">not reported<//>
        </header>
        <p class="hint">
          The firmware does not publish this servo's feedback - it is the derived
          half of the coupled pair, and ${joint ? joint.label : 'the joint'} carries
          the primary's numbers. Its real position, load and temperature are only
          visible in the register map below.
        </p>
      </section>`;
  }

  return html`
    <section class="live-strip">
      <header>
        <h2>Live feedback</h2>
        <${Chip} tone=${live ? 'ok' : 'muted'}>
          ${live ? 'state topic · 50 Hz' : 'no feedback'}
        <//>
        ${live && feedback.faults && feedback.faults.length
          ? feedback.faults.map((f) => html`<${Chip} key=${f} tone="danger">${f}<//>`)
          : null}
      </header>

      <div class="live-stats">
        <${Stat} label="position" value=${live ? fmt(deg(feedback.current_location), 1) : '--'} unit="°" />
        <${Stat} label="target" value=${live ? fmt(deg(feedback.target_location), 1) : '--'} unit="°" />
        <${Stat} label="speed" value=${live ? fmt(feedback.current_speed, 2) : '--'} unit=" rad/s" />
        <${Stat} label="load" value=${live ? fmt(feedback.current_load, 1) : '--'} unit="%" />
        <${Stat} label="temp" value=${live ? feedback.current_temperature : '--'} unit="°C"
                 tone=${live ? tempTone(feedback.current_temperature) : null} />
        <${Stat} label="volts" value=${live ? fmt(feedback.current_voltage, 1) : '--'} unit=" V" />
        <${Stat} label="current" value=${live ? fmt(feedback.current_current, 0) : '--'} unit=" mA" />
        <${Stat} label="torque lim" value=${live ? fmt(feedback.target_torque, 0) : '--'} />
      </div>

      <${BiBar} value=${live ? feedback.current_load : 0} max=${100}
                tone=${live && Math.abs(feedback.current_load) > 60 ? 'warn' : 'accent'} />
    </section>`;
}
