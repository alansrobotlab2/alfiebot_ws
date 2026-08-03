// Back / spine linear actuator. Not a bus servo, but it shares the command_mux
// and it is the other thing you reach for during bring-up.

import { html, fmt } from '../api.js';
import { Chip, NumberField, Panel, SliderRow, Stat } from './ui.js';

// Mirrors the ERROR_* constants in alfie_msgs/msg/BackState.msg.
const ERROR_NAMES = [
  'none', 'invalid command', 'motor fault', 'encoder fault', 'low battery',
  'overtemperature', 'communication', 'motor stall', 'driver fault',
  'motor blocked', 'load exceeded', 'motor runaway',
];
const CALIBRATION_NAMES = ['idle', 'lowering', 'raising', 'complete', 'error'];

export function BackPanel({
  back, draft, owned, limits, estopped, onTake, onRelease, onChange, onCalibrate,
}) {
  const online = back && back.online;
  const target = owned && draft ? draft : null;
  const position = target ? target.position : (online ? back.current_position : 0);
  const faulted = online && back.error_code !== 0;

  const actions = html`
    <button type="button" class="btn btn-ghost" disabled=${!online || estopped}
            onClick=${onCalibrate}>calibrate</button>
    <button type="button" class=${`btn ${owned ? 'btn-active' : 'btn-primary'}`}
            disabled=${!online || estopped}
            onClick=${owned ? onRelease : onTake}>
      ${owned ? 'release control' : 'take control'}
    </button>`;

  const subtitle = html`
    <${Chip} tone=${online ? 'ok' : 'muted'}>${online ? 'online' : 'no state'}<//>
    ${online && !back.is_calibrated ? html`<${Chip} tone="warn">uncalibrated<//>` : null}
    ${faulted
      ? html`<${Chip} tone="danger">
               ${ERROR_NAMES[back.error_code] || `error ${back.error_code}`}
               ${back.fault_latched ? ' (latched)' : ''}
             <//>`
      : null}
    ${online && back.calibration_status
      ? html`<${Chip} tone="accent">${CALIBRATION_NAMES[back.calibration_status]}<//>`
      : null}`;

  return html`
    <${Panel} title="Back / spine" subtitle=${subtitle} actions=${actions}
              tone=${owned ? 'owned' : null}>
      <div class="back-body">
        <div class="joint-telemetry">
          <${Stat} label="position" value=${online ? fmt(back.current_position, 3) : '--'} unit=" m" />
          <${Stat} label="commanded" value=${online ? fmt(back.command_position, 3) : '--'} unit=" m" />
          <${Stat} label="velocity" value=${online ? fmt(back.current_velocity, 3) : '--'} unit=" m/s" />
          <${Stat} label="pwm" value=${online ? back.pwm_output : '--'} />
          <${Stat} label="board" value=${online ? back.board_temp : '--'} unit="°C" />
          <${Stat} label="stalls" value=${online ? back.stall_count : '--'}
                   tone=${online && back.stall_count > 0 ? 'warn' : null} />
        </div>

        <${SliderRow}
          label="height"
          value=${position}
          feedback=${online ? back.current_position : null}
          min=${limits.min_position} max=${limits.max_position} step=${0.001}
          unit=" m" digits=${3}
          disabled=${!owned}
          onChange=${(v) => onChange({ position: v })} />

        <div class="back-motion">
          <${NumberField} label="velocity" unit="m/s" min=${0} max=${limits.max_velocity}
            step=${0.005} value=${target ? target.velocity : 0} disabled=${!owned}
            onChange=${(v) => onChange({ velocity: v })} />
          <${NumberField} label="accel" unit="m/s²" min=${0} max=${limits.max_acceleration}
            step=${0.01} value=${target ? target.acceleration : 0} disabled=${!owned}
            onChange=${(v) => onChange({ acceleration: v })} />
        </div>

        ${online && back.stall_count > 0
          ? html`<p class="hint">
                   last stall at ${fmt(back.stall_position, 3)} m. Two stalls at the same
                   position means something is physically in the way - send the back
                   <em>down</em> to release it.
                 </p>`
          : null}
      </div>
    <//>`;
}
