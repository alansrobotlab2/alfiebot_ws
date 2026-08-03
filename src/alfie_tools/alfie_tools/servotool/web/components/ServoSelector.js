// Target + servo pickers. Everything on the servo page follows this selection.

import { html, fmt } from '../api.js';
import { Chip } from './ui.js';

const TARGET_LABELS = {
  left_arm: 'Left arm',
  right_arm: 'Right arm',
  head: 'Head',
};

export function ServoSelector({ targets, busServos, selection, module, onSelect }) {
  const servos = busServos[selection.subsystem] || [];
  const online = module && module.online;

  return html`
    <div class="selector">
      <label class="field">
        <span>Target</span>
        <select
          value=${selection.subsystem}
          onChange=${(e) => onSelect(e.target.value, 1)}>
          ${targets.map((t) => html`
            <option key=${t} value=${t}>${TARGET_LABELS[t] || t}</option>`)}
        </select>
      </label>

      <label class="field">
        <span>Servo</span>
        <select
          value=${String(selection.bus_id)}
          onChange=${(e) => onSelect(selection.subsystem, parseInt(e.target.value, 10))}>
          ${servos.map((s) => html`
            <option key=${s.bus_id} value=${String(s.bus_id)}>
              bus ${s.bus_id} — ${s.label}
            </option>`)}
        </select>
      </label>

      <div class="selector-status">
        <${Chip} tone=${online ? 'ok' : 'muted'}>
          ${online ? `${fmt(module.hz, 0)} Hz` : 'module offline'}
        <//>
        ${module && module.side
          ? html`<${Chip} tone=${module.side === 'unknown' ? 'danger' : 'muted'}
                          title=${`board ${module.board_serial || '?'}`}>
                   side: ${module.side}
                 <//>`
          : null}
      </div>
    </div>`;
}
