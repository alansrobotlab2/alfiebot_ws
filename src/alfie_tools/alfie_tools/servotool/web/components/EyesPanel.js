// Eye LEDs. Their own command_mux channel, so they are held independently of
// head-servo motion.

import { html } from '../api.js';
import { Chip, Panel, SliderRow } from './ui.js';

const PRESETS = [
  ['off', 0],
  ['dim', 200],
  ['half', 2048],
  ['full', 4095],
];

export function EyesPanel({ eyes, draft, owned, max, estopped, onTake, onRelease, onChange }) {
  const online = eyes && eyes.online;
  const measured = (eyes && eyes.state) || [0, 0];
  const value = owned && draft ? draft : measured;

  const actions = html`
    <button
      type="button"
      class=${`btn ${owned ? 'btn-active' : 'btn-primary'}`}
      disabled=${!online || estopped}
      onClick=${owned ? onRelease : onTake}>
      ${owned ? 'release control' : 'take control'}
    </button>`;

  const set = (index, v) => {
    const next = [value[0], value[1]];
    next[index] = Math.round(v);
    onChange(next);
  };

  return html`
    <${Panel}
      title="Eyes"
      subtitle=${html`<${Chip} tone=${online ? 'ok' : 'muted'}>
                        ${online ? 'head module online' : 'no head state'}
                      <//>`}
      actions=${actions}
      tone=${owned ? 'owned' : null}>
      <div class="eyes-body">
        <${SliderRow} label="left" value=${value[0]} min=${0} max=${max} step=${1}
                      digits=${0} disabled=${!owned}
                      onChange=${(v) => set(0, v)} />
        <${SliderRow} label="right" value=${value[1]} min=${0} max=${max} step=${1}
                      digits=${0} disabled=${!owned}
                      onChange=${(v) => set(1, v)} />
        <div class="preset-row">
          ${PRESETS.map(([name, pwm]) => html`
            <button key=${name} type="button" class="btn btn-ghost" disabled=${!owned}
                    onClick=${() => onChange([pwm, pwm])}>${name}</button>`)}
        </div>
        <p class="hint">
          Raw PWM duty 0-${max} at 100 Hz. Measured now: ${measured[0]} / ${measured[1]}.
        </p>
      </div>
    <//>`;
}
