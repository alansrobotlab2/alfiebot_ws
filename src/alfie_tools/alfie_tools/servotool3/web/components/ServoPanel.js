// A servo module (left arm / right arm / head): status header + joint rows.

import { html, fmt } from '../api.js';
import { Chip, Panel } from './ui.js';
import { JointRow } from './JointRow.js';

export function ServoPanel({
  subsystem, label, module, joints, states, draft, owned, limits,
  estopped, onTake, onRelease, onTorqueOff, onJointChange,
}) {
  const online = module && module.online;
  const sideMismatch =
    module && module.side && module.side !== 'unknown' &&
    !subsystem.startsWith(module.side);

  const subtitle = html`
    <${Chip} tone=${online ? 'ok' : 'muted'}>
      ${online ? `${fmt(module.hz, 0)} Hz` : 'no state'}
    <//>
    ${module && module.side
      ? html`<${Chip} tone=${module.side === 'unknown' || sideMismatch ? 'danger' : 'muted'}
                      title=${`board ${module.board_serial || 'unknown'}`}>
               side: ${module.side}
             <//>`
      : null}
    ${sideMismatch
      ? html`<${Chip} tone="danger">board reports the wrong side<//>`
      : null}`;

  const actions = html`
    ${owned
      ? html`<button type="button" class="btn btn-warn" onClick=${onTorqueOff}>torque off</button>`
      : null}
    <button
      type="button"
      class=${`btn ${owned ? 'btn-active' : 'btn-primary'}`}
      disabled=${!online || estopped}
      title=${estopped ? 'reset the e-stop first' : ''}
      onClick=${owned ? onRelease : onTake}>
      ${owned ? 'release control' : 'take control'}
    </button>`;

  return html`
    <${Panel} title=${label} subtitle=${subtitle} actions=${actions}
              tone=${owned ? 'owned' : null}>
      ${owned
        ? html`<p class="panel-note">
                 servotool3 is commanding this module. Releasing stops the commands -
                 the firmware watchdog limps these servos ~500 ms later.
               </p>`
        : null}
      <div class="joint-list">
        ${joints.map((info, i) => html`
          <${JointRow}
            key=${info.key}
            info=${info}
            state=${states[info.key]}
            target=${draft ? draft[i] : null}
            owned=${owned}
            limits=${limits}
            onChange=${(patch) => onJointChange(i, patch)} />`)}
      </div>
    <//>`;
}
