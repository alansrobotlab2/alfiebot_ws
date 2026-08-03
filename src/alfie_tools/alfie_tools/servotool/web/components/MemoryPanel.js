// Right-hand side: the selected servo's register map, grouped and editable.
//
// Three things this view has to keep honest:
//
//  * Which values are live. EEPROM registers only change when something writes
//    them; SRAM and feedback registers change under you constantly. They are
//    read in the same transaction, so nothing in the raw data distinguishes
//    them - the `kind` from /api/config does.
//  * Whether EEPROM is writable. Register 0x37 is 0 when UNLOCKED. A tool that
//    shows values but not the lock invites someone to conclude a write silently
//    failed, which is exactly what the servo does when locked.
//  * That an edit landed. The firmware re-reads after every write and the row
//    redraws from that read-back, so a value that snaps back was rejected by
//    the servo, not by the browser.

import { html, useEffect, useState, fmt } from '../api.js';
import { Chip } from './ui.js';

const KIND_LABEL = {
  eeprom: 'persistent',
  sram: 'live · command',
  feedback: 'live · measured',
};

function formatValue(register, raw) {
  if (raw === undefined || raw === null) return '--';
  if (!register.scale) return String(raw);
  const scaled = raw * register.scale;
  const digits = register.scale >= 1 ? 0 : 1;
  return `${raw} (${fmt(scaled, digits)} ${register.scaled_unit})`;
}

/** Editable register value. Commits on Enter or blur, reverts on Escape. */
function RegisterField({ register, value, disabled, onWrite }) {
  const [text, setText] = useState('');
  const [editing, setEditing] = useState(false);
  const [busy, setBusy] = useState(false);

  useEffect(() => {
    if (!editing) setText(value === undefined || value === null ? '' : String(value));
  }, [value, editing]);

  const commit = async () => {
    setEditing(false);
    const parsed = parseInt(text, 10);
    if (Number.isNaN(parsed) || parsed === value) {
      setText(String(value));
      return;
    }
    setBusy(true);
    await onWrite(parsed);
    setBusy(false);
  };

  return html`
    <input
      class=${`reg-input${busy ? ' is-busy' : ''}`}
      type="text"
      inputMode="numeric"
      value=${text}
      disabled=${disabled || busy}
      title=${`${register.vmin}..${register.vmax}`}
      onFocus=${() => setEditing(true)}
      onInput=${(e) => setText(e.target.value)}
      onBlur=${commit}
      onKeyDown=${(e) => {
        if (e.key === 'Enter') e.target.blur();
        if (e.key === 'Escape') { setEditing(false); setText(String(value)); e.target.blur(); }
      }} />`;
}

function RegisterRow({ register, value, editable, onWrite }) {
  const scaled = register.scale && value !== undefined && value !== null
    ? `${fmt(value * register.scale, register.scale >= 1 ? 0 : 1)} ${register.scaled_unit}`
    : null;

  return html`
    <div class=${`reg reg-${register.kind}${editable ? ' reg-editable' : ''}`}
         title=${register.note || ''}>
      <span class="reg-addr">0x${register.address.toString(16).toUpperCase().padStart(2, '0')}</span>
      <span class="reg-label">
        ${register.label}
        ${register.note ? html`<i class="reg-note-dot" aria-hidden="true">·</i>` : null}
      </span>
      ${editable
        ? html`<${RegisterField} register=${register} value=${value}
                                 disabled=${false} onWrite=${onWrite} />`
        : html`<span class="reg-value">${formatValue(register, value)}</span>`}
      <span class="reg-unit">${editable && scaled ? scaled : register.unit}</span>
    </div>`;
}

function LockControl({ locked, busy, onToggle }) {
  if (locked === null || locked === undefined) {
    return html`<${Chip} tone="muted">lock unknown<//>`;
  }
  return html`
    <${Chip} tone=${locked ? 'muted' : 'warn'}
             title=${locked
               ? 'register 0x37 = 1: the servo rejects EEPROM writes'
               : 'register 0x37 = 0: EEPROM writes are accepted'}>
      EEPROM ${locked ? 'locked' : 'UNLOCKED'}
    <//>
    <button type="button" class=${`btn ${locked ? 'btn-primary' : 'btn-warn'}`}
            disabled=${busy}
            title=${locked
              ? 'clear the write lock so EEPROM registers can be edited'
              : 'restore the write lock'}
            onClick=${() => onToggle(!locked)}>
      ${locked ? 'unlock' : 'lock'}
    </button>`;
}

export function MemoryPanel({
  memory, groups, registers, writable, faults, torqueOn, onWrite, onLock,
}) {
  const [lockBusy, setLockBusy] = useState(false);
  const values = (memory && memory.values) || null;
  const fresh = values && memory.age !== null && memory.age < 3;
  const locked = memory ? memory.locked : null;

  // Mirrors the firmware's own guards, so a row that cannot be written looks
  // that way instead of failing on submit.
  const canWrite = (register) => {
    if (!writable || !register.writable || !values) return false;
    if (register.kind !== 'eeprom') return false;
    return locked === false && !torqueOn;
  };

  const byGroup = {};
  for (const register of registers) {
    (byGroup[register.group] = byGroup[register.group] || []).push(register);
  }

  const blockedReason = !values ? null
    : torqueOn ? 'Torque is on for this servo - EEPROM writes are refused while it is holding.'
    : locked !== false ? 'EEPROM is locked. Unlock to edit persistent registers.'
    : null;

  return html`
    <section class="memory">
      <header class="memory-head">
        <div>
          <h2>Register map</h2>
          <p class="memory-sub">
            bus ${memory ? memory.bus_id : '--'} · ${memory ? memory.label : ''}
          </p>
        </div>
        <div class="memory-status">
          ${memory && memory.error
            ? html`<${Chip} tone="danger">${memory.error}<//>`
            : html`<${Chip} tone=${fresh ? 'ok' : 'muted'}>
                     ${values ? (fresh ? `read ${fmt(memory.age, 1)}s ago` : 'stale') : 'no read yet'}
                   <//>`}
          <${LockControl} locked=${locked} busy=${lockBusy}
                          onToggle=${async (next) => {
                            setLockBusy(true);
                            await onLock(next);
                            setLockBusy(false);
                          }} />
        </div>
      </header>

      ${faults && faults.length
        ? html`<div class="memory-faults">
                 ${faults.map((f) => html`<${Chip} key=${f} tone="danger">${f}<//>`)}
               </div>`
        : null}

      ${blockedReason
        ? html`<p class="hint memory-note">${blockedReason}</p>`
        : null}

      ${!values && memory && !memory.error
        ? html`<p class="hint memory-empty">Waiting for the first register read…</p>`
        : null}

      <div class="memory-groups">
        ${groups.map((group) => {
          const rows = byGroup[group.key] || [];
          if (!rows.length) return null;
          const kind = rows[0].kind;
          return html`
            <article class=${`reg-group reg-group-${kind}`} key=${group.key}>
              <header>
                <h3>${group.label}</h3>
                <${Chip} tone=${kind === 'eeprom' ? 'muted' : 'accent'}>
                  ${KIND_LABEL[kind] || kind}
                <//>
              </header>
              <div class="reg-list">
                ${rows.map((register) => html`
                  <${RegisterRow} key=${register.field} register=${register}
                                  value=${values ? values[register.field] : null}
                                  editable=${canWrite(register)}
                                  onWrite=${(v) => onWrite(register, v)} />`)}
              </div>
            </article>`;
        })}
      </div>

      <p class="hint memory-foot">
        EEPROM has a finite write endurance, so each field writes only on Enter or
        blur - never per keystroke. Every write is verified by read-back on the
        module; a value that snaps back was refused by the servo.
      </p>
    </section>`;
}
