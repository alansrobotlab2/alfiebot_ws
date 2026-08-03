// Small shared presentational pieces.

import { html, fmt } from '../api.js';

export function Chip({ tone = 'muted', children, title }) {
  return html`<span class=${`chip chip-${tone}`} title=${title || ''}>${children}</span>`;
}

export function Stat({ label, value, unit, tone }) {
  return html`
    <div class=${`stat${tone ? ` stat-${tone}` : ''}`}>
      <span class="stat-label">${label}</span>
      <span class="stat-value">${value}${unit ? html`<i>${unit}</i>` : null}</span>
    </div>`;
}

export function Toggle({ checked, disabled, onChange, on = 'ON', off = 'OFF' }) {
  return html`
    <button
      type="button"
      class=${`toggle${checked ? ' toggle-on' : ''}`}
      disabled=${disabled}
      aria-pressed=${checked ? 'true' : 'false'}
      onClick=${() => onChange(!checked)}>
      <span class="toggle-track"><span class="toggle-knob"></span></span>
      <span class="toggle-text">${checked ? on : off}</span>
    </button>`;
}

/**
 * Slider + numeric readout. `feedback` (optional) draws a second marker for the
 * measured value, so target-vs-actual error is visible without reading numbers.
 */
export function SliderRow({
  label, value, min, max, step, unit, digits = 3,
  feedback, disabled, onChange, onCommit,
}) {
  const span = max - min;
  const pct = (v) => (span > 0 ? Math.min(100, Math.max(0, ((v - min) / span) * 100)) : 0);
  return html`
    <div class=${`slider-row${disabled ? ' is-disabled' : ''}`}>
      <label class="slider-label">${label}</label>
      <div class="slider-track-wrap">
        <input
          type="range"
          min=${min} max=${max} step=${step}
          value=${value}
          disabled=${disabled}
          onInput=${(e) => onChange(parseFloat(e.target.value))}
          onChange=${(e) => (onCommit || onChange)(parseFloat(e.target.value))} />
        ${feedback === undefined || feedback === null
          ? null
          : html`<span class="slider-feedback" style=${{ left: `${pct(feedback)}%` }}
                       title=${`measured ${fmt(feedback, digits)}${unit || ''}`}></span>`}
      </div>
      <output class="slider-value">${fmt(value, digits)}<i>${unit || ''}</i></output>
    </div>`;
}

export function NumberField({ label, value, min, max, step, unit, disabled, onChange }) {
  return html`
    <label class=${`number-field${disabled ? ' is-disabled' : ''}`}>
      <span>${label}${unit ? html` <i>${unit}</i>` : null}</span>
      <input
        type="number"
        min=${min} max=${max} step=${step}
        value=${value}
        disabled=${disabled}
        onChange=${(e) => {
          const parsed = parseFloat(e.target.value);
          if (!Number.isNaN(parsed)) onChange(parsed);
        }} />
    </label>`;
}

/** Signed bar for values that swing either side of zero (load, speed). */
export function BiBar({ value, max, tone = 'accent' }) {
  const ratio = Math.max(-1, Math.min(1, max > 0 ? value / max : 0));
  const width = Math.abs(ratio) * 50;
  const left = ratio >= 0 ? 50 : 50 - width;
  return html`
    <div class="bibar">
      <span class=${`bibar-fill bibar-${tone}`} style=${{ left: `${left}%`, width: `${width}%` }}></span>
      <span class="bibar-zero"></span>
    </div>`;
}

export function Panel({ title, subtitle, actions, children, tone }) {
  return html`
    <section class=${`panel${tone ? ` panel-${tone}` : ''}`}>
      <header class="panel-head">
        <div>
          <h2>${title}</h2>
          ${subtitle ? html`<p class="panel-sub">${subtitle}</p>` : null}
        </div>
        <div class="panel-actions">${actions}</div>
      </header>
      ${children}
    </section>`;
}
