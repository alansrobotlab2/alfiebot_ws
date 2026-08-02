// Global status + the controls that must always be one click away.

import { html } from '../api.js';
import { Chip } from './ui.js';

export function TopBar({ connected, snapshot, config, held, onEstop, onReset, onReleaseAll }) {
  const estop = (snapshot && snapshot.estop) || { known: false, engaged: false };

  return html`
    <header class="topbar">
      <div class="brand">
        <h1>ServoTool<span>3</span></h1>
        <p>gen2 servo bring-up & diagnostics</p>
      </div>

      <div class="topbar-status">
        <${Chip} tone=${connected ? 'ok' : 'danger'}>
          ${connected ? 'streaming' : 'disconnected'}
        <//>
        <${Chip} tone=${estop.known ? 'muted' : 'warn'}
                 title="command_mux publishes the latched e-stop state">
          mux ${estop.known ? 'up' : 'not seen'}
        <//>
        ${config ? html`<${Chip} tone="muted">source: ${config.source}<//>` : null}
        ${held.length
          ? html`<${Chip} tone="accent">holding ${held.join(', ')}<//>`
          : html`<${Chip} tone="muted">observing<//>`}
      </div>

      <div class="topbar-actions">
        ${held.length
          ? html`<button type="button" class="btn btn-ghost" onClick=${onReleaseAll}>
                   release all
                 </button>`
          : null}
        ${estop.engaged
          ? html`<button type="button" class="btn btn-ok" onClick=${onReset}>reset e-stop</button>`
          : html`<button type="button" class="btn btn-estop" onClick=${onEstop}>E-STOP</button>`}
      </div>
    </header>

    ${estop.engaged
      ? html`<div class="banner banner-danger">
               E-stop latched. The mux is ignoring every command source until it is reset.
             </div>`
      : null}`;
}
