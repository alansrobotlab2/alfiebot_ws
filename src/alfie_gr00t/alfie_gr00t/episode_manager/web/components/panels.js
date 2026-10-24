// Top bar, episode editor, details table, converter log and the confirm dialog.

import { html, useEffect, useRef, episodeName, fmtDuration, taskTone, CAMERAS } from '../api.js';

function RecorderPill({ status }) {
  if (!status) return html`<span class="pill dim" title="No message on the recorder status topic yet">recorder: not seen</span>`;
  if (status.startsWith('RECORDING')) {
    return html`<span class="pill rec"><i class="rec-dot"></i>recording ${status.split(':').slice(1).join(':')}</span>`;
  }
  return html`<span class="pill">recorder idle</span>`;
}

export const NEW_TASK = -1;

export function TopBar({ info, summary, busy, recorder, pending, convertTask, onConvertTask,
                         newTaskName, onNewTaskName, onDataset, onConvert }) {
  const naming = convertTask === NEW_TASK;
  return html`
    <header class="topbar">
      <div class="brand">
        <h1>Alfie <span>Episode Manager</span></h1>
        <p class="mono" title=${summary.path}>${summary.path}</p>
      </div>
      <label class="field">
        <span>Dataset</span>
        <select value=${summary.name} disabled=${busy} onChange=${(e) => onDataset(e.target.value)}>
          ${info.datasets.map((d) => html`<option key=${d} value=${d}>${d}</option>`)}
        </select>
      </label>
      <div class="stats">
        <span class="pill"><b>${summary.episodes.length}</b> episodes</span>
        <span class="pill"><b>${summary.total_frames.toLocaleString()}</b> frames</span>
        <span class="pill"><b>${summary.fps ?? '--'}</b> fps</span>
        <${RecorderPill} status=${recorder} />
      </div>
      <div class="convert">
        <label class="field">
          <span>Label new bags as</span>
          <select value=${convertTask} disabled=${busy}
                  onChange=${(e) => onConvertTask(parseInt(e.target.value, 10))}>
            ${summary.tasks.map((t) => html`
              <option key=${t.index} value=${t.index}>${t.index}: ${t.task}</option>`)}
            <option value=${NEW_TASK}>+ new task…</option>
          </select>
        </label>
        ${naming
          ? html`<label class="field new-task">
                   <span>Task ${nextTaskIndex(summary.tasks)} instruction</span>
                   <input type="text" value=${newTaskName} disabled=${busy}
                          placeholder="e.g. find the can and pick it up"
                          onInput=${(e) => onNewTaskName(e.target.value)} />
                 </label>`
          : null}
        <button type="button" class=${`btn${pending ? ' primary' : ''}`}
                disabled=${busy || (naming && !newTaskName.trim())}
                onClick=${onConvert}
                title="Run rosbag_to_groot.py: add new bags, rebuild any with missing files">
          ${busy ? 'Converting…' : pending ? `Convert ${pending} new` : 'Convert'}
        </button>
      </div>
    </header>`;
}

export const nextTaskIndex = (tasks) =>
  tasks.reduce((max, t) => Math.max(max, t.index + 1), 0);

export function Editor({ episode, tasks, busy, autoAdvance, onAutoAdvance, onTask, onDelete }) {
  return html`
    <section class="panel editor">
      <div class="panel-head">
        <h2>Label</h2>
        <label class="check">
          <input type="checkbox" checked=${autoAdvance}
                 onChange=${(e) => onAutoAdvance(e.target.checked)} />
          next episode after labelling
        </label>
      </div>
      <div class="task-buttons">
        ${tasks.map((t, i) => html`
          <button type="button" key=${t.index} disabled=${busy || !episode}
                  class=${`task-btn ${taskTone(t.index)}${episode && episode.task_index === t.index ? ' current' : ''}`}
                  onClick=${() => onTask(t.index)}>
            ${i < 9 ? html`<kbd>${i + 1}</kbd>` : null}
            <span class="task-idx">${t.index}</span>
            <span class="task-text">${t.task}</span>
          </button>`)}
      </div>
      <div class="editor-foot">
        <button type="button" class="btn danger" disabled=${busy || !episode} onClick=${onDelete}>
          Delete episode…
        </button>
        <span class="dim keys">
          <kbd>J</kbd>/<kbd>K</kbd> next/prev · <kbd>Space</kbd> play · <kbd>←</kbd><kbd>→</kbd> frame
          · <kbd>Shift</kbd>+<kbd>←</kbd><kbd>→</kbd> 1 s · <kbd>Del</kbd> delete
        </span>
      </div>
    </section>`;
}

export function Details({ episode, tasks, fps }) {
  if (!episode) return null;
  const task = tasks.find((t) => t.index === episode.task_index);
  const rows = [
    ['Task', task ? `${task.index}: ${task.task}` : `${episode.task_index} (not in tasks.jsonl)`],
    ['Frames', episode.length],
    ['Duration', `${fmtDuration(episode.duration)} at ${fps ?? '?'} fps`],
    ['Source bag', html`${episode.source}${episode.has_source ? null
      : html` <span class="warn">(not in demonstrations/)</span>`}`],
    ['Recorded', episode.recorded || '--'],
  ];
  return html`
    <section class="panel details">
      <h2>${episodeName(episode.index)}</h2>
      <table>
        <tbody>
          ${rows.map(([k, v]) => html`<tr key=${k}><th>${k}</th><td>${v}</td></tr>`)}
          <tr>
            <th>Files</th>
            <td class="files">
              ${[['parquet', 'parquet'], ...CAMERAS.map((c) => [c.key, c.label])].map(([key, label]) => html`
                <span key=${key} class=${`file ${episode.files[key] ? 'ok' : 'bad'}`}>
                  ${episode.files[key] ? '✓' : '✗'} ${label}
                </span>`)}
            </td>
          </tr>
        </tbody>
      </table>
    </section>`;
}

export function JobPanel({ job, onCancel, onClose }) {
  const logRef = useRef(null);
  useEffect(() => {
    if (logRef.current) logRef.current.scrollTop = logRef.current.scrollHeight;
  }, [job.log.length]);

  const running = job.state === 'running';
  const tone = { running: 'accent', ok: 'ok', failed: 'danger', cancelled: 'warn' }[job.state] || '';
  return html`
    <section class="panel job">
      <div class="panel-head">
        <h2>Converter <span class=${`pill ${tone}`}>${job.state}</span>
          <span class="dim mono"> ${Math.round(job.elapsed || 0)}s</span></h2>
        ${running
          ? html`<button type="button" class="btn danger" onClick=${onCancel}>Cancel</button>`
          : html`<button type="button" class="btn" onClick=${onClose}>Close</button>`}
      </div>
      <pre class="log" ref=${logRef}>${job.log.join('\n')}</pre>
    </section>`;
}

export function Confirm({ title, body, confirm, danger, onConfirm, onCancel }) {
  // Destructive dialogs focus Cancel, so a stray Enter never deletes anything.
  const okRef = useRef(null);
  const cancelRef = useRef(null);
  useEffect(() => {
    const el = danger ? cancelRef.current : okRef.current;
    if (el) el.focus();
  }, []);
  return html`
    <div class="modal-back" onClick=${onCancel}
         onKeyDown=${(e) => { if (e.key === 'Escape') onCancel(); }}>
      <div class="modal panel" role="dialog" aria-modal="true" onClick=${(e) => e.stopPropagation()}>
        <h2>${title}</h2>
        <div class="modal-body">${body}</div>
        <div class="modal-actions">
          <button type="button" ref=${cancelRef} class="btn" onClick=${onCancel}>Cancel</button>
          <button type="button" ref=${okRef} class=${`btn ${danger ? 'danger solid' : 'primary'}`}
                  onClick=${onConfirm}>${confirm}</button>
        </div>
      </div>
    </div>`;
}
