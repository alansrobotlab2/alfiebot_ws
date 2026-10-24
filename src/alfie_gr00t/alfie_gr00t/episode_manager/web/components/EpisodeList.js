// Left column: filter by task, search, and the episode list itself (newest first).

import { html, useEffect, useRef, episodeName, fmtDuration, taskTone, filesOk } from '../api.js';

export function EpisodeList({ episodes, tasks, counts, filter, onFilter, query, onQuery,
                              selected, onSelect }) {
  const listRef = useRef(null);

  // Keep the selected row in view as j/k walk the list.
  useEffect(() => {
    const row = listRef.current && listRef.current.querySelector('.ep-row.selected');
    if (row) row.scrollIntoView({ block: 'nearest' });
  }, [selected]);

  return html`
    <aside class="episodes panel">
      <div class="filters">
        <button type="button" class=${`filter${filter === null ? ' on' : ''}`}
                onClick=${() => onFilter(null)}>
          All <b>${counts.all}</b>
        </button>
        ${tasks.map((t) => html`
          <button type="button" key=${t.index}
                  class=${`filter ${taskTone(t.index)}${filter === t.index ? ' on' : ''}`}
                  title=${t.task}
                  onClick=${() => onFilter(t.index)}>
            <i class="dot"></i>${t.index} <b>${counts[t.index] || 0}</b>
          </button>`)}
      </div>
      <input class="search" type="search" placeholder="Search index or bag…"
             value=${query} onInput=${(e) => onQuery(e.target.value)} />
      <div class="ep-list" ref=${listRef} role="listbox" aria-label="Episodes">
        ${episodes.length === 0 ? html`<div class="empty">No episodes match.</div>` : null}
        ${episodes.map((ep) => {
          const problems = [];
          if (!filesOk(ep)) problems.push('missing output files');
          if (!ep.has_source) problems.push('no source bag');
          return html`
            <button type="button" key=${ep.index} role="option"
                    aria-selected=${ep.index === selected ? 'true' : 'false'}
                    class=${`ep-row${ep.index === selected ? ' selected' : ''}`}
                    onClick=${() => onSelect(ep.index)}>
              <span class="ep-name mono">${episodeName(ep.index)}</span>
              <span class=${`task-chip ${taskTone(ep.task_index)}`}>${ep.task_index ?? '?'}</span>
              <span class="ep-dur mono">${fmtDuration(ep.duration)}</span>
              <span class=${`ep-flag${!filesOk(ep) ? ' bad' : ''}`} title=${problems.join(', ')}>
                ${!filesOk(ep) ? '!' : !ep.has_source ? '·' : ''}
              </span>
            </button>`;
        })}
      </div>
    </aside>`;
}
