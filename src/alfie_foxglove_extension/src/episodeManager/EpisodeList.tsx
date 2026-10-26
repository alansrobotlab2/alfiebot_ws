// Left column: filter by task, search, and the episode list itself (newest first).

import { ReactElement, useEffect, useRef } from "react";

import { Episode, Task, episodeName, filesOk, fmtDuration, taskTone } from "./api";

export function EpisodeList({
  episodes,
  tasks,
  counts,
  filter,
  onFilter,
  query,
  onQuery,
  selected,
  onSelect,
}: {
  episodes: Episode[];
  tasks: Task[];
  counts: Map<number | "all", number>;
  filter: number | undefined;
  onFilter: (filter: number | undefined) => void;
  query: string;
  onQuery: (query: string) => void;
  selected: number | undefined;
  onSelect: (index: number) => void;
}): ReactElement {
  const listRef = useRef<HTMLDivElement>(null);

  // Keep the selected row in view as j/k walk the list.
  useEffect(() => {
    const row = listRef.current?.querySelector(".ep-row.selected");
    row?.scrollIntoView({ block: "nearest" });
  }, [selected]);

  return (
    <aside className="episodes panel">
      <div className="filters">
        <button
          type="button"
          className={`filter${filter == undefined ? " on" : ""}`}
          onClick={() => {
            onFilter(undefined);
          }}
        >
          All <b>{counts.get("all") ?? 0}</b>
        </button>
        {tasks.map((t) => (
          <button
            type="button"
            key={t.index}
            className={`filter ${taskTone(t.index)}${filter === t.index ? " on" : ""}`}
            title={t.task}
            onClick={() => {
              onFilter(t.index);
            }}
          >
            <i className="dot"></i>
            {t.index} <b>{counts.get(t.index) ?? 0}</b>
          </button>
        ))}
      </div>
      <input
        className="search"
        type="search"
        placeholder="Search index or bag…"
        value={query}
        onChange={(e) => {
          onQuery(e.target.value);
        }}
      />
      <div className="ep-list" ref={listRef} role="listbox" aria-label="Episodes">
        {episodes.length === 0 && <div className="empty">No episodes match.</div>}
        {episodes.map((ep) => {
          const ok = filesOk(ep);
          const problems = [];
          if (!ok) {
            problems.push("missing output files");
          }
          if (!ep.has_source) {
            problems.push("no source bag");
          }
          return (
            <button
              type="button"
              key={ep.index}
              role="option"
              aria-selected={ep.index === selected}
              className={`ep-row${ep.index === selected ? " selected" : ""}`}
              onClick={() => {
                onSelect(ep.index);
              }}
            >
              <span className="ep-name mono">{episodeName(ep.index)}</span>
              <span className={`task-chip ${taskTone(ep.task_index)}`}>{ep.task_index ?? "?"}</span>
              <span className="ep-dur mono">{fmtDuration(ep.duration)}</span>
              <span className={`ep-flag${!ok ? " bad" : ""}`} title={problems.join(", ")}>
                {!ok ? "!" : !ep.has_source ? "·" : ""}
              </span>
            </button>
          );
        })}
      </div>
    </aside>
  );
}
