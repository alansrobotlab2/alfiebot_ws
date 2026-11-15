// Left column: filter by task, search, and the episode list itself (newest first).

import { ReactElement, useEffect, useRef } from "react";

import { Episode, SOURCE_LABELS, Task, episodeName, filesOk, fmtDuration, taskTone } from "./api";

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
        {tasks.map((t) => {
          const index = t.index;
          if (index == undefined) {
            return undefined; // registry task with no episodes here yet
          }
          return (
            <button
              type="button"
              key={index}
              className={`filter ${taskTone(index)}${filter === index ? " on" : ""}`}
              title={t.task}
              onClick={() => {
                onFilter(index);
              }}
            >
              <i className="dot"></i>
              {t.id ?? index} <b>{counts.get(index) ?? 0}</b>
            </button>
          );
        })}
      </div>
      <input
        className="search"
        type="search"
        placeholder="Search index, bag, task or source…"
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
          if (ep.quality !== "ok") {
            problems.push(`quality: ${ep.quality}`);
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
              <span className="ep-name mono">
                {episodeName(ep.index)}
                {ep.source !== "demo" && (
                  <span
                    className="ep-source"
                    title={`${SOURCE_LABELS[ep.source] ?? ep.source}, round ${ep.round ?? "?"}`}
                  >
                    {ep.source === "correction"
                      ? `C${ep.round ?? ""}`
                      : ep.source.slice(0, 1).toUpperCase()}
                  </span>
                )}
              </span>
              <span
                className={`task-chip ${taskTone(ep.task_index)}`}
                title={ep.task_id ?? undefined}
              >
                {ep.task_index ?? "?"}
              </span>
              <span className="ep-dur mono">{fmtDuration(ep.duration)}</span>
              <span
                className={`ep-flag${!ok || ep.quality === "bad" ? " bad" : ""}`}
                title={problems.join(", ")}
              >
                {!ok
                  ? "!"
                  : ep.quality === "bad"
                    ? "✗"
                    : ep.quality === "marginal"
                      ? "~"
                      : !ep.has_source
                        ? "·"
                        : ""}
              </span>
            </button>
          );
        })}
      </div>
    </aside>
  );
}
