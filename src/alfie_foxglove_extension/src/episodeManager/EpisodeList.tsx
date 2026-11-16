// Left column: filter by task, search, and the episode list itself (newest first).

import { ReactElement, useEffect, useRef } from "react";

import {
  Episode,
  SOURCE_LABELS,
  Task,
  TaskFilter,
  episodeName,
  filesOk,
  fmtDuration,
  taskTone,
} from "./api";

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
  counts: Map<number | "all" | "unlabelled", number>;
  filter: TaskFilter;
  onFilter: (filter: TaskFilter) => void;
  query: string;
  onQuery: (query: string) => void;
  selected: number | undefined;
  onSelect: (index: number) => void;
}): ReactElement {
  const listRef = useRef<HTMLDivElement>(null);

  // The chip shows the task's registry number, the one typed in the task
  // picker; a string that is not a registry task shows its tasks.jsonl index.
  const chip = (taskIndex: number | null): string => {
    if (taskIndex == undefined) {
      return "?";
    }
    const task = tasks.find((t) => t.index === taskIndex);
    return task?.number != undefined ? String(task.number) : `#${taskIndex}`;
  };

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
        {(counts.get("unlabelled") ?? 0) !== 0 && (
          <button
            type="button"
            className={`filter task-none${filter === "unlabelled" ? " on" : ""}`}
            title="Converted without a task; tag them with the task picker"
            onClick={() => {
              onFilter("unlabelled");
            }}
          >
            <i className="dot"></i>
            Unlabelled <b>{counts.get("unlabelled") ?? 0}</b>
          </button>
        )}
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
          if (ep.excluded != undefined) {
            problems.push(`excluded from training: ${ep.excluded || "no reason given"}`);
          }
          if (ep.heldout != undefined) {
            problems.push(`held out in build ${ep.heldout}`);
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
                title={ep.task_id ?? (ep.task_index == undefined ? "unlabelled" : undefined)}
              >
                {chip(ep.task_index)}
              </span>
              <span className="ep-dur mono">{fmtDuration(ep.duration)}</span>
              <span
                className={`ep-flag${!ok || ep.quality === "bad" || ep.excluded != undefined ? " bad" : ""}`}
                title={problems.join(", ")}
              >
                {!ok
                  ? "!"
                  : ep.quality === "bad" || ep.excluded != undefined
                    ? "✗"
                    : ep.quality === "marginal"
                      ? "~"
                      : ep.heldout != undefined
                        ? "h"
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
