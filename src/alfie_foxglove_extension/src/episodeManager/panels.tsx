// Top bar, episode editor, details table, converter log and the confirm dialog.

import { ReactElement, ReactNode, RefObject, useEffect, useMemo, useRef, useState } from "react";

import {
  CAMERAS,
  DatasetInfo,
  DatasetSummary,
  Episode,
  EpisodeFields,
  Job,
  RecordingContext,
  Registry,
  SOURCE_LABELS,
  Task,
  episodeName,
  fmtDuration,
  searchTasks,
  taskKey,
  taskTone,
} from "./api";

function RecorderPill({ status }: { status: string }): ReactElement {
  if (!status) {
    return (
      <span className="pill dim" title="No message on the recorder status topic yet">
        recorder: not seen
      </span>
    );
  }
  if (status.startsWith("RECORDING")) {
    return (
      <span className="pill rec">
        <i className="rec-dot"></i>recording {status.split(":").slice(1).join(":")}
      </span>
    );
  }
  if (status.startsWith("STARTING")) {
    return (
      <span className="pill" title="rosbag2 is subscribing; not recording yet">
        recorder starting {status.split(":").slice(1).join(":")}
      </span>
    );
  }
  return <span className="pill">recorder idle</span>;
}

/** Rows the task picker shows before it scrolls. */
const PICKER_ROWS = 6;

/**
 * Pick a task for an episode: type a task number or a few words, Enter takes
 * the highlighted match. The box clears and keeps the focus, so a run of
 * episodes can be tagged as "1 Enter 2 Enter 1 Enter".
 */
export function TaskPicker({
  tasks,
  episode,
  disabled,
  inputRef,
  onPick,
}: {
  tasks: Task[];
  episode: Episode;
  disabled: boolean;
  inputRef: RefObject<HTMLInputElement>;
  onPick: (task: Task) => void;
}): ReactElement {
  const [query, setQuery] = useState("");
  const [at, setAt] = useState(0);
  const listRef = useRef<HTMLDivElement>(null);
  const matches = useMemo(() => searchTasks(tasks, query), [tasks, query]);
  const cursor = Math.min(at, Math.max(0, matches.length - 1));

  useEffect(() => {
    listRef.current?.querySelector(".pick-row.cursor")?.scrollIntoView({ block: "nearest" });
  }, [cursor, matches]);

  const pick = (task: Task | undefined) => {
    if (task && !disabled) {
      onPick(task);
      setQuery("");
      setAt(0);
    }
  };
  const isCurrent = (t: Task) => (t.index != undefined ? episode.task_index === t.index : false);

  return (
    <div className="task-picker">
      <input
        ref={inputRef}
        className="search"
        type="search"
        value={query}
        disabled={disabled}
        placeholder="Task number or words, Enter to tag…"
        aria-label="Find a task"
        onChange={(e) => {
          setQuery(e.target.value);
          setAt(0);
        }}
        onKeyDown={(e) => {
          if (e.key === "ArrowDown") {
            setAt(Math.min(cursor + 1, matches.length - 1));
          } else if (e.key === "ArrowUp") {
            setAt(Math.max(cursor - 1, 0));
          } else if (e.key === "Enter") {
            pick(matches[cursor]);
          } else if (e.key === "Escape") {
            setQuery("");
            e.currentTarget.blur();
          } else {
            return;
          }
          e.preventDefault();
          e.stopPropagation();
        }}
      />
      <div
        className="pick-list"
        ref={listRef}
        role="listbox"
        aria-label="Tasks"
        style={{ maxHeight: `calc(${PICKER_ROWS} * var(--pick-row))` }}
      >
        {matches.length === 0 && <div className="empty">No task matches.</div>}
        {matches.map((t, i) => (
          <button
            type="button"
            key={taskKey(t)}
            role="option"
            aria-selected={isCurrent(t)}
            disabled={disabled}
            className={`pick-row ${taskTone(t.index)}${isCurrent(t) ? " current" : ""}${
              i === cursor && query !== "" ? " cursor" : ""
            }`}
            title={t.description ?? undefined}
            onClick={() => {
              pick(t);
            }}
          >
            <span className="pick-num mono">{t.number ?? "–"}</span>
            <span className="pick-id mono">{t.id ?? `#${t.index ?? ""}`}</span>
            <span className="pick-text">{t.description ?? t.instruction ?? t.task}</span>
          </button>
        ))}
      </div>
    </div>
  );
}

/** A text box that commits on blur or Enter, so typing doesn't post every key. */
function CommitInput({
  value,
  disabled,
  placeholder,
  onCommit,
}: {
  value: string;
  disabled?: boolean;
  placeholder?: string;
  onCommit: (value: string) => void;
}): ReactElement {
  const [draft, setDraft] = useState(value);
  useEffect(() => {
    setDraft(value);
  }, [value]);
  const commit = () => {
    if (draft.trim() !== value) {
      onCommit(draft.trim());
    }
  };
  return (
    <input
      type="text"
      value={draft}
      disabled={disabled}
      placeholder={placeholder}
      onChange={(e) => {
        setDraft(e.target.value);
      }}
      onBlur={commit}
      onKeyDown={(e) => {
        if (e.key === "Enter") {
          e.currentTarget.blur();
        }
      }}
    />
  );
}

/** What the recorder writes with the next episodes, and where. */
function RecordingBar({
  info,
  summary,
  busy,
  onContext,
}: {
  info: DatasetInfo;
  summary: DatasetSummary;
  busy: boolean;
  onContext: (update: Partial<RecordingContext>) => void;
}): ReactElement {
  const ctx = info.context;
  if (!ctx) {
    return (
      <div className="recording">
        <h2>Next recording</h2>
        <span className="dim">
          The data recorder is not running, so the task for new episodes cannot be set here.
        </span>
      </div>
    );
  }
  const tasks = info.registry.tasks.filter((t) => t.recordable || t.id === ctx.task_id);
  const here = ctx.session === summary.name;
  const canRecordHere = summary.is_session && summary.closed == undefined && !here;
  return (
    <div className="recording">
      <h2>Next recording</h2>
      <label className="field">
        <span>Task</span>
        <select
          value={ctx.task_id}
          onChange={(e) => {
            onContext({ task_id: e.target.value });
          }}
        >
          <option value="">(not set)</option>
          {tasks.map((t) => (
            <option key={t.id} value={t.id}>
              {t.number != undefined ? `${t.number}  ` : ""}
              {t.id}: {t.task}
            </option>
          ))}
        </select>
      </label>
      <label className="field">
        <span>Source</span>
        <select
          value={ctx.source}
          title="A VR takeover of a GR00T run is recorded as a correction whatever is set here"
          onChange={(e) => {
            onContext({ source: e.target.value });
          }}
        >
          {info.registry.sources.map((src) => (
            <option key={src} value={src}>
              {SOURCE_LABELS[src] ?? src}
            </option>
          ))}
        </select>
      </label>
      <label className="field">
        <span>Correction round</span>
        <input
          type="number"
          min={1}
          value={ctx.round}
          title="Stored with corrections and other non-demo episodes, VR takeovers included"
          onChange={(e) => {
            const round = parseInt(e.target.value, 10);
            if (round >= 1) {
              onContext({ round });
            }
          }}
        />
      </label>
      <label className="field">
        <span>Operator</span>
        <CommitInput
          value={ctx.operator}
          placeholder="name"
          onCommit={(operator) => {
            onContext({ operator });
          }}
        />
      </label>
      <label className="field">
        <span>Surface</span>
        <CommitInput
          value={ctx.surface}
          placeholder="floor, table…"
          onCommit={(surface) => {
            onContext({ surface });
          }}
        />
      </label>
      <div className="target">
        <span className="pill" title="Where the recorder writes the next episode">
          into <b>{ctx.session || "today's session"}</b> · {ctx.hw}
        </span>
        {canRecordHere && (
          <button
            type="button"
            className="btn"
            disabled={busy}
            onClick={() => {
              onContext({ session: summary.name });
            }}
          >
            Record into this session
          </button>
        )}
      </div>
    </div>
  );
}

export function TopBar({
  info,
  summary,
  busy,
  recorder,
  pending,
  onDataset,
  onConvert,
  onNewSession,
  onCloseSession,
  onContext,
}: {
  info: DatasetInfo;
  summary: DatasetSummary;
  busy: boolean;
  recorder: string;
  pending: number;
  onDataset: (name: string) => void;
  onConvert: () => void;
  onNewSession: () => void;
  onCloseSession: () => void;
  onContext: (update: Partial<RecordingContext>) => void;
}): ReactElement {
  const closed = summary.closed != undefined;
  // Counted here, not taken from the summary, so it follows tags made since the last reload.
  const untagged = summary.episodes.filter((ep) => ep.task_index == undefined).length;
  return (
    <header className="topbar">
      <div className="brand">
        <h1>
          Alfie <span>Episode Manager</span>
        </h1>
        <p className="mono" title={summary.path}>
          {summary.path}
        </p>
      </div>
      <div className="session-actions">
        <label className="field">
          <span>Session</span>
          <select
            value={summary.name}
            disabled={busy}
            onChange={(e) => {
              onDataset(e.target.value);
            }}
          >
            {!info.datasets.includes(summary.name) && (
              <option value={summary.name}>(no session yet)</option>
            )}
            {info.datasets.map((d) => (
              <option key={d} value={d}>
                {d}
              </option>
            ))}
          </select>
        </label>
        <button
          type="button"
          className="btn"
          disabled={busy}
          onClick={onNewSession}
          title={`Start the next session of today on ${info.registry.hw ?? "this hardware"} and record into it`}
        >
          New session
        </button>
        {summary.is_session && !closed && (
          <button
            type="button"
            className="btn"
            disabled={busy || summary.episodes.length === 0 || untagged !== 0}
            onClick={onCloseSession}
            title={
              untagged !== 0
                ? `${untagged} episodes have no task yet; tag or delete them first`
                : "Freeze this session: no relabelling, deleting or converting afterwards"
            }
          >
            Close session…
          </button>
        )}
      </div>
      <div className="stats">
        {closed && (
          <span className="pill warn" title={`Closed ${summary.closed ?? ""}; read-only`}>
            closed
          </span>
        )}
        <span className="pill">
          <b>{summary.episodes.length}</b> episodes
        </span>
        <span className="pill">
          <b>{summary.total_frames.toLocaleString()}</b> frames
        </span>
        <span className="pill">
          <b>{summary.fps ?? "--"}</b> fps
        </span>
        <RecorderPill status={recorder} />
      </div>
      {!closed && (
        <div className="convert">
          <button
            type="button"
            className={`btn${pending !== 0 ? " primary" : ""}`}
            disabled={busy}
            onClick={onConvert}
            title="Run rosbag_to_groot.py: add new bags, rebuild any with missing files. Bags recorded without a task become unlabelled episodes"
          >
            {busy ? "Converting…" : pending !== 0 ? `Convert ${pending} new` : "Convert"}
          </button>
        </div>
      )}
      <RecordingBar info={info} summary={summary} busy={busy} onContext={onContext} />
    </header>
  );
}

export function Editor({
  episode,
  tasks,
  registry,
  busy,
  readOnly,
  autoAdvance,
  onAutoAdvance,
  pickerRef,
  onTask,
  onFields,
  onDelete,
}: {
  episode: Episode;
  tasks: Task[];
  registry: Registry;
  busy: boolean;
  /** A closed session: nothing can be changed. */
  readOnly: boolean;
  autoAdvance: boolean;
  onAutoAdvance: (on: boolean) => void;
  /** The task picker's search box, so the panel can focus it from the keyboard. */
  pickerRef: RefObject<HTMLInputElement>;
  onTask: (task: Task) => void;
  onFields: (fields: EpisodeFields) => void;
  onDelete: () => void;
}): ReactElement {
  const locked = busy || readOnly;
  return (
    <section className="panel editor">
      <div className="panel-head">
        <h2>{readOnly ? "Label (session closed)" : "Label"}</h2>
        <label className="check">
          <input
            type="checkbox"
            checked={autoAdvance}
            onChange={(e) => {
              onAutoAdvance(e.target.checked);
            }}
          />
          next episode after labelling
        </label>
      </div>
      <TaskPicker
        tasks={tasks}
        episode={episode}
        disabled={locked}
        inputRef={pickerRef}
        onPick={onTask}
      />
      <div className="review">
        <div className="field">
          <span>Quality</span>
          <div className="quality-buttons">
            {registry.qualities.map((q) => (
              <button
                type="button"
                key={q}
                disabled={locked}
                className={`btn${episode.quality === q ? " current" : ""}`}
                onClick={() => {
                  onFields({ quality: q });
                }}
              >
                {q}
              </button>
            ))}
          </div>
        </div>
        {episode.source === "correction" && (
          <label className="field">
            <span>Why taken over</span>
            <select
              value={episode.failure_mode ?? ""}
              disabled={locked}
              onChange={(e) => {
                onFields({ failure_mode: e.target.value || null });
              }}
            >
              <option value="">(not tagged)</option>
              {registry.failure_modes.map((m) => (
                <option key={m} value={m}>
                  {m.replace("_", " ")}
                </option>
              ))}
            </select>
          </label>
        )}
        <label className="field notes">
          <span>Notes</span>
          <CommitInput
            value={episode.notes ?? ""}
            disabled={locked}
            onCommit={(notes) => {
              onFields({ notes });
            }}
          />
        </label>
      </div>
      <div className="editor-foot">
        <button type="button" className="btn danger" disabled={locked} onClick={onDelete}>
          Delete episode…
        </button>
        <span className="dim keys">
          <kbd>T</kbd> task · <kbd>J</kbd>/<kbd>K</kbd> next/prev · <kbd>Space</kbd> play ·{" "}
          <kbd>←</kbd>
          <kbd>→</kbd> frame · <kbd>Shift</kbd>+<kbd>←</kbd>
          <kbd>→</kbd> 1 s · <kbd>Del</kbd> delete
        </span>
      </div>
    </section>
  );
}

export function Details({
  episode,
  tasks,
  fps,
}: {
  episode: Episode;
  tasks: Task[];
  fps: number | null;
}): ReactElement {
  const task = tasks.find((t) => t.index === episode.task_index);
  const correction = episode.source !== "demo";
  const rows: [string, ReactNode][] = [
    [
      "Task",
      task
        ? `${task.id ?? String(task.index)}: ${task.task}`
        : episode.task_index == undefined
          ? "unlabelled"
          : `${episode.task_index} (not in tasks.jsonl)`,
    ],
    [
      "Source",
      correction
        ? `${SOURCE_LABELS[episode.source] ?? episode.source}, round ${episode.round ?? "?"}`
        : "demo",
    ],
    ...(correction
      ? ([
          ["Policy", episode.policy_ckpt ?? "--"],
          [
            "Run",
            `${episode.run_id ?? "--"}${episode.takeover_s != undefined ? `, taken over at ${episode.takeover_s.toFixed(1)} s` : ""}`,
          ],
          ["Policy heard", episode.instruction_sent ?? "--"],
        ] as [string, ReactNode][])
      : []),
    ["Frames", episode.length],
    ["Duration", `${fmtDuration(episode.duration)} at ${fps ?? "?"} fps`],
    ...(episode.close_frame != undefined || episode.done_frame != undefined
      ? ([
          ["Grasp / done", `frame ${episode.close_frame ?? "--"} / ${episode.done_frame ?? "--"}`],
        ] as [string, ReactNode][])
      : []),
    [
      "Operator",
      episode.operator != undefined && episode.operator !== "" ? episode.operator : "--",
    ],
    ["Scene", [episode.objects?.join(", "), episode.surface].filter(Boolean).join(" on ") || "--"],
    ["Hardware", episode.hw ?? "--"],
    [
      "Source bag",
      <>
        {episode.bag}
        {!episode.has_source && <span className="warn"> (not in demonstrations/)</span>}
      </>,
    ],
    ["Recorded", episode.recorded ?? "--"],
    ...(episode.legacy_episode_index != undefined
      ? ([
          [
            "Was",
            `episode ${episode.legacy_episode_index} of ${episode.legacy_dataset ?? "an older dataset"}`,
          ],
        ] as [string, ReactNode][])
      : []),
  ];
  const files: [string, string][] = [
    ["parquet", "parquet"],
    ...CAMERAS.map((c): [string, string] => [c.key, c.label]),
  ];
  return (
    <section className="panel details">
      <h2>{episode.episode_uid ?? episodeName(episode.index)}</h2>
      <table>
        <tbody>
          {rows.map(([k, v]) => (
            <tr key={k}>
              <th>{k}</th>
              <td>{v}</td>
            </tr>
          ))}
          <tr>
            <th>Files</th>
            <td className="files">
              {files.map(([key, label]) => (
                <span key={key} className={`file ${episode.files[key] === true ? "ok" : "bad"}`}>
                  {episode.files[key] === true ? "✓" : "✗"} {label}
                </span>
              ))}
            </td>
          </tr>
        </tbody>
      </table>
    </section>
  );
}

const JOB_TONES: Record<Job["state"], string> = {
  idle: "",
  running: "accent",
  ok: "ok",
  failed: "danger",
  cancelled: "warn",
};

export function JobPanel({
  job,
  onCancel,
  onClose,
}: {
  job: Job;
  onCancel: () => void;
  onClose: () => void;
}): ReactElement {
  const logRef = useRef<HTMLPreElement>(null);
  useEffect(() => {
    if (logRef.current) {
      logRef.current.scrollTop = logRef.current.scrollHeight;
    }
  }, [job.log.length]);

  return (
    <section className="panel job">
      <div className="panel-head">
        <h2>
          Converter <span className={`pill ${JOB_TONES[job.state]}`}>{job.state}</span>
          <span className="dim mono"> {Math.round(job.elapsed ?? 0)}s</span>
        </h2>
        {job.state === "running" ? (
          <button type="button" className="btn danger" onClick={onCancel}>
            Cancel
          </button>
        ) : (
          <button type="button" className="btn" onClick={onClose}>
            Close
          </button>
        )}
      </div>
      <pre className="log" ref={logRef}>
        {job.log.join("\n")}
      </pre>
    </section>
  );
}

export type ConfirmProps = {
  title: string;
  body: ReactNode;
  confirm: string;
  danger?: boolean;
  onConfirm: () => void | Promise<void>;
};

export function Confirm({
  title,
  body,
  confirm,
  danger = false,
  onConfirm,
  onCancel,
}: ConfirmProps & { onCancel: () => void }): ReactElement {
  // Destructive dialogs focus Cancel, so a stray Enter never deletes anything.
  const okRef = useRef<HTMLButtonElement>(null);
  const cancelRef = useRef<HTMLButtonElement>(null);
  useEffect(() => {
    (danger ? cancelRef : okRef).current?.focus();
  }, [danger]);
  return (
    <div
      className="modal-back"
      onClick={onCancel}
      onKeyDown={(e) => {
        // Keep keys inside the dialog; the panel's shortcuts must not fire under it.
        e.stopPropagation();
        if (e.key === "Escape") {
          onCancel();
        }
      }}
    >
      <div
        className="modal panel"
        role="dialog"
        aria-modal="true"
        onClick={(e) => {
          e.stopPropagation();
        }}
      >
        <h2>{title}</h2>
        <div className="modal-body">{body}</div>
        <div className="modal-actions">
          <button type="button" ref={cancelRef} className="btn" onClick={onCancel}>
            Cancel
          </button>
          <button
            type="button"
            ref={okRef}
            className={`btn ${danger ? "danger solid" : "primary"}`}
            onClick={() => void onConfirm()}
          >
            {confirm}
          </button>
        </div>
      </div>
    </div>
  );
}
