// Top bar, episode editor, details table, converter log and the confirm dialog.

import { ReactElement, ReactNode, useEffect, useRef } from "react";

import {
  CAMERAS,
  DatasetInfo,
  DatasetSummary,
  Episode,
  Job,
  Task,
  episodeName,
  fmtDuration,
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
  return <span className="pill">recorder idle</span>;
}

export const NEW_TASK = -1;

export const nextTaskIndex = (tasks: Task[]): number =>
  tasks.reduce((max, t) => Math.max(max, t.index + 1), 0);

export function TopBar({
  info,
  summary,
  busy,
  recorder,
  pending,
  convertTask,
  onConvertTask,
  newTaskName,
  onNewTaskName,
  onDataset,
  onConvert,
}: {
  info: DatasetInfo;
  summary: DatasetSummary;
  busy: boolean;
  recorder: string;
  pending: number;
  convertTask: number;
  onConvertTask: (task: number) => void;
  newTaskName: string;
  onNewTaskName: (name: string) => void;
  onDataset: (name: string) => void;
  onConvert: () => void;
}): ReactElement {
  const naming = convertTask === NEW_TASK;
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
      <label className="field">
        <span>Dataset</span>
        <select
          value={summary.name}
          disabled={busy}
          onChange={(e) => {
            onDataset(e.target.value);
          }}
        >
          {info.datasets.map((d) => (
            <option key={d} value={d}>
              {d}
            </option>
          ))}
        </select>
      </label>
      <div className="stats">
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
      <div className="convert">
        <label className="field">
          <span>Label new bags as</span>
          <select
            value={convertTask}
            disabled={busy}
            onChange={(e) => {
              onConvertTask(parseInt(e.target.value, 10));
            }}
          >
            {summary.tasks.map((t) => (
              <option key={t.index} value={t.index}>
                {t.index}: {t.task}
              </option>
            ))}
            <option value={NEW_TASK}>+ new task…</option>
          </select>
        </label>
        {naming && (
          <label className="field new-task">
            <span>Task {nextTaskIndex(summary.tasks)} instruction</span>
            <input
              type="text"
              value={newTaskName}
              disabled={busy}
              placeholder="e.g. find the can and pick it up"
              onChange={(e) => {
                onNewTaskName(e.target.value);
              }}
            />
          </label>
        )}
        <button
          type="button"
          className={`btn${pending !== 0 ? " primary" : ""}`}
          disabled={busy || (naming && !newTaskName.trim())}
          onClick={onConvert}
          title="Run rosbag_to_groot.py: add new bags, rebuild any with missing files"
        >
          {busy ? "Converting…" : pending !== 0 ? `Convert ${pending} new` : "Convert"}
        </button>
      </div>
    </header>
  );
}

export function Editor({
  episode,
  tasks,
  busy,
  autoAdvance,
  onAutoAdvance,
  onTask,
  onDelete,
}: {
  episode: Episode;
  tasks: Task[];
  busy: boolean;
  autoAdvance: boolean;
  onAutoAdvance: (on: boolean) => void;
  onTask: (task: number) => void;
  onDelete: () => void;
}): ReactElement {
  return (
    <section className="panel editor">
      <div className="panel-head">
        <h2>Label</h2>
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
      <div className="task-buttons">
        {tasks.map((t, i) => (
          <button
            type="button"
            key={t.index}
            disabled={busy}
            className={`task-btn ${taskTone(t.index)}${episode.task_index === t.index ? " current" : ""}`}
            onClick={() => {
              onTask(t.index);
            }}
          >
            {i < 9 && <kbd>{i + 1}</kbd>}
            <span className="task-idx">{t.index}</span>
            <span className="task-text">{t.task}</span>
          </button>
        ))}
      </div>
      <div className="editor-foot">
        <button type="button" className="btn danger" disabled={busy} onClick={onDelete}>
          Delete episode…
        </button>
        <span className="dim keys">
          <kbd>J</kbd>/<kbd>K</kbd> next/prev · <kbd>Space</kbd> play · <kbd>←</kbd>
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
  const rows: [string, ReactNode][] = [
    [
      "Task",
      task ? `${task.index}: ${task.task}` : `${episode.task_index ?? "?"} (not in tasks.jsonl)`,
    ],
    ["Frames", episode.length],
    ["Duration", `${fmtDuration(episode.duration)} at ${fps ?? "?"} fps`],
    [
      "Source bag",
      <>
        {episode.source}
        {!episode.has_source && <span className="warn"> (not in demonstrations/)</span>}
      </>,
    ],
    ["Recorded", episode.recorded ?? "--"],
  ];
  const files: [string, string][] = [
    ["parquet", "parquet"],
    ...CAMERAS.map((c): [string, string] => [c.key, c.label]),
  ];
  return (
    <section className="panel details">
      <h2>{episodeName(episode.index)}</h2>
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
