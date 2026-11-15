// Episode manager - review, relabel and delete GR00T training episodes.
//
// A port of the episode manager node's own web UI (alfie_gr00t/episode_manager/web)
// talking to the same HTTP API. The server owns every file. This view holds the
// dataset summary it was last sent, and reloads it whenever the cheap
// /api/status poll says something changed underneath (a new bag landed, a
// conversion finished, someone else switched datasets). Relabels patch the
// local copy so labelling a run of episodes with the number keys doesn't wait
// on a full reload each time.

import {
  KeyboardEvent,
  ReactElement,
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import { createPortal } from "react-dom";

import { EpisodeList } from "./EpisodeList";
import {
  Api,
  CAMERAS,
  DatasetInfo,
  EpisodeFields,
  Job,
  RecordingContext,
  Status,
  Task,
  TaskFilter,
  episodeName,
} from "./api";
import { Confirm, ConfirmProps, Details, Editor, JobPanel, TopBar } from "./panels";
import { Transport, VideoGrid, usePlayer } from "./player";

const STATUS_MS = 2000;
const JOB_MS = 1000;

/** Call `fn` every `periodMs` while `active`; the latest `fn` is always used. */
function useInterval(fn: () => void | Promise<void>, periodMs: number, active = true): void {
  const saved = useRef(fn);
  saved.current = fn;
  useEffect(() => {
    if (!active) {
      return undefined;
    }
    const id = setInterval(() => void saved.current(), periodMs);
    return () => {
      clearInterval(id);
    };
  }, [periodMs, active]);
}

export function EpisodeManager({
  api,
  serverUrl,
  overlay,
  autoAdvance,
  onAutoAdvance,
  loop,
  onLoop,
}: {
  api: Api;
  serverUrl: string;
  /** Element above the scrolling panel body; dialogs and toasts render there. */
  overlay: HTMLElement;
  autoAdvance: boolean;
  onAutoAdvance: (on: boolean) => void;
  loop: boolean;
  onLoop: (on: boolean) => void;
}): ReactElement {
  const [info, setInfo] = useState<DatasetInfo | undefined>();
  const [loadError, setLoadError] = useState<string | undefined>();
  const [rev, setRev] = useState(0); // bumps video URLs after a conversion
  const [selected, setSelected] = useState<number | undefined>();
  const [filter, setFilter] = useState<TaskFilter>();
  const pickerRef = useRef<HTMLInputElement>(null);
  const [query, setQuery] = useState("");
  const [status, setStatus] = useState<Status>({ busy: false, recorder: "", pending: 0 });
  const [job, setJob] = useState<Job | undefined>();
  const [confirm, setConfirm] = useState<ConfirmProps | undefined>();
  const [toast, setToast] = useState<{ msg: string; id: number } | undefined>();

  const summary = info?.dataset;
  const { player, playing, setPlaying, rate } = usePlayer(summary?.fps);

  const say = (msg: string) => {
    setToast({ msg, id: Date.now() });
  };
  useEffect(() => {
    if (!toast) {
      return undefined;
    }
    const id = setTimeout(() => {
      setToast(undefined);
    }, 4000);
    return () => {
      clearTimeout(id);
    };
  }, [toast]);

  const load = useCallback(async () => {
    try {
      setInfo(await api.dataset());
      setLoadError(undefined);
    } catch (err) {
      setLoadError(String(err));
    }
  }, [api]);

  useEffect(() => {
    void load();
    api
      .job()
      .then((j) => {
        if (j.state === "running") {
          setJob(j);
        }
      })
      .catch(() => undefined);
  }, [api, load]);

  // --- derived --------------------------------------------------------------

  const visible = useMemo(() => {
    if (!summary) {
      return [];
    }
    const q = query.trim().toLowerCase();
    return summary.episodes.filter(
      (ep) =>
        (filter == undefined ||
          (filter === "unlabelled" ? ep.task_index == undefined : ep.task_index === filter)) &&
        (!q ||
          String(ep.index).includes(q) ||
          ep.bag.toLowerCase().includes(q) ||
          ep.source.includes(q) ||
          (ep.task_id ?? "").includes(q)),
    );
  }, [summary, filter, query]);

  const counts = useMemo(() => {
    const c = new Map<number | "all" | "unlabelled", number>([
      ["all", summary?.episodes.length ?? 0],
    ]);
    for (const ep of summary?.episodes ?? []) {
      const key = ep.task_index ?? "unlabelled";
      c.set(key, (c.get(key) ?? 0) + 1);
    }
    return c;
  }, [summary]);

  const episode = summary?.episodes.find((ep) => ep.index === selected);

  // Keep a valid selection: the first visible episode when the old one is gone.
  useEffect(() => {
    if (summary && !summary.episodes.some((ep) => ep.index === selected)) {
      setSelected(visible[0]?.index);
    }
  }, [summary, visible, selected]);

  // Nothing left to tag: drop the Unlabelled filter instead of showing an empty list.
  useEffect(() => {
    if (filter === "unlabelled" && summary && (counts.get("unlabelled") ?? 0) === 0) {
      setFilter(undefined);
    }
  }, [filter, summary, counts]);

  // A new episode starts playing on its own (the <video>s autoplay).
  useEffect(() => {
    setPlaying(selected != undefined);
  }, [selected, rev, setPlaying]);

  // --- polling --------------------------------------------------------------

  useInterval(async () => {
    try {
      const s = await api.status();
      setStatus(s);
      if (
        !summary ||
        s.dataset !== summary.name ||
        s.pending !== summary.pending.length ||
        (s.closed ?? false) !== (summary.closed != undefined)
      ) {
        await load();
      } else if ("context" in s) {
        // The recorder's own view of the recording context (it may have moved
        // to a new session, or been changed from another panel).
        const context = s.context ?? null;
        setInfo((prev) =>
          prev && JSON.stringify(prev.context) !== JSON.stringify(context)
            ? { ...prev, context }
            : prev,
        );
      }
      if (s.busy && !job) {
        setJob(await api.job());
      }
    } catch (err) {
      // Server unreachable or restarting; the next poll will tell.
      setLoadError(String(err));
    }
  }, STATUS_MS);

  const jobRunning = job?.state === "running";
  useInterval(
    async () => {
      const j = await api.job();
      setJob(j);
      if (j.state !== "running") {
        say(`conversion ${j.state}`);
        await load();
        setRev((r) => r + 1);
      }
    },
    JOB_MS,
    jobRunning,
  );

  // --- actions --------------------------------------------------------------

  const move = (delta: number) => {
    if (visible.length === 0) {
      return;
    }
    const at = visible.findIndex((ep) => ep.index === selected);
    const next = Math.max(0, Math.min(visible.length - 1, at < 0 ? 0 : at + delta));
    setSelected(visible[next]?.index);
  };

  const patchEpisode = (
    index: number,
    patch: Partial<DatasetInfo["dataset"]["episodes"][number]>,
  ) => {
    setInfo(
      (prev) =>
        prev && {
          ...prev,
          dataset: {
            ...prev.dataset,
            stats_stale: true,
            episodes: prev.dataset.episodes.map((ep) =>
              ep.index === index ? { ...ep, ...patch } : ep,
            ),
          },
        },
    );
  };

  const label = async (task: Task) => {
    if (!episode || summary?.closed != undefined) {
      return;
    }
    const index = episode.index;
    const res = await api.setTask(index, task);
    if (!res.ok) {
      say(`relabel failed: ${res.error ?? "unknown error"}`);
      return;
    }
    // Advance first, from the list as it was, so a task filter that hides the
    // relabelled episode doesn't throw the selection back to the top.
    if (autoAdvance) {
      move(1);
    }
    if (task.index != undefined) {
      patchEpisode(index, { task_index: task.index, task_id: task.id });
    } else {
      await load(); // the task is new to this dataset: tasks.jsonl got a row
    }
    say(`${episodeName(index)} → ${task.id ?? `task ${task.index ?? "?"}`}`);
  };

  const review = async (fields: EpisodeFields) => {
    if (!episode) {
      return;
    }
    const index = episode.index;
    const res = await api.setFields(index, fields);
    if (!res.ok) {
      say(`not saved: ${res.error ?? "unknown error"}`);
      return;
    }
    patchEpisode(index, fields);
  };

  const setContext = async (update: Partial<RecordingContext>) => {
    const res = await api.setContext(update);
    if (!res.ok) {
      say(res.error ?? "recording context not changed");
      return;
    }
    setInfo((prev) =>
      prev?.context ? { ...prev, context: { ...prev.context, ...update } } : prev,
    );
  };

  const newSession = async () => {
    const res = await api.newSession();
    if (!res.ok) {
      say(res.error ?? "cannot start a session");
      return;
    }
    setFilter(undefined);
    setQuery("");
    setSelected(undefined);
    say(`recording into ${res.session ?? "the new session"}`);
    await load();
  };

  const closeSession = (force: boolean, pending: number) => {
    if (!summary) {
      return;
    }
    const name = summary.name;
    setConfirm({
      title: `Close ${name}?`,
      body: (
        <>
          <p>
            A closed session is final: its <b>{summary.episodes.length}</b> episodes can no longer
            be relabelled, re-rated, deleted or converted. Do this once the review is done.
          </p>
          {pending !== 0 && (
            <p>
              <b>
                {pending} {pending === 1 ? "bag is" : "bags are"} not converted
              </b>{" "}
              and will stay out of the session for good.
            </p>
          )}
          <p className="dim">
            The task registry and the code versions in use are saved with it. The recorder moves on
            to a new session.
          </p>
        </>
      ),
      confirm: pending !== 0 ? `Close without ${pending}` : "Close session",
      danger: true,
      onConfirm: async () => {
        setConfirm(undefined);
        const res = await api.closeSession(force || pending !== 0);
        if (!res.ok) {
          say(`not closed: ${res.error ?? "unknown error"}`);
          return;
        }
        say(`${name} closed`);
        await load();
      },
    });
  };

  const askDelete = () => {
    if (!episode) {
      return;
    }
    const ep = episode;
    setConfirm({
      title: `Delete ${episodeName(ep.index)}?`,
      body: (
        <>
          <p>This permanently removes:</p>
          <ul>
            <li>
              the source bag <code>{ep.bag}</code>
              {!ep.has_source && " (already gone)"}
            </li>
            <li>its parquet and four camera videos</li>
            <li>
              its row in <code>episodes.jsonl</code>
            </li>
          </ul>
          <p className="dim">
            Other episodes keep their indices. Regenerate stats before training.
          </p>
        </>
      ),
      confirm: "Delete",
      danger: true,
      onConfirm: async () => {
        setConfirm(undefined);
        const at = visible.findIndex((e) => e.index === ep.index);
        const neighbour = visible[at + 1] ?? visible[at - 1];
        const res = await api.remove(ep.index);
        if (!res.ok) {
          say(`delete failed: ${res.error ?? "unknown error"}`);
          return;
        }
        if (neighbour) {
          setSelected(neighbour.index);
        }
        say(`deleted ${episodeName(ep.index)}`);
        await load();
      },
    });
  };

  const startConvert = async (force: boolean) => {
    if (!summary) {
      return;
    }
    const res = await api.convert(force);
    if (res.needs_force === true) {
      const orphans = res.orphans ?? 0;
      setConfirm({
        title: "Converting will delete episodes",
        body: (
          <>
            <p>
              <b>{orphans}</b> episodes in this dataset have no source bag in{" "}
              <code>{summary.demos_path}</code>.
            </p>
            <p>
              rosbag_to_groot treats a missing bag as a deleted demo, so this conversion will{" "}
              <b>remove all {orphans} of them</b> from the dataset.
            </p>
            <p className="dim">
              If the bags live elsewhere, copy them back into demonstrations/ first.
            </p>
          </>
        ),
        confirm: `Convert and drop ${orphans}`,
        danger: true,
        onConfirm: async () => {
          setConfirm(undefined);
          await startConvert(true);
        },
      });
      return;
    }
    if (!res.ok) {
      say(`convert: ${res.error ?? "unknown error"}`);
      return;
    }
    player.pause();
    setJob(await api.job());
  };

  const switchDataset = async (name: string) => {
    const res = await api.selectDataset(name);
    if (!res.ok) {
      say(res.error ?? "cannot switch dataset");
      return;
    }
    setFilter(undefined);
    setQuery("");
    setSelected(undefined);
    await load();
  };

  const onEnded = () => {
    if (loop) {
      player.seek(0);
      player.play();
    } else {
      player.pause();
    }
  };

  // --- keyboard -------------------------------------------------------------

  // Keys only count while focus is inside this panel. Handled keys stop here so
  // Foxglove's own shortcuts (Space plays the data source) don't fire as well.
  const onKeyDown = (e: KeyboardEvent<HTMLDivElement>) => {
    if (confirm || e.metaKey || e.ctrlKey || e.altKey) {
      return;
    }
    const tag = (e.target as HTMLElement).tagName;
    if (tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA") {
      return;
    }
    const fps = player.fps();
    const k = e.key;
    const readOnly = summary?.closed != undefined;
    let handled = true;
    if (k === "j" || k === "ArrowDown") {
      move(1);
    } else if (k === "k" || k === "ArrowUp") {
      move(-1);
    } else if (k === " ") {
      player.toggle();
    } else if (k === "ArrowRight") {
      player.step(e.shiftKey ? fps : 1);
    } else if (k === "ArrowLeft") {
      player.step(e.shiftKey ? -fps : -1);
    } else if (k === "Home") {
      player.seek(0);
    } else if (k === "l") {
      onLoop(!loop);
    } else if ((k === "Delete" || k === "Backspace") && !readOnly) {
      askDelete();
    } else if (k === "t" || k === "/") {
      pickerRef.current?.focus();
    } else {
      handled = false;
    }
    if (handled) {
      e.preventDefault();
      e.stopPropagation();
    }
  };

  // --- render ---------------------------------------------------------------

  if (!info || !summary) {
    return (
      <div className="boot">
        {loadError != undefined ? (
          <div className="boot-error">
            <p>
              Cannot reach the episode manager at <code>{serverUrl}</code>
            </p>
            <p className="dim mono">{loadError}</p>
            <p className="dim">
              Is <code>ros2 run alfie_gr00t episode_manager</code> running, and the URL right in the
              panel settings? Retrying…
            </p>
          </div>
        ) : (
          "Loading episode manager…"
        )}
      </div>
    );
  }

  const busy = status.busy || jobRunning;
  const urls = CAMERAS.map((c) =>
    episode?.files[c.key] === true ? api.videoUrl(episode.index, c.key, rev) : undefined,
  );

  return (
    <div className="app" tabIndex={-1} onKeyDown={onKeyDown}>
      <TopBar
        info={info}
        summary={summary}
        busy={busy}
        recorder={status.recorder}
        pending={summary.pending.length}
        onDataset={(name) => void switchDataset(name)}
        onConvert={() => void startConvert(false)}
        onNewSession={() => void newSession()}
        onCloseSession={() => {
          closeSession(false, summary.pending.length);
        }}
        onContext={(update) => void setContext(update)}
      />

      {loadError != undefined && (
        <div className="banner">
          Lost contact with <code>{serverUrl}</code> ({loadError}). Retrying…
        </div>
      )}
      {!summary.converted && (
        <div className="banner">
          No converted data in <code>{summary.path}</code> yet.
          {summary.pending.length !== 0
            ? ` ${summary.pending.length} ${summary.pending.length === 1 ? "bag is" : "bags are"} waiting; press Convert.`
            : " Pick the task under Next recording, then record some demonstrations."}
        </div>
      )}
      {/* Stats for sessions are made on the training machine, for the merged training folders. */}
      {summary.converted && summary.stats_stale && !summary.is_session && (
        <div className="banner">
          <code>meta/stats.json</code> is older than <code>episodes.jsonl</code>. Regenerate it
          before training:
          <code className="cmd">
            python gr00t/data/stats.py --dataset-path {summary.path} --embodiment-tag NEW_EMBODIMENT
            --modality-config-path alfiebot_config.py
          </code>
        </div>
      )}

      <main className="layout">
        <EpisodeList
          episodes={visible}
          tasks={summary.tasks}
          counts={counts}
          filter={filter}
          onFilter={setFilter}
          query={query}
          onQuery={setQuery}
          selected={selected}
          onSelect={setSelected}
        />

        <div className="viewer">
          {episode ? (
            <>
              <div className="stage">
                <VideoGrid
                  key={`${summary.name}:${episode.index}:${rev}`}
                  player={player}
                  cameras={CAMERAS}
                  urls={urls}
                  rate={rate}
                  autoplay={true}
                  onEnded={onEnded}
                />
                <Transport
                  player={player}
                  playing={playing}
                  rate={rate}
                  frames={episode.length}
                  loop={loop}
                  onLoop={onLoop}
                />
              </div>
              <div className="side">
                <Editor
                  episode={episode}
                  tasks={summary.tasks}
                  registry={info.registry}
                  busy={busy}
                  readOnly={summary.closed != undefined}
                  autoAdvance={autoAdvance}
                  onAutoAdvance={onAutoAdvance}
                  pickerRef={pickerRef}
                  onTask={(task) => void label(task)}
                  onFields={(fields) => void review(fields)}
                  onDelete={askDelete}
                />
                <Details episode={episode} tasks={summary.tasks} fps={summary.fps} />
              </div>
            </>
          ) : (
            <div className="panel empty-viewer">
              {summary.episodes.length !== 0
                ? "Select an episode."
                : "No episodes in this dataset."}
            </div>
          )}
          {job && job.state !== "idle" && (
            <JobPanel
              job={job}
              onCancel={() => void api.cancel()}
              onClose={() => {
                setJob(undefined);
              }}
            />
          )}
        </div>
      </main>

      {createPortal(
        <>
          {confirm && (
            <Confirm
              {...confirm}
              onCancel={() => {
                setConfirm(undefined);
              }}
            />
          )}
          {toast && (
            <div className="toast" key={toast.id}>
              {toast.msg}
            </div>
          )}
        </>,
        overlay,
      )}
    </div>
  );
}
