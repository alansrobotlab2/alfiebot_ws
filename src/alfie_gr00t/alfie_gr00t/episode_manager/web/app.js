// Episode manager - React front end for reviewing and labelling GR00T episodes.
//
// The server owns every file. This page holds the dataset summary it was last
// sent, and reloads it whenever the cheap /api/status poll says something
// changed underneath (a new bag landed, a conversion finished, someone else
// switched datasets). Relabels patch the local copy so labelling a run of
// episodes with the number keys doesn't wait on a full reload each time.

import { React, ReactDOM, html, api, CAMERAS, videoUrl, episodeName,
         useInterval, useStoredFlag, useEffect, useMemo, useState, useCallback } from './api.js';
import { usePlayer, VideoGrid, Transport } from './components/player.js';
import { EpisodeList } from './components/EpisodeList.js';
import { TopBar, Editor, Details, JobPanel, Confirm, NEW_TASK, nextTaskIndex } from './components/panels.js';

const STATUS_MS = 2000;
const JOB_MS = 1000;

function App() {
  const [info, setInfo] = useState(null);
  const [rev, setRev] = useState(0);              // bumps video URLs after a conversion
  const [selected, setSelected] = useState(null);
  const [filter, setFilter] = useState(null);
  const [query, setQuery] = useState('');
  const [status, setStatus] = useState({ busy: false, recorder: '', pending: 0 });
  const [job, setJob] = useState(null);
  const [confirm, setConfirm] = useState(null);
  const [toast, setToast] = useState(null);
  const [convertTask, setConvertTask] = useState(0);
  const [newTaskName, setNewTaskName] = useState('');
  const [autoAdvance, setAutoAdvance] = useStoredFlag('em.autoAdvance', true);
  const [loop, setLoop] = useStoredFlag('em.loop', false);

  const summary = info ? info.dataset : null;
  const { player, playing, setPlaying, rate } = usePlayer(summary && summary.fps);

  const say = (msg) => setToast({ msg, id: Date.now() });
  useEffect(() => {
    if (!toast) return undefined;
    const id = setTimeout(() => setToast(null), 4000);
    return () => clearTimeout(id);
  }, [toast]);

  const load = useCallback(async () => {
    try {
      setInfo(await api.dataset());
    } catch (err) {
      say(`cannot load dataset: ${err}`);
    }
  }, []);

  useEffect(() => {
    load();
    api.job().then((j) => { if (j.state === 'running') setJob(j); });
  }, [load]);

  // --- derived --------------------------------------------------------------

  const visible = useMemo(() => {
    if (!summary) return [];
    const q = query.trim().toLowerCase();
    return summary.episodes.filter((ep) =>
      (filter === null || ep.task_index === filter) &&
      (!q || String(ep.index).includes(q) || ep.source.toLowerCase().includes(q)));
  }, [summary, filter, query]);

  const counts = useMemo(() => {
    const c = { all: summary ? summary.episodes.length : 0 };
    for (const ep of (summary ? summary.episodes : [])) c[ep.task_index] = (c[ep.task_index] || 0) + 1;
    return c;
  }, [summary]);

  const episode = summary ? summary.episodes.find((ep) => ep.index === selected) || null : null;

  // Keep a valid selection: the first visible episode when the old one is gone.
  useEffect(() => {
    if (!summary) return;
    if (!summary.episodes.some((ep) => ep.index === selected)) {
      setSelected(visible.length ? visible[0].index : null);
    }
  }, [summary, visible]);

  // A dataset with no tasks yet can only be converted by naming one. Once a
  // named task shows up in tasks.jsonl, switch the picker over to it.
  useEffect(() => {
    if (!summary) return;
    if (convertTask === NEW_TASK) {
      const made = summary.tasks.find((t) => t.task === newTaskName.trim());
      if (made) { setConvertTask(made.index); setNewTaskName(''); }
    } else if (!summary.tasks.some((t) => t.index === convertTask)) {
      setConvertTask(summary.tasks.length ? summary.tasks[0].index : NEW_TASK);
    }
  }, [summary]);

  // A new episode starts playing on its own (the <video>s autoplay).
  useEffect(() => { setPlaying(selected !== null); }, [selected, rev]);

  // --- polling --------------------------------------------------------------

  useInterval(async () => {
    try {
      const s = await api.status();
      setStatus(s);
      if (summary && (s.dataset !== summary.name || s.pending !== summary.pending.length)) load();
      if (s.busy && !job) setJob(await api.job());
    } catch (err) {
      /* server restarting; the next poll will tell */
    }
  }, STATUS_MS);

  const jobRunning = !!(job && job.state === 'running');
  useInterval(async () => {
    const j = await api.job();
    setJob(j);
    if (j.state !== 'running') {
      say(`conversion ${j.state}`);
      await load();
      setRev((r) => r + 1);
    }
  }, JOB_MS, jobRunning);

  // --- actions --------------------------------------------------------------

  const move = (delta) => {
    if (!visible.length) return;
    const at = visible.findIndex((ep) => ep.index === selected);
    const next = Math.max(0, Math.min(visible.length - 1, (at < 0 ? 0 : at + delta)));
    setSelected(visible[next].index);
  };

  const label = async (taskIndex) => {
    if (!episode) return;
    const index = episode.index;
    const res = await api.setTask(index, taskIndex);
    if (!res.ok) { say(`relabel failed: ${res.error}`); return; }
    // Advance first, from the list as it was, so a task filter that hides the
    // relabelled episode doesn't throw the selection back to the top.
    if (autoAdvance) move(1);
    setInfo((prev) => ({
      ...prev,
      dataset: {
        ...prev.dataset,
        stats_stale: true,
        episodes: prev.dataset.episodes.map((ep) =>
          (ep.index === index ? { ...ep, task_index: taskIndex } : ep)),
      },
    }));
    say(`${episodeName(index)} → task ${taskIndex}`);
  };

  const askDelete = () => {
    if (!episode) return;
    const ep = episode;
    setConfirm({
      title: `Delete ${episodeName(ep.index)}?`,
      body: html`
        <p>This permanently removes:</p>
        <ul>
          <li>the source bag <code>${ep.source}</code>${ep.has_source ? null : ' (already gone)'}</li>
          <li>its parquet and four camera videos</li>
          <li>its row in <code>episodes.jsonl</code></li>
        </ul>
        <p class="dim">Other episodes keep their indices. Regenerate stats before training.</p>`,
      confirm: 'Delete',
      danger: true,
      onConfirm: async () => {
        setConfirm(null);
        const at = visible.findIndex((e) => e.index === ep.index);
        const neighbour = visible[at + 1] || visible[at - 1];
        const res = await api.remove(ep.index);
        if (!res.ok) { say(`delete failed: ${res.error}`); return; }
        if (neighbour) setSelected(neighbour.index);
        say(`deleted ${episodeName(ep.index)}`);
        load();
      },
    });
  };

  const startConvert = async (force = false) => {
    const naming = convertTask === NEW_TASK;
    const res = await api.convert(naming ? nextTaskIndex(summary.tasks) : convertTask, force,
                                  naming ? newTaskName.trim() : '');
    if (res.needs_force) {
      setConfirm({
        title: 'Converting will delete episodes',
        body: html`
          <p><b>${res.orphans}</b> episodes in this dataset have no source bag in
             <code>${summary.demos_path}</code>.</p>
          <p>rosbag_to_groot treats a missing bag as a deleted demo, so this
             conversion will <b>remove all ${res.orphans} of them</b> from the dataset.</p>
          <p class="dim">If the bags live elsewhere, copy them back into
             demonstrations/ first.</p>`,
        confirm: `Convert and drop ${res.orphans}`,
        danger: true,
        onConfirm: () => { setConfirm(null); startConvert(true); },
      });
      return;
    }
    if (!res.ok) { say(`convert: ${res.error}`); return; }
    player.pause();
    setJob(await api.job());
  };

  const switchDataset = async (name) => {
    const res = await api.selectDataset(name);
    if (!res.ok) { say(res.error); return; }
    setFilter(null);
    setQuery('');
    setSelected(null);
    await load();
  };

  const onEnded = () => {
    if (loop) { player.seek(0); player.play(); } else { player.pause(); }
  };

  // --- keyboard -------------------------------------------------------------

  useEffect(() => {
    const onKey = (e) => {
      if (confirm || e.metaKey || e.ctrlKey || e.altKey) return;
      const tag = e.target.tagName;
      if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
      const fps = player.fps();
      const k = e.key;
      let handled = true;
      if (k === 'j' || k === 'ArrowDown') move(1);
      else if (k === 'k' || k === 'ArrowUp') move(-1);
      else if (k === ' ') player.toggle();
      else if (k === 'ArrowRight') player.step(e.shiftKey ? fps : 1);
      else if (k === 'ArrowLeft') player.step(e.shiftKey ? -fps : -1);
      else if (k === 'Home') player.seek(0);
      else if (k === 'l') setLoop(!loop);
      else if (k === 'Delete' || k === 'Backspace') askDelete();
      else if (/^[1-9]$/.test(k) && summary && summary.tasks[Number(k) - 1]) {
        label(summary.tasks[Number(k) - 1].index);
      } else handled = false;
      if (handled) e.preventDefault();
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  });

  // --- render ---------------------------------------------------------------

  if (!info) return html`<div class="boot">Loading episode manager…</div>`;

  const busy = status.busy || jobRunning;
  const jobPanel = job && job.state !== 'idle'
    ? html`<${JobPanel} job=${job} onCancel=${() => api.cancel()} onClose=${() => setJob(null)} />`
    : null;
  const urls = episode
    ? CAMERAS.map((c) => (episode.files[c.key] ? videoUrl(episode.index, c.key, rev) : null))
    : [null, null, null, null];

  return html`
    <div class="app">
      <${TopBar}
        info=${info}
        summary=${summary}
        busy=${busy}
        recorder=${status.recorder}
        pending=${summary.pending.length}
        convertTask=${convertTask}
        onConvertTask=${setConvertTask}
        newTaskName=${newTaskName}
        onNewTaskName=${setNewTaskName}
        onDataset=${switchDataset}
        onConvert=${() => startConvert(false)} />

      ${!summary.converted
        ? html`<div class="banner">
                 No converted data in <code>${summary.path}</code> yet.
                 ${summary.pending.length
                   ? ` ${summary.pending.length} ${summary.pending.length === 1 ? 'bag is' : 'bags are'} waiting; name the task and press Convert.`
                   : ' Record some demonstrations first.'}
               </div>`
        : null}
      ${summary.converted && summary.stats_stale
        ? html`<div class="banner">
                 <code>meta/stats.json</code> is older than <code>episodes.jsonl</code>.
                 Regenerate it before training:
                 <code class="cmd">python gr00t/data/stats.py --dataset-path ${summary.path} --embodiment-tag NEW_EMBODIMENT --modality-config-path alfiebot_config.py</code>
               </div>`
        : null}

      <main class="layout">
        <${EpisodeList}
          episodes=${visible}
          tasks=${summary.tasks}
          counts=${counts}
          filter=${filter}
          onFilter=${setFilter}
          query=${query}
          onQuery=${setQuery}
          selected=${selected}
          onSelect=${setSelected} />

        <div class="viewer">
          ${episode
            ? html`
              <div class="stage">
                <${VideoGrid}
                  key=${`${summary.name}:${episode.index}:${rev}`}
                  player=${player}
                  cameras=${CAMERAS}
                  urls=${urls}
                  rate=${rate}
                  autoplay=${true}
                  onEnded=${onEnded} />
                <${Transport}
                  player=${player}
                  playing=${playing}
                  rate=${rate}
                  frames=${episode.length}
                  loop=${loop}
                  onLoop=${setLoop} />
              </div>
              <div class="side">
                <${Editor}
                  episode=${episode}
                  tasks=${summary.tasks}
                  busy=${busy}
                  autoAdvance=${autoAdvance}
                  onAutoAdvance=${setAutoAdvance}
                  onTask=${label}
                  onDelete=${askDelete} />
                <${Details} episode=${episode} tasks=${summary.tasks} fps=${summary.fps} />
              </div>`
            : html`<div class="panel empty-viewer">
                     ${summary.episodes.length ? 'Select an episode.' : 'No episodes in this dataset.'}
                   </div>`}
          ${jobPanel}
        </div>
      </main>

      ${confirm ? html`<${Confirm} ...${confirm} onCancel=${() => setConfirm(null)} />` : null}
      ${toast ? html`<div class="toast" key=${toast.id}>${toast.msg}</div>` : null}
    </div>`;
}

ReactDOM.createRoot(document.getElementById('root')).render(html`<${App} />`);
