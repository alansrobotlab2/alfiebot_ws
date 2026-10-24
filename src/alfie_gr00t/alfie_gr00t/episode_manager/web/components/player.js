// Four <video> elements played as one: a shared transport, a master clock, and
// drift correction. The first camera that has loaded is the master; the others
// are nudged back onto it whenever they wander more than a couple of frames.

import { html, useState, useEffect, useRef, useMemo, fmtDuration } from '../api.js';

const DRIFT_FRAMES = 2;

export function usePlayer(fps) {
  const [playing, setPlaying] = useState(false);
  const [rate, setRateState] = useState(1);
  const fpsRef = useRef(fps || 15);
  fpsRef.current = fps || 15;

  const player = useMemo(() => {
    const videos = [];
    const live = () => videos.filter((v) => v && v.readyState >= 1);
    const master = () => live()[0] || null;
    const p = {
      videos,
      master,
      fps: () => fpsRef.current,
      time: () => (master() ? master().currentTime : 0),
      duration: () => (master() && Number.isFinite(master().duration) ? master().duration : 0),
      play() {
        const m = master();
        if (m && m.ended) p.seek(0);
        live().forEach((v) => v.play().catch(() => {}));
        setPlaying(true);
      },
      pause() {
        live().forEach((v) => v.pause());
        setPlaying(false);
      },
      toggle() { (master() && !master().paused) ? p.pause() : p.play(); },
      seek(t) {
        const end = p.duration();
        const target = Math.max(0, end ? Math.min(t, end - 1e-3) : t);
        live().forEach((v) => { v.currentTime = target; });
      },
      step(frames) {
        p.pause();
        // Land mid-frame so rounding never shows the neighbour.
        const frame = Math.round(p.time() * p.fps()) + frames;
        p.seek((Math.max(0, frame) + 0.5) / p.fps());
      },
      setRate(r) {
        videos.forEach((v) => { if (v) v.playbackRate = r; });
        setRateState(r);
      },
      sync() {
        const m = master();
        if (!m || m.paused) return;
        const slack = DRIFT_FRAMES / p.fps();
        for (const v of live()) {
          if (v === m) continue;
          if (Math.abs(v.currentTime - m.currentTime) > slack) v.currentTime = m.currentTime;
          if (v.paused) v.play().catch(() => {});
        }
      },
    };
    return p;
  }, []);

  return { player, playing, setPlaying, rate };
}

export function VideoGrid({ player, cameras, urls, rate, autoplay, onEnded }) {
  // Drift correction runs on its own animation loop so it never re-renders React.
  useEffect(() => {
    let id = requestAnimationFrame(function tick() {
      player.sync();
      id = requestAnimationFrame(tick);
    });
    return () => cancelAnimationFrame(id);
  }, [player]);

  return html`
    <div class="video-grid">
      ${cameras.map((cam, i) => html`
        <figure class="video-tile" key=${cam.key}>
          ${urls[i]
            ? html`<video
                ref=${(el) => { player.videos[i] = el; }}
                src=${urls[i]}
                muted
                playsInline
                preload="auto"
                autoPlay=${autoplay}
                onLoadedMetadata=${(e) => { e.target.playbackRate = rate; }}
                onEnded=${(e) => { if (e.target === player.master()) onEnded(); }} />`
            : html`<div class="video-missing"
                        ref=${() => { player.videos[i] = null; }}>no video</div>`}
          <figcaption>${cam.label}</figcaption>
        </figure>`)}
    </div>`;
}

const RATES = [0.25, 0.5, 1, 2, 4];

export function Transport({ player, playing, rate, frames, loop, onLoop }) {
  const [time, setTime] = useState(0);
  const [duration, setDuration] = useState(0);
  const dragging = useRef(false);

  // The scrubber reads the master clock every frame without touching the rest
  // of the tree.
  useEffect(() => {
    let id = requestAnimationFrame(function tick() {
      if (!dragging.current) setTime(player.time());
      setDuration(player.duration());
      id = requestAnimationFrame(tick);
    });
    return () => cancelAnimationFrame(id);
  }, [player]);

  const fps = player.fps();
  const frame = Math.min(Math.floor(time * fps + 1e-6), Math.max(0, (frames || 1) - 1));

  return html`
    <div class="transport">
      <div class="transport-buttons">
        <button type="button" class="btn icon" title="Restart (Home)"
                onClick=${() => { player.seek(0); }}>⏮</button>
        <button type="button" class="btn icon" title="Back one frame (←)"
                onClick=${() => player.step(-1)}>◀︎</button>
        <button type="button" class="btn primary play" title="Play / pause (Space)"
                onClick=${() => player.toggle()}>${playing ? '❚❚' : '▶'}</button>
        <button type="button" class="btn icon" title="Forward one frame (→)"
                onClick=${() => player.step(1)}>▶︎</button>
        <button type="button" class=${`btn icon${loop ? ' on' : ''}`} title="Loop (L)"
                aria-pressed=${loop ? 'true' : 'false'}
                onClick=${() => onLoop(!loop)}>⟲</button>
      </div>
      <input class="scrubber" type="range" min="0" max=${duration || 0} step="any"
             value=${Math.min(time, duration || 0)}
             onPointerDown=${() => { dragging.current = true; }}
             onPointerUp=${() => { dragging.current = false; }}
             onInput=${(e) => { const t = parseFloat(e.target.value); setTime(t); player.seek(t); }} />
      <div class="transport-readout">
        <span class="mono">${fmtDuration(time)} / ${fmtDuration(duration)}</span>
        <span class="mono dim">frame ${frame} / ${frames ?? '--'}</span>
      </div>
      <div class="rates" role="group" aria-label="Playback speed">
        ${RATES.map((r) => html`
          <button type="button" key=${r} class=${`btn tiny${r === rate ? ' on' : ''}`}
                  onClick=${() => player.setRate(r)}>${r}×</button>`)}
      </div>
    </div>`;
}
