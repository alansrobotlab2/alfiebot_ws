// Four <video> elements played as one: a shared transport, a master clock, and
// drift correction. The first camera that has loaded is the master; the others
// are nudged back onto it whenever they wander more than a couple of frames.

import { ReactElement, useEffect, useMemo, useRef, useState } from "react";

import { fmtDuration } from "./api";

const DRIFT_FRAMES = 2;

export type Player = {
  videos: (HTMLVideoElement | null)[];
  master: () => HTMLVideoElement | undefined;
  fps: () => number;
  time: () => number;
  duration: () => number;
  play: () => void;
  pause: () => void;
  toggle: () => void;
  seek: (t: number) => void;
  step: (frames: number) => void;
  setRate: (r: number) => void;
  sync: () => void;
};

export function usePlayer(fps: number | null | undefined): {
  player: Player;
  playing: boolean;
  setPlaying: (playing: boolean) => void;
  rate: number;
} {
  const [playing, setPlaying] = useState(false);
  const [rate, setRateState] = useState(1);
  const fpsRef = useRef(fps ?? 15);
  fpsRef.current = fps ?? 15;

  const player = useMemo(() => {
    const videos: (HTMLVideoElement | null)[] = [];
    const live = () =>
      videos.filter((v): v is HTMLVideoElement => v != undefined && v.readyState >= 1);
    const master = () => live()[0];
    const p: Player = {
      videos,
      master,
      fps: () => fpsRef.current,
      time: () => master()?.currentTime ?? 0,
      duration: () => {
        const d = master()?.duration;
        return d != undefined && Number.isFinite(d) ? d : 0;
      },
      play() {
        if (master()?.ended === true) {
          p.seek(0);
        }
        live().forEach((v) => {
          v.play().catch(() => undefined);
        });
        setPlaying(true);
      },
      pause() {
        live().forEach((v) => {
          v.pause();
        });
        setPlaying(false);
      },
      toggle() {
        const m = master();
        if (m && !m.paused) {
          p.pause();
        } else {
          p.play();
        }
      },
      seek(t) {
        const end = p.duration();
        const target = Math.max(0, end !== 0 ? Math.min(t, end - 1e-3) : t);
        live().forEach((v) => {
          v.currentTime = target;
        });
      },
      step(frames) {
        p.pause();
        // Land mid-frame so rounding never shows the neighbour.
        const frame = Math.round(p.time() * p.fps()) + frames;
        p.seek((Math.max(0, frame) + 0.5) / p.fps());
      },
      setRate(r) {
        videos.forEach((v) => {
          if (v) {
            v.playbackRate = r;
          }
        });
        setRateState(r);
      },
      sync() {
        const m = master();
        if (!m || m.paused) {
          return;
        }
        const slack = DRIFT_FRAMES / p.fps();
        for (const v of live()) {
          if (v === m) {
            continue;
          }
          if (Math.abs(v.currentTime - m.currentTime) > slack) {
            v.currentTime = m.currentTime;
          }
          if (v.paused) {
            v.play().catch(() => undefined);
          }
        }
      },
    };
    return p;
  }, []);

  return { player, playing, setPlaying, rate };
}

type Camera = { key: string; label: string };

export function VideoGrid({
  player,
  cameras,
  urls,
  rate,
  autoplay,
  onEnded,
}: {
  player: Player;
  cameras: readonly Camera[];
  urls: (string | undefined)[];
  rate: number;
  autoplay: boolean;
  onEnded: () => void;
}): ReactElement {
  // Drift correction runs on its own animation loop so it never re-renders React.
  useEffect(() => {
    let id = requestAnimationFrame(function tick() {
      player.sync();
      id = requestAnimationFrame(tick);
    });
    return () => {
      cancelAnimationFrame(id);
    };
  }, [player]);

  return (
    <div className="video-grid">
      {cameras.map((cam, i) => {
        const url = urls[i];
        return (
          <figure className="video-tile" key={cam.key}>
            {url != undefined ? (
              <video
                ref={(el) => {
                  player.videos[i] = el;
                }}
                src={url}
                muted
                playsInline
                preload="auto"
                autoPlay={autoplay}
                onLoadedMetadata={(e) => {
                  e.currentTarget.playbackRate = rate;
                }}
                onEnded={(e) => {
                  if (e.currentTarget === player.master()) {
                    onEnded();
                  }
                }}
              />
            ) : (
              <div
                className="video-missing"
                ref={() => {
                  player.videos[i] = null;
                }}
              >
                no video
              </div>
            )}
            <figcaption>{cam.label}</figcaption>
          </figure>
        );
      })}
    </div>
  );
}

const RATES = [0.25, 0.5, 1, 2, 4];

export function Transport({
  player,
  playing,
  rate,
  frames,
  loop,
  onLoop,
}: {
  player: Player;
  playing: boolean;
  rate: number;
  frames: number | null;
  loop: boolean;
  onLoop: (loop: boolean) => void;
}): ReactElement {
  const [time, setTime] = useState(0);
  const [duration, setDuration] = useState(0);
  const dragging = useRef(false);

  // The scrubber reads the master clock every frame without touching the rest
  // of the tree.
  useEffect(() => {
    let id = requestAnimationFrame(function tick() {
      if (!dragging.current) {
        setTime(player.time());
      }
      setDuration(player.duration());
      id = requestAnimationFrame(tick);
    });
    return () => {
      cancelAnimationFrame(id);
    };
  }, [player]);

  const fps = player.fps();
  const frame = Math.min(Math.floor(time * fps + 1e-6), Math.max(0, (frames ?? 1) - 1));

  return (
    <div className="transport">
      <div className="transport-buttons">
        <button
          type="button"
          className="btn icon"
          title="Restart (Home)"
          onClick={() => {
            player.seek(0);
          }}
        >
          ⏮
        </button>
        <button
          type="button"
          className="btn icon"
          title="Back one frame (←)"
          onClick={() => {
            player.step(-1);
          }}
        >
          ◀︎
        </button>
        <button
          type="button"
          className="btn primary play"
          title="Play / pause (Space)"
          onClick={() => {
            player.toggle();
          }}
        >
          {playing ? "❚❚" : "▶"}
        </button>
        <button
          type="button"
          className="btn icon"
          title="Forward one frame (→)"
          onClick={() => {
            player.step(1);
          }}
        >
          ▶︎
        </button>
        <button
          type="button"
          className={`btn icon${loop ? " on" : ""}`}
          title="Loop (L)"
          aria-pressed={loop}
          onClick={() => {
            onLoop(!loop);
          }}
        >
          ⟲
        </button>
      </div>
      <input
        className="scrubber"
        type="range"
        min="0"
        max={duration}
        step="any"
        value={Math.min(time, duration)}
        onPointerDown={() => {
          dragging.current = true;
        }}
        onPointerUp={() => {
          dragging.current = false;
        }}
        onChange={(e) => {
          const t = parseFloat(e.target.value);
          setTime(t);
          player.seek(t);
        }}
      />
      <div className="transport-readout">
        <span className="mono">
          {fmtDuration(time)} / {fmtDuration(duration)}
        </span>
        <span className="mono dim">
          frame {frame} / {frames ?? "--"}
        </span>
      </div>
      <div className="rates" role="group" aria-label="Playback speed">
        {RATES.map((r) => (
          <button
            type="button"
            key={r}
            className={`btn tiny${r === rate ? " on" : ""}`}
            onClick={() => {
              player.setRate(r);
            }}
          >
            {r}×
          </button>
        ))}
      </div>
    </div>
  );
}
