// Foxglove glue for the episode manager: persisted panel config, the settings
// sidebar, light/dark theme, and the render handshake. The UI itself is in
// EpisodeManager.tsx.

import { PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useMemo, useState } from "react";
import { createRoot } from "react-dom/client";

import { EpisodeManager } from "./EpisodeManager";
import { makeApi } from "./api";
import "./styles.css";
import { URLS_TOPIC, advertisedUrls, dataSourceUrl } from "../robot";

type Config = {
  /** Episode manager base URL; empty means find it through the Foxglove connection. */
  serverUrl: string;
  autoAdvance: boolean;
  loop: boolean;
};

const DEFAULT_CONFIG: Config = { serverUrl: "", autoAdvance: true, loop: false };

// Where the episode manager lives relative to foxglove_bridge on the robot
// (alfie_bringup starts both; systemd/foxglove-proxy.conf fronts both on 8082).
const EPISODE_MANAGER_PORT = 7880;
const PROXY_PORT = 8082;
const PROXY_PATH = "/episodes";

/**
 * The episode manager on the same robot as the bridge: ws:// is a direct
 * connection, so talk to the node directly; wss:// comes through the nginx
 * proxy, which also serves the episode manager under /episodes.
 */
export function serverUrlFor(bridgeUrl: string | undefined): string | undefined {
  if (bridgeUrl == undefined) {
    return undefined;
  }
  try {
    const url = new URL(bridgeUrl);
    if (url.protocol === "wss:") {
      return `https://${url.host}${PROXY_PATH}`;
    }
    if (url.protocol === "ws:") {
      return `http://${url.hostname}:${EPISODE_MANAGER_PORT}`;
    }
  } catch {
    // not a URL; fall through
  }
  return undefined;
}

/**
 * The episode manager URL to use from the node's best advertised URL. An HTTPS
 * page (the Foxglove web app) can't call plain HTTP, so it goes through the
 * nginx proxy on the same host instead.
 */
function advertisedServerUrl(first: string): string | undefined {
  try {
    if (window.location.protocol === "https:") {
      return `https://${new URL(first).hostname}:${PROXY_PORT}${PROXY_PATH}`;
    }
    return first.replace(/\/+$/, "");
  } catch {
    return undefined;
  }
}

function Setup({ onUrl }: { onUrl: (url: string) => void }): ReactElement {
  const [draft, setDraft] = useState(`http://alfiebot.local:${EPISODE_MANAGER_PORT}`);
  return (
    <div className="boot">
      <form
        className="setup panel"
        onSubmit={(e) => {
          e.preventDefault();
          onUrl(draft.trim());
        }}
      >
        <h2>Connect to the episode manager</h2>
        <p className="dim">
          Waiting for the robot to advertise the episode manager on <code>{URLS_TOPIC}</code>. That
          needs Foxglove connected to the robot&apos;s bridge and the <code>episode_manager</code>{" "}
          node running (bringup starts it). Or enter the <code>episode_manager</code> node&apos;s
          URL: <code>http://&lt;robot&gt;:7880</code>, or{" "}
          <code>https://&lt;robot&gt;:8082/episodes</code> through the nginx proxy (the only option
          from the Foxglove web app, which can&apos;t call plain HTTP).
        </p>
        <input
          type="url"
          value={draft}
          onChange={(e) => {
            setDraft(e.target.value);
          }}
        />
        <button type="submit" className="btn primary" disabled={!draft.trim()}>
          Connect
        </button>
      </form>
    </div>
  );
}

function EpisodeManagerPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [config, setConfig] = useState<Config>(() => ({
    ...DEFAULT_CONFIG,
    ...(context.initialState as Partial<Config> | undefined),
  }));
  const [colorScheme, setColorScheme] = useState<"dark" | "light">("dark");
  const [bridgeUrl, setBridgeUrl] = useState(dataSourceUrl);
  const [advertisedUrl, setAdvertisedUrl] = useState<string | undefined>();
  const [overlay, setOverlay] = useState<HTMLDivElement | null>(null);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const update = (patch: Partial<Config>) => {
    setConfig((prev) => ({ ...prev, ...patch }));
  };

  useEffect(() => {
    context.saveState(config);
  }, [context, config]);

  useEffect(() => {
    context.updatePanelSettingsEditor({
      actionHandler: (action: SettingsTreeAction) => {
        if (action.action !== "update") {
          return;
        }
        const [, field] = action.payload.path;
        const { value } = action.payload;
        if (field === "serverUrl") {
          update({ serverUrl: typeof value === "string" ? value.trim() : "" });
        } else if (field === "autoAdvance" || field === "loop") {
          update({ [field]: value === true });
        }
      },
      nodes: {
        general: {
          label: "Episode manager",
          fields: {
            serverUrl: {
              label: "Server URL",
              input: "string",
              value: config.serverUrl,
              placeholder:
                serverUrlFor(bridgeUrl) ??
                advertisedUrl ??
                `http://<robot>:${EPISODE_MANAGER_PORT}`,
              help:
                "Base URL of the episode_manager node (alfie_gr00t). Leave empty to use the " +
                "robot Foxglove is connected to.",
            },
            autoAdvance: {
              label: "Next after labelling",
              input: "boolean",
              value: config.autoAdvance,
            },
            loop: { label: "Loop playback", input: "boolean", value: config.loop },
          },
        },
      },
    });
  }, [context, config, bridgeUrl, advertisedUrl]);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      if (renderState.colorScheme) {
        setColorScheme(renderState.colorScheme);
      }
      // Topics change whenever Foxglove (re)connects, which is when ds.url can change.
      setBridgeUrl(dataSourceUrl());
      const first = advertisedUrls(renderState.currentFrame)?.[0];
      if (first != undefined) {
        setAdvertisedUrl(advertisedServerUrl(first));
      }
      setRenderDone(() => done);
    };
    context.watch("colorScheme");
    context.watch("topics");
    context.watch("currentFrame");
    context.subscribe([{ topic: URLS_TOPIC }]);
  }, [context]);

  // Foxglove waits for done() before sending the next render.
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // A URL set by hand wins; then the bridge address (web app only: it keeps
  // ds.url in its page URL); then whatever the node advertises over the bridge.
  const serverUrl = config.serverUrl || (serverUrlFor(bridgeUrl) ?? advertisedUrl);
  const api = useMemo(() => makeApi(serverUrl ?? ""), [serverUrl]);

  return (
    <div className="em" data-theme={colorScheme}>
      <div className="em-scroll">
        {serverUrl == undefined ? (
          <Setup
            onUrl={(url) => {
              update({ serverUrl: url });
            }}
          />
        ) : (
          overlay && (
            <EpisodeManager
              key={serverUrl}
              api={api}
              serverUrl={serverUrl}
              overlay={overlay}
              autoAdvance={config.autoAdvance}
              onAutoAdvance={(autoAdvance) => {
                update({ autoAdvance });
              }}
              loop={config.loop}
              onLoop={(loop) => {
                update({ loop });
              }}
            />
          )
        )}
      </div>
      <div className="em-overlay" ref={setOverlay} />
    </div>
  );
}

export function initEpisodeManagerPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<EpisodeManagerPanel context={context} />);
  return () => {
    root.unmount();
  };
}
