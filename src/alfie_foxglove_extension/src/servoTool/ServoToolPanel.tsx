// Foxglove glue for the servo tool: persisted panel config, the settings
// sidebar, light/dark theme, finding the servotool node, and the render
// handshake. The UI itself is in ServoTool.tsx.

import { PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useMemo, useState } from "react";
import { createRoot } from "react-dom/client";

import { ServoTool } from "./ServoTool";
import { makeApi } from "./api";
import "./styles.css";
import { SERVOTOOL_URLS_TOPIC, advertisedUrls, dataSourceUrl } from "../robot";

type PanelConfig = {
  /** servotool base URL; empty means find it through the Foxglove connection. */
  serverUrl: string;
};

const DEFAULT_CONFIG: PanelConfig = { serverUrl: "" };

// Where servotool lives relative to foxglove_bridge on the robot
// (systemd/foxglove-proxy.conf fronts both on 8082).
const SERVOTOOL_PORT = 7870;
const PROXY_PORT = 8082;
const PROXY_PATH = "/servotool";

/**
 * servotool on the same robot as the bridge: ws:// is a direct connection, so
 * talk to the node directly; wss:// comes through the nginx proxy, which also
 * serves servotool under /servotool.
 */
function serverUrlFor(bridgeUrl: string | undefined): string | undefined {
  if (bridgeUrl == undefined) {
    return undefined;
  }
  try {
    const url = new URL(bridgeUrl);
    if (url.protocol === "wss:") {
      return `https://${url.host}${PROXY_PATH}`;
    }
    if (url.protocol === "ws:") {
      return `http://${url.hostname}:${SERVOTOOL_PORT}`;
    }
  } catch {
    // not a URL; fall through
  }
  return undefined;
}

/**
 * The URL to use from the node's best advertised URL. An HTTPS page (the
 * Foxglove web app) can't call plain HTTP, so it goes through the nginx proxy
 * on the same host instead.
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
  const [draft, setDraft] = useState(`http://alfiebot.local:${SERVOTOOL_PORT}`);
  return (
    <div className="boot">
      <form
        className="boot-card"
        onSubmit={(e) => {
          e.preventDefault();
          onUrl(draft.trim());
        }}
      >
        <h2>Connect to servotool</h2>
        <p className="hint">
          Waiting for the robot to advertise servotool on <code>{SERVOTOOL_URLS_TOPIC}</code>. That
          needs Foxglove connected to the robot&apos;s bridge and the servotool node running (
          <code>ros2 run alfie_tools servotool</code>; bringup does not start it). Or enter its URL:{" "}
          <code>http://&lt;robot&gt;:{SERVOTOOL_PORT}</code>, or{" "}
          <code>https://&lt;robot&gt;:{PROXY_PORT}/servotool</code> through the nginx proxy (the
          only option from the Foxglove web app, which can&apos;t call plain HTTP).
        </p>
        <div className="boot-row">
          <input
            type="url"
            value={draft}
            onChange={(e) => {
              setDraft(e.target.value);
            }}
          />
          <button type="submit" className="btn btn-primary" disabled={!draft.trim()}>
            Connect
          </button>
        </div>
      </form>
    </div>
  );
}

function ServoToolPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [config, setConfig] = useState<PanelConfig>(() => ({
    ...DEFAULT_CONFIG,
    ...(context.initialState as Partial<PanelConfig> | undefined),
  }));
  const [colorScheme, setColorScheme] = useState<"dark" | "light">("dark");
  const [bridgeUrl, setBridgeUrl] = useState(dataSourceUrl);
  const [advertisedUrl, setAdvertisedUrl] = useState<string | undefined>();
  const [overlay, setOverlay] = useState<HTMLDivElement | null>(null);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  useEffect(() => {
    context.saveState(config);
  }, [context, config]);

  useEffect(() => {
    context.updatePanelSettingsEditor({
      actionHandler: (action: SettingsTreeAction) => {
        if (action.action === "update" && action.payload.path[1] === "serverUrl") {
          const { value } = action.payload;
          setConfig({ serverUrl: typeof value === "string" ? value.trim() : "" });
        }
      },
      nodes: {
        general: {
          label: "Servo tool",
          fields: {
            serverUrl: {
              label: "Server URL",
              input: "string",
              value: config.serverUrl,
              placeholder:
                serverUrlFor(bridgeUrl) ?? advertisedUrl ?? `http://<robot>:${SERVOTOOL_PORT}`,
              help:
                "Base URL of the servotool node (alfie_tools). Leave empty to use the robot " +
                "Foxglove is connected to.",
            },
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
      const first = advertisedUrls(renderState.currentFrame, SERVOTOOL_URLS_TOPIC)?.[0];
      if (first != undefined) {
        setAdvertisedUrl(advertisedServerUrl(first));
      }
      setRenderDone(() => done);
    };
    context.watch("colorScheme");
    context.watch("topics");
    context.watch("currentFrame");
    context.subscribe([{ topic: SERVOTOOL_URLS_TOPIC }]);
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
    <div className="st" data-theme={colorScheme}>
      <div className="st-scroll">
        {serverUrl == undefined ? (
          <Setup
            onUrl={(url) => {
              setConfig({ serverUrl: url });
            }}
          />
        ) : (
          overlay && <ServoTool key={serverUrl} api={api} serverUrl={serverUrl} overlay={overlay} />
        )}
      </div>
      <div className="st-overlay" ref={setOverlay} />
    </div>
  );
}

export function initServoToolPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<ServoToolPanel context={context} />);
  return () => {
    root.unmount();
  };
}
