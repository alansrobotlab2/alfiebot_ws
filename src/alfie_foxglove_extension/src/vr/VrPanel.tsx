// AlfieVR web UI (alfie_vr, served by alfie_teleop_vr) embedded in a panel:
// the stereo preview, robot viewer and ROS log panels, next to the rest of a
// Foxglove layout.
//
// The page is HTTPS with the robot's self-signed certificate (alfie_vr/cert.pem),
// and so are the connections it opens itself (VR websocket on 8442, the bridge
// through nginx on 8082). A browser lets you click through the warning on a
// top-level page, but never inside an iframe, and the Foxglove desktop app not
// at all: the frame just stays blank until the certificate is trusted.

import { PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

import { URLS_TOPIC, advertisedUrls, dataSourceUrl, robotHostname } from "../robot";
import "./vr.css";

/** alfie_vr/config.yaml network.https_port */
const VR_HTTPS_PORT = 8443;

type Config = {
  /** Page URL; empty means https://<robot>:8443/ for the robot Foxglove is connected to. */
  url: string;
};

const DEFAULT_CONFIG: Config = { url: "" };

function VrPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [config, setConfig] = useState<Config>(() => ({
    ...DEFAULT_CONFIG,
    ...(context.initialState as Partial<Config> | undefined),
  }));
  const [colorScheme, setColorScheme] = useState<"dark" | "light">("dark");
  const [bridgeUrl, setBridgeUrl] = useState(dataSourceUrl);
  const [advertised, setAdvertised] = useState<string | undefined>();
  const [reloads, setReloads] = useState(0);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const host = robotHostname(bridgeUrl, advertised == undefined ? undefined : [advertised]);
  const derived = host == undefined ? undefined : `https://${host}:${VR_HTTPS_PORT}/`;
  const url = config.url || derived;

  useEffect(() => {
    context.saveState(config);
  }, [context, config]);

  useEffect(() => {
    context.updatePanelSettingsEditor({
      actionHandler: (action: SettingsTreeAction) => {
        if (action.action === "update" && action.payload.path[1] === "url") {
          const { value } = action.payload;
          setConfig({ url: typeof value === "string" ? value.trim() : "" });
        }
      },
      nodes: {
        general: {
          label: "AlfieVR",
          fields: {
            url: {
              label: "Page URL",
              input: "string",
              value: config.url,
              placeholder: derived ?? `https://<robot>:${VR_HTTPS_PORT}/`,
              help:
                "The alfie_teleop_vr web UI. Leave empty to use the robot Foxglove is " +
                "connected to.",
            },
          },
        },
      },
    });
  }, [context, config, derived]);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      if (renderState.colorScheme) {
        setColorScheme(renderState.colorScheme);
      }
      setBridgeUrl(dataSourceUrl());
      const first = advertisedUrls(renderState.currentFrame)?.[0];
      if (first != undefined) {
        setAdvertised(first);
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

  if (url == undefined) {
    return (
      <div className="vr" data-theme={colorScheme}>
        <div className="vr-empty">
          <p>Looking for the robot…</p>
          <p className="vr-dim">
            The panel uses the robot Foxglove is connected to, which the episode manager advertises
            on <code>{URLS_TOPIC}</code>. Or set the page URL in the panel settings, e.g.{" "}
            <code>https://&lt;robot&gt;:{VR_HTTPS_PORT}/</code>.
          </p>
        </div>
      </div>
    );
  }

  return (
    <div className="vr" data-theme={colorScheme}>
      <div className="vr-bar">
        <code className="vr-url" title={url}>
          {url}
        </code>
        <button
          type="button"
          title="Reload the page"
          onClick={() => {
            setReloads((n) => n + 1);
          }}
        >
          ⟳ Reload
        </button>
        <a href={url} target="_blank" rel="noreferrer" title="Open in its own window">
          ↗ Open
        </a>
        <span
          className="vr-dim vr-hint"
          title={
            "The robot's certificate (alfie_vr/cert.pem) has to be trusted for the page, its " +
            "VR websocket (:8442) and the bridge proxy (:8082). Inside a panel there is no " +
            "warning to click through: import cert.pem as a trusted CA on this computer."
          }
        >
          Blank? Trust the robot&apos;s certificate.
        </span>
      </div>
      <iframe
        key={`${url}#${reloads}`}
        className="vr-frame"
        src={url}
        title="AlfieVR"
        allow="xr-spatial-tracking; fullscreen; autoplay"
      />
    </div>
  );
}

export function initVrPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<VrPanel context={context} />);
  return () => {
    root.unmount();
  };
}
