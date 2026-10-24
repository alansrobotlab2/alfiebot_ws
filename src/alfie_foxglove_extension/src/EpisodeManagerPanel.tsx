// Placeholder for the GR00T episode manager panel. The backend stays the
// episode manager node's HTTP API (alfie_gr00t/episode_manager); this panel
// will replace its web UI inside a Foxglove layout.

import { PanelExtensionContext } from "@foxglove/extension";
import { useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

function EpisodeManagerPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  useLayoutEffect(() => {
    context.onRender = (_renderState, done) => {
      setRenderDone(() => done);
    };
  }, [context]);

  // Foxglove waits for done() before sending the next render.
  useLayoutEffect(() => {
    renderDone?.();
  }, [renderDone]);

  return (
    <div style={{ padding: "1rem" }}>
      <h2>Episode Manager</h2>
      <p>Coming soon.</p>
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
