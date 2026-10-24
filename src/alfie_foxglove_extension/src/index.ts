import { ExtensionContext } from "@foxglove/extension";

import { initEpisodeManagerPanel } from "./EpisodeManagerPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Episode Manager", initPanel: initEpisodeManagerPanel });
}
