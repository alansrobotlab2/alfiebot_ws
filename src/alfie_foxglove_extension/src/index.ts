import { ExtensionContext } from "@foxglove/extension";

import { initEpisodeManagerPanel } from "./episodeManager/EpisodeManagerPanel";
import { initVrPanel } from "./vr/VrPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Episode Manager", initPanel: initEpisodeManagerPanel });
  extensionContext.registerPanel({ name: "AlfieVR", initPanel: initVrPanel });
}
