// Finding the robot Foxglove is connected to, for panels that talk to its web
// services directly rather than through topics.
//
// There is no extension API for the data source, so there are two routes:
//  - the web app mirrors it into its own page URL as ?ds.url=... (the desktop
//    app does not);
//  - the episode manager node advertises its URLs on URLS_TOPIC, which any
//    panel can subscribe to over the bridge Foxglove is already connected to.

import { Immutable, MessageEvent } from "@foxglove/extension";

/** std_msgs/String, JSON {"urls": ["http://<lan ip>:7880/", ...]}, best first. */
export const URLS_TOPIC = "/alfie/episode_manager/urls";

/** The bridge URL Foxglove is connected to (web app only), e.g. ws://192.168.50.201:8765. */
export function dataSourceUrl(): string | undefined {
  try {
    return new URL(window.location.href).searchParams.get("ds.url") ?? undefined;
  } catch {
    return undefined;
  }
}

/** The episode manager's advertised URLs from this frame's messages, if any arrived. */
export function advertisedUrls(frame: Immutable<MessageEvent[]> | undefined): string[] | undefined {
  let urls: string[] | undefined;
  for (const event of frame ?? []) {
    if (event.topic !== URLS_TOPIC) {
      continue;
    }
    try {
      const data = (event.message as { data?: unknown }).data;
      const parsed = JSON.parse(String(data)) as { urls?: unknown };
      if (Array.isArray(parsed.urls)) {
        urls = parsed.urls.map(String);
      }
    } catch {
      // malformed message; keep looking
    }
  }
  return urls;
}

/** The robot's hostname: from the bridge URL when known, else the first advertised URL. */
export function robotHostname(
  bridgeUrl: string | undefined,
  urls: readonly string[] | undefined,
): string | undefined {
  for (const candidate of [bridgeUrl, urls?.[0]]) {
    if (candidate == undefined) {
      continue;
    }
    try {
      return new URL(candidate).hostname;
    } catch {
      // not a URL; try the next
    }
  }
  return undefined;
}
