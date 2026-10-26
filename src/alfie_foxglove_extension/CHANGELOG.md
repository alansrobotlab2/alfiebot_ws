# Changelog

## 0.2.0

- AlfieVR panel: the alfie_vr web UI (https://<robot>:8443/) in an iframe,
  pointed at the robot Foxglove is connected to.
- Robot lookup (ds.url, /alfie/episode_manager/urls) shared by both panels.

## 0.1.2

- Episode Manager: find the server through the bridge. The node advertises
  its URLs on /alfie/episode_manager/urls, which the panel subscribes to; the
  desktop app has no ds.url for 0.1.1's lookup to read.

## 0.1.1

- Episode Manager: with no Server URL set, use the robot Foxglove is connected
  to (ws://<robot>:8765 -> http://<robot>:7880, wss://<robot>:8082 ->
  https://<robot>:8082/episodes).

## 0.1.0

- Episode Manager panel: a port of the alfie_gr00t episode manager web UI,
  talking to the episode_manager node's HTTP API.
