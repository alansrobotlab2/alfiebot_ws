# Changelog

## 0.4.0

- Episode Manager: recording sessions and the task registry. Start, switch and
  close sessions; set what the next recordings are (task by ID from the
  registry, source, correction round, operator, surface); relabel by task ID;
  rate episodes (ok / marginal / bad), tag why a correction was taken over, add
  notes. A closed session is read-only. Needs the alfie_gr00t episode manager
  and data recorder of the same change.
- Episode Manager: record first, tag later. Convert takes every new bag; one
  recorded without a task becomes an unlabelled episode. An Unlabelled filter
  lists them, and the task picker tags them: type a task number or a few
  words, Enter. The number keys 1-9 no longer tag; T focuses the picker.

## 0.3.0

- Servo Tool panel: a port of the servotool web UI (alfie_tools), talking to
  the servotool node's HTTP API and state stream. Finds the node through
  /alfie/servotool/urls or the bridge address, like the Episode Manager.

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
