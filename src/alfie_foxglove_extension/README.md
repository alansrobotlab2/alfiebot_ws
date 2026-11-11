# alfie_foxglove_extension

Custom Foxglove panels for Alfie, packaged as one Foxglove extension.

| Panel | Backend |
|---|---|
| Episode Manager | `alfie_gr00t` episode manager node (HTTP API) |
| AlfieVR | `alfie_vr` web UI served by `alfie_teleop_vr` (HTTPS, port 8443) |
| Servo Tool | `alfie_tools` servotool node (HTTP API + SSE, port 7870) |

## Build

Needs Node.js 18+ and npm (not part of the ROS install). On the robot, Node is
unpacked in `~/.local/node` and is not on `PATH` by default:

```bash
export PATH=$HOME/.local/node/bin:$PATH    # on the robot
npm install
npm run build          # compile to dist/
npm run package        # -> alfie.alfie-foxglove-extension-<version>.foxe
npm run lint
```

`colcon build` does not run npm. It installs whatever `.foxe` is in this folder
to `share/alfie_foxglove_extension/extensions/`, so run `npm run package`
before `colcon build`.

Bump `version` in `package.json` (and add a `CHANGELOG.md` entry) when you ship
a change so Foxglove replaces the installed copy, and delete the previous
`.foxe` from this folder (`*.foxe` is gitignored, so it is never committed).

The panels that call the robot's web services through nginx (Episode Manager
and Servo Tool from the web app) need the current `systemd/foxglove-proxy.conf`
installed on the robot; see `systemd/FOXGLOVE-PROXY.md`.

## Install

The extension lives in the Foxglove app, not on the robot: connecting to
`foxglove_bridge` does not install it. On each machine, either

- drag the `.foxe` onto the Foxglove window (desktop or web app), or
- run `npm run local-install` there (desktop app only; copies into
  `~/.foxglove-studio/extensions/`).

## Episode Manager panel

A port of the episode manager's own web UI (`alfie_gr00t/episode_manager/web`)
that talks to the same HTTP API, so both work side by side.

1. The node runs with bringup (`launch_episode_manager:=true`, the default), or
   by hand: `ros2 run alfie_gr00t episode_manager`.
2. Add the **Episode Manager** panel. With **Server URL** left empty it finds
   the node through the robot Foxglove is connected to:
   - The node advertises its URLs on `/alfie/episode_manager/urls`
     (`std_msgs/String`, JSON `{"urls": [...]}`, latched), and the panel
     subscribes over the bridge. Works in the desktop and web apps.
   - The web app also has the bridge address in its page URL (`ds.url`), which
     the panel uses first: `ws://<robot>:8765` -> `http://<robot>:7880`,
     `wss://<robot>:8082` -> `https://<robot>:8082/episodes`.
   - From the web app (an HTTPS page) the panel always uses the nginx route,
     `https://<robot>:8082/episodes` (`systemd/foxglove-proxy.conf`), since it
     can't call plain HTTP; accept the robot's certificate once at
     `https://<robot>:8082/health`. The desktop app rejects that self-signed
     certificate (`ERR_CERT_AUTHORITY_INVALID`), so it uses `http://<robot>:7880`.

   Set **Server URL** in the panel settings to override it.
3. The node only answers cross-origin calls from its `allowed_origins`
   parameter (default `https://app.foxglove.dev`). It logs each origin it
   refuses once; if the panel can't connect, add the origin from that log line.

Keyboard shortcuts (J/K, Space, arrows, 1-9, Del) work while the panel has
focus, and take precedence over Foxglove's own while it does.

## Servo Tool panel

A port of servotool's own web UI (`alfie_tools/servotool/web`): pick a servo,
command its joint, read and edit its register map, plus the eyes and back.
It talks to the same HTTP API and state stream, so both work side by side and
share one set of control ownership; the node keeps every safety rule (take /
release per subsystem, the heartbeat deadman, clamping to URDF limits).

1. servotool is a bench tool and not part of bringup:
   `ros2 run alfie_tools servotool` on the robot.
2. Add the **Servo Tool** panel. With **Server URL** left empty it finds the
   node the same way the Episode Manager does: the node advertises its URLs on
   `/alfie/servotool/urls` (latched), and the web app's `ds.url` maps
   `ws://<robot>:8765` -> `http://<robot>:7870` and
   `wss://<robot>:8082` -> `https://<robot>:8082/servotool`
   (`systemd/foxglove-proxy.conf`; from the HTTPS web app the panel always goes
   through that route).
3. Cross-origin calls are allowed only from the node's `allowed_origins`
   parameter (default `https://app.foxglove.dev`); refused origins are logged
   once.

Holding control needs the panel to keep heartbeating. If Foxglove is hidden
long enough for the browser to throttle its timers, or the panel is closed,
the node releases everything after `control_timeout` (1.5 s) - and a released
arm goes limp.

## AlfieVR panel

The AlfieVR web UI (stereo preview, robot viewer, ROS logs) in an iframe, at
`https://<robot>:8443/` for the robot Foxglove is connected to (found the same
way as the Episode Manager; override with **Page URL** in the panel settings).

The page, and the connections it opens itself (VR websocket on 8442, the bridge
through nginx on 8082), all use the robot's self-signed certificate
(`src/alfie_vr/alfie_vr/cert.pem`, a CA certificate covering the robot's IPs).
An iframe can't show the browser's "proceed anyway" page, and the desktop app
never does, so until the certificate is trusted the panel stays blank. Import
`cert.pem` once as a trusted root CA on the computer running Foxglove:

- macOS: open it in Keychain Access, add to System, set "Always Trust".
- Windows: `certutil -addstore -f Root cert.pem` (admin prompt).
- Linux (Chrome/Electron use the NSS store):
  `certutil -d sql:$HOME/.pki/nssdb -A -t "C,," -n alfie -i cert.pem`

Restart Foxglove afterwards. This also makes the Episode Manager's HTTPS route
(`https://<robot>:8082/episodes`) work from the desktop app.
