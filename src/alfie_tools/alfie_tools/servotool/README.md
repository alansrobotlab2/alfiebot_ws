# ServoTool

Servo bring-up and diagnostics for **gen2 Alfie**, in a browser.

```bash
ros2 run alfie_tools servotool          # then open http://alfiebot.local:7870/
```

It is a bench tool, not part of bringup: start it by hand when you need it, and
stop it with Ctrl-C (it does not exit on SIGTERM; use `kill -INT <pid>`). The
same UI is also available as the **Servo Tool** panel in Foxglove (see
[Foxglove panel](#foxglove-panel)).

## What changed from servotool / servotool2

Gen1 exposed a `GDBServoService` that read and wrote raw servo registers, and a
`GDBState` topic carrying the whole memory map. **Neither exists in gen2.** Each
Pico driver board owns its servo bus and its memory map, and publishes only SI
units:

| module | state topic | command topic |
|---|---|---|
| left arm | `alfie/low/left_arm/armstate` (`ArmState`) | `alfie/low/left_arm/armcmd` |
| right arm | `alfie/low/right_arm/armstate` (`ArmState`) | `alfie/low/right_arm/armcmd` |
| head | `alfie/low/headstate` (`HeadState`) | `alfie/low/headcmd` |
| back | `alfie/low/backstate` (`BackState`) | `alfie/low/backcmd` |

Joint control works in those SI units: radians, rad/s, rad/s², and the 0-1000
torque limit, exactly as `ServoCmd`/`ServoState` define them.

Registers are reached per module through a `ServoService`
(`alfie/low/left_arm/servoservice`, `alfie/low/right_arm/servoservice`,
`alfie/low/headservoservice`), served by the Pico firmware from its servo tick.
The page shows **one physical servo at a time**: its joint control on the left
and its register map on the right, polled at `memory_poll_hz`. EEPROM registers
(PID gains, angle limits, protection settings) can be edited in place once the
servo's write lock (0x37) is cleared and its torque is off; every write is read
back by the firmware. Servo IDs are still set with the standalone firmware
utilities in `alfie_firmware/` (`picosetservoid`, `picoservoping`).

## Command path

servotool never publishes to the firmware topics. It publishes to
**command_mux** as the source `tool`:

```
cmd/left_arm/tool   ArmCmd     cmd/head/tool  HeadCmd
cmd/right_arm/tool  ArmCmd     cmd/eyes/tool  EyeCmd
cmd/back/tool       BackCmd
```

`tool` sits at priority 110 — above `vr` (100) and everything autonomous. That
is deliberate: it is a bench tool with an explicit take/release per subsystem
and an operator deadman, so whoever is standing at the robot with it open wins.
It publishes nothing at all unless a subsystem is held, so it is invisible the
rest of the time. It does not drive the base.

## Safety model

* **Take control** per subsystem, explicitly. Targets are seeded from live
  feedback, so the first command is "stay exactly where you are".
* **Operator deadman.** The browser heartbeats at ~2.5 Hz; after
  `control_timeout` (default 1.5 s) without one, every held subsystem is
  released. Closing the tab, losing wifi, or killing the browser all release.
* **Release means limp.** Releasing stops publishing. The mux forwards the next
  fresh source and, if there is none, the module firmware watchdog torques the
  servos off ~500 ms later — a held arm falls. Same as any commander dying.
  Lower the arm before releasing.
* **E-stop** is always one click away and latches in the mux. Reset is a
  deliberate `estop_reset` service call; the mux's resume fence then ignores any
  source that hasn't published since.
* Commanded values are clamped server-side to the URDF joint limits and to
  `max_speed`, not just in the UI.

Two servo-command gotchas the UI surfaces, because both bite during bring-up:
`target_speed = 0` means *unlimited* to the servo, and `target_torque = 0` means
the joint has no torque limit budget at all and cannot hold.

## Parameters

| parameter | default | meaning |
|---|---|---|
| `host` | `0.0.0.0` | bind address |
| `port` | `7870` | HTTP port |
| `stream_rate_hz` | `10` | state push rate over SSE |
| `robot_namespace` | `alfie` | namespace the robot runs in |
| `source` | `tool` | command_mux source name to publish as |
| `forward_rate_hz` | `20` | command rate while holding control |
| `control_timeout` | `1.5` | seconds without a heartbeat before releasing |
| `max_speed` | `6.0` | rad/s ceiling on commanded joint speed |
| `memory_poll_hz` | `2.0` | register-map read rate for the selected servo |
| `allowed_origins` | `["https://app.foxglove.dev"]` | origins allowed to call the API cross-origin (the Foxglove panel); `"*"` allows any |
| `urls_topic` | `/alfie/servotool/urls` | latched `std_msgs/String` advertising the UI URLs, for the Foxglove panel |

```bash
ros2 run alfie_tools servotool --ros-args -p port:=8080 -p host:=127.0.0.1
```

## Foxglove panel

The **Servo Tool** panel in `alfie_foxglove_extension` is a second front end
on the same API and state stream. It finds the node through `urls_topic` over
the bridge, or through the nginx route `https://<robot>:8082/servotool/`
(`systemd/foxglove-proxy.conf`) from the HTTPS web app. Cross-origin POSTs from
anything outside `allowed_origins` are refused with 403, since a stray page on
the LAN could otherwise move the arms.

## Getting to it over wifi

The default bind is `0.0.0.0`, so it is already reachable from any laptop,
tablet or phone on the same wifi as the robot. Startup logs the URLs that
actually work, e.g.

```
[INFO] [servotool]: servotool web UI: http://192.168.50.201:7870/  http://alfiebot.local:7870/  http://127.0.0.1:7870/
```

Prefer the `alfiebot.local` form — avahi publishes it, so it survives the robot
picking up a different DHCP lease. It resolves out of the box on macOS, iOS and
Windows 10+; on Linux clients it needs `libnss-mdns`. Fall back to the raw IP if
mDNS is filtered (some guest/isolated wifi networks block multicast, which also
means those networks may block the connection entirely).

Nothing else is required: there is no ufw/firewalld on the robot image, and the
port is not privileged.

**There is no authentication.** Anyone who can reach the port can move the arms.
That is the trade for being able to open it from a phone while standing at the
robot — keep it on a trusted network. On an untrusted one, bind to loopback and
forward the port over SSH instead:

```bash
ros2 run alfie_tools servotool --ros-args -p host:=127.0.0.1     # on the robot
ssh -N -L 7870:127.0.0.1:7870 alfie@alfiebot.local               # on the laptop
```

## Layout

```
servotool/
├── servotool_node.py        ROS entry point (node + executor + HTTP server)
├── ros/
│   ├── joint_config.py      joint table, URDF limits, status-bit decode
│   └── servo_bridge.py      subscriptions, mux publishing, control ownership,
│                            register reads/writes
├── server/
│   └── http_server.py       stdlib HTTP: static files, JSON API, SSE stream
└── web/
    ├── index.html, app.js, api.js, styles.css
    ├── components/          TopBar, ServoSelector, AngleColumn, LiveStrip,
    │                        MemoryPanel, EyesPanel, BackPanel
    └── vendor/              React 18 + htm (UMD, vendored)
```

No build step and no Python dependencies beyond `rclpy`. The Jetson has no
node/npm, so React is vendored as UMD and `htm` provides JSX-like syntax through
tagged templates — `ros2 run` works on a fresh image with only ROS installed.

## HTTP API

| method | path | body |
|---|---|---|
| GET | `/api/config` | static layout, limits, defaults |
| GET | `/api/state` | one state snapshot |
| GET | `/api/stream` | SSE, a snapshot per tick |
| POST | `/api/select` | `{subsystem, bus_id}` - which servo the register map shows |
| POST | `/api/register` | `{subsystem, bus_id, address, value}` |
| POST | `/api/lock` | `{subsystem, bus_id, locked}` |
| POST | `/api/control` | `{subsystem, action: take\|release\|release_all}` |
| POST | `/api/command` | `{subsystem, joints:[{index, ...}]}` / `{eye_pwm}` / `{back}` |
| POST | `/api/torque_off` | `{subsystem}` (omit for all servo modules) |
| POST | `/api/heartbeat` | — |
| POST | `/api/estop` | `{action: engage\|reset}` |
| POST | `/api/calibrate_back` | — |

## Troubleshooting

**"no state from …" when taking control** — that module isn't publishing. Check
the micro-ROS agent and the board:

```bash
ros2 topic hz /alfie/low/left_arm/armstate
ros2 topic hz /alfie/low/headstate
```

**Panels show data but nothing moves** — `command_mux` has to be running; the
tool only ever talks to it. Check `ros2 topic echo /alfie/estop_state`, and that
the mux config knows the `tool` source (`alfie_bringup/config/command_mux.yaml`).

**An arm reports `side: unknown`** — the board's serial isn't in the firmware's
provisioning table, and `applyArmCmd` refuses to energise servos in that state.
The board serial shown on the panel is what needs adding to `arm_control.cpp`.
