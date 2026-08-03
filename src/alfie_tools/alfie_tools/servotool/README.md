# ServoTool3

Servo bring-up and diagnostics for **gen2 Alfie**, in a browser.

```bash
ros2 run alfie_tools servotool          # then open http://alfiebot.local:7870/
```

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

So servotool is not a register editor — there is nothing to edit over ROS. It
is a **live monitor and joint commander**: radians, rad/s, rad/s², and the
0-1000 torque limit, exactly as `ServoCmd`/`ServoState` define them.

Changing EEPROM parameters (PID gains, angle limits, servo IDs) now means
talking to the bus directly with the standalone firmware utilities in
`alfie_firmware/` (`picosetservoid`, `picoservoping`), not through ROS.

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

```bash
ros2 run alfie_tools servotool --ros-args -p port:=8080 -p host:=127.0.0.1
```

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
├── servotool3_node.py       ROS entry point (node + executor + HTTP server)
├── ros/
│   ├── joint_config.py      joint table, URDF limits, status-bit decode
│   └── servo_bridge.py      subscriptions, mux publishing, control ownership
├── server/
│   └── http_server.py       stdlib HTTP: static files, JSON API, SSE stream
└── web/
    ├── index.html, app.js, api.js, styles.css
    ├── components/          TopBar, ServoPanel, JointRow, EyesPanel, BackPanel
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
