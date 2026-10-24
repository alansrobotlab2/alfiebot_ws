# alfie_foxglove_extension

Custom Foxglove panels for Alfie, packaged as one Foxglove extension.

| Panel | Backend |
|---|---|
| Episode Manager | `alfie_gr00t` episode manager node (HTTP API) |

## Build

Needs Node.js 18+ and npm (not part of the ROS install):

```bash
npm install
npm run build          # compile to dist/
npm run package        # -> alfie.alfie-foxglove-extension-<version>.foxe
```

`colcon build` does not run npm. It installs whatever `.foxe` is in this folder
to `share/alfie_foxglove_extension/extensions/`, so run `npm run package`
before `colcon build`.

## Install

The extension lives in the Foxglove app, not on the robot: connecting to
`foxglove_bridge` does not install it. On each machine, either

- drag the `.foxe` onto the Foxglove window (desktop or web app), or
- run `npm run local-install` there (desktop app only; copies into
  `~/.foxglove-studio/extensions/`).

Bump `version` in `package.json` when you ship a change so Foxglove replaces
the installed copy.
