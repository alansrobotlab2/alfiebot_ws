# Stereo VR Integration Gameplan

Integrate features from `index_old.html` into `index.html` to create a unified stereo VR teleoperation interface.

Don't check anything off on your own.  I will check them off when I confirm the task is complete.

---

## Phase 2: VR Controller Tracking

- [X] **2.1 Add WebSocket connection for controller data** – Connect to WSS port `8442` for streaming controller telemetry
- [X] **2.2 Implement XR input source tracking** – Use WebXR gamepad API to read controller position, rotation, buttons, triggers, thumbsticks
- [X] **2.3 Stream controller data to WebSocket** – Send left/right hand poses, grip, trigger, thumbstick, and button states
- [X] **2.4 Add headset pose tracking** – Include head position/rotation in the data stream

---

## Phase 3: Robot State Visualization (Foxglove)

- [ ] **3.1 Add Foxglove WebSocket connection** – Connect to WSS port `8082` for ROS bridge
- [ ] **3.2 Subscribe to robot topics** – `/robot_description` (URDF) and `/tf` (transforms)
- [ ] **3.3 Create robot status panel (desktop)** – Display Foxglove connection status, TF rate, joint states
- [ ] **3.4 Create robot status panel (VR)** – Head-locked panel showing robot state in immersive mode
- [ ] **3.5 (Optional) URDF mesh rendering** – Parse URDF and load OBJ meshes for 3D robot visualization

---

## Phase 4: Settings & Configuration

- [ ] **4.1 Add settings button and modal** – Port settings UI from `modal.js`
- [ ] **4.2 Add configuration fields** – Robot arm names, ports, host IP, control parameters
- [ ] **4.3 Add API integration** – Settings load/save via `/api/settings`, restart via `/api/restart`
- [ ] **4.4 Persist settings to localStorage** – Merge with existing stereo adjustment settings

---

## Phase 5: Status Indicators & Controls

- [ ] **5.1 Add connection status indicators** – Dots for Left Arm, Right Arm, VR, Foxglove, Video streams
- [ ] **5.2 Add keyboard control (desktop)** – WASD/arrow key teleoperation fallback
- [ ] **5.3 Add Start Controller Tracking button** – Dynamic XR session control button
- [ ] **5.4 Add passthrough dimmer control** – Adjustable background opacity in AR mode

---

## Phase 6: Code Cleanup & Optimization

- [ ] **6.1 Unify port configuration** – Consolidate all ports (8082, 8083, 8084, 8442) in one config object
- [ ] **6.2 Add reconnection logic** – Auto-reconnect for all WebSocket/WebRTC connections
- [ ] **6.3 Add error handling UI** – User-friendly error messages for connection failures
- [ ] **6.4 Test on Meta Quest 3** – Verify stereo rendering, controller tracking, and all panels in VR

---

## Hardware Configuration

- **Stereo Camera**: USB-C stereo camera (replaced OAK-D Lite)
- **Video Format**: Single frame with left/right images side-by-side (SBS)
- **Processing**: WebGL shader splits the SBS frame and renders each half to the corresponding eye

---

## Notes

- **Keeping raw WebXR/WebGL** for stereo rendering (not switching to A-Frame)
- **SBS video handling**: The stereo camera publishes a single combined frame; the WebGL shader samples the left half for the left eye and the right half for the right eye
- **URDF rendering** can be simplified to joint-state text display initially
