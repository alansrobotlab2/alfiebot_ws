# Stereo VR Integration Gameplan

Integrate features into `index.html` to create a unified stereo VR teleoperation interface.

Don't check anything off on your own.  I will check them off when I confirm the task is complete.

leverage the existing implementation of key features from vr_app.js and urdf_viewer.js for reference.

---

## Phase 2: VR Controller Tracking

- [X] **2.1 Add WebSocket connection for controller data** – Connect to WSS port `8442` for streaming controller telemetry
- [X] **2.2 Implement XR input source tracking** – Use WebXR gamepad API to read controller position, rotation, buttons, triggers, thumbsticks
- [X] **2.3 Stream controller data to WebSocket** – Send left/right hand poses, grip, trigger, thumbstick, and button states
- [X] **2.4 Add headset pose tracking** – Include head position/rotation in the data stream

---

## Phase 3: Robot State Visualization (Foxglove)

### Analysis of Existing Code

**`urdf_viewer.js`** (1075 lines) - Desktop Three.js URDF viewer:
- `URDFViewer` class with full Foxglove WebSocket integration
- Connects via `wss://hostname:8082` using `foxglove.sdk.v1` subprotocol
- Subscribes to `/alfie/tf`, `/alfie/tf_static`, `/alfie/joint_states`, `/alfie/robot_description`
- Handles binary CDR-encoded messages for TF and robot_description
- Parses URDF XML, builds link hierarchy, loads OBJ meshes
- TF rate tracking (updates every 5 seconds)
- Auto-reconnect on disconnect

**`vr_app.js`** (1573 lines) - A-Frame VR robot viewer component:
- `vr-robot-viewer` A-Frame component
- Same Foxglove connection pattern as urdf_viewer.js
- URDF parsing creates A-Frame entities instead of Three.js objects
- Batched/throttled updates (100ms interval) to prevent VR jitter
- Pending update queues applied in `tick()` function

**`stereo_vr.js`** (1518 lines) - Current stereo VR system:
- Raw WebXR/WebGL for stereo video rendering
- Already has controller WebSocket on port 8442
- Uses Foxglove for compressed image subscription
- No robot visualization yet

### Step-by-Step Integration Plan

**3.1 Desktop Status Panel (HTML/CSS only)**
- [X] **3.1.1** Add robot viewer panel HTML to `index.html` (container div, status elements)
- [X] **3.1.2** Add CSS for panel styling in `stereo_styles.css` (collapsible, semi-transparent)
- [X] **3.1.3** Test: Panel appears on page, shows placeholder text

**3.2 Foxglove Connection (stereo_vr.js)**
- [X] **3.2.1** Add Foxglove connection function to foxglove_conn.js (reuse pattern from existing code)
- [X] **3.2.2** Add connection status indicator update function
- [X] **3.2.3** Test: Status indicator shows "Connected" when Foxglove bridge is running

**3.3 Topic Subscription (stereo_vr.js)**
- [X] **3.3.1** Subscribe to `/alfie/robot_description` on advertise
- [X] **3.3.2** Subscribe to `/alfie/tf` for transform updates
- [X] **3.3.3** Add CDR binary message decoder (copy from urdf_viewer.js)
- [X] **3.3.4** Test: Console logs show "Subscribed to N topics" and TF messages arriving

**3.4 TF Rate Display (desktop panel)**
- [X] **3.4.1** Add TF rate counter variable and 5-second update interval
- [X] **3.4.2** Update DOM element with Hz rate
- [X] **3.4.3** Test: Panel shows live TF rate (e.g., "50.0 Hz")

**3.5 VR Status Panel (WebXR overlay)**
- [X] **3.5.1** Create head-locked WebGL quad for status text in VR
- [X] **3.5.2** Render Foxglove connection status on quad
- [X] **3.5.3** Render TF rate on quad
- [X] **3.5.4** Test: In VR, status panel visible near bottom of view

**3.6 URDF Text Display (simplified first)**
- [X] **3.6.1** Parse robot_description to extract link names
- [X] **3.6.2** Display link count in desktop panel
- [X] **3.6.3** Display link count in VR panel
- [X] **3.6.4** Test: Shows "12 links" (or actual count) when URDF received

**3.7 URDF 3D Visualization (desktop only first)**
- [X] **3.7.1** Add Three.js canvas to desktop panel
- [X] **3.7.2** Initialize URDFViewer class in stereo_vr.js
- [X] **3.7.3** Test: 3D robot model appears in desktop panel

**3.8 URDF in VR**
- [ ] **3.8.1** Add WebGL robot rendering to VR scene (position in world space)
- [ ] **3.8.2** Apply TF transforms to update robot pose
- [ ] **3.8.3** Test: 3D robot model visible in VR, moves with TF updates

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
