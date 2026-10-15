// Controller Tracking Module
// Handles VR controller WebSocket connection and input processing
//
// Client -> server protocol (JSON text frames):
//   Tracking update (every 3rd XR frame):
//     { timestamp, seq, leftController?, rightController?, headset? }
//     A hand/headset key is OMITTED when it is not tracked this frame; the server
//     treats a key that is absent for ~150ms as lost and releases that arm/head.
//   { type: 'release_all', reason, timestamp }          - session end / page hidden
//   { type: 'release_hand', hand: 'left'|'right', reason, timestamp } - input source removed

import {
    CONFIG,
    connectionState,
    leftController,
    rightController,
    headsetPose,
    passthroughMode,
    centerViewState,
    resetController,
    resetHeadsetPose,
} from './state.js';
import { vrLog } from './ui_status.js';

// Per-frame tracking validity (only tracked data is ever sent)
const tracked = {
    left: false,
    right: false,
    headset: false,
};

// Monotonic sequence number for tracking messages
let messageSeq = 0;

// Reconnect state
let reconnectDelayMs = CONFIG.CONTROLLER_WS_BACKOFF_INITIAL_MS;
let reconnectTimer = null;

// ========================================
// Quaternion Math
// ========================================

const RAD_TO_DEG = 180 / Math.PI;

// Quaternion to Euler angles conversion (in degrees)
// WebXR is Y-up, -Z forward. Decomposes as intrinsic Y-X-Z:
// yaw about Y, then pitch about X, then roll about Z (R = Ry * Rx * Rz).
// Returns { x: pitch, y: yaw, z: roll }. Yaw covers the full +/-180 range;
// pitch is limited to +/-90 (gimbal lock at straight up/down).
export function quaternionToEuler(q) {
    const { x, y, z, w } = q;

    // Rotation matrix elements needed for the YXZ decomposition
    const m13 = 2 * (x * z + w * y);
    const m23 = 2 * (y * z - w * x);
    const m33 = 1 - 2 * (x * x + y * y);
    const m21 = 2 * (x * y + w * z);
    const m22 = 1 - 2 * (x * x + z * z);
    const m31 = 2 * (x * z - w * y);
    const m11 = 1 - 2 * (y * y + z * z);

    const pitch = Math.asin(Math.max(-1, Math.min(1, -m23)));
    let yaw, roll;
    if (Math.abs(m23) < 0.9999999) {
        yaw = Math.atan2(m13, m33);
        roll = Math.atan2(m21, m22);
    } else {
        // Gimbal lock: looking straight up/down, fold roll into yaw
        yaw = Math.atan2(-m31, m11);
        roll = 0;
    }

    return {
        x: pitch * RAD_TO_DEG,
        y: yaw * RAD_TO_DEG,
        z: roll * RAD_TO_DEG
    };
}

// ========================================
// Controller WebSocket Functions
// ========================================

function isSocketOpen() {
    const ws = connectionState.controllerWS;
    return !!ws && ws.readyState === WebSocket.OPEN;
}

function scheduleReconnect() {
    if (reconnectTimer) return;
    const delay = reconnectDelayMs;
    reconnectDelayMs = Math.min(reconnectDelayMs * 2, CONFIG.CONTROLLER_WS_BACKOFF_MAX_MS);
    console.log(`Controller WebSocket reconnecting in ${delay}ms`);
    reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        initControllerWebSocket();
    }, delay);
}

export function initControllerWebSocket() {
    const wsUrl = `wss://${window.location.hostname}:${CONFIG.CONTROLLER_WS_PORT}`;
    console.log(`Connecting to controller WebSocket: ${wsUrl}`);

    let ws;
    try {
        ws = new WebSocket(wsUrl);
    } catch (error) {
        console.error('Failed to create controller WebSocket:', error);
        scheduleReconnect();
        return;
    }
    connectionState.controllerWS = ws;

    ws.onopen = () => {
        console.log('Controller WebSocket connected');
        reconnectDelayMs = CONFIG.CONTROLLER_WS_BACKOFF_INITIAL_MS;
        vrLog('Controller WS: Connected');
    };

    ws.onerror = (error) => {
        console.error('Controller WebSocket error:', error);
        vrLog('Controller WS: Error');
    };

    ws.onclose = (event) => {
        console.log(`Controller WebSocket closed. Code: ${event.code}, Reason: ${event.reason}`);
        if (connectionState.controllerWS === ws) {
            connectionState.controllerWS = null;
        }
        vrLog('Controller WS: Disconnected');
        scheduleReconnect();  // Retry forever with backoff
    };

    ws.onmessage = (event) => {
        // Handle commands from the server
        try {
            const data = JSON.parse(event.data);
            if (data.command === 'passthrough_mode') {
                passthroughMode.active = data.active;
                console.log(`Passthrough mode: ${data.active ? 'ON' : 'OFF'}`);
                vrLog(`Passthrough: ${data.active ? 'ON' : 'OFF'}`);
            } else {
                console.log('Controller WS message:', event.data);
            }
        } catch (e) {
            console.log('Controller WS message:', event.data);
        }
    };
}

function serializeController(controller) {
    // Deep copies so later state mutation can't affect a queued message
    return {
        hand: controller.hand,
        position: { ...controller.position },
        rotation: controller.rotation ? { ...controller.rotation } : null,
        quaternion: controller.quaternion ? { ...controller.quaternion } : null,
        gripActive: controller.gripActive,
        trigger: controller.trigger,
        thumbstick: { ...controller.thumbstick },
        buttons: { ...controller.buttons }
    };
}

export function sendControllerData() {
    if (!isSocketOpen()) return;
    const ws = connectionState.controllerWS;

    // Drop rather than queue: a backlog of stale commands is worse than a gap
    if (ws.bufferedAmount > CONFIG.CONTROLLER_WS_MAX_BUFFERED) return;

    const hasValidLeft = tracked.left && leftController.position !== null;
    const hasValidRight = tracked.right && rightController.position !== null;
    // Suppress headset data when passthrough mode is active
    const hasValidHeadset = tracked.headset && !passthroughMode.active && headsetPose.position !== null;

    if (!hasValidLeft && !hasValidRight && !hasValidHeadset) return;

    const data = {
        timestamp: Date.now(),
        seq: ++messageSeq,
    };
    // Untracked keys are omitted entirely (never null / never stale)
    if (hasValidLeft) data.leftController = serializeController(leftController);
    if (hasValidRight) data.rightController = serializeController(rightController);
    if (hasValidHeadset) {
        data.headset = {
            position: { ...headsetPose.position },
            rotation: headsetPose.rotation ? { ...headsetPose.rotation } : null,
            quaternion: headsetPose.quaternion ? { ...headsetPose.quaternion } : null
        };
    }
    ws.send(JSON.stringify(data));
}

// Best-effort safety messages: sent regardless of bufferedAmount
function sendSafetyMessage(message) {
    if (!isSocketOpen()) return false;
    try {
        connectionState.controllerWS.send(JSON.stringify(message));
        return true;
    } catch (e) {
        console.warn('Failed to send safety message:', e);
        return false;
    }
}

export function sendReleaseAll(reason) {
    if (sendSafetyMessage({ type: 'release_all', reason: reason, timestamp: Date.now() })) {
        console.log(`Sent release_all (${reason})`);
    }
}

export function sendReleaseHand(hand, reason) {
    if (sendSafetyMessage({ type: 'release_hand', hand: hand, reason: reason, timestamp: Date.now() })) {
        console.log(`Sent release_hand ${hand} (${reason})`);
    }
}

// Clear all tracking state (session end / page hidden)
export function resetTracking() {
    tracked.left = false;
    tracked.right = false;
    tracked.headset = false;
    resetController(leftController);
    resetController(rightController);
    resetHeadsetPose();
    centerViewState.lastButtonState = false;
}

// ========================================
// XR Input Processing
// ========================================

function controllerForHand(hand) {
    return hand === 'left' ? leftController : (hand === 'right' ? rightController : null);
}

function hasOtherGamepadSource(session, hand, removedSources) {
    for (const source of session.inputSources || []) {
        if (removedSources.includes(source)) continue;
        if (source.handedness === hand && source.gamepad && source.targetRayMode === 'tracked-pointer') {
            return true;
        }
    }
    return false;
}

// XRSession 'inputsourceschange': release a hand whose controller disappeared
export function handleInputSourcesChange(event) {
    const session = event.session;
    const removed = Array.from(event.removed || []);
    for (const source of removed) {
        const hand = source.handedness;
        const controller = controllerForHand(hand);
        if (!controller) continue;
        // Another gamepad source may already have taken over this hand
        if (session && hasOtherGamepadSource(session, hand, removed)) continue;
        tracked[hand] = false;
        resetController(controller);
        if (hand === 'left') centerViewState.lastButtonState = false;
        sendReleaseHand(hand, 'input_source_removed');
        vrLog(`Controller lost: ${hand}`);
    }
}

// Process XR input sources to extract controller data
// A hand only counts as tracked this frame if it has a gamepad AND a grip pose
export function processInputSources(frame, refSpace, session) {
    const seen = { left: false, right: false };
    // Input is not delivered to blurred/hidden sessions (system menu etc.)
    const inputAvailable = session && session.inputSources &&
        (!session.visibilityState || session.visibilityState === 'visible');

    if (inputAvailable) {
        for (const inputSource of session.inputSources) {
            if (inputSource.targetRayMode !== 'tracked-pointer') continue;

            const handedness = inputSource.handedness; // 'left' or 'right'
            const controller = controllerForHand(handedness);
            const gamepad = inputSource.gamepad;
            if (!controller || !gamepad || !inputSource.gripSpace) continue;

            // Get grip pose (hand position/rotation)
            const gripPose = frame.getPose(inputSource.gripSpace, refSpace);
            if (!gripPose) continue;

            const pos = gripPose.transform.position;
            const ori = gripPose.transform.orientation;
            controller.position = { x: pos.x, y: pos.y, z: pos.z };
            controller.quaternion = { x: ori.x, y: ori.y, z: ori.z, w: ori.w };
            controller.rotation = quaternionToEuler(controller.quaternion);

            // Trigger (index 0)
            controller.trigger = gamepad.buttons[0] ? (gamepad.buttons[0].value || 0) : 0;

            // Squeeze/Grip (index 1)
            const isGripPressed = gamepad.buttons[1]?.pressed || false;
            controller.gripActive = isGripPressed;

            // Thumbstick axes (axes 2, 3)
            controller.thumbstick = {
                x: gamepad.axes[2] || 0,
                y: gamepad.axes[3] || 0
            };

            // Buttons
            if (handedness === 'left') {
                const thumbstickPressed = !!gamepad.buttons[3]?.pressed;
                controller.buttons = {
                    squeeze: isGripPressed,
                    thumbstick: thumbstickPressed,
                    x: !!gamepad.buttons[4]?.pressed,
                    y: !!gamepad.buttons[5]?.pressed,
                    menu: !!gamepad.buttons[6]?.pressed
                };

                // Toggle center view on left thumbstick button press (edge detection)
                if (thumbstickPressed && !centerViewState.lastButtonState) {
                    centerViewState.visible = !centerViewState.visible;
                    vrLog(`Center view: ${centerViewState.visible ? 'ON' : 'OFF'}`);
                }
                centerViewState.lastButtonState = thumbstickPressed;
            } else {
                controller.buttons = {
                    squeeze: isGripPressed,
                    thumbstick: !!gamepad.buttons[3]?.pressed,
                    a: !!gamepad.buttons[4]?.pressed,
                    b: !!gamepad.buttons[5]?.pressed,
                    menu: !!gamepad.buttons[6]?.pressed
                };
            }

            seen[handedness] = true;
        }
    }

    // Hands not seen this frame are cleared so their keys are omitted from the next send
    for (const hand of ['left', 'right']) {
        if (!seen[hand] && tracked[hand]) {
            resetController(controllerForHand(hand));
            if (hand === 'left') centerViewState.lastButtonState = false;
        }
        tracked[hand] = seen[hand];
    }
}

// Process headset pose (null / emulatedPosition = tracking lost -> omitted)
export function processHeadsetPose(pose) {
    if (!pose || !pose.transform || pose.emulatedPosition) {
        tracked.headset = false;
        resetHeadsetPose();
        return;
    }

    const pos = pose.transform.position;
    const ori = pose.transform.orientation;

    headsetPose.position = { x: pos.x, y: pos.y, z: pos.z };
    headsetPose.quaternion = { x: ori.x, y: ori.y, z: ori.z, w: ori.w };
    headsetPose.rotation = quaternionToEuler(headsetPose.quaternion);
    tracked.headset = true;
}
