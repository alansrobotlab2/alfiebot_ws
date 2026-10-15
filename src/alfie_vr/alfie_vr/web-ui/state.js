// Shared state module for VR application
// All modules import state from here to avoid circular dependencies

// ========================================
// Configuration Constants
// ========================================

export const CONFIG = {
    CONTROLLER_WS_PORT: 8442,
    CONTROLLER_WS_BACKOFF_INITIAL_MS: 1000,  // First reconnect delay
    CONTROLLER_WS_BACKOFF_MAX_MS: 5000,      // Reconnect delay cap (retries forever)
    CONTROLLER_WS_MAX_BUFFERED: 16384,       // Drop a send if this many bytes are already queued
};

// ========================================
// 3D View Configuration (Immersive Mode)
// ========================================

export const immersive3DConfig = {
    verticalOffset: -0.17,     // Meters - vertical position (positive = up)
    ipdOffset: -0.018,         // Meters - IPD convergence adjustment
    screenDistance: 0.6,       // Meters - distance to virtual screen
    screenScale: 0.5,          // Multiplier for screen size
    cornerRadius: 0.05,        // Normalized 0-0.5 - corner radius for rounded corners

    // Camera resolution configuration
    wideSourceWidth: 800,      // Wide camera source resolution width
    wideSourceHeight: 600,     // Wide camera source resolution height
    wideOutputWidth: 320,      // Wide camera output resolution width
    wideOutputHeight: 240,     // Wide camera output resolution height

    centerCropWidth: 320,      // Center crop width from source
    centerCropHeight: 200,     // Center crop height from source
    centerOutputWidth: 320,    // Center camera output resolution width
    centerOutputHeight: 240,   // Center camera output resolution height
};

// Legacy alias for backward compatibility with overlay panels
export const stereoSettings = immersive3DConfig;

// ========================================
// Connection State
// ========================================

export const connectionState = {
    controllerWS: null,
};

// ========================================
// Controller Tracking State
// ========================================

export const leftController = {
    hand: 'left',
    position: null,
    rotation: null,
    quaternion: null,
    gripActive: false,
    trigger: 0,
    thumbstick: { x: 0, y: 0 },
    buttons: { squeeze: false, thumbstick: false, x: false, y: false, menu: false }
};

export const rightController = {
    hand: 'right',
    position: null,
    rotation: null,
    quaternion: null,
    gripActive: false,
    trigger: 0,
    thumbstick: { x: 0, y: 0 },
    buttons: { squeeze: false, thumbstick: false, a: false, b: false, menu: false }
};

export const headsetPose = {
    position: null,
    rotation: null,
    quaternion: null
};

// ========================================
// Battery State (headset battery, shown in VR stream panel)
// ========================================

export const batteryState = {
    level: null,
    charging: false,
};

// ========================================
// Passthrough Mode State
// ========================================

export const passthroughMode = {
    active: false,  // When true: 0% opacity, hide panels/video, don't send headset data
};

// ========================================
// Center View Toggle State
// ========================================

export const centerViewState = {
    visible: true,  // Defaults to on, toggle with left joystick button
    lastButtonState: false,  // Track button state for edge detection
};

// ========================================
// Pre-allocated Buffers (for performance)
// ========================================

export const preAllocatedBuffers = {
    viewMatrix: new Float32Array(16),
};

// ========================================
// Helper Functions for State Updates
// ========================================

// Reset controller state (hand lost / released) so nothing stale is ever sent
export function resetController(controller) {
    controller.position = null;
    controller.rotation = null;
    controller.quaternion = null;
    controller.gripActive = false;
    controller.trigger = 0;
    controller.thumbstick = { x: 0, y: 0 };
    for (const key of Object.keys(controller.buttons)) {
        controller.buttons[key] = false;
    }
}

// Reset headset pose
export function resetHeadsetPose() {
    headsetPose.position = null;
    headsetPose.rotation = null;
    headsetPose.quaternion = null;
}
