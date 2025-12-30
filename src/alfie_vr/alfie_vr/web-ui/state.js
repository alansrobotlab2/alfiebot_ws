// Shared state module for VR application
// All modules import state from here to avoid circular dependencies

// ========================================
// Configuration Constants
// ========================================

export const CONFIG = {
    CONTROLLER_WS_PORT: 8442,
    CONTROLLER_WS_MAX_RETRIES: 10,
    MAX_RETRIES: 3,
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
    foxgloveConn: null,
    controllerWS: null,
    controllerWSRetryCount: 0,
    retryCount: 0,
};

// ========================================
// Subscription IDs
// ========================================

export const subscriptions = {
    compressedImage: null,
    robotDescription: null,
    tf: null,
    tfStatic: null,
};

// ========================================
// XR Session State
// ========================================

export const xrState = {
    session: null,
    refSpace: null,
    gl: null,
    frameCounter: 0,
};

// ========================================
// Video/Texture State
// ========================================

export const videoState = {
    currentFrameBitmap: null,
    videoElement: null,
    leftTexture: null,
    rightTexture: null,
    leftCanvas: null,
    rightCanvas: null,
    leftCtx: null,
    rightCtx: null,
    useDirectVideoTexture: false,  // TEMPORARILY DISABLED - use canvas mode for debugging
    videoTexture: null,
    texturesInitialized: false,
    lastVideoWidth: 0,
    lastVideoHeight: 0,
    lastVideoTime: -1,
    videoFrameCallbackId: null,
    newVideoFrameAvailable: false,
};

// ========================================
// Robot State Tracking
// ========================================

export const robotState = {
    urdfString: null,
    linkNames: [],
    tfCount: 0,
    lastTfTime: performance.now(),
    currentTfRate: 0,  // Hz - updated every 5 seconds
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

// Grip state tracking for relative rotation
export const gripState = {
    leftInitialQuaternion: null,
    rightInitialQuaternion: null,
    leftGripDown: false,
    rightGripDown: false,
};

// ========================================
// FPS Overlay State
// ========================================

export const fpsOverlayState = {
    canvas: null,
    ctx: null,
    texture: null,
    shader: null,
    positionBuffer: null,
    texCoordBuffer: null,
    cachedLocations: null,
    currentVrFps: 0,  // This tracks RECEIVED frame rate
};

// ========================================
// Battery State
// ========================================

export const batteryState = {
    level: null,
    charging: false,
};

// ========================================
// State Panel State
// ========================================

export const statePanelState = {
    canvas: null,
    ctx: null,
    texture: null,
    positionBuffer: null,
    texCoordBuffer: null,
    initialized: false,
};

// ========================================
// Stream Panel State
// ========================================

export const streamPanelState = {
    canvas: null,
    ctx: null,
    texture: null,
    positionBuffer: null,
    texCoordBuffer: null,
    initialized: false,
};

// ========================================
// Frame/Latency Tracking
// ========================================

export const frameTracking = {
    receivedFrameCount: 0,
    lastReceivedFpsTime: performance.now(),
    lastFrameReceivedTimestamp: 0,
    lastFrameTime: 0,
    frameCount: 0,
    totalLatency: 0,
};

// ========================================
// Shader State
// ========================================

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
// Recording Toggle State
// ========================================

export const recordingState = {
    isRecording: false,  // Tracks current recording state
    lastButtonState: false,  // Track button state for edge detection
};

export const shaderState = {
    program: null,
    positionBuffer: null,
    texCoordBuffer: null,
    cachedLocations: null,
    lastSettingsHash: '',
};

// ========================================
// Pre-allocated Buffers (for performance)
// ========================================

export const preAllocatedBuffers = {
    viewMatrix: new Float32Array(16),
    rightPanelModelMatrix: new Float32Array(16),
    leftPanelModelMatrix: new Float32Array(16),
};

// ========================================
// Helper Functions for State Updates
// ========================================

// Invalidate settings hash to trigger buffer rebuild
export function invalidateSettingsHash() {
    shaderState.lastSettingsHash = '';
}

// Reset controller state
export function resetController(controller) {
    controller.position = null;
    controller.rotation = null;
    controller.quaternion = null;
    controller.gripActive = false;
    controller.trigger = 0;
    controller.thumbstick = { x: 0, y: 0 };
}

// Reset headset pose
export function resetHeadsetPose() {
    headsetPose.position = null;
    headsetPose.rotation = null;
    headsetPose.quaternion = null;
}
