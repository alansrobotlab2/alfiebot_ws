// Controller Tracking Module
// Handles VR controller WebSocket connection and input processing

import {
    CONFIG,
    stereoSettings,
    connectionState,
    xrState,
    leftController,
    rightController,
    headsetPose,
    gripState,
    invalidateSettingsHash,
    passthroughMode,
} from './state.js';

// Forward declaration for vrLog (will be set by main module)
let vrLogFn = (msg) => console.log('[VR]', msg);

export function setVrLogFunction(fn) {
    vrLogFn = fn;
}

// ========================================
// Quaternion Math
// ========================================

// Quaternion to Euler angles conversion (in degrees)
export function quaternionToEuler(q) {
    // Roll (x-axis rotation)
    const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    const sinp = 2 * (q.w * q.y - q.z * q.x);
    let pitch;
    if (Math.abs(sinp) >= 1) {
        pitch = Math.sign(sinp) * Math.PI / 2; // Use 90 degrees if out of range
    } else {
        pitch = Math.asin(sinp);
    }
    
    // Yaw (z-axis rotation)
    const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);
    
    return {
        x: roll * 180 / Math.PI,
        y: pitch * 180 / Math.PI,
        z: yaw * 180 / Math.PI
    };
}

// ========================================
// Controller WebSocket Functions
// ========================================

export function initControllerWebSocket() {
    const wsUrl = `wss://${window.location.hostname}:${CONFIG.CONTROLLER_WS_PORT}`;
    console.log(`Connecting to controller WebSocket: ${wsUrl}`);
    
    try {
        connectionState.controllerWS = new WebSocket(wsUrl);
        
        connectionState.controllerWS.onopen = () => {
            console.log('Controller WebSocket connected');
            connectionState.controllerWSRetryCount = 0;
            vrLogFn('Controller WS: Connected');
        };
        
        connectionState.controllerWS.onerror = (error) => {
            console.error('Controller WebSocket error:', error);
            vrLogFn('Controller WS: Error');
        };
        
        connectionState.controllerWS.onclose = (event) => {
            console.log(`Controller WebSocket closed. Code: ${event.code}, Reason: ${event.reason}`);
            connectionState.controllerWS = null;
            vrLogFn('Controller WS: Disconnected');
            
            // Retry connection
            if (connectionState.controllerWSRetryCount < CONFIG.CONTROLLER_WS_MAX_RETRIES) {
                connectionState.controllerWSRetryCount++;
                setTimeout(initControllerWebSocket, 3000);
            }
        };
        
        connectionState.controllerWS.onmessage = (event) => {
            // Handle commands from the server
            try {
                const data = JSON.parse(event.data);
                if (data.command === 'passthrough_mode') {
                    passthroughMode.active = data.active;
                    console.log(`Passthrough mode: ${data.active ? 'ON' : 'OFF'}`);
                    vrLogFn(`Passthrough: ${data.active ? 'ON' : 'OFF'}`);
                } else {
                    console.log('Controller WS message:', event.data);
                }
            } catch (e) {
                console.log('Controller WS message:', event.data);
            }
        };
    } catch (error) {
        console.error('Failed to create controller WebSocket:', error);
        if (connectionState.controllerWSRetryCount < CONFIG.CONTROLLER_WS_MAX_RETRIES) {
            connectionState.controllerWSRetryCount++;
            setTimeout(initControllerWebSocket, 3000);
        }
    }
}

export function sendControllerData() {
    if (!connectionState.controllerWS || connectionState.controllerWS.readyState !== WebSocket.OPEN) return;
    
    const hasValidLeft = leftController.position !== null;
    const hasValidRight = rightController.position !== null;
    // Suppress headset data when passthrough mode is active
    const hasValidHeadset = !passthroughMode.active && headsetPose.position !== null;
    
    if (hasValidLeft || hasValidRight || hasValidHeadset) {
        // Create deep copies to avoid race conditions with concurrent state access
        const data = {
            timestamp: Date.now(),
            leftController: hasValidLeft ? {
                hand: leftController.hand,
                position: { ...leftController.position },
                rotation: leftController.rotation ? { ...leftController.rotation } : null,
                quaternion: leftController.quaternion ? { ...leftController.quaternion } : null,
                gripActive: leftController.gripActive,
                trigger: leftController.trigger,
                thumbstick: { ...leftController.thumbstick },
                buttons: { ...leftController.buttons }
            } : null,
            rightController: hasValidRight ? {
                hand: rightController.hand,
                position: { ...rightController.position },
                rotation: rightController.rotation ? { ...rightController.rotation } : null,
                quaternion: rightController.quaternion ? { ...rightController.quaternion } : null,
                gripActive: rightController.gripActive,
                trigger: rightController.trigger,
                thumbstick: { ...rightController.thumbstick },
                buttons: { ...rightController.buttons }
            } : null,
            headset: hasValidHeadset ? {
                position: { ...headsetPose.position },
                rotation: headsetPose.rotation ? { ...headsetPose.rotation } : null,
                quaternion: headsetPose.quaternion ? { ...headsetPose.quaternion } : null
            } : null
        };
        connectionState.controllerWS.send(JSON.stringify(data));
    }
}

// ========================================
// XR Input Processing
// ========================================

// Process XR input sources to extract controller data
export function processInputSources(frame, refSpace, session = null) {
    const xrSession = session || xrState.session;
    if (!xrSession || !xrSession.inputSources) return;
    
    for (const inputSource of xrSession.inputSources) {
        if (inputSource.targetRayMode !== 'tracked-pointer') continue;
        
        const handedness = inputSource.handedness; // 'left' or 'right'
        const controller = handedness === 'left' ? leftController : rightController;
        
        // Get grip pose (hand position/rotation)
        const gripSpace = inputSource.gripSpace;
        if (gripSpace) {
            const gripPose = frame.getPose(gripSpace, refSpace);
            if (gripPose) {
                const pos = gripPose.transform.position;
                const ori = gripPose.transform.orientation;
                
                controller.position = { x: pos.x, y: pos.y, z: pos.z };
                controller.quaternion = { x: ori.x, y: ori.y, z: ori.z, w: ori.w };
                controller.rotation = quaternionToEuler(controller.quaternion);
            }
        }
        
        // Get gamepad data (buttons, triggers, thumbsticks)
        const gamepad = inputSource.gamepad;
        if (gamepad) {
            // Trigger (index 0)
            if (gamepad.buttons[0]) {
                controller.trigger = gamepad.buttons[0].value || 0;
            }
            
            // Squeeze/Grip (index 1)
            const isGripPressed = gamepad.buttons[1]?.pressed || false;
            
            // Track grip state changes
            if (handedness === 'left') {
                if (isGripPressed && !gripState.leftGripDown) {
                    gripState.leftGripDown = true;
                    gripState.leftInitialQuaternion = controller.quaternion ? { ...controller.quaternion } : null;
                } else if (!isGripPressed && gripState.leftGripDown) {
                    gripState.leftGripDown = false;
                    gripState.leftInitialQuaternion = null;
                    // Send grip release message
                    if (connectionState.controllerWS && connectionState.controllerWS.readyState === WebSocket.OPEN) {
                        connectionState.controllerWS.send(JSON.stringify({ hand: 'left', gripReleased: true }));
                    }
                }
                controller.gripActive = gripState.leftGripDown;
            } else {
                if (isGripPressed && !gripState.rightGripDown) {
                    gripState.rightGripDown = true;
                    gripState.rightInitialQuaternion = controller.quaternion ? { ...controller.quaternion } : null;
                } else if (!isGripPressed && gripState.rightGripDown) {
                    gripState.rightGripDown = false;
                    gripState.rightInitialQuaternion = null;
                    // Send grip release message
                    if (connectionState.controllerWS && connectionState.controllerWS.readyState === WebSocket.OPEN) {
                        connectionState.controllerWS.send(JSON.stringify({ hand: 'right', gripReleased: true }));
                    }
                }
                controller.gripActive = gripState.rightGripDown;
            }
            
            // Thumbstick axes (axes 2, 3)
            controller.thumbstick = {
                x: gamepad.axes[2] || 0,
                y: gamepad.axes[3] || 0
            };
            
            // Buttons
            if (handedness === 'left') {
                controller.buttons = {
                    squeeze: isGripPressed,
                    thumbstick: !!gamepad.buttons[3]?.pressed,
                    x: !!gamepad.buttons[4]?.pressed,
                    y: !!gamepad.buttons[5]?.pressed,
                    menu: !!gamepad.buttons[6]?.pressed
                };
            } else {
                controller.buttons = {
                    squeeze: isGripPressed,
                    thumbstick: !!gamepad.buttons[3]?.pressed,
                    a: !!gamepad.buttons[4]?.pressed,
                    b: !!gamepad.buttons[5]?.pressed,
                    menu: !!gamepad.buttons[6]?.pressed
                };
            }
        }
    }
}

// Process headset pose
export function processHeadsetPose(pose) {
    if (!pose || !pose.transform) return;
    
    const pos = pose.transform.position;
    const ori = pose.transform.orientation;
    
    headsetPose.position = { x: pos.x, y: pos.y, z: pos.z };
    headsetPose.quaternion = { x: ori.x, y: ori.y, z: ori.z, w: ori.w };
    headsetPose.rotation = quaternionToEuler(headsetPose.quaternion);
}

// Use thumbsticks to adjust stereo settings in VR
export function processThumbstickAdjustments() {
    // Left thumbstick Y: vertical offset - DISABLED to prevent conflict with robot control
    /*
    if (Math.abs(leftController.thumbstick?.y || 0) > 0.1) {
        stereoSettings.verticalOffset += leftController.thumbstick.y * 0.005;
        stereoSettings.verticalOffset = Math.max(-1, Math.min(1, stereoSettings.verticalOffset));
        invalidateSettingsHash();
    }
    */
    
    // Right thumbstick X: IPD offset - DISABLED
    /*
    if (Math.abs(rightController.thumbstick?.x || 0) > 0.1) {
        stereoSettings.ipdOffset += rightController.thumbstick.x * 0.0005;
        stereoSettings.ipdOffset = Math.max(-0.05, Math.min(0.05, stereoSettings.ipdOffset));
    }
    */
    
    // Triggers: screen distance - DISABLED
    /*
    if ((leftController.trigger || 0) > 0.5) {
        stereoSettings.screenDistance -= 0.02;
        stereoSettings.screenDistance = Math.max(0.5, stereoSettings.screenDistance);
    }
    if ((rightController.trigger || 0) > 0.5) {
        stereoSettings.screenDistance += 0.02;
        stereoSettings.screenDistance = Math.max(0.5, Math.min(5, stereoSettings.screenDistance));
    }
    */
}
