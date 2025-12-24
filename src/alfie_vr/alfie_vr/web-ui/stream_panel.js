// Stream Panel Module
// Displays stream info (FPS, latency, battery, controller status, headset orientation)
// Positioned along the left edge in immersive VR mode

import {
    stereoSettings,
    connectionState,
    headsetPose,
    preAllocatedBuffers,
} from './state.js';

// ========================================
// Stream Panel Configuration
// ========================================

const STREAM_PANEL_CONFIG = {
    width: 0.15,
    height: 0.12,
    horizontalOffset: 0.15,     // Offset from left edge (0 = flush with left edge)
    verticalOffset: 0.125,       // Centered vertically relative to stereoSettings.verticalOffset
    angle: 10,                 // Yaw rotation in degrees (positive = angled toward viewer)
    backgroundAlpha: 0.5,      // Background transparency (0-1)
    canvasScale: 1280,         // Multiplier to convert panel size to canvas pixels
    titleFontSize: 17,         // Title font size in pixels
    bodyFontSize: 13           // Body text font size in pixels
};

// ========================================
// Module State
// ========================================

let streamPanelCanvas = null;
let streamPanelCtx = null;
let streamPanelTexture = null;
let streamPanelPositionBuffer = null;
let streamPanelTexCoordBuffer = null;
let streamPanelInitialized = false;

// Pre-allocated buffer for model matrix
const streamPanelModelMatrix = preAllocatedBuffers.leftPanelModelMatrix;

// ========================================
// Context and Callback References
// ========================================

let gl = null;
let vrLogFn = (msg) => console.log('[VR]', msg);
let getFrameCounterFn = () => 0;
let getCurrentVrFpsFn = () => 0;
let getCurrentBatteryLevelFn = () => null;
let getCurrentBatteryChargingFn = () => false;
let getLastFrameReceivedTimestampFn = () => 0;
let getShaderProgramFn = () => null;
let getCachedLocationsFn = () => null;
let invertMatrixFn = null;
let viewMatrixBuffer = null;

// Set the GL context and callbacks from main module
export function setStreamPanelContext(context) {
    gl = context.gl;
    vrLogFn = context.vrLog || vrLogFn;
    getFrameCounterFn = context.getFrameCounter || getFrameCounterFn;
    getCurrentVrFpsFn = context.getCurrentVrFps || getCurrentVrFpsFn;
    getCurrentBatteryLevelFn = context.getCurrentBatteryLevel || getCurrentBatteryLevelFn;
    getCurrentBatteryChargingFn = context.getCurrentBatteryCharging || getCurrentBatteryChargingFn;
    getLastFrameReceivedTimestampFn = context.getLastFrameReceivedTimestamp || getLastFrameReceivedTimestampFn;
    getShaderProgramFn = context.getShaderProgram || getShaderProgramFn;
    getCachedLocationsFn = context.getCachedLocations || getCachedLocationsFn;
    invertMatrixFn = context.invertMatrix || invertMatrixFn;
    viewMatrixBuffer = context.viewMatrixBuffer || preAllocatedBuffers.viewMatrix;
}

// Update GL context (called when XR session starts)
export function updateStreamPanelGlContext(newGl) {
    gl = newGl;
    // Reset initialization flag since textures need to be recreated
    streamPanelInitialized = false;
}

// ========================================
// Helper Functions
// ========================================

// Helper function to draw a rounded rectangle
function drawRoundedRect(ctx, x, y, width, height, radius) {
    ctx.beginPath();
    ctx.moveTo(x + radius, y);
    ctx.lineTo(x + width - radius, y);
    ctx.arcTo(x + width, y, x + width, y + radius, radius);
    ctx.lineTo(x + width, y + height - radius);
    ctx.arcTo(x + width, y + height, x + width - radius, y + height, radius);
    ctx.lineTo(x + radius, y + height);
    ctx.arcTo(x, y + height, x, y + height - radius, radius);
    ctx.lineTo(x, y + radius);
    ctx.arcTo(x, y, x + radius, y, radius);
    ctx.closePath();
    ctx.fill();
}

// Helper function to create a rotated model matrix (yaw rotation around Y axis)
function createRotatedModelMatrix(baseMatrix, yawDegrees, outMatrix) {
    const yawRad = yawDegrees * Math.PI / 180;
    const cosY = Math.cos(yawRad);
    const sinY = Math.sin(yawRad);
    
    const m = baseMatrix;
    
    // Column 0: m * [cosY, 0, -sinY, 0]
    outMatrix[0] = m[0] * cosY + m[8] * (-sinY);
    outMatrix[1] = m[1] * cosY + m[9] * (-sinY);
    outMatrix[2] = m[2] * cosY + m[10] * (-sinY);
    outMatrix[3] = m[3] * cosY + m[11] * (-sinY);
    
    // Column 1: m * [0, 1, 0, 0] = column 1 of m
    outMatrix[4] = m[4];
    outMatrix[5] = m[5];
    outMatrix[6] = m[6];
    outMatrix[7] = m[7];
    
    // Column 2: m * [sinY, 0, cosY, 0]
    outMatrix[8] = m[0] * sinY + m[8] * cosY;
    outMatrix[9] = m[1] * sinY + m[9] * cosY;
    outMatrix[10] = m[2] * sinY + m[10] * cosY;
    outMatrix[11] = m[3] * sinY + m[11] * cosY;
    
    // Column 3: m * [0, 0, 0, 1] = column 3 of m (translation)
    outMatrix[12] = m[12];
    outMatrix[13] = m[13];
    outMatrix[14] = m[14];
    outMatrix[15] = m[15];
}

// ========================================
// Stream Panel Functions
// ========================================

function initStreamPanel() {
    if (!gl) {
        vrLogFn('Stream panel: no GL context');
        return;
    }
    
    vrLogFn('Init stream panel...');
    
    // Create canvas for status text (dimensions derived from panel size)
    streamPanelCanvas = document.createElement('canvas');
    streamPanelCanvas.width = Math.round(STREAM_PANEL_CONFIG.width * STREAM_PANEL_CONFIG.canvasScale);
    streamPanelCanvas.height = Math.round(STREAM_PANEL_CONFIG.height * STREAM_PANEL_CONFIG.canvasScale);
    streamPanelCtx = streamPanelCanvas.getContext('2d');
    
    // Create texture
    streamPanelTexture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, streamPanelTexture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    
    // Create position buffer for 3D quad
    // Position along the left edge of the viewport
    const mainHeight = stereoSettings.screenScale;
    const mainWidth = mainHeight * (16/9);
    const mainLeftEdge = -mainWidth / 2;
    
    // Stream panel size and position
    const panelHeight = STREAM_PANEL_CONFIG.height;
    const panelWidth = STREAM_PANEL_CONFIG.width;
    
    // Position flush with left edge
    const panelLeft = mainLeftEdge + STREAM_PANEL_CONFIG.horizontalOffset;
    const panelCenterY = stereoSettings.verticalOffset + STREAM_PANEL_CONFIG.verticalOffset;
    
    streamPanelPositionBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, streamPanelPositionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
        panelLeft, panelCenterY - panelHeight/2,
        panelLeft + panelWidth, panelCenterY - panelHeight/2,
        panelLeft, panelCenterY + panelHeight/2,
        panelLeft + panelWidth, panelCenterY + panelHeight/2,
    ]), gl.STATIC_DRAW);
    
    streamPanelTexCoordBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, streamPanelTexCoordBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
        0, 1,
        1, 1,
        0, 0,
        1, 0,
    ]), gl.STATIC_DRAW);
    
    // Initial update
    updateStreamPanelCanvas();
    
    streamPanelInitialized = true;
    vrLogFn('Stream panel: OK');
}

function updateStreamPanelCanvas() {
    if (!streamPanelCtx) return;
    
    const currentVrFps = getCurrentVrFpsFn();
    const currentBatteryLevel = getCurrentBatteryLevelFn();
    const currentBatteryCharging = getCurrentBatteryChargingFn();
    const lastFrameReceivedTimestamp = getLastFrameReceivedTimestampFn();
    
    const canvas = streamPanelCanvas;
    const ctx = streamPanelCtx;
    
    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Semi-transparent dark background with rounded corners
    ctx.fillStyle = `rgba(20, 20, 30, ${STREAM_PANEL_CONFIG.backgroundAlpha})`;
    drawRoundedRect(ctx, 0, 0, canvas.width, canvas.height, 12);
    
    // Title
    ctx.fillStyle = '#88ccff';
    ctx.font = `bold ${STREAM_PANEL_CONFIG.titleFontSize}px monospace`;
    ctx.textBaseline = 'top';
    ctx.fillText('Stream Info', 10, 8);
    
    // FPS
    ctx.fillStyle = '#cccccc';
    ctx.font = `${STREAM_PANEL_CONFIG.bodyFontSize}px monospace`;
    ctx.fillText('FPS:', 10, 28);
    const fpsColor = currentVrFps >= 25 ? '#44ff44' : (currentVrFps >= 15 ? '#ffff00' : '#ff4444');
    ctx.fillStyle = fpsColor;
    ctx.fillText(`${currentVrFps.toFixed(1)}`, 80, 28);
    
    // Frame latency
    const frameAge = performance.now() - lastFrameReceivedTimestamp;
    const lagColor = frameAge > 150 ? '#ff4444' : '#44ff44';
    ctx.fillStyle = '#cccccc';
    ctx.fillText('Latency:', 10, 43);
    ctx.fillStyle = lagColor;
    ctx.fillText(`${frameAge.toFixed(0)} ms`, 80, 43);
    
    // Battery status
    ctx.fillStyle = '#cccccc';
    ctx.fillText('Battery:', 10, 58);
    if (currentBatteryLevel !== null) {
        const batteryColor = currentBatteryLevel > 20 ? '#44ff44' : '#ff4444';
        ctx.fillStyle = batteryColor;
        const chargingIcon = currentBatteryCharging ? ' ⚡' : '';
        ctx.fillText(`${currentBatteryLevel}%${chargingIcon}`, 80, 58);
    } else {
        ctx.fillStyle = '#888888';
        ctx.fillText('N/A', 80, 58);
    }
    
    // Controller WS status
    ctx.fillStyle = '#cccccc';
    ctx.fillText('Ctrl WS:', 10, 73);
    const ctrlStatus = connectionState.controllerWS && connectionState.controllerWS.readyState === WebSocket.OPEN ? 'Connected' : 'Disconnected';
    const ctrlColor = connectionState.controllerWS && connectionState.controllerWS.readyState === WebSocket.OPEN ? '#44ff44' : '#ff4444';
    ctx.fillStyle = ctrlColor;
    ctx.fillText(ctrlStatus, 80, 73);
    
    // Headset orientation
    const pitch = headsetPose.rotation ? Math.round(headsetPose.rotation.x) : 0;
    const roll = headsetPose.rotation ? Math.round(headsetPose.rotation.z) : 0;
    const yaw = headsetPose.rotation ? Math.round(headsetPose.rotation.y) : 0;
    
    ctx.fillStyle = '#88ccff';
    ctx.fillText('Pitch:', 10, 88);
    ctx.fillText(`${pitch}°`, 80, 88);
    
    ctx.fillText('Roll:', 10, 103);
    ctx.fillText(`${roll}°`, 80, 103);
    
    ctx.fillText('Yaw:', 10, 118);
    ctx.fillStyle = (Math.abs(yaw) > 10) ? '#ff4444' : '#88ccff';
    ctx.fillText(`${yaw}°`, 80, 118);
    
    // Update texture
    if (gl && streamPanelTexture) {
        gl.bindTexture(gl.TEXTURE_2D, streamPanelTexture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, canvas);
    }
}

export function drawStreamPanel(view, viewport, modelMatrix) {
    const frameCounter = getFrameCounterFn();
    const shaderProgram = getShaderProgramFn();
    const cachedLocations = getCachedLocationsFn();
    
    // Initialize on first call
    if (!streamPanelInitialized) {
        initStreamPanel();
    }
    
    if (!streamPanelInitialized || !shaderProgram || !streamPanelTexture || !cachedLocations) {
        return;
    }
    
    // Update canvas content periodically
    if (frameCounter % 30 === 0) {
        updateStreamPanelCanvas();
    }
    
    // Save current state
    const prevProgram = gl.getParameter(gl.CURRENT_PROGRAM);
    
    // Enable blending for transparency
    gl.enable(gl.BLEND);
    gl.blendFuncSeparate(
        gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA,
        gl.ZERO, gl.ONE
    );
    
    // Disable depth test so panel is always visible on top
    gl.disable(gl.DEPTH_TEST);
    
    gl.useProgram(shaderProgram);
    
    // Set up position buffer
    if (cachedLocations.position !== -1 && cachedLocations.position !== null) {
        gl.bindBuffer(gl.ARRAY_BUFFER, streamPanelPositionBuffer);
        gl.enableVertexAttribArray(cachedLocations.position);
        gl.vertexAttribPointer(cachedLocations.position, 2, gl.FLOAT, false, 0, 0);
    }
    
    // Set up tex coord buffer
    if (cachedLocations.texCoord !== -1 && cachedLocations.texCoord !== null) {
        gl.bindBuffer(gl.ARRAY_BUFFER, streamPanelTexCoordBuffer);
        gl.enableVertexAttribArray(cachedLocations.texCoord);
        gl.vertexAttribPointer(cachedLocations.texCoord, 2, gl.FLOAT, false, 0, 0);
    }
    
    // Set uniforms
    if (cachedLocations.projection !== null) {
        gl.uniformMatrix4fv(cachedLocations.projection, false, view.projectionMatrix);
    }
    
    if (cachedLocations.view !== null && invertMatrixFn) {
        invertMatrixFn(view.transform.matrix, viewMatrixBuffer);
        gl.uniformMatrix4fv(cachedLocations.view, false, viewMatrixBuffer);
    }
    
    if (cachedLocations.model !== null) {
        if (modelMatrix) {
            createRotatedModelMatrix(modelMatrix, STREAM_PANEL_CONFIG.angle, streamPanelModelMatrix);
            gl.uniformMatrix4fv(cachedLocations.model, false, streamPanelModelMatrix);
        } else {
            gl.uniformMatrix4fv(cachedLocations.model, false, cachedLocations.identityMatrix);
        }
    }
    
    if (cachedLocations.distance !== null) {
        gl.uniform1f(cachedLocations.distance, stereoSettings.screenDistance);
    }
    
    if (cachedLocations.ipdOffset !== null) {
        gl.uniform1f(cachedLocations.ipdOffset, stereoSettings.ipdOffset);
    }
    
    if (cachedLocations.isLeftEye !== null) {
        gl.uniform1f(cachedLocations.isLeftEye, view.eye === 'left' ? 1.0 : 0.0);
    }
    
    if (cachedLocations.useDirectVideo !== null) {
        gl.uniform1f(cachedLocations.useDirectVideo, 0.0);
    }
    
    if (cachedLocations.texture !== null) {
        gl.activeTexture(gl.TEXTURE0);
        gl.bindTexture(gl.TEXTURE_2D, streamPanelTexture);
        gl.uniform1i(cachedLocations.texture, 0);
    }
    
    // Draw the panel
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
    
    // Restore state
    gl.disable(gl.BLEND);
    gl.enable(gl.DEPTH_TEST);
    
    if (prevProgram) {
        gl.useProgram(prevProgram);
    }
}
