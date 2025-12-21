// VR Overlays Module
// Handles FPS overlay, status panels (left and right) for immersive VR mode

import {
    stereoSettings,
    connectionState,
    headsetPose,
    preAllocatedBuffers,
} from './state.js';

// ========================================
// Module State
// ========================================

// FPS overlay state
let fpsOverlayCanvas = null;
let fpsOverlayCtx = null;
let fpsOverlayTexture = null;
let fpsOverlayShader = null;
let fpsOverlayPositionBuffer = null;
let fpsOverlayTexCoordBuffer = null;
let fpsOverlayCachedLocations = null;

// Right status panel state
let statusPanelCanvas = null;
let statusPanelCtx = null;
let statusPanelTexture = null;
let statusPanelPositionBuffer = null;
let statusPanelTexCoordBuffer = null;
let statusPanelInitialized = false;

// Left status panel state
let leftStatusPanelCanvas = null;
let leftStatusPanelCtx = null;
let leftStatusPanelTexture = null;
let leftStatusPanelPositionBuffer = null;
let leftStatusPanelTexCoordBuffer = null;
let leftStatusPanelInitialized = false;

// Pre-allocated buffers
const rightPanelModelMatrix = preAllocatedBuffers.rightPanelModelMatrix;
const leftPanelModelMatrix = preAllocatedBuffers.leftPanelModelMatrix;

// ========================================
// Context and Callback References
// ========================================

// These are set by the main module via setOverlayContext
let gl = null;
let vrLogFn = (msg) => console.log('[VR]', msg);
let getFrameCounterFn = () => 0;
let getCurrentVrFpsFn = () => 0;
let getCurrentBatteryLevelFn = () => null;
let getCurrentBatteryChargingFn = () => false;
let getLastFrameReceivedTimestampFn = () => 0;
let getFoxgloveConnFn = () => null;
let getTfRateFn = () => 0;
let getLinkNamesLengthFn = () => 0;
let getUrdfStringFn = () => null;
let getShaderProgramFn = () => null;
let getCachedLocationsFn = () => null;
let invertMatrixFn = null;
let viewMatrixBuffer = null;

// Set the GL context and callbacks from main module
export function setOverlayContext(context) {
    gl = context.gl;
    vrLogFn = context.vrLog || vrLogFn;
    getFrameCounterFn = context.getFrameCounter || getFrameCounterFn;
    getCurrentVrFpsFn = context.getCurrentVrFps || getCurrentVrFpsFn;
    getCurrentBatteryLevelFn = context.getCurrentBatteryLevel || getCurrentBatteryLevelFn;
    getCurrentBatteryChargingFn = context.getCurrentBatteryCharging || getCurrentBatteryChargingFn;
    getLastFrameReceivedTimestampFn = context.getLastFrameReceivedTimestamp || getLastFrameReceivedTimestampFn;
    getFoxgloveConnFn = context.getFoxgloveConn || getFoxgloveConnFn;
    getTfRateFn = context.getTfRate || getTfRateFn;
    getLinkNamesLengthFn = context.getLinkNamesLength || getLinkNamesLengthFn;
    getUrdfStringFn = context.getUrdfString || getUrdfStringFn;
    getShaderProgramFn = context.getShaderProgram || getShaderProgramFn;
    getCachedLocationsFn = context.getCachedLocations || getCachedLocationsFn;
    invertMatrixFn = context.invertMatrix || invertMatrixFn;
    viewMatrixBuffer = context.viewMatrixBuffer || preAllocatedBuffers.viewMatrix;
}

// Update GL context (called when XR session starts)
export function updateGlContext(newGl) {
    gl = newGl;
    // Reset initialization flags since textures need to be recreated
    statusPanelInitialized = false;
    leftStatusPanelInitialized = false;
    fpsOverlayCanvas = null;
    fpsOverlayTexture = null;
    fpsOverlayShader = null;
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
    
    // Rotation matrix around Y axis
    // [cosY  0  sinY  0]
    // [0     1  0     0]
    // [-sinY 0  cosY  0]
    // [0     0  0     1]
    
    // Multiply baseMatrix * rotationMatrix
    const m = baseMatrix;
    
    outMatrix[0] = m[0] * cosY + m[2] * (-sinY);
    outMatrix[1] = m[1] * cosY + m[3] * (-sinY);
    outMatrix[2] = m[0] * sinY + m[2] * cosY;
    outMatrix[3] = m[3];
    
    outMatrix[4] = m[4] * cosY + m[6] * (-sinY);
    outMatrix[5] = m[5] * cosY + m[7] * (-sinY);
    outMatrix[6] = m[4] * sinY + m[6] * cosY;
    outMatrix[7] = m[7];
    
    outMatrix[8] = m[8] * cosY + m[10] * (-sinY);
    outMatrix[9] = m[9] * cosY + m[11] * (-sinY);
    outMatrix[10] = m[8] * sinY + m[10] * cosY;
    outMatrix[11] = m[11];
    
    outMatrix[12] = m[12] * cosY + m[14] * (-sinY);
    outMatrix[13] = m[13] * cosY + m[15] * (-sinY);
    outMatrix[14] = m[12] * sinY + m[14] * cosY;
    outMatrix[15] = m[15];
}

// ========================================
// FPS Overlay Functions
// ========================================

function initFpsOverlay() {
    if (!gl) {
        vrLogFn('FPS overlay: no GL context');
        return;
    }
    
    vrLogFn('Init FPS overlay...');
    
    // Create canvas for FPS text (larger for better visibility)
    fpsOverlayCanvas = document.createElement('canvas');
    fpsOverlayCanvas.width = 256;
    fpsOverlayCanvas.height = 64;
    fpsOverlayCtx = fpsOverlayCanvas.getContext('2d');
    
    // Create texture
    fpsOverlayTexture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, fpsOverlayTexture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    
    // Initial update
    updateFpsOverlayCanvas();
    
    // Initialize overlay shader
    initFpsOverlayShader();
    
    vrLogFn('FPS overlay: ' + (fpsOverlayShader ? 'OK' : 'FAIL'));
}

function updateFpsOverlayCanvas() {
    if (!fpsOverlayCtx) return;
    
    const currentVrFps = getCurrentVrFpsFn();
    const currentBatteryLevel = getCurrentBatteryLevelFn();
    const currentBatteryCharging = getCurrentBatteryChargingFn();
    
    // Clear canvas
    fpsOverlayCtx.clearRect(0, 0, fpsOverlayCanvas.width, fpsOverlayCanvas.height);
    
    // Bright red background for high visibility (testing)
    fpsOverlayCtx.fillStyle = 'rgba(200, 0, 0, 0.8)';
    fpsOverlayCtx.fillRect(0, 0, fpsOverlayCanvas.width, fpsOverlayCanvas.height);
    
    // White border
    fpsOverlayCtx.strokeStyle = '#ffffff';
    fpsOverlayCtx.lineWidth = 3;
    fpsOverlayCtx.strokeRect(2, 2, fpsOverlayCanvas.width - 4, fpsOverlayCanvas.height - 4);
    
    // Draw FPS text in white
    fpsOverlayCtx.fillStyle = '#ffffff';
    fpsOverlayCtx.font = 'bold 24px monospace';
    fpsOverlayCtx.textBaseline = 'middle';
    fpsOverlayCtx.textAlign = 'center';
    
    // Build display text with FPS and battery
    let displayText = `FPS: ${currentVrFps.toFixed(1)}`;
    if (currentBatteryLevel !== null) {
        const batteryIcon = currentBatteryCharging ? 'CHG' : 'BAT';
        displayText += ` | ${batteryIcon}: ${currentBatteryLevel}%`;
    }
    fpsOverlayCtx.fillText(displayText, fpsOverlayCanvas.width / 2, fpsOverlayCanvas.height / 2);
    
    // Update texture if GL context is available
    if (gl && fpsOverlayTexture) {
        gl.bindTexture(gl.TEXTURE_2D, fpsOverlayTexture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, fpsOverlayCanvas);
    }
}

function initFpsOverlayShader() {
    if (!gl) return;
    
    // Simple 2D overlay shader (screen-space, no 3D transforms)
    const vertexShader = gl.createShader(gl.VERTEX_SHADER);
    gl.shaderSource(vertexShader, `
        attribute vec2 a_position;
        attribute vec2 a_texCoord;
        varying vec2 v_texCoord;
        
        void main() {
            gl_Position = vec4(a_position, 0.0, 1.0);
            v_texCoord = a_texCoord;
        }
    `);
    gl.compileShader(vertexShader);
    
    if (!gl.getShaderParameter(vertexShader, gl.COMPILE_STATUS)) {
        console.error('FPS overlay vertex shader error:', gl.getShaderInfoLog(vertexShader));
        return;
    }
    
    const fragmentShader = gl.createShader(gl.FRAGMENT_SHADER);
    gl.shaderSource(fragmentShader, `
        precision mediump float;
        varying vec2 v_texCoord;
        uniform sampler2D u_texture;
        
        void main() {
            gl_FragColor = texture2D(u_texture, v_texCoord);
        }
    `);
    gl.compileShader(fragmentShader);
    
    if (!gl.getShaderParameter(fragmentShader, gl.COMPILE_STATUS)) {
        console.error('FPS overlay fragment shader error:', gl.getShaderInfoLog(fragmentShader));
        return;
    }
    
    fpsOverlayShader = gl.createProgram();
    gl.attachShader(fpsOverlayShader, vertexShader);
    gl.attachShader(fpsOverlayShader, fragmentShader);
    gl.linkProgram(fpsOverlayShader);
    
    if (!gl.getProgramParameter(fpsOverlayShader, gl.LINK_STATUS)) {
        vrLogFn('FPS shader link FAIL');
        fpsOverlayShader = null;
        return;
    }
    
    vrLogFn('FPS shader linked');
    
    // Create buffers for overlay quad (upper left corner, screen space coords)
    const overlayWidth = 0.5;
    const overlayHeight = 0.15;
    const marginX = -0.95;
    const marginY = 0.95;
    
    fpsOverlayPositionBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, fpsOverlayPositionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
        marginX, marginY - overlayHeight,
        marginX + overlayWidth, marginY - overlayHeight,
        marginX, marginY,
        marginX + overlayWidth, marginY,
    ]), gl.STATIC_DRAW);
    
    fpsOverlayTexCoordBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, fpsOverlayTexCoordBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
        0, 1,
        1, 1,
        0, 0,
        1, 0,
    ]), gl.STATIC_DRAW);
    
    // Cache locations
    fpsOverlayCachedLocations = {
        position: gl.getAttribLocation(fpsOverlayShader, 'a_position'),
        texCoord: gl.getAttribLocation(fpsOverlayShader, 'a_texCoord'),
        texture: gl.getUniformLocation(fpsOverlayShader, 'u_texture')
    };
}

export function drawFpsOverlay(view, viewport) {
    const frameCounter = getFrameCounterFn();
    
    // Initialize on first call
    if (!fpsOverlayCanvas) {
        initFpsOverlay();
    }
    
    // Check if initialization succeeded
    if (!fpsOverlayShader || !fpsOverlayTexture || !fpsOverlayCachedLocations) {
        if (frameCounter < 20 && frameCounter % 5 === 0) {
            vrLogFn('FPS ovl not ready');
        }
        return;
    }
    
    if (frameCounter === 15) {
        vrLogFn('Drawing FPS overlay');
    }
    
    // Save current WebGL state
    const prevProgram = gl.getParameter(gl.CURRENT_PROGRAM);
    
    // Enable blending for transparency
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    
    // Disable depth test so overlay is always visible
    gl.disable(gl.DEPTH_TEST);
    
    gl.useProgram(fpsOverlayShader);
    
    // Disable any previously enabled vertex attrib arrays that might interfere
    for (let i = 0; i < 8; i++) {
        gl.disableVertexAttribArray(i);
    }
    
    // Bind position buffer
    gl.bindBuffer(gl.ARRAY_BUFFER, fpsOverlayPositionBuffer);
    gl.enableVertexAttribArray(fpsOverlayCachedLocations.position);
    gl.vertexAttribPointer(fpsOverlayCachedLocations.position, 2, gl.FLOAT, false, 0, 0);
    
    // Bind texcoord buffer
    gl.bindBuffer(gl.ARRAY_BUFFER, fpsOverlayTexCoordBuffer);
    gl.enableVertexAttribArray(fpsOverlayCachedLocations.texCoord);
    gl.vertexAttribPointer(fpsOverlayCachedLocations.texCoord, 2, gl.FLOAT, false, 0, 0);
    
    // Bind texture
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, fpsOverlayTexture);
    gl.uniform1i(fpsOverlayCachedLocations.texture, 0);
    
    // Draw overlay
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
    
    // Restore state
    gl.disable(gl.BLEND);
    gl.enable(gl.DEPTH_TEST);
    
    if (prevProgram) {
        gl.useProgram(prevProgram);
    }
}

// ========================================
// Right Status Panel Functions
// ========================================

function initStatusPanel() {
    if (!gl) {
        vrLogFn('Status panel: no GL context');
        return;
    }
    
    vrLogFn('Init status panel...');
    
    // Create canvas for status text
    statusPanelCanvas = document.createElement('canvas');
    statusPanelCanvas.width = 256;
    statusPanelCanvas.height = 96;
    statusPanelCtx = statusPanelCanvas.getContext('2d');
    
    // Create texture
    statusPanelTexture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, statusPanelTexture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    
    // Create position buffer for 3D quad
    const mainHeight = stereoSettings.screenScale;
    const mainWidth = mainHeight * (16/9);
    const mainRightEdge = mainWidth / 2;
    
    // Status panel size
    const panelHeight = 0.09;
    const panelWidth = 0.20;
    
    const panelLeft = mainRightEdge - panelWidth - 0.175;
    const panelCenterY = stereoSettings.verticalOffset + 0.30;
    
    statusPanelPositionBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, statusPanelPositionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
        panelLeft, panelCenterY - panelHeight/2,
        panelLeft + panelWidth, panelCenterY - panelHeight/2,
        panelLeft, panelCenterY + panelHeight/2,
        panelLeft + panelWidth, panelCenterY + panelHeight/2,
    ]), gl.STATIC_DRAW);
    
    statusPanelTexCoordBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, statusPanelTexCoordBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
        0, 1,
        1, 1,
        0, 0,
        1, 0,
    ]), gl.STATIC_DRAW);
    
    // Initial update
    updateStatusPanelCanvas();
    
    statusPanelInitialized = true;
    vrLogFn('Status panel: OK');
}

function updateStatusPanelCanvas() {
    if (!statusPanelCtx) return;
    
    const foxgloveConn = getFoxgloveConnFn();
    const tfRate = getTfRateFn();
    const linkNamesLength = getLinkNamesLengthFn();
    const urdfString = getUrdfStringFn();
    
    const canvas = statusPanelCanvas;
    const ctx = statusPanelCtx;
    
    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Semi-transparent dark background with rounded corners
    ctx.fillStyle = 'rgba(20, 20, 30, 0.5)';
    drawRoundedRect(ctx, 0, 0, canvas.width, canvas.height, 12);
    
    // Title
    ctx.fillStyle = '#88ccff';
    ctx.font = 'bold 14px monospace';
    ctx.textBaseline = 'top';
    ctx.fillText('Robot Status', 10, 8);
    
    // Foxglove connection status
    const foxgloveStatus = foxgloveConn && foxgloveConn.isConnected ? 'Connected' : 'Disconnected';
    const foxgloveColor = foxgloveConn && foxgloveConn.isConnected ? '#44ff44' : '#ff4444';
    ctx.fillStyle = '#cccccc';
    ctx.font = '11px monospace';
    ctx.fillText('Foxglove:', 10, 28);
    ctx.fillStyle = foxgloveColor;
    ctx.fillText(foxgloveStatus, 80, 28);
    
    // TF Rate
    const tfColor = tfRate > 10 ? '#44ff44' : (tfRate > 0 ? '#ffff00' : '#ff4444');
    ctx.fillStyle = '#cccccc';
    ctx.fillText('TF Rate:', 10, 43);
    ctx.fillStyle = tfColor;
    ctx.fillText(`${tfRate.toFixed(1)} Hz`, 80, 43);
    
    // Link count
    ctx.fillStyle = '#cccccc';
    ctx.fillText('Links:', 10, 58);
    ctx.fillStyle = '#ffffff';
    ctx.fillText(`${linkNamesLength}`, 80, 58);
    
    // URDF status
    ctx.fillStyle = '#cccccc';
    ctx.fillText('URDF:', 10, 73);
    ctx.fillStyle = urdfString ? '#44ff44' : '#888888';
    ctx.fillText(urdfString ? 'Loaded' : 'Waiting...', 80, 73);
    
    // Update texture
    if (gl && statusPanelTexture) {
        gl.bindTexture(gl.TEXTURE_2D, statusPanelTexture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, canvas);
    }
}

export function drawStatusPanel(view, viewport, modelMatrix) {
    const frameCounter = getFrameCounterFn();
    const shaderProgram = getShaderProgramFn();
    const cachedLocations = getCachedLocationsFn();
    
    // Initialize on first call
    if (!statusPanelInitialized) {
        initStatusPanel();
    }
    
    if (!statusPanelInitialized || !shaderProgram || !statusPanelTexture || !cachedLocations) {
        if (frameCounter < 20 && frameCounter % 5 === 0) {
            vrLogFn(`Status panel skip: init=${statusPanelInitialized}, shader=${!!shaderProgram}, tex=${!!statusPanelTexture}, locs=${!!cachedLocations}`);
        }
        return;
    }
    
    if (frameCounter === 15) {
        vrLogFn('Drawing status panel');
    }
    
    // Update canvas content periodically
    if (frameCounter % 30 === 0) {
        updateStatusPanelCanvas();
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
        gl.bindBuffer(gl.ARRAY_BUFFER, statusPanelPositionBuffer);
        gl.enableVertexAttribArray(cachedLocations.position);
        gl.vertexAttribPointer(cachedLocations.position, 2, gl.FLOAT, false, 0, 0);
    }
    
    // Set up tex coord buffer
    if (cachedLocations.texCoord !== -1 && cachedLocations.texCoord !== null) {
        gl.bindBuffer(gl.ARRAY_BUFFER, statusPanelTexCoordBuffer);
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
            createRotatedModelMatrix(modelMatrix, 15, rightPanelModelMatrix);
            gl.uniformMatrix4fv(cachedLocations.model, false, rightPanelModelMatrix);
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
        gl.bindTexture(gl.TEXTURE_2D, statusPanelTexture);
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

// ========================================
// Left Status Panel Functions
// ========================================

function initLeftStatusPanel() {
    if (!gl) {
        vrLogFn('Left status panel: no GL context');
        return;
    }
    
    vrLogFn('Init left status panel...');
    
    // Create canvas for status text (taller for more info)
    leftStatusPanelCanvas = document.createElement('canvas');
    leftStatusPanelCanvas.width = 256;
    leftStatusPanelCanvas.height = 144;
    leftStatusPanelCtx = leftStatusPanelCanvas.getContext('2d');
    
    // Create texture
    leftStatusPanelTexture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, leftStatusPanelTexture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    
    // Create position buffer for 3D quad
    const mainHeight = stereoSettings.screenScale;
    const mainWidth = mainHeight * (16/9);
    const mainLeftEdge = -mainWidth / 2;
    
    // Left status panel size
    const panelHeight = 0.135;
    const panelWidth = 0.20;
    
    const panelLeft = mainLeftEdge + 0.24;
    const panelCenterY = stereoSettings.verticalOffset - 0.175;
    
    leftStatusPanelPositionBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, leftStatusPanelPositionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
        panelLeft, panelCenterY - panelHeight/2,
        panelLeft + panelWidth, panelCenterY - panelHeight/2,
        panelLeft, panelCenterY + panelHeight/2,
        panelLeft + panelWidth, panelCenterY + panelHeight/2,
    ]), gl.STATIC_DRAW);
    
    leftStatusPanelTexCoordBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, leftStatusPanelTexCoordBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
        0, 1,
        1, 1,
        0, 0,
        1, 0,
    ]), gl.STATIC_DRAW);
    
    // Initial update
    updateLeftStatusPanelCanvas();
    
    leftStatusPanelInitialized = true;
    vrLogFn('Left status panel: OK');
}

function updateLeftStatusPanelCanvas() {
    if (!leftStatusPanelCtx) return;
    
    const currentVrFps = getCurrentVrFpsFn();
    const currentBatteryLevel = getCurrentBatteryLevelFn();
    const currentBatteryCharging = getCurrentBatteryChargingFn();
    const lastFrameReceivedTimestamp = getLastFrameReceivedTimestampFn();
    
    const canvas = leftStatusPanelCanvas;
    const ctx = leftStatusPanelCtx;
    
    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Semi-transparent dark background with rounded corners
    ctx.fillStyle = 'rgba(20, 20, 30, 0.5)';
    drawRoundedRect(ctx, 0, 0, canvas.width, canvas.height, 12);
    
    // Title
    ctx.fillStyle = '#88ccff';
    ctx.font = 'bold 14px monospace';
    ctx.textBaseline = 'top';
    ctx.fillText('Stream Info', 10, 8);
    
    // FPS
    ctx.fillStyle = '#cccccc';
    ctx.font = '11px monospace';
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
    if (gl && leftStatusPanelTexture) {
        gl.bindTexture(gl.TEXTURE_2D, leftStatusPanelTexture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, canvas);
    }
}

export function drawLeftStatusPanel(view, viewport, modelMatrix) {
    const frameCounter = getFrameCounterFn();
    const shaderProgram = getShaderProgramFn();
    const cachedLocations = getCachedLocationsFn();
    
    // Initialize on first call
    if (!leftStatusPanelInitialized) {
        initLeftStatusPanel();
    }
    
    if (!leftStatusPanelInitialized || !shaderProgram || !leftStatusPanelTexture || !cachedLocations) {
        return;
    }
    
    // Update canvas content periodically
    if (frameCounter % 30 === 0) {
        updateLeftStatusPanelCanvas();
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
        gl.bindBuffer(gl.ARRAY_BUFFER, leftStatusPanelPositionBuffer);
        gl.enableVertexAttribArray(cachedLocations.position);
        gl.vertexAttribPointer(cachedLocations.position, 2, gl.FLOAT, false, 0, 0);
    }
    
    // Set up tex coord buffer
    if (cachedLocations.texCoord !== -1 && cachedLocations.texCoord !== null) {
        gl.bindBuffer(gl.ARRAY_BUFFER, leftStatusPanelTexCoordBuffer);
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
            createRotatedModelMatrix(modelMatrix, -15, leftPanelModelMatrix);
            gl.uniformMatrix4fv(cachedLocations.model, false, leftPanelModelMatrix);
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
        gl.bindTexture(gl.TEXTURE_2D, leftStatusPanelTexture);
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
