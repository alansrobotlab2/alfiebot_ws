// State Panel Module
// Displays robot state info (Foxglove connection, TF rate, Links, URDF status)
// Positioned along the left edge in immersive VR mode

import {
    stereoSettings,
    preAllocatedBuffers,
} from './state.js';

// ========================================
// State Panel Configuration
// ========================================

const STATE_PANEL_CONFIG = {
    width: 0.15,
    height: 0.09,
    horizontalOffset: 0.2,     // Offset from left edge (matching stream panel)
    verticalOffset: 0.0,       // Centered vertically relative to stereoSettings.verticalOffset
    angle: 10,                 // Yaw rotation in degrees (positive = angled toward viewer)
    backgroundAlpha: 0.5,      // Background transparency (0-1)
    canvasScale: 1280,         // Multiplier to convert panel size to canvas pixels
    titleFontSize: 17,         // Title font size in pixels
    bodyFontSize: 13           // Body text font size in pixels
};

// ========================================
// Module State
// ========================================

let statePanelCanvas = null;
let statePanelCtx = null;
let statePanelTexture = null;
let statePanelPositionBuffer = null;
let statePanelTexCoordBuffer = null;
let statePanelInitialized = false;

// Pre-allocated buffer for model matrix
const statePanelModelMatrix = preAllocatedBuffers.rightPanelModelMatrix;

// ========================================
// Context and Callback References
// ========================================

let gl = null;
let vrLogFn = (msg) => console.log('[VR]', msg);
let getFrameCounterFn = () => 0;
let getFoxgloveConnFn = () => null;
let getTfRateFn = () => 0;
let getLinkNamesLengthFn = () => 0;
let getUrdfStringFn = () => null;
let getShaderProgramFn = () => null;
let getCachedLocationsFn = () => null;
let invertMatrixFn = null;
let viewMatrixBuffer = null;

// Set the GL context and callbacks from main module
export function setStatePanelContext(context) {
    gl = context.gl;
    vrLogFn = context.vrLog || vrLogFn;
    getFrameCounterFn = context.getFrameCounter || getFrameCounterFn;
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
export function updateStatePanelGlContext(newGl) {
    gl = newGl;
    // Reset initialization flag since textures need to be recreated
    statePanelInitialized = false;
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
// State Panel Functions
// ========================================

function initStatePanel() {
    if (!gl) {
        vrLogFn('State panel: no GL context');
        return;
    }
    
    vrLogFn('Init state panel...');
    
    // Create canvas for status text (dimensions derived from panel size)
    statePanelCanvas = document.createElement('canvas');
    statePanelCanvas.width = Math.round(STATE_PANEL_CONFIG.width * STATE_PANEL_CONFIG.canvasScale);
    statePanelCanvas.height = Math.round(STATE_PANEL_CONFIG.height * STATE_PANEL_CONFIG.canvasScale);
    statePanelCtx = statePanelCanvas.getContext('2d');
    
    // Create texture
    statePanelTexture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, statePanelTexture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    
    // Create position buffer for 3D quad
    // Position along the left edge of the viewport
    const mainHeight = stereoSettings.screenScale;
    const mainWidth = mainHeight * (16/9);
    const mainLeftEdge = -mainWidth / 2;
    
    // State panel size and position
    const panelHeight = STATE_PANEL_CONFIG.height;
    const panelWidth = STATE_PANEL_CONFIG.width;
    
    // Position along left edge
    const panelLeft = mainLeftEdge + STATE_PANEL_CONFIG.horizontalOffset;
    const panelCenterY = stereoSettings.verticalOffset + STATE_PANEL_CONFIG.verticalOffset;
    
    statePanelPositionBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, statePanelPositionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
        panelLeft, panelCenterY - panelHeight/2,
        panelLeft + panelWidth, panelCenterY - panelHeight/2,
        panelLeft, panelCenterY + panelHeight/2,
        panelLeft + panelWidth, panelCenterY + panelHeight/2,
    ]), gl.STATIC_DRAW);
    
    statePanelTexCoordBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, statePanelTexCoordBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
        0, 1,
        1, 1,
        0, 0,
        1, 0,
    ]), gl.STATIC_DRAW);
    
    // Initial update
    updateStatePanelCanvas();
    
    statePanelInitialized = true;
    vrLogFn('State panel: OK');
}

function updateStatePanelCanvas() {
    if (!statePanelCtx) return;
    
    const foxgloveConn = getFoxgloveConnFn();
    const tfRate = getTfRateFn();
    const linkNamesLength = getLinkNamesLengthFn();
    const urdfString = getUrdfStringFn();
    
    const canvas = statePanelCanvas;
    const ctx = statePanelCtx;
    
    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Semi-transparent dark background with rounded corners
    ctx.fillStyle = `rgba(20, 20, 30, ${STATE_PANEL_CONFIG.backgroundAlpha})`;
    drawRoundedRect(ctx, 0, 0, canvas.width, canvas.height, 12);
    
    // Title
    ctx.fillStyle = '#88ccff';
    ctx.font = `bold ${STATE_PANEL_CONFIG.titleFontSize}px monospace`;
    ctx.textBaseline = 'top';
    ctx.fillText('Robot State', 10, 8);
    
    // Foxglove connection status
    const foxgloveStatus = foxgloveConn && foxgloveConn.isConnected ? 'Connected' : 'Disconnected';
    const foxgloveColor = foxgloveConn && foxgloveConn.isConnected ? '#44ff44' : '#ff4444';
    ctx.fillStyle = '#cccccc';
    ctx.font = `${STATE_PANEL_CONFIG.bodyFontSize}px monospace`;
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
    if (gl && statePanelTexture) {
        gl.bindTexture(gl.TEXTURE_2D, statePanelTexture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, canvas);
    }
}

export function drawStatePanel(view, viewport, modelMatrix) {
    const frameCounter = getFrameCounterFn();
    const shaderProgram = getShaderProgramFn();
    const cachedLocations = getCachedLocationsFn();
    
    // Initialize on first call
    if (!statePanelInitialized) {
        initStatePanel();
    }
    
    if (!statePanelInitialized || !shaderProgram || !statePanelTexture || !cachedLocations) {
        if (frameCounter < 20 && frameCounter % 5 === 0) {
            vrLogFn(`State panel skip: init=${statePanelInitialized}, shader=${!!shaderProgram}, tex=${!!statePanelTexture}, locs=${!!cachedLocations}`);
        }
        return;
    }
    
    if (frameCounter === 15) {
        vrLogFn('Drawing state panel');
    }
    
    // Update canvas content periodically
    if (frameCounter % 30 === 0) {
        updateStatePanelCanvas();
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
        gl.bindBuffer(gl.ARRAY_BUFFER, statePanelPositionBuffer);
        gl.enableVertexAttribArray(cachedLocations.position);
        gl.vertexAttribPointer(cachedLocations.position, 2, gl.FLOAT, false, 0, 0);
    }
    
    // Set up tex coord buffer
    if (cachedLocations.texCoord !== -1 && cachedLocations.texCoord !== null) {
        gl.bindBuffer(gl.ARRAY_BUFFER, statePanelTexCoordBuffer);
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
            createRotatedModelMatrix(modelMatrix, STATE_PANEL_CONFIG.angle, statePanelModelMatrix);
            gl.uniformMatrix4fv(cachedLocations.model, false, statePanelModelMatrix);
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
        gl.bindTexture(gl.TEXTURE_2D, statePanelTexture);
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
