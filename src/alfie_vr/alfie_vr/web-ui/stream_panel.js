// Stream Panel Module
// Displays stream info (FPS, latency, battery, controller status, headset orientation)
// Positioned along the left edge in immersive VR mode

import {
    stereoSettings,
    connectionState,
    headsetPose,
    batteryState,
} from './state.js';
import { TexturedPanel, drawRoundedRect } from './gl_panel.js';
import {
    getCurrentVrFps,
    getLastFrameReceivedTimestamp,
} from './robot_state.js';
import { vrLog } from './ui_status.js';

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
    bodyFontSize: 13,          // Body text font size in pixels
    cornerRadius: 0.0,         // Shader corner radius (canvas draws its own rounded background)
    refreshFrames: 30,         // Repaint canvas every N XR frames
};

// ========================================
// Module State
// ========================================

const panel = new TexturedPanel({
    name: 'stream',
    canvasWidth: Math.round(STREAM_PANEL_CONFIG.width * STREAM_PANEL_CONFIG.canvasScale),
    canvasHeight: Math.round(STREAM_PANEL_CONFIG.height * STREAM_PANEL_CONFIG.canvasScale),
    angle: STREAM_PANEL_CONFIG.angle,
    cornerRadius: STREAM_PANEL_CONFIG.cornerRadius,
    ipdScale: 1.0,
    computeRect: () => {
        // Position along the left edge of the main screen
        const mainHeight = stereoSettings.screenScale;
        const mainWidth = mainHeight * (16/9);
        const mainLeftEdge = -mainWidth / 2;
        const panelCenterY = stereoSettings.verticalOffset + STREAM_PANEL_CONFIG.verticalOffset;
        return {
            left: mainLeftEdge + STREAM_PANEL_CONFIG.horizontalOffset,
            bottom: panelCenterY - STREAM_PANEL_CONFIG.height / 2,
            width: STREAM_PANEL_CONFIG.width,
            height: STREAM_PANEL_CONFIG.height,
        };
    },
});

// ========================================
// Stream Panel Functions
// ========================================

function updateStreamPanelCanvas() {
    const currentVrFps = getCurrentVrFps();
    const currentBatteryLevel = batteryState.level;
    const currentBatteryCharging = batteryState.charging;
    const lastFrameReceivedTimestamp = getLastFrameReceivedTimestamp();

    const canvas = panel.canvas;
    const ctx = panel.ctx;

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
    const ctrlConnected = !!(connectionState.controllerWS && connectionState.controllerWS.readyState === WebSocket.OPEN);
    ctx.fillStyle = '#cccccc';
    ctx.fillText('Ctrl WS:', 10, 73);
    ctx.fillStyle = ctrlConnected ? '#44ff44' : '#ff4444';
    ctx.fillText(ctrlConnected ? 'Connected' : 'Disconnected', 80, 73);

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

    panel.markDirty();
}

// Called once per XR frame: (re)create GL resources, repaint periodically, upload if changed
export function updateStreamPanel(gl, frameCounter) {
    const created = panel.init(gl);
    if (created) vrLog('Stream panel: OK');
    if (created || frameCounter % STREAM_PANEL_CONFIG.refreshFrames === 0) {
        updateStreamPanelCanvas();
    }
    panel.uploadIfDirty();
}

export function drawStreamPanel(view, viewMatrix, headMatrix, prog) {
    panel.draw(view, viewMatrix, headMatrix, prog);
}

// Release GL resources (session end / context loss)
export function disposeStreamPanel() {
    panel.dispose();
}
