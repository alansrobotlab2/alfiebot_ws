// State Panel Module
// Displays robot state info (Foxglove connection, TF rate, Links, URDF status)
// Positioned along the left edge in immersive VR mode

import { stereoSettings } from './state.js';
import { TexturedPanel, drawRoundedRect } from './gl_panel.js';
import {
    getFoxgloveConn,
    getTfRate,
    getLinkNames,
    getUrdfString,
} from './robot_state.js';
import { vrLog } from './ui_status.js';

// ========================================
// State Panel Configuration
// ========================================

const STATE_PANEL_CONFIG = {
    width: 0.15,
    height: 0.09,
    horizontalOffset: 0.15,     // Offset from left edge (matching stream panel)
    verticalOffset: -0.05,       // Centered vertically relative to stereoSettings.verticalOffset
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
    name: 'state',
    canvasWidth: Math.round(STATE_PANEL_CONFIG.width * STATE_PANEL_CONFIG.canvasScale),
    canvasHeight: Math.round(STATE_PANEL_CONFIG.height * STATE_PANEL_CONFIG.canvasScale),
    angle: STATE_PANEL_CONFIG.angle,
    cornerRadius: STATE_PANEL_CONFIG.cornerRadius,
    ipdScale: 1.0,
    computeRect: () => {
        // Position along the left edge of the main screen
        const mainHeight = stereoSettings.screenScale;
        const mainWidth = mainHeight * (16/9);
        const mainLeftEdge = -mainWidth / 2;
        const panelCenterY = stereoSettings.verticalOffset + STATE_PANEL_CONFIG.verticalOffset;
        return {
            left: mainLeftEdge + STATE_PANEL_CONFIG.horizontalOffset,
            bottom: panelCenterY - STATE_PANEL_CONFIG.height / 2,
            width: STATE_PANEL_CONFIG.width,
            height: STATE_PANEL_CONFIG.height,
        };
    },
});

// ========================================
// State Panel Functions
// ========================================

function updateStatePanelCanvas() {
    const foxgloveConn = getFoxgloveConn();
    const tfRate = getTfRate();
    const linkNamesLength = getLinkNames().length;
    const urdfString = getUrdfString();

    const canvas = panel.canvas;
    const ctx = panel.ctx;

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
    const connected = !!(foxgloveConn && foxgloveConn.isConnected);
    ctx.fillStyle = '#cccccc';
    ctx.font = `${STATE_PANEL_CONFIG.bodyFontSize}px monospace`;
    ctx.fillText('Foxglove:', 10, 28);
    ctx.fillStyle = connected ? '#44ff44' : '#ff4444';
    ctx.fillText(connected ? 'Connected' : 'Disconnected', 80, 28);

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

    panel.markDirty();
}

// Called once per XR frame: (re)create GL resources, repaint periodically, upload if changed
export function updateStatePanel(gl, frameCounter) {
    const created = panel.init(gl);
    if (created) vrLog('State panel: OK');
    if (created || frameCounter % STATE_PANEL_CONFIG.refreshFrames === 0) {
        updateStatePanelCanvas();
    }
    panel.uploadIfDirty();
}

export function drawStatePanel(view, viewMatrix, headMatrix, prog) {
    panel.draw(view, viewMatrix, headMatrix, prog);
}

// Release GL resources (session end / context loss)
export function disposeStatePanel() {
    panel.dispose();
}
