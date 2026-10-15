// VR Overlays Module
// Rosout (bottom) and bearing (top) panels, plus the aggregate per-frame
// update / draw / dispose entry points for every head-locked VR panel.

import {
    stereoSettings,
    headsetPose,
} from './state.js';
import { TexturedPanel, drawRoundedRect } from './gl_panel.js';
import { getRosoutMessages } from './robot_state.js';
import { vrLog } from './ui_status.js';
import { updateStatePanel, drawStatePanel, disposeStatePanel } from './state_panel.js';
import { updateStreamPanel, drawStreamPanel, disposeStreamPanel } from './stream_panel.js';
import { updateUrdfPanel, drawUrdfPanel } from './vr_urdf_panel.js';

// ========================================
// Panel Configuration (adjust these values)
// ========================================

// Bottom rosout log panel configuration
const ROSOUT_PANEL_CONFIG = {
    width: 0.45,                // Wide panel at bottom
    height: 0.06,
    horizontalOffset: 0.0,      // Centered horizontally
    verticalOffset: -0.20,      // Below main screen
    angle: 0,                   // No rotation
    backgroundAlpha: 0.0,      // Background transparency (0-1)
    canvasScale: 1280,          // Multiplier to convert panel size to canvas pixels
    titleFontSize: 17,          // Title font size in pixels
    bodyFontSize: 13,           // Body text font size in pixels
    maxMessages: 3,             // Number of log messages to display
    cornerRadius: 0.02,         // Shader corner radius
    refreshFrames: 10,          // Repaint canvas every N XR frames (reduce CPU load)
};

// Top bearing indicator panel configuration
const BEARING_PANEL_CONFIG = {
    width: 0.12,                // Compact centered panel
    height: 0.04,
    horizontalOffset: 0.0,      // Centered horizontally
    verticalOffset: 0.22,       // Above main screen
    angle: 0,                   // No rotation
    backgroundAlpha: 0.0,       // Background transparency (0-1)
    canvasScale: 1280,          // Multiplier to convert panel size to canvas pixels
    fontSize: 32,               // Main yaw display font size
    nominalThreshold: 5,        // Degrees for green (nominal)
    warningThreshold: 30,       // Degrees for yellow (warning), beyond is red
    displayMultiplier: 2,       // Multiplier for displayed bearing value
    cornerRadius: 0.0,          // Shader corner radius
    refreshFrames: 5,           // Repaint every N XR frames (bearing needs responsive updates)
};

const LEVEL_COLORS = {
    'DEBUG': '#888888',
    'INFO': '#44ff44',
    'WARN': '#f39c12',
    'ERROR': '#e74c3c',
    'FATAL': '#ff00ff'
};

// Centered panel rect helper (no IPD offset, no rotation)
function centeredRect(config) {
    const panelCenterX = config.horizontalOffset;
    const panelCenterY = stereoSettings.verticalOffset + config.verticalOffset;
    return {
        left: panelCenterX - config.width / 2,
        bottom: panelCenterY - config.height / 2,
        width: config.width,
        height: config.height,
    };
}

// ========================================
// Module State
// ========================================

const rosoutPanel = new TexturedPanel({
    name: 'rosout',
    canvasWidth: Math.round(ROSOUT_PANEL_CONFIG.width * ROSOUT_PANEL_CONFIG.canvasScale),
    canvasHeight: Math.round(ROSOUT_PANEL_CONFIG.height * ROSOUT_PANEL_CONFIG.canvasScale),
    angle: ROSOUT_PANEL_CONFIG.angle,
    cornerRadius: ROSOUT_PANEL_CONFIG.cornerRadius,
    ipdScale: 0.0,  // Zero IPD offset for centered UI panels
    computeRect: () => centeredRect(ROSOUT_PANEL_CONFIG),
});

const bearingPanel = new TexturedPanel({
    name: 'bearing',
    canvasWidth: Math.round(BEARING_PANEL_CONFIG.width * BEARING_PANEL_CONFIG.canvasScale),
    canvasHeight: Math.round(BEARING_PANEL_CONFIG.height * BEARING_PANEL_CONFIG.canvasScale),
    angle: BEARING_PANEL_CONFIG.angle,
    cornerRadius: BEARING_PANEL_CONFIG.cornerRadius,
    ipdScale: 0.0,  // Zero IPD offset for centered UI panels
    computeRect: () => centeredRect(BEARING_PANEL_CONFIG),
});

// ========================================
// Rosout Log Panel Functions (Bottom Panel)
// ========================================

function updateRosoutPanelCanvas() {
    const messages = getRosoutMessages();
    const canvas = rosoutPanel.canvas;
    const ctx = rosoutPanel.ctx;

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Semi-transparent dark background with rounded corners
    ctx.fillStyle = `rgba(20, 20, 30, ${ROSOUT_PANEL_CONFIG.backgroundAlpha})`;
    drawRoundedRect(ctx, 0, 0, canvas.width, canvas.height, 12);

    // Title
    ctx.fillStyle = '#f39c12';
    ctx.font = `bold ${ROSOUT_PANEL_CONFIG.titleFontSize}px monospace`;
    ctx.textBaseline = 'top';
    ctx.fillText('📋 ROS Logs', 10, 6);

    // Draw last N messages
    const displayMessages = messages.slice(-ROSOUT_PANEL_CONFIG.maxMessages);
    ctx.font = `${ROSOUT_PANEL_CONFIG.bodyFontSize}px monospace`;

    const startY = 24;
    const lineHeight = 15;

    displayMessages.forEach((msg, i) => {
        const y = startY + i * lineHeight;

        // Format: [LEVEL] message
        ctx.fillStyle = LEVEL_COLORS[msg.levelName] || '#cccccc';
        ctx.fillText(`[${msg.levelName}]`, 10, y);

        // Truncate message if too long
        const maxMsgLen = 90;
        const truncatedMsg = msg.msg.length > maxMsgLen ?
            msg.msg.substring(0, maxMsgLen) + '...' : msg.msg;
        ctx.fillText(truncatedMsg, 70, y);
    });

    // If no messages, show placeholder
    if (displayMessages.length === 0) {
        ctx.fillStyle = '#666666';
        ctx.fillText('Waiting for log messages...', 10, startY);
    }

    rosoutPanel.markDirty();
}

// ========================================
// Bearing Panel Functions (Top Panel)
// ========================================

function updateBearingPanelCanvas() {
    const canvas = bearingPanel.canvas;
    const ctx = bearingPanel.ctx;

    // Get current yaw from headset pose (multiplied for display)
    const yaw = headsetPose.rotation ? Math.round(headsetPose.rotation.y * BEARING_PANEL_CONFIG.displayMultiplier) : 0;
    const absYaw = Math.abs(yaw);

    // Determine color based on yaw threshold
    let color;
    if (absYaw <= BEARING_PANEL_CONFIG.nominalThreshold) {
        color = '#44ff44';  // Green - nominal
    } else if (absYaw <= BEARING_PANEL_CONFIG.warningThreshold) {
        color = '#ffff00';  // Yellow - warning
    } else {
        color = '#ff4444';  // Red - danger
    }

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Semi-transparent dark background with rounded corners
    ctx.fillStyle = `rgba(20, 20, 30, ${BEARING_PANEL_CONFIG.backgroundAlpha})`;
    drawRoundedRect(ctx, 0, 0, canvas.width, canvas.height, 12);

    // Set font for measurements
    ctx.font = `bold ${BEARING_PANEL_CONFIG.fontSize}px monospace`;
    ctx.textBaseline = 'middle';

    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;

    // Format yaw text
    const yawText = `${yaw}°`;

    // Measure text width for centering
    const textWidth = ctx.measureText(yawText).width;

    // Draw yaw value centered
    ctx.fillStyle = color;
    ctx.textAlign = 'center';
    ctx.fillText(yawText, centerX, centerY);

    // Draw arrow if outside nominal range (pointing direction to return to center)
    if (absYaw > BEARING_PANEL_CONFIG.nominalThreshold) {
        const arrowSize = BEARING_PANEL_CONFIG.fontSize * 1.6;
        const arrowOffset = textWidth / 2 + 25;

        ctx.font = `bold ${arrowSize}px monospace`;

        if (yaw > 0) {
            // Yaw is positive (looking right), arrow points right
            ctx.fillText('▶', centerX + arrowOffset, centerY);
        } else {
            // Yaw is negative (looking left), arrow points left
            ctx.fillText('◀', centerX - arrowOffset, centerY);
        }
    }

    bearingPanel.markDirty();
}

function updatePanel(panel, gl, frameCounter, refreshFrames, repaint) {
    const created = panel.init(gl);
    if (created) vrLog(`${panel.name} panel: OK`);
    if (created || frameCounter % refreshFrames === 0) {
        repaint();
    }
    panel.uploadIfDirty();
}

// ========================================
// Aggregate Panel Entry Points
// ========================================

// Call once per XR frame (before the per-eye loop): repaint + upload changed panels
export function updateOverlayPanels(gl, frameCounter) {
    updateStatePanel(gl, frameCounter);
    updateStreamPanel(gl, frameCounter);
    updatePanel(bearingPanel, gl, frameCounter, BEARING_PANEL_CONFIG.refreshFrames, updateBearingPanelCanvas);
    updatePanel(rosoutPanel, gl, frameCounter, ROSOUT_PANEL_CONFIG.refreshFrames, updateRosoutPanelCanvas);
    updateUrdfPanel(gl, frameCounter);
}

// Draw all head-locked panels for one eye
export function drawOverlayPanels(view, viewMatrix, headMatrix, prog) {
    if (!prog) return;
    drawStatePanel(view, viewMatrix, headMatrix, prog);
    drawStreamPanel(view, viewMatrix, headMatrix, prog);
    bearingPanel.draw(view, viewMatrix, headMatrix, prog);
    rosoutPanel.draw(view, viewMatrix, headMatrix, prog);
    drawUrdfPanel(view, viewMatrix, headMatrix, prog);
}

// Release GL resources of the canvas panels (URDF panel is disposed separately,
// since it also owns a Three.js renderer and TF/URDF listeners)
export function disposeOverlayPanels() {
    disposeStatePanel();
    disposeStreamPanel();
    bearingPanel.dispose();
    rosoutPanel.dispose();
}
