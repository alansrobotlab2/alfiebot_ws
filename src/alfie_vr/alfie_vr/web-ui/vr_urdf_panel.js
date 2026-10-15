// VR Alfie Top View Panel Module
// Renders a Three.js URDF robot viewer from a top-down perspective to an offscreen canvas,
// then displays it as a textured panel in the VR scene.
// This approach avoids WebGL context conflicts between Three.js and custom shaders.

import { stereoSettings } from './state.js';
import { TexturedPanel } from './gl_panel.js';
import { buildUrdfRobot, applyLinkTransform, disposeObject3D } from './urdf_builder.js';
import { onUrdfAvailable, addTfListener, getTfTransforms } from './robot_state.js';
import { vrLog } from './ui_status.js';

// ========================================
// Panel Configuration
// ========================================

export const URDF_PANEL_CONFIG = {
    width: 0.18,                // Panel width in world units
    height: 0.38,               // Panel height in world units
    horizontalOffset: -0.150,    // Small gap from right edge of main screen
    verticalOffset: 0.00,       // Vertical offset from screen center (adjusted for taller panel)
    angle: -10,                 // Yaw rotation in degrees (negative = angled toward center)
    backgroundAlpha: 0.5,       // Background opacity
    canvasWidth: 512,           // Canvas width in pixels
    canvasHeight: 1088,         // Canvas height in pixels
    topViewRatio: 0.30,         // Top-down view gets 30% of height
    forwardOffset: 0.175,         // Shift both views forward (robot appears further back)
    viewSize: 0.6,              // Orthographic view size (same for both views, larger = see more)
    isoTargetHeight: 0.5,      // How high up the robot the side camera looks (centered on torso)
    renderFps: 15,              // Target FPS for Three.js rendering
    titleFontSize: 16,          // Title font size in pixels
    cornerRadius: 0.08,         // Shader corner radius (more rounded corners)
    ipdScale: 0.5,              // Reduced IPD offset for right-side panel
};

// ========================================
// Module State
// ========================================

// Three.js scene for URDF rendering
let urdfScene = null;
let urdfCamera = null;      // Top-down view
let urdfCameraIso = null;   // Side view
let urdfRenderer = null;
let urdfModel = null;       // Result of buildUrdfRobot()
let threeCanvas = null;     // Canvas where Three.js renders

let lastRenderTime = 0;

// Listener unsubscribe functions (active while a VR session is running)
let unsubscribeUrdf = null;
let unsubscribeTf = null;

// Composite canvas + GL texture/quad
const panel = new TexturedPanel({
    name: 'URDF',
    canvasWidth: URDF_PANEL_CONFIG.canvasWidth,
    canvasHeight: URDF_PANEL_CONFIG.canvasHeight,
    angle: URDF_PANEL_CONFIG.angle,
    cornerRadius: URDF_PANEL_CONFIG.cornerRadius,
    ipdScale: URDF_PANEL_CONFIG.ipdScale,
    computeRect: () => {
        // Right side of main screen
        const mainHeight = stereoSettings.screenScale;
        const mainWidth = mainHeight * (16/9);
        const mainRightEdge = mainWidth / 2;
        const panelLeft = mainRightEdge + URDF_PANEL_CONFIG.horizontalOffset - URDF_PANEL_CONFIG.width;
        const panelCenterY = stereoSettings.verticalOffset + URDF_PANEL_CONFIG.verticalOffset;
        vrLog(`URDF panel pos: L=${panelLeft.toFixed(2)} Y=${panelCenterY.toFixed(2)}`);
        return {
            left: panelLeft,
            bottom: panelCenterY - URDF_PANEL_CONFIG.height / 2,
            width: URDF_PANEL_CONFIG.width,
            height: URDF_PANEL_CONFIG.height,
        };
    },
});

// ========================================
// Initialization
// ========================================

/**
 * Initialize the Three.js URDF renderer (offscreen)
 */
function initThreeJsRenderer() {
    if (typeof THREE === 'undefined') {
        vrLog('Three.js not available');
        return false;
    }

    const canvasWidth = URDF_PANEL_CONFIG.canvasWidth;
    const canvasHeight = URDF_PANEL_CONFIG.canvasHeight;

    // Create offscreen canvas for Three.js rendering
    threeCanvas = document.createElement('canvas');
    threeCanvas.width = canvasWidth;
    threeCanvas.height = canvasHeight;

    // Create Three.js renderer on the offscreen canvas
    urdfRenderer = new THREE.WebGLRenderer({
        canvas: threeCanvas,
        antialias: true,
        alpha: true,  // Enable transparent background
        preserveDrawingBuffer: true,  // Needed to read pixels
    });
    urdfRenderer.setSize(canvasWidth, canvasHeight);
    urdfRenderer.setScissorTest(true);  // Enable scissor for split-view rendering
    urdfRenderer.autoClear = false;     // Manual clear for each viewport

    // Set clear color with alpha based on config
    const bgAlpha = URDF_PANEL_CONFIG.backgroundAlpha;
    if (bgAlpha > 0) {
        urdfRenderer.setClearColor(0x1a1a2e, bgAlpha);  // Dark blue-gray with configured alpha
    } else {
        urdfRenderer.setClearColor(0x000000, 0);  // Fully transparent
    }
    urdfRenderer.shadowMap.enabled = false;  // Disable shadows for performance

    // Create scene
    urdfScene = new THREE.Scene();

    // Calculate viewport dimensions for proper aspect ratios
    const topViewHeight = Math.floor(canvasHeight * URDF_PANEL_CONFIG.topViewRatio);
    const sideViewHeight = canvasHeight - topViewHeight;
    const viewSize = URDF_PANEL_CONFIG.viewSize;  // Base scale (used for side view)

    // Calculate aspect ratios
    const topAspect = canvasWidth / topViewHeight;
    const sideAspect = canvasWidth / sideViewHeight;

    // Side view camera dimensions (reference for alignment)
    const sideHalfWidth = viewSize * sideAspect;

    // Top view: match horizontal scale with side view for alignment
    // topHalfWidth should equal sideHalfWidth so horizontal pixels map to same world units
    const topHalfWidth = sideHalfWidth;
    const topViewSize = topHalfWidth / topAspect;  // Derived to match horizontal scale

    // Camera 1: Top-down orthographic view (top portion of canvas)
    urdfCamera = new THREE.OrthographicCamera(-topHalfWidth, topHalfWidth, topViewSize, -topViewSize, 0.01, 10);
    const forwardOffset = URDF_PANEL_CONFIG.forwardOffset;  // Shift view forward
    urdfCamera.position.set(0, 1, forwardOffset);
    urdfCamera.up.set(-1, 0, 0);  // Set "up" to -X for 90° CCW rotation (robot faces left)
    urdfCamera.lookAt(0, 0, forwardOffset);

    // Camera 2: Side view from the right (bottom portion of canvas)
    // Camera positioned on the right side (+X), looking at the robot
    // Robot faces right (forward = -X direction in view)
    urdfCameraIso = new THREE.OrthographicCamera(-sideHalfWidth, sideHalfWidth, viewSize, -viewSize, 0.01, 10);
    const sideDistance = 1.0;
    const isoTargetY = URDF_PANEL_CONFIG.isoTargetHeight;  // Look at this height on robot
    urdfCameraIso.position.set(sideDistance, isoTargetY, forwardOffset);  // Right side of robot, with forward offset
    urdfCameraIso.up.set(0, 1, 0);  // Y is up
    urdfCameraIso.lookAt(0, isoTargetY, forwardOffset);

    // Add lighting - works for both views
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    urdfScene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(1, 2, 1);
    urdfScene.add(directionalLight);

    const hemiLight = new THREE.HemisphereLight(0xffffff, 0x444444, 0.3);
    hemiLight.position.set(0, 10, 0);
    urdfScene.add(hemiLight);

    vrLog('Three.js dual-view URDF renderer initialized');
    return true;
}

/**
 * Start the URDF panel for a VR session: create the Three.js renderer and
 * subscribe to URDF / TF updates (unsubscribed again by disposeUrdfPanel()).
 */
export function startUrdfPanel() {
    if (!urdfRenderer && !initThreeJsRenderer()) {
        return;
    }
    if (!unsubscribeUrdf) {
        unsubscribeUrdf = onUrdfAvailable(loadUrdfForPanel);
    }
    if (!unsubscribeTf) {
        unsubscribeTf = addTfListener(applyUrdfPanelTransform);
    }
    vrLog('URDF Panel TF listener set');
}

// ========================================
// URDF Loading
// ========================================

/**
 * Load URDF string into the Three.js scene
 * @param {string} urdfString - The URDF XML string
 */
function loadUrdfForPanel(urdfString) {
    if (!urdfString || urdfModel || !urdfScene) return;

    try {
        const built = buildUrdfRobot(urdfString, { log: vrLog });
        if (!built) return;

        urdfModel = built;
        urdfScene.add(built.robot);

        // Apply the latest known transforms (TF static only arrives once)
        const transforms = getTfTransforms();
        for (const frameId of Object.keys(transforms)) {
            applyLinkTransform(urdfModel.links, frameId, transforms[frameId]);
        }

        vrLog('URDF Panel: loaded');
    } catch (error) {
        vrLog('Error loading URDF: ' + error.message);
        console.error('URDF load error:', error);
    }
}

// ========================================
// TF Transform Updates
// ========================================

/**
 * Apply a TF transform to the robot model
 * @param {string} childFrameId - The frame ID to update
 * @param {Object} transform - Transform with position {x,y,z} and quaternion {x,y,z,w}
 */
function applyUrdfPanelTransform(childFrameId, transform) {
    if (!urdfModel) return;
    applyLinkTransform(urdfModel.links, childFrameId, transform);
}

// ========================================
// Rendering
// ========================================

/**
 * Render the Three.js scene to the composite canvas (dual views).
 * Throttled to renderFps; returns true if the canvas changed.
 */
function renderThreeJsToCanvas() {
    const urdfCtx = panel.ctx;

    const now = performance.now();
    const frameInterval = 1000 / URDF_PANEL_CONFIG.renderFps;

    if (now - lastRenderTime < frameInterval) {
        return false;  // Skip frame to maintain target FPS
    }
    lastRenderTime = now;

    const canvasWidth = URDF_PANEL_CONFIG.canvasWidth;
    const canvasHeight = URDF_PANEL_CONFIG.canvasHeight;
    const topViewHeight = Math.floor(canvasHeight * URDF_PANEL_CONFIG.topViewRatio);
    const isoViewHeight = canvasHeight - topViewHeight;
    const urdfLoaded = !!urdfModel;

    // Clear canvas with transparency
    urdfCtx.clearRect(0, 0, canvasWidth, canvasHeight);

    // If Three.js is ready, render both views
    if (urdfRenderer && urdfScene && urdfCamera && urdfCameraIso) {
        // Top portion: Top-down view
        urdfRenderer.setViewport(0, isoViewHeight, canvasWidth, topViewHeight);
        urdfRenderer.setScissor(0, isoViewHeight, canvasWidth, topViewHeight);
        urdfRenderer.clear();  // Clear this viewport region
        urdfRenderer.render(urdfScene, urdfCamera);

        // Bottom portion: Side view
        urdfRenderer.setViewport(0, 0, canvasWidth, isoViewHeight);
        urdfRenderer.setScissor(0, 0, canvasWidth, isoViewHeight);
        urdfRenderer.clear();  // Clear this viewport region
        urdfRenderer.render(urdfScene, urdfCameraIso);

        // Copy to composite canvas (preserves transparency)
        if (threeCanvas) {
            urdfCtx.drawImage(threeCanvas, 0, 0);
        }
    } else {
        // Fallback: Draw a placeholder when Three.js isn't ready
        if (URDF_PANEL_CONFIG.backgroundAlpha > 0) {
            urdfCtx.fillStyle = `rgba(26, 26, 46, ${URDF_PANEL_CONFIG.backgroundAlpha})`;
            urdfCtx.fillRect(0, 0, canvasWidth, canvasHeight);
        }

        // Draw placeholder text
        urdfCtx.fillStyle = '#666666';
        urdfCtx.font = 'bold 24px sans-serif';
        urdfCtx.textAlign = 'center';
        urdfCtx.textBaseline = 'middle';
        urdfCtx.fillText('Initializing 3D...', canvasWidth / 2, canvasHeight / 2);
        urdfCtx.textAlign = 'left';
    }

    // Draw title bar for top view (top of canvas)
    const titleHeight = 28;
    urdfCtx.fillStyle = 'rgba(20, 20, 40, 0.85)';
    urdfCtx.fillRect(0, 0, canvasWidth, titleHeight);
    urdfCtx.fillStyle = '#88ccff';
    urdfCtx.font = `bold ${URDF_PANEL_CONFIG.titleFontSize}px sans-serif`;
    urdfCtx.textBaseline = 'middle';
    urdfCtx.fillText('⬇️ Top View', 10, titleHeight / 2);

    // Draw divider line between views (at the split point)
    urdfCtx.strokeStyle = '#4488cc';
    urdfCtx.lineWidth = 2;
    urdfCtx.beginPath();
    urdfCtx.moveTo(0, topViewHeight);
    urdfCtx.lineTo(canvasWidth, topViewHeight);
    urdfCtx.stroke();

    // Draw title bar for side view (at split point)
    urdfCtx.fillStyle = 'rgba(20, 20, 40, 0.85)';
    urdfCtx.fillRect(0, topViewHeight, canvasWidth, titleHeight);
    urdfCtx.fillStyle = '#88ccff';
    urdfCtx.fillText('👈 Side View', 10, topViewHeight + titleHeight / 2);

    // Draw status indicator on top title bar
    const statusColor = urdfLoaded ? '#44ff44' : '#ff8844';
    const statusText = urdfLoaded ? `${Object.keys(urdfModel.links).length} links` : 'Loading...';
    urdfCtx.fillStyle = statusColor;
    urdfCtx.font = '13px sans-serif';
    const textWidth = urdfCtx.measureText(statusText).width;
    urdfCtx.fillText(statusText, canvasWidth - textWidth - 10, titleHeight / 2);

    // Draw border only if background is visible
    if (URDF_PANEL_CONFIG.backgroundAlpha > 0) {
        urdfCtx.strokeStyle = '#4488cc';
        urdfCtx.lineWidth = 3;
        urdfCtx.strokeRect(1, 1, canvasWidth - 2, canvasHeight - 2);
    }
    return true;
}

/**
 * Called once per XR frame: render Three.js (throttled) and upload the texture
 * only when the canvas was re-rendered.
 */
export function updateUrdfPanel(gl, frameCounter) {
    const created = panel.init(gl);
    if (created) {
        lastRenderTime = 0;  // Force an initial render for the new texture
        vrLog('URDF panel: OK');
    }
    if (renderThreeJsToCanvas()) {
        panel.markDirty();
    }
    panel.uploadIfDirty();
}

/**
 * Draw the URDF panel in VR for one eye
 */
export function drawUrdfPanel(view, viewMatrix, headMatrix, prog) {
    panel.draw(view, viewMatrix, headMatrix, prog);
}

// ========================================
// Cleanup
// ========================================

/**
 * Dispose of URDF panel resources (session end): listeners, scene, Three.js
 * renderer/context and the panel's GL texture.
 */
export function disposeUrdfPanel() {
    if (unsubscribeUrdf) {
        unsubscribeUrdf();
        unsubscribeUrdf = null;
    }
    if (unsubscribeTf) {
        unsubscribeTf();
        unsubscribeTf = null;
    }

    if (urdfModel) {
        urdfModel.dispose();
        urdfModel = null;
    }

    if (urdfScene) {
        disposeObject3D(urdfScene);
        urdfScene = null;
    }

    if (urdfRenderer) {
        urdfRenderer.dispose();
        urdfRenderer.forceContextLoss();  // Free the offscreen WebGL context now
        urdfRenderer = null;
    }

    threeCanvas = null;
    urdfCamera = null;
    urdfCameraIso = null;
    lastRenderTime = 0;

    panel.dispose();
}
