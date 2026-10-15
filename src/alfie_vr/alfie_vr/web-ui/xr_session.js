// XR Session Module
// WebXR session lifecycle (enter/exit button, GL context, render loop),
// controller input wiring and teleop safety releases.

import {
    passthroughMode,
    preAllocatedBuffers,
} from './state.js';
import {
    sendControllerData,
    processInputSources,
    processHeadsetPose,
    handleInputSourcesChange,
    sendReleaseAll,
    resetTracking,
} from './controller_tracking.js';
import {
    initVideoRenderer,
    disposeVideoRenderer,
    uploadVideoFrames,
    drawVideo,
    getQuadProgram,
} from './video_renderer.js';
import {
    updateOverlayPanels,
    drawOverlayPanels,
    disposeOverlayPanels,
} from './vr_overlays.js';
import {
    startUrdfPanel,
    disposeUrdfPanel,
} from './vr_urdf_panel.js';
import { getCurrentFrameBitmap } from './robot_state.js';
import { invertMatrix } from './gl_utils.js';
import { setStatus, vrLog } from './ui_status.js';

// ========================================
// Module State
// ========================================

let xrSession = null;
let xrRefSpace = null;
let gl = null;
let frameCounter = 0;
let sessionRequestPending = false;   // Guards against concurrent requestSession
let xrSupported = false;
let streamReady = false;
let onSessionEndedFn = () => {};

// Pre-allocated per-eye view matrix (inverse of view.transform.matrix)
const viewMatrixBuffer = preAllocatedBuffers.viewMatrix;

const NO_VIDEO_FRAMES = { hasWide: false, hasCenter: false };

export function isXrActive() {
    return xrSession !== null;
}

function getVrButton() {
    return document.getElementById('vrButton');
}

function updateVrButtonEnabled() {
    const btn = getVrButton();
    if (btn) btn.disabled = !(xrSupported && streamReady) || sessionRequestPending;
}

// Called when the first stereo frame arrives
export function setStreamReady() {
    streamReady = true;
    updateVrButtonEnabled();
}

// ========================================
// Initialization
// ========================================

/**
 * Initialize WebXR support and the Enter/Exit button.
 * @param {Object} opts
 * @param {Function} opts.onSessionEnded - Called after an XR session ends
 */
export async function initXR({ onSessionEnded } = {}) {
    if (onSessionEnded) onSessionEndedFn = onSessionEnded;
    installPageLifecycleHandlers();

    if (!navigator.xr) {
        setStatus('WebXR not supported', 'error');
        return;
    }

    const supported = await navigator.xr.isSessionSupported('immersive-ar');
    if (!supported) {
        setStatus('Immersive AR not supported', 'error');
        updateVrButtonEnabled();
        return;
    }

    xrSupported = true;
    // Single handler that toggles based on session state
    getVrButton().addEventListener('click', onVrButtonClick);
    updateVrButtonEnabled();
}

function onVrButtonClick() {
    if (xrSession) {
        xrSession.end();
    } else if (!sessionRequestPending) {
        startVRSession();
    }
}

// Release the robot whenever the page stops being visible or is unloaded
function installPageLifecycleHandlers() {
    document.addEventListener('visibilitychange', () => {
        if (document.visibilityState !== 'hidden') return;
        // A visible immersive session keeps rendering/teleoperating even if the
        // browser reports the 2D page as hidden; its own visibilitychange covers blur.
        if (xrSession && xrSession.visibilityState === 'visible') return;
        sendReleaseAll('page_hidden');
        resetTracking();
    });
    window.addEventListener('pagehide', () => {
        sendReleaseAll('pagehide');
        resetTracking();
    });
}

// ========================================
// Session Lifecycle
// ========================================

// Start immersive VR session
async function startVRSession() {
    sessionRequestPending = true;
    updateVrButtonEnabled();
    let session = null;
    try {
        setStatus('Starting VR session...');
        vrLog('Starting VR session...');

        // Request immersive AR session with local-floor reference space and passthrough
        session = await navigator.xr.requestSession('immersive-ar', {
            requiredFeatures: ['local-floor'],
            optionalFeatures: ['hand-tracking', 'dom-overlay'],
            domOverlay: { root: document.body }
        });
        xrSession = session;

        vrLog('XR session created');

        // Show debug overlay in VR
        document.getElementById('vrDebug').style.display = 'block';

        session.addEventListener('end', onSessionEnd);
        session.addEventListener('inputsourceschange', handleInputSourcesChange);
        session.addEventListener('visibilitychange', () => {
            // Input is not delivered while blurred/hidden (e.g. system menu)
            if (session.visibilityState !== 'visible') {
                sendReleaseAll('xr_' + session.visibilityState);
            }
        });

        // Set up WebGL context for XR - optimized for performance
        const canvas = document.createElement('canvas');
        const glOptions = {
            xrCompatible: true,
            antialias: false,           // Disable AA for performance
            depth: true,                // Enable depth buffer for WebXR
            stencil: false,             // No stencil needed
            powerPreference: 'high-performance',
            preserveDrawingBuffer: false
        };
        const glCtx = canvas.getContext('webgl2', glOptions) || canvas.getContext('webgl', glOptions);
        if (!glCtx) {
            throw new Error('Failed to create WebGL context');
        }
        gl = glCtx;

        console.log('WebGL context created:', gl.getParameter(gl.VERSION));
        vrLog('WebGL: ' + gl.getParameter(gl.VERSION));

        await session.updateRenderState({
            baseLayer: new XRWebGLLayer(session, gl, {
                antialias: false,       // Faster rendering
                depth: true,            // Enable depth buffer
                stencil: false,         // No stencil needed
                alpha: true,            // Enable alpha for proper blending
                framebufferScaleFactor: 1.0  // Native resolution
            })
        });

        assertSessionCurrent(session);

        // Enable depth testing and blending
        gl.enable(gl.DEPTH_TEST);
        gl.depthFunc(gl.LEQUAL);
        gl.enable(gl.BLEND);
        gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
        vrLog('Depth/blend enabled');

        xrRefSpace = await session.requestReferenceSpace('local-floor');
        assertSessionCurrent(session);
        vrLog('Got reference space');

        // Fresh GL resources for this context (previous session's were disposed)
        if (!initVideoRenderer(gl)) {
            vrLog('ERROR: No shader!');
        }

        vrLog('Video ready: ' + (getCurrentFrameBitmap() ? 'YES' : 'NO'));
        if (getCurrentFrameBitmap()) {
            vrLog('Video: ' + getCurrentFrameBitmap().width + 'x' + getCurrentFrameBitmap().height);
        }

        document.getElementById('preview').classList.add('hidden');
        getVrButton().textContent = '🛑 Exit AR';

        setStatus('AR Active - Stereo 3D Vision (95% opacity)', 'connected');

        // URDF panel: offscreen Three.js renderer + URDF/TF listeners for this session
        startUrdfPanel();

        // Start XR render loop
        session.requestAnimationFrame(onXRFrame);

    } catch (err) {
        if (session && xrSession !== session) {
            // Session already ended (and was cleaned up) while we were initializing
            console.log('VR session ended during startup');
        } else {
            console.error('VR session error:', err);
            setStatus('VR error: ' + err.message, 'error');
            if (session) {
                // Tear down a half-initialized session ('end' runs onSessionEnd)
                session.end().catch(() => {});
            }
        }
    } finally {
        sessionRequestPending = false;
        updateVrButtonEnabled();
    }
}

// The session can end while startVRSession is awaiting; stop initializing if so
function assertSessionCurrent(session) {
    if (xrSession !== session) {
        throw new Error('XR session ended during startup');
    }
}

function onSessionEnd() {
    // Safety first: tell the server to release everything
    sendReleaseAll('xr_session_end');
    resetTracking();

    // Dispose all GL-context-bound state so re-entering VR starts clean
    disposeUrdfPanel();
    disposeOverlayPanels();
    disposeVideoRenderer();
    if (gl) {
        const loseContext = gl.getExtension('WEBGL_lose_context');
        if (loseContext) loseContext.loseContext();
    }

    xrSession = null;
    xrRefSpace = null;
    gl = null;
    frameCounter = 0;

    setStatus('AR session ended', '');
    getVrButton().textContent = '🥽 Enter AR';
    updateVrButtonEnabled();
    document.getElementById('preview').classList.remove('hidden');

    onSessionEndedFn();
}

// ========================================
// Render Loop
// ========================================

// XR render loop - optimized for low latency
function onXRFrame(time, frame) {
    if (!xrSession || frame.session !== xrSession) return;
    const session = xrSession;

    session.requestAnimationFrame(onXRFrame);

    const pose = frame.getViewerPose(xrRefSpace);

    // Process controller and headset tracking (untracked parts are omitted from sends)
    processInputSources(frame, xrRefSpace, session);
    processHeadsetPose(pose);

    // Send controller data to WebSocket (throttled - every 3 frames)
    if (frameCounter % 3 === 0) {
        sendControllerData();
    }

    if (!pose) {
        if (frameCounter % 60 === 0) {
            console.log('No pose available');
            vrLog('No pose');
        }
        frameCounter++;
        return;
    }

    if (frameCounter === 5) {
        vrLog('Pose OK, views: ' + pose.views.length);
    }

    const glLayer = session.renderState.baseLayer;
    gl.bindFramebuffer(gl.FRAMEBUFFER, glLayer.framebuffer);

    // Check passthrough mode - if active, use 0% opacity (full passthrough) and skip rendering
    if (passthroughMode.active) {
        gl.clearColor(0.0, 0.0, 0.0, 0.0);  // 0% opacity - full passthrough
        gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
        frameCounter++;
        return;  // Skip all rendering - panels, video, everything
    }

    // Clear with 95% opacity black to dim passthrough (5% passthrough visible)
    gl.clearColor(0.0, 0.0, 0.0, 0.95);  // 95% opacity for immersive AR dimming
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    const hasVideo = !!getCurrentFrameBitmap();
    if (!hasVideo && frameCounter % 60 === 0) {
        console.log('Waiting for video frame');
        vrLog('Wait video');
    }
    frameCounter++;

    // Once per frame: upload new video frames and changed panel canvases
    const frames = hasVideo ? uploadVideoFrames(frameCounter) : NO_VIDEO_FRAMES;
    updateOverlayPanels(gl, frameCounter);
    const prog = getQuadProgram();
    const headMatrix = pose.transform.matrix;

    // Render to both eyes
    for (const view of pose.views) {
        const viewport = glLayer.getViewport(view);
        gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);

        // Invert view transform once per eye (view.transform is viewer pose)
        invertMatrix(view.transform.matrix, viewMatrixBuffer);

        if (frameCounter === 12) {
            vrLog(view.eye + ' eye: ' + viewport.width + 'x' + viewport.height);
        }

        // Draw wide view, center overlay and outline
        drawVideo(view, viewMatrixBuffer, headMatrix, frames);

        // Draw status panels (head-locked)
        drawOverlayPanels(view, viewMatrixBuffer, headMatrix, prog);
    }
}
