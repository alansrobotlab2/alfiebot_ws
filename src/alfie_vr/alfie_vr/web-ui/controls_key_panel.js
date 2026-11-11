// Controls Key Panel Module
// Head-locked 16:9 reference sheet of the controller mapping (controls_key.png),
// sized and placed like the video screen. Shown instead of the HUD while left X is held.

import { immersive3DConfig } from './state.js';
import { TexturedPanel } from './gl_panel.js';
import { vrLog } from './ui_status.js';

const IMAGE_URL = 'controls_key.png';
const CANVAS_WIDTH = 1920;
const CANVAS_HEIGHT = 1080;

let imageLoaded = false;

const keyPanel = new TexturedPanel({
    name: 'controls key',
    canvasWidth: CANVAS_WIDTH,
    canvasHeight: CANVAS_HEIGHT,
    cornerRadius: 0.03,
    ipdScale: 0.0,  // Flat UI: true per-eye geometry, no convergence shift
    computeRect: () => {
        // Same footprint as the video screen (video_renderer.js)
        const height = immersive3DConfig.screenScale;
        const width = height * 16 / 9;
        return {
            left: -width / 2,
            bottom: -height / 2 + immersive3DConfig.verticalOffset,
            width,
            height,
        };
    },
});

function paintPlaceholder() {
    const ctx = keyPanel.ctx;
    ctx.fillStyle = '#1a2027';
    ctx.fillRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);
    ctx.fillStyle = '#e6eaee';
    ctx.font = 'bold 64px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('Controls key not loaded', CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2);
    keyPanel.markDirty();
}

// Load once at module import; the canvas outlives XR sessions
const image = new Image();
image.onload = () => {
    keyPanel.ctx.drawImage(image, 0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);
    imageLoaded = true;
    keyPanel.markDirty();
};
image.onerror = () => vrLog(`Controls key: failed to load ${IMAGE_URL}`);
paintPlaceholder();
image.src = IMAGE_URL;

// Call once per XR frame while the key is shown (before the per-eye loop)
export function updateControlsKeyPanel(gl) {
    if (keyPanel.init(gl)) {
        vrLog(`controls key panel: ${imageLoaded ? 'OK' : 'placeholder'}`);
        if (imageLoaded) keyPanel.ctx.drawImage(image, 0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);
        keyPanel.markDirty();
    }
    keyPanel.uploadIfDirty();
}

export function drawControlsKeyPanel(view, viewMatrix, headMatrix, prog) {
    keyPanel.draw(view, viewMatrix, headMatrix, prog);
}

export function disposeControlsKeyPanel() {
    keyPanel.dispose();
}
