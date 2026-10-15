// Video Renderer Module
// Shaders, quads and textures for the stereo video screen (wide view + center
// overlay + center outline). The video quad program is also shared by the VR panels.
// All state here is bound to one GL context: call disposeVideoRenderer() on session end.

import {
    immersive3DConfig,
    centerViewState,
} from './state.js';
import {
    IDENTITY_MATRIX,
    QUAD_TEX_COORDS,
    writeRectPositions,
    createArrayBuffer,
    createLinearTexture,
    createSolidTexture,
    createProgram,
    getLocations,
    bindAttribute,
    deleteGlObjects,
    checkGlError,
} from './gl_utils.js';
import {
    getCurrentFrameBitmap,
    getLeftCenterFrameBitmap,
    getRightCenterFrameBitmap,
    getStereoFrameSeq,
    getCenterFrameSeq,
} from './robot_state.js';
import { vrLog } from './ui_status.js';

// ========================================
// Shader Sources
// ========================================

const QUAD_VERTEX_SHADER = `
    attribute vec2 a_position;
    attribute vec2 a_texCoord;
    varying vec2 v_texCoord;
    uniform mat4 u_projection;
    uniform mat4 u_view;
    uniform mat4 u_model;
    uniform float u_distance;
    uniform float u_ipdOffset;
    uniform float u_isLeftEye;

    void main() {
        // Apply IPD offset (inward for each eye to adjust convergence)
        float xOffset = u_ipdOffset * (u_isLeftEye > 0.5 ? 1.0 : -1.0);
        vec4 pos = vec4(a_position.x + xOffset, a_position.y, -u_distance, 1.0);
        gl_Position = u_projection * u_view * u_model * pos;
        v_texCoord = a_texCoord;
    }
`;

// Textured quad with rounded corners
const QUAD_FRAGMENT_SHADER = `
    precision mediump float;
    varying vec2 v_texCoord;
    uniform sampler2D u_texture;
    uniform float u_cornerRadius;

    // Signed distance function for rounded rectangle
    float roundedBoxSDF(vec2 centerPos, vec2 halfSize, float radius) {
        vec2 q = abs(centerPos) - halfSize + radius;
        return min(max(q.x, q.y), 0.0) + length(max(q, 0.0)) - radius;
    }

    void main() {
        // Apply rounded corners using SDF
        // Convert tex coords to centered coordinates (-0.5 to 0.5)
        vec2 centered = v_texCoord - 0.5;
        float dist = roundedBoxSDF(centered, vec2(0.5), u_cornerRadius);

        // Discard pixels outside rounded rectangle
        if (dist > 0.0) {
            discard;
        }

        // Smooth edge with anti-aliasing (slight softness at edges)
        float alpha = 1.0 - smoothstep(-0.005, 0.0, dist);

        vec4 color = texture2D(u_texture, v_texCoord);
        gl_FragColor = vec4(color.rgb, color.a * alpha);
    }
`;

// Hollow rounded rectangle outline
const OUTLINE_FRAGMENT_SHADER = `
    precision highp float;
    varying vec2 v_texCoord;
    uniform sampler2D u_texture;
    uniform float u_cornerRadius;
    uniform float u_outlineWidth;
    uniform float u_quadAspect;

    // Signed distance function for rounded rectangle with aspect ratio
    float roundedBoxSDF(vec2 centerPos, vec2 halfSize, float radius) {
        vec2 q = abs(centerPos) - halfSize + radius;
        return min(max(q.x, q.y), 0.0) + length(max(q, 0.0)) - radius;
    }

    void main() {
        // Convert tex coords to centered coordinates (-0.5 to 0.5)
        vec2 centered = v_texCoord - 0.5;

        // Apply aspect ratio correction to make perfect circular corners
        vec2 aspectCorrected = centered;
        aspectCorrected.x *= u_quadAspect;

        // Calculate distance from rounded rectangle edge (aspect-corrected half size)
        vec2 halfSize = vec2(u_quadAspect * 0.5, 0.5);
        float dist = roundedBoxSDF(aspectCorrected, halfSize, u_cornerRadius);

        // Hollow outline: discard pixels outside the outer edge and inside the inner edge
        if (dist > 0.002) {
            discard;
        }
        if (dist < -u_outlineWidth - 0.002) {
            discard;
        }

        // Smooth anti-aliasing at edges for crisp appearance
        float outerAlpha = 1.0 - smoothstep(-0.002, 0.0, dist);
        float innerAlpha = smoothstep(-u_outlineWidth - 0.002, -u_outlineWidth, dist);
        float alpha = outerAlpha * innerAlpha;

        // Sample texture (black)
        vec4 color = texture2D(u_texture, v_texCoord);
        gl_FragColor = vec4(color.rgb, color.a * alpha);
    }
`;

const QUAD_ATTRIBUTES = { position: 'a_position', texCoord: 'a_texCoord' };
const QUAD_UNIFORMS = {
    projection: 'u_projection',
    view: 'u_view',
    model: 'u_model',
    texture: 'u_texture',
    distance: 'u_distance',
    ipdOffset: 'u_ipdOffset',
    isLeftEye: 'u_isLeftEye',
    cornerRadius: 'u_cornerRadius',
};
const OUTLINE_UNIFORMS = {
    ...QUAD_UNIFORMS,
    outlineWidth: 'u_outlineWidth',
    quadAspect: 'u_quadAspect',
};

// Center outline styling
const OUTLINE_CORNER_RADIUS = 0.08;  // Larger rounded corners
const OUTLINE_LINE_WIDTH = 0.008;    // Line width in normalized space
const OUTLINE_PADDING = 0.01;        // Extra padding to prevent edge clipping
const CENTER_HEIGHT_SCALE = 1.2;     // Center overlay vertical height increased by 20%

// ========================================
// Module State (all bound to the current GL context)
// ========================================

let gl = null;
let quadProg = null;      // { program, locations } - shared with panels
let outlineProg = null;   // { program, locations }

let videoPosBuffer = null;
let centerPosBuffer = null;
let outlinePosBuffer = null;
let texCoordBuffer = null;
let lastSettingsHash = '';
let outlineQuadAspect = 1.0;
const scratchPositions = new Float32Array(8);

let leftTexture = null;
let rightTexture = null;
let leftCenterTexture = null;
let rightCenterTexture = null;
let blackTexture = null;  // 1x1 black texture for drawing outlines

// Canvases for splitting the side-by-side stereo frame into left/right halves
let leftCanvas = null, rightCanvas = null, leftCtx = null, rightCtx = null;

// Last uploaded frame sequence numbers (upload only when a new frame arrived)
let uploadedStereoSeq = -1;
let uploadedCenterSeq = -1;
let hasWideTextures = false;
let hasCenterTextures = false;

// ========================================
// Initialization / Disposal
// ========================================

export function initVideoRenderer(newGl) {
    if (gl === newGl && quadProg) return true;
    if (gl) disposeVideoRenderer();
    gl = newGl;

    const program = createProgram(gl, QUAD_VERTEX_SHADER, QUAD_FRAGMENT_SHADER, 'Video', vrLog);
    if (!program) {
        return false;
    }
    quadProg = { program, locations: getLocations(gl, program, QUAD_ATTRIBUTES, QUAD_UNIFORMS) };

    // Verify all locations are valid
    const invalidLocs = [];
    for (const [name, loc] of Object.entries(quadProg.locations)) {
        if (name in QUAD_ATTRIBUTES ? loc === -1 : loc === null) {
            invalidLocs.push(name);
        }
    }
    vrLog(invalidLocs.length > 0 ? 'Missing: ' + invalidLocs.join(', ') : 'Shaders compiled OK');

    const outline = createProgram(gl, QUAD_VERTEX_SHADER, OUTLINE_FRAGMENT_SHADER, 'Outline', vrLog);
    if (outline) {
        outlineProg = { program: outline, locations: getLocations(gl, outline, QUAD_ATTRIBUTES, OUTLINE_UNIFORMS) };
        vrLog('Outline shader OK');
    }

    // Quad buffers (positions are filled by updatePositionBuffersIfNeeded)
    videoPosBuffer = createArrayBuffer(gl, scratchPositions);
    centerPosBuffer = createArrayBuffer(gl, scratchPositions);
    outlinePosBuffer = createArrayBuffer(gl, scratchPositions);
    texCoordBuffer = createArrayBuffer(gl, QUAD_TEX_COORDS);
    lastSettingsHash = '';
    updatePositionBuffersIfNeeded();

    // Textures for left and right eye views
    leftTexture = createLinearTexture(gl);
    rightTexture = createLinearTexture(gl);
    leftCenterTexture = createLinearTexture(gl);
    rightCenterTexture = createLinearTexture(gl);
    blackTexture = createSolidTexture(gl, [0, 0, 0, 255]);  // Black with full alpha

    // Canvases for extracting left/right from the stereo frame
    leftCanvas = document.getElementById('leftCanvas') || document.createElement('canvas');
    rightCanvas = document.getElementById('rightCanvas') || document.createElement('canvas');
    leftCtx = leftCanvas.getContext('2d', { willReadFrequently: false, desynchronized: true });
    rightCtx = rightCanvas.getContext('2d', { willReadFrequently: false, desynchronized: true });

    uploadedStereoSeq = -1;
    uploadedCenterSeq = -1;
    hasWideTextures = false;
    hasCenterTextures = false;

    vrLog('Textures created');
    return true;
}

// Shared textured-quad program for panels: { program, locations } or null
export function getQuadProgram() {
    return quadProg;
}

// Release every GL object and reset all context-bound state
export function disposeVideoRenderer() {
    deleteGlObjects(gl, {
        buffers: [videoPosBuffer, centerPosBuffer, outlinePosBuffer, texCoordBuffer],
        textures: [leftTexture, rightTexture, leftCenterTexture, rightCenterTexture, blackTexture],
        programs: [quadProg && quadProg.program, outlineProg && outlineProg.program],
    });
    gl = null;
    quadProg = null;
    outlineProg = null;
    videoPosBuffer = centerPosBuffer = outlinePosBuffer = texCoordBuffer = null;
    leftTexture = rightTexture = leftCenterTexture = rightCenterTexture = blackTexture = null;
    lastSettingsHash = '';
    uploadedStereoSeq = -1;
    uploadedCenterSeq = -1;
    hasWideTextures = false;
    hasCenterTextures = false;
}

// ========================================
// Geometry
// ========================================

// Rebuild position buffers only when settings change
function updatePositionBuffersIfNeeded() {
    const cfg = immersive3DConfig;
    const hash = `${cfg.screenScale}_${cfg.verticalOffset}_${cfg.centerCropWidth}_${cfg.centerCropHeight}_${cfg.wideSourceWidth}_${cfg.wideSourceHeight}`;
    if (hash === lastSettingsHash || !videoPosBuffer) return;
    lastSettingsHash = hash;

    // Screen-sized quad positioned in front of viewer
    const aspect = 16 / 9;  // Assume 16:9 aspect ratio per eye
    const height = cfg.screenScale;
    const width = height * aspect;
    const yOffset = cfg.verticalOffset;

    gl.bindBuffer(gl.ARRAY_BUFFER, videoPosBuffer);
    gl.bufferData(gl.ARRAY_BUFFER,
        writeRectPositions(scratchPositions, -width / 2, -height / 2 + yOffset, width, height), gl.STATIC_DRAW);

    // Center overlay size based on crop dimensions
    const coverageX = cfg.centerCropWidth / cfg.wideSourceWidth;   // 320/800 = 0.4
    const coverageY = cfg.centerCropHeight / cfg.wideSourceHeight; // 200/600 = 0.333
    const overlayWidth = width * coverageX;
    const overlayHeight = height * coverageY * CENTER_HEIGHT_SCALE;

    gl.bindBuffer(gl.ARRAY_BUFFER, centerPosBuffer);
    gl.bufferData(gl.ARRAY_BUFFER,
        writeRectPositions(scratchPositions, -overlayWidth / 2, -overlayHeight / 2 + yOffset, overlayWidth, overlayHeight),
        gl.STATIC_DRAW);

    // Outline quad: slightly larger than the overlay to prevent clipping
    const pad = OUTLINE_PADDING;
    gl.bindBuffer(gl.ARRAY_BUFFER, outlinePosBuffer);
    gl.bufferData(gl.ARRAY_BUFFER,
        writeRectPositions(scratchPositions, -(overlayWidth / 2 + pad), -(overlayHeight / 2 + pad) + yOffset,
            overlayWidth + 2 * pad, overlayHeight + 2 * pad),
        gl.STATIC_DRAW);
    outlineQuadAspect = overlayWidth / overlayHeight;
}

// ========================================
// Per-frame Upload (once per frame, not per eye)
// ========================================

/**
 * Upload new video frames to textures. Only uploads when a new frame arrived.
 * @returns {{hasWide: boolean, hasCenter: boolean}}
 */
export function uploadVideoFrames(frameCounter) {
    if (!gl || !quadProg) return { hasWide: false, hasCenter: false };

    updatePositionBuffersIfNeeded();

    // Wide stereo frame: split side-by-side into left/right halves
    const frameBitmap = getCurrentFrameBitmap();
    const stereoSeq = getStereoFrameSeq();
    if (frameBitmap && stereoSeq !== uploadedStereoSeq) {
        const videoWidth = frameBitmap.width;
        const videoHeight = frameBitmap.height;
        const halfWidth = videoWidth / 2;

        if (leftCanvas.width !== halfWidth || leftCanvas.height !== videoHeight) {
            leftCanvas.width = halfWidth;
            leftCanvas.height = videoHeight;
            rightCanvas.width = halfWidth;
            rightCanvas.height = videoHeight;
            vrLog('Canvas: ' + halfWidth + 'x' + videoHeight);
        }

        leftCtx.drawImage(frameBitmap, 0, 0, halfWidth, videoHeight, 0, 0, halfWidth, videoHeight);
        rightCtx.drawImage(frameBitmap, halfWidth, 0, halfWidth, videoHeight, 0, 0, halfWidth, videoHeight);

        gl.bindTexture(gl.TEXTURE_2D, leftTexture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, leftCanvas);
        gl.bindTexture(gl.TEXTURE_2D, rightTexture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, rightCanvas);

        if (uploadedStereoSeq === -1) {
            vrLog('Rendering video: ' + videoWidth + 'x' + videoHeight);
        }
        uploadedStereoSeq = stereoSeq;
        hasWideTextures = true;
    }

    // Center camera textures (higher resolution overlay)
    const leftCenterBitmap = getLeftCenterFrameBitmap();
    const rightCenterBitmap = getRightCenterFrameBitmap();
    const centerSeq = getCenterFrameSeq();
    if (leftCenterBitmap && rightCenterBitmap && centerSeq !== uploadedCenterSeq) {
        gl.bindTexture(gl.TEXTURE_2D, leftCenterTexture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, leftCenterBitmap);
        gl.bindTexture(gl.TEXTURE_2D, rightCenterTexture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, rightCenterBitmap);
        uploadedCenterSeq = centerSeq;
        hasCenterTextures = true;
    }

    checkGlError(gl, 'uploadVideoFrames', vrLog);
    return { hasWide: hasWideTextures, hasCenter: hasCenterTextures };
}

// ========================================
// Drawing (per eye)
// ========================================

function setQuadUniforms(prog, view, viewMatrix, modelMatrix, ipdOffset, cornerRadius) {
    const loc = prog.locations;
    gl.uniformMatrix4fv(loc.projection, false, view.projectionMatrix);
    gl.uniformMatrix4fv(loc.view, false, viewMatrix);
    gl.uniformMatrix4fv(loc.model, false, modelMatrix || IDENTITY_MATRIX);
    gl.uniform1f(loc.distance, immersive3DConfig.screenDistance);
    gl.uniform1f(loc.ipdOffset, ipdOffset);
    gl.uniform1f(loc.isLeftEye, view.eye === 'left' ? 1.0 : 0.0);
    gl.uniform1f(loc.cornerRadius, cornerRadius);
    gl.uniform1i(loc.texture, 0);
}

/**
 * Draw the video screen for one eye: wide view, center overlay, center outline.
 * @param {XRView} view
 * @param {Float32Array} viewMatrix - Inverse of view.transform.matrix
 * @param {Float32Array} headMatrix - Head pose (screen is head-locked)
 * @param {{hasWide: boolean, hasCenter: boolean}} frames - From uploadVideoFrames()
 */
export function drawVideo(view, viewMatrix, headMatrix, frames) {
    if (!gl || !quadProg || !frames.hasWide) return;
    const isLeft = view.eye === 'left';
    const loc = quadProg.locations;

    // Wide view base layer
    gl.useProgram(quadProg.program);
    bindAttribute(gl, loc.position, videoPosBuffer);
    bindAttribute(gl, loc.texCoord, texCoordBuffer);
    setQuadUniforms(quadProg, view, viewMatrix, headMatrix, immersive3DConfig.ipdOffset, immersive3DConfig.cornerRadius);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, isLeft ? leftTexture : rightTexture);

    // Disable blending for video quad to ensure full opacity (no passthrough behind video)
    gl.disable(gl.BLEND);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
    gl.enable(gl.BLEND);

    if (!centerViewState.visible) return;

    // Blend color, preserve destination alpha
    gl.blendFuncSeparate(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA, gl.ZERO, gl.ONE);

    // Center overlay (no rounded corners)
    if (frames.hasCenter) {
        bindAttribute(gl, loc.position, centerPosBuffer);
        bindAttribute(gl, loc.texCoord, texCoordBuffer);
        gl.uniform1f(loc.cornerRadius, 0.0);
        gl.bindTexture(gl.TEXTURE_2D, isLeft ? leftCenterTexture : rightCenterTexture);
        gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
    }

    // Outline box around center view area (without IPD offset)
    if (outlineProg && blackTexture) {
        const oloc = outlineProg.locations;
        gl.useProgram(outlineProg.program);
        bindAttribute(gl, oloc.position, outlinePosBuffer);
        bindAttribute(gl, oloc.texCoord, texCoordBuffer);
        setQuadUniforms(outlineProg, view, viewMatrix, headMatrix, 0.0, OUTLINE_CORNER_RADIUS);
        gl.uniform1f(oloc.outlineWidth, OUTLINE_LINE_WIDTH);
        gl.uniform1f(oloc.quadAspect, outlineQuadAspect);
        gl.bindTexture(gl.TEXTURE_2D, blackTexture);
        gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
    }

    checkGlError(gl, 'drawVideo', vrLog);
}
