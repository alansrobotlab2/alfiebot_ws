// Textured Panel Module
// Shared helper for head-locked VR panels: a 2D canvas uploaded to a texture and
// drawn as a quad with the video shader program (see video_renderer.js).
// GL resources are bound to one context; dispose() on session end / context loss.

import { stereoSettings } from './state.js';
import {
    IDENTITY_MATRIX,
    QUAD_TEX_COORDS,
    createRotatedModelMatrix,
    writeRectPositions,
    createArrayBuffer,
    createLinearTexture,
    bindAttribute,
    deleteGlObjects,
} from './gl_utils.js';

// Helper function to draw a filled rounded rectangle on a 2D canvas
export function drawRoundedRect(ctx, x, y, width, height, radius) {
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

export class TexturedPanel {
    /**
     * @param {Object} opts
     * @param {string} opts.name - Label for logs
     * @param {number} opts.canvasWidth - Canvas pixel width
     * @param {number} opts.canvasHeight - Canvas pixel height
     * @param {Function} opts.computeRect - () => {left, bottom, width, height} in panel space
     *        (evaluated when GL resources are created)
     * @param {number} [opts.angle=0] - Yaw rotation in degrees applied to the head matrix
     * @param {number} [opts.cornerRadius=0] - Shader corner radius (normalized tex space)
     * @param {number} [opts.ipdScale=0] - Fraction of stereoSettings.ipdOffset applied
     */
    constructor({ name, canvasWidth, canvasHeight, computeRect, angle = 0, cornerRadius = 0, ipdScale = 0 }) {
        this.name = name;
        this.computeRect = computeRect;
        this.angle = angle;
        this.cornerRadius = cornerRadius;
        this.ipdScale = ipdScale;

        // Canvas is context-independent and survives sessions
        this.canvas = document.createElement('canvas');
        this.canvas.width = canvasWidth;
        this.canvas.height = canvasHeight;
        this.ctx = this.canvas.getContext('2d');

        // GL resources (per context)
        this.gl = null;
        this.texture = null;
        this.positionBuffer = null;
        this.texCoordBuffer = null;
        this.needsUpload = true;
        this.modelMatrix = new Float32Array(16);
    }

    // Create GL resources for this context. Returns true if they were (re)created,
    // meaning the caller should repaint the canvas.
    init(gl) {
        if (this.gl === gl && this.texture) return false;
        if (this.gl && this.gl !== gl) this.dispose();
        if (!gl) return false;

        this.gl = gl;
        this.texture = createLinearTexture(gl);
        const rect = this.computeRect();
        this.positionBuffer = createArrayBuffer(gl,
            writeRectPositions(new Float32Array(8), rect.left, rect.bottom, rect.width, rect.height));
        this.texCoordBuffer = createArrayBuffer(gl, QUAD_TEX_COORDS);
        this.needsUpload = true;
        return true;
    }

    isReady() {
        return !!(this.gl && this.texture);
    }

    // Canvas content changed - upload on next uploadIfDirty()
    markDirty() {
        this.needsUpload = true;
    }

    // Upload the canvas to the texture at most once per call (call once per frame)
    uploadIfDirty() {
        if (!this.needsUpload || !this.isReady()) return;
        const gl = this.gl;
        gl.bindTexture(gl.TEXTURE_2D, this.texture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, this.canvas);
        this.needsUpload = false;
    }

    /**
     * Draw the panel for one eye.
     * @param {XRView} view - The XR view
     * @param {Float32Array} viewMatrix - Inverse of view.transform.matrix (computed once per eye)
     * @param {Float32Array} headMatrix - Head pose matrix (panels are head-locked)
     * @param {{program: WebGLProgram, locations: Object}} prog - Video quad shader program
     */
    draw(view, viewMatrix, headMatrix, prog) {
        if (!this.isReady() || !prog) return;
        const gl = this.gl;
        const loc = prog.locations;

        // Blend color but preserve destination alpha (prevents AR passthrough showing through)
        gl.enable(gl.BLEND);
        gl.blendFuncSeparate(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA, gl.ZERO, gl.ONE);
        // Disable depth test so panel is always visible on top
        gl.disable(gl.DEPTH_TEST);

        gl.useProgram(prog.program);
        bindAttribute(gl, loc.position, this.positionBuffer);
        bindAttribute(gl, loc.texCoord, this.texCoordBuffer);

        let model = IDENTITY_MATRIX;
        if (headMatrix) {
            if (this.angle !== 0) {
                createRotatedModelMatrix(headMatrix, this.angle, this.modelMatrix);
                model = this.modelMatrix;
            } else {
                model = headMatrix;
            }
        }

        gl.uniformMatrix4fv(loc.projection, false, view.projectionMatrix);
        gl.uniformMatrix4fv(loc.view, false, viewMatrix);
        gl.uniformMatrix4fv(loc.model, false, model);
        gl.uniform1f(loc.distance, stereoSettings.screenDistance);
        gl.uniform1f(loc.ipdOffset, stereoSettings.ipdOffset * this.ipdScale);
        gl.uniform1f(loc.isLeftEye, view.eye === 'left' ? 1.0 : 0.0);
        gl.uniform1f(loc.cornerRadius, this.cornerRadius);

        gl.activeTexture(gl.TEXTURE0);
        gl.bindTexture(gl.TEXTURE_2D, this.texture);
        gl.uniform1i(loc.texture, 0);

        gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);

        // Restore state
        gl.disable(gl.BLEND);
        gl.enable(gl.DEPTH_TEST);
    }

    // Release GL resources (canvas is kept for the next session)
    dispose() {
        deleteGlObjects(this.gl, {
            buffers: [this.positionBuffer, this.texCoordBuffer],
            textures: [this.texture],
        });
        this.gl = null;
        this.texture = null;
        this.positionBuffer = null;
        this.texCoordBuffer = null;
        this.needsUpload = true;
    }
}
