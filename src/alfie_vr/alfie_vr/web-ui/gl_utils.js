// GL Utilities Module
// Matrix math, buffer/texture/shader helpers shared by the video renderer and VR panels

// Identity model matrix (never changes)
export const IDENTITY_MATRIX = new Float32Array([
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
]);

// Texture coordinates for a TRIANGLE_STRIP quad (bottom-left, bottom-right, top-left, top-right)
export const QUAD_TEX_COORDS = new Float32Array([
    0, 1,
    1, 1,
    0, 0,
    1, 0,
]);

// Set to true to check gl.getError() after draws (slow - stalls the GPU pipeline)
export const DEBUG_GL_ERRORS = false;

// ========================================
// Matrix Helpers
// ========================================

// 4x4 matrix inversion (column-major)
export function invertMatrix(m, out) {
    const m00 = m[0], m01 = m[1], m02 = m[2], m03 = m[3];
    const m10 = m[4], m11 = m[5], m12 = m[6], m13 = m[7];
    const m20 = m[8], m21 = m[9], m22 = m[10], m23 = m[11];
    const m30 = m[12], m31 = m[13], m32 = m[14], m33 = m[15];

    const tmp0 = m22 * m33 - m32 * m23;
    const tmp1 = m21 * m33 - m31 * m23;
    const tmp2 = m21 * m32 - m31 * m22;
    const tmp3 = m20 * m33 - m30 * m23;
    const tmp4 = m20 * m32 - m30 * m22;
    const tmp5 = m20 * m31 - m30 * m21;

    const t0 = m11 * tmp0 - m12 * tmp1 + m13 * tmp2;
    const t1 = -(m10 * tmp0 - m12 * tmp3 + m13 * tmp4);
    const t2 = m10 * tmp1 - m11 * tmp3 + m13 * tmp5;
    const t3 = -(m10 * tmp2 - m11 * tmp4 + m12 * tmp5);

    const det = 1.0 / (m00 * t0 + m01 * t1 + m02 * t2 + m03 * t3);

    out[0] = t0 * det;
    out[1] = (-(m01 * tmp0 - m02 * tmp1 + m03 * tmp2)) * det;
    out[2] = (m01 * (m12 * m33 - m32 * m13) - m02 * (m11 * m33 - m31 * m13) + m03 * (m11 * m32 - m31 * m12)) * det;
    out[3] = (-(m01 * (m12 * m23 - m22 * m13) - m02 * (m11 * m23 - m21 * m13) + m03 * (m11 * m22 - m21 * m12))) * det;
    out[4] = t1 * det;
    out[5] = (m00 * tmp0 - m02 * tmp3 + m03 * tmp4) * det;
    out[6] = (-(m00 * (m12 * m33 - m32 * m13) - m02 * (m10 * m33 - m30 * m13) + m03 * (m10 * m32 - m30 * m12))) * det;
    out[7] = (m00 * (m12 * m23 - m22 * m13) - m02 * (m10 * m23 - m20 * m13) + m03 * (m10 * m22 - m20 * m12)) * det;
    out[8] = t2 * det;
    out[9] = (-(m00 * tmp1 - m01 * tmp3 + m03 * tmp5)) * det;
    out[10] = (m00 * (m11 * m33 - m31 * m13) - m01 * (m10 * m33 - m30 * m13) + m03 * (m10 * m31 - m30 * m11)) * det;
    out[11] = (-(m00 * (m11 * m23 - m21 * m13) - m01 * (m10 * m23 - m20 * m13) + m03 * (m10 * m21 - m20 * m11))) * det;
    out[12] = t3 * det;
    out[13] = (m00 * tmp2 - m01 * tmp4 + m02 * tmp5) * det;
    out[14] = (-(m00 * (m11 * m32 - m31 * m12) - m01 * (m10 * m32 - m30 * m12) + m02 * (m10 * m31 - m30 * m11))) * det;
    out[15] = (m00 * (m11 * m22 - m21 * m12) - m01 * (m10 * m22 - m20 * m12) + m02 * (m10 * m21 - m20 * m11)) * det;
}

// Create a rotated model matrix: baseMatrix * RotY(yawDegrees)
// WebGL uses column-major 4x4 matrices
export function createRotatedModelMatrix(baseMatrix, yawDegrees, outMatrix) {
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
// Buffer / Texture Helpers
// ========================================

// Write a TRIANGLE_STRIP rectangle into a Float32Array(8)
export function writeRectPositions(out, left, bottom, width, height) {
    out[0] = left;          out[1] = bottom;
    out[2] = left + width;  out[3] = bottom;
    out[4] = left;          out[5] = bottom + height;
    out[6] = left + width;  out[7] = bottom + height;
    return out;
}

// Create a vertex buffer and upload data (STATIC_DRAW)
export function createArrayBuffer(gl, data) {
    const buffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
    return buffer;
}

// Create a texture with linear filtering and clamp-to-edge wrapping
export function createLinearTexture(gl) {
    const texture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    return texture;
}

// Create a 1x1 solid-color texture
export function createSolidTexture(gl, rgba) {
    const texture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, 1, 1, 0, gl.RGBA, gl.UNSIGNED_BYTE, new Uint8Array(rgba));
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    return texture;
}

// Bind a 2-component float attribute to a buffer (skips missing attributes)
export function bindAttribute(gl, location, buffer) {
    if (location === -1 || location === null || location === undefined) return;
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
    gl.enableVertexAttribArray(location);
    gl.vertexAttribPointer(location, 2, gl.FLOAT, false, 0, 0);
}

// Delete GL objects, ignoring nulls and errors from an already-lost context
export function deleteGlObjects(gl, { buffers = [], textures = [], programs = [] } = {}) {
    if (!gl) return;
    try {
        for (const b of buffers) if (b) gl.deleteBuffer(b);
        for (const t of textures) if (t) gl.deleteTexture(t);
        for (const p of programs) if (p) gl.deleteProgram(p);
    } catch (e) {
        console.warn('GL cleanup error (context lost?):', e);
    }
}

// ========================================
// Shader Helpers
// ========================================

function compileShader(gl, type, source, label, log) {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
        const err = gl.getShaderInfoLog(shader) || '';
        const kind = type === gl.VERTEX_SHADER ? 'VTX' : 'FRAG';
        console.error(`${label} ${kind} shader compilation error:`, err);
        log(`${label} ${kind} ERR: ` + err.substring(0, 50));
        gl.deleteShader(shader);
        return null;
    }
    return shader;
}

// Compile + link a program; returns null (and logs) on failure
export function createProgram(gl, vertexSource, fragmentSource, label, log = console.log) {
    const vs = compileShader(gl, gl.VERTEX_SHADER, vertexSource, label, log);
    const fs = vs && compileShader(gl, gl.FRAGMENT_SHADER, fragmentSource, label, log);
    if (!vs || !fs) {
        if (vs) gl.deleteShader(vs);
        return null;
    }

    const program = gl.createProgram();
    gl.attachShader(program, vs);
    gl.attachShader(program, fs);
    gl.linkProgram(program);
    // Shaders can be flagged for deletion once linked
    gl.deleteShader(vs);
    gl.deleteShader(fs);

    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
        const err = gl.getProgramInfoLog(program) || '';
        console.error(`${label} shader program linking error:`, err);
        log(`${label} LINK ERR: ` + err.substring(0, 50));
        gl.deleteProgram(program);
        return null;
    }
    return program;
}

// Look up attribute and uniform locations: names map key -> GLSL name
export function getLocations(gl, program, attributes, uniforms) {
    const locations = {};
    for (const [key, name] of Object.entries(attributes)) {
        locations[key] = gl.getAttribLocation(program, name);
    }
    for (const [key, name] of Object.entries(uniforms)) {
        locations[key] = gl.getUniformLocation(program, name);
    }
    return locations;
}

// Log any GL error (only when DEBUG_GL_ERRORS is enabled)
export function checkGlError(gl, where, log = console.error) {
    if (!DEBUG_GL_ERRORS) return false;
    const err = gl.getError();
    if (err !== gl.NO_ERROR) {
        log(`GL ERROR in ${where}: ${err}`);
        return true;
    }
    return false;
}
