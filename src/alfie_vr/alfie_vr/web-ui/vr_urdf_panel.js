// VR Alfie Top View Panel Module
// Renders a Three.js URDF robot viewer from a top-down perspective to an offscreen canvas,
// then displays it as a textured panel in the VR scene.
// This approach avoids WebGL context conflicts between Three.js and custom shaders.

import {
    stereoSettings,
    preAllocatedBuffers,
} from './state.js';

// ========================================
// Panel Configuration
// ========================================

export const URDF_PANEL_CONFIG = {
    width: 0.18,                // Panel width in world units
    height: 0.36,               // Panel height in world units (tall for dual views)
    horizontalOffset: -0.225,    // Small gap from right edge of main screen
    verticalOffset: 0.00,       // Vertical offset from screen center (adjusted for taller panel)
    angle: -15,                 // Yaw rotation in degrees (negative = angled toward center)
    backgroundAlpha: 0.5,       // Background opacity
    canvasWidth: 512,           // Canvas width in pixels
    canvasHeight: 1024,         // Canvas height in pixels (tall for dual views)
    topViewRatio: 0.35,         // Top-down view gets 35% of height
    topViewForwardOffset: 0.15, // Shift top-down view forward (robot appears lower, room for arms)
    isoTargetHeight: 0.45,      // How high up the robot the isometric camera looks (head level)
    renderFps: 15,              // Target FPS for Three.js rendering
    titleFontSize: 16,          // Title font size in pixels
};

// ========================================
// Module State
// ========================================

// Three.js scene for URDF rendering
let urdfScene = null;
let urdfCamera = null;
let urdfRenderer = null;
let urdfRobot = null;
let urdfLinks = {};
let urdfJoints = {};
let urdfMaterialDefs = {};
let urdfLoaded = false;

// Offscreen canvas for rendering
let urdfCanvas = null;
let urdfCtx = null;
let threeCanvas = null;  // Canvas where Three.js renders

// WebGL resources for VR display
let urdfPanelTexture = null;
let urdfPanelPositionBuffer = null;
let urdfPanelTexCoordBuffer = null;
let urdfPanelInitialized = false;
const urdfPanelModelMatrix = new Float32Array(16);

// Animation/camera state
// Top-down view: camera looks straight down at the robot
// Isometric view: classic isometric angle from the side
let cameraRadius = 0.6;     // Distance from target
let cameraTarget = { x: 0, y: 0, z: 0 };  // Look at origin (robot base)
let lastRenderTime = 0;

// Second camera for isometric view
let urdfCameraIso = null;

// Context references (set by main module)
let gl = null;
let vrLogFn = (msg) => console.log('[URDF Panel]', msg);
let getFrameCounterFn = () => 0;
let getShaderProgramFn = () => null;
let getCachedLocationsFn = () => null;
let invertMatrixFn = null;
let viewMatrixBuffer = null;
let getUrdfStringFn = () => null;  // Callback to get URDF string from robot_state

// ========================================
// Initialization
// ========================================

/**
 * Set the GL context and callbacks from main module
 */
export function setUrdfPanelContext(context) {
    gl = context.gl;
    vrLogFn = context.vrLog || vrLogFn;
    getFrameCounterFn = context.getFrameCounter || getFrameCounterFn;
    getShaderProgramFn = context.getShaderProgram || getShaderProgramFn;
    getCachedLocationsFn = context.getCachedLocations || getCachedLocationsFn;
    invertMatrixFn = context.invertMatrix || invertMatrixFn;
    viewMatrixBuffer = context.viewMatrixBuffer || preAllocatedBuffers.viewMatrix;
    getUrdfStringFn = context.getUrdfString || getUrdfStringFn;
}

/**
 * Update GL context (called when XR session starts)
 */
export function updateUrdfPanelGlContext(newGl) {
    gl = newGl;
    urdfPanelInitialized = false;
    urdfPanelTexture = null;
}

/**
 * Initialize the Three.js URDF renderer (offscreen)
 */
function initThreeJsRenderer() {
    if (typeof THREE === 'undefined') {
        vrLogFn('Three.js not available');
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
    
    // Camera 1: Top-down orthographic view (top half of canvas)
    const viewSize = 0.5;  // Half-size of the view in world units
    urdfCamera = new THREE.OrthographicCamera(-viewSize, viewSize, viewSize, -viewSize, 0.01, 10);
    const topViewOffset = URDF_PANEL_CONFIG.topViewForwardOffset;  // Shift view forward
    urdfCamera.position.set(0, 1, topViewOffset);
    urdfCamera.up.set(0, 0, 1);  // Set "up" to +Z so robot faces down on screen
    urdfCamera.lookAt(0, 0, topViewOffset);
    
    // Camera 2: Isometric view (bottom portion of canvas)
    // Classic isometric: 45° azimuth, ~35.264° elevation (arctan(1/√2))
    urdfCameraIso = new THREE.OrthographicCamera(-viewSize, viewSize, viewSize, -viewSize, 0.01, 10);
    const isoDistance = 1.0;
    const isoAzimuth = Math.PI / 4;  // 45 degrees around Y
    const isoElevation = Math.atan(1 / Math.sqrt(2));  // ~35.264 degrees up
    const isoTargetY = URDF_PANEL_CONFIG.isoTargetHeight;  // Look at this height on robot
    urdfCameraIso.position.set(
        isoDistance * Math.cos(isoElevation) * Math.sin(isoAzimuth),
        isoTargetY + isoDistance * Math.sin(isoElevation),
        isoDistance * Math.cos(isoElevation) * Math.cos(isoAzimuth)
    );
    urdfCameraIso.up.set(0, 1, 0);
    urdfCameraIso.lookAt(0, isoTargetY, 0);
    
    // Add lighting - works for both views
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    urdfScene.add(ambientLight);
    
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(1, 2, 1);
    urdfScene.add(directionalLight);
    
    const hemiLight = new THREE.HemisphereLight(0xffffff, 0x444444, 0.3);
    hemiLight.position.set(0, 10, 0);
    urdfScene.add(hemiLight);
    
    vrLogFn('Three.js dual-view URDF renderer initialized');
    return true;
}

/**
 * Update camera positions (called if target changes)
 */
function updateCameraPositions() {
    if (!urdfCamera || !urdfCameraIso) return;
    
    // Top-down camera stays fixed above robot (with forward offset for arm room)
    const topViewOffset = URDF_PANEL_CONFIG.topViewForwardOffset;
    urdfCamera.position.set(cameraTarget.x, cameraTarget.y + 1, cameraTarget.z + topViewOffset);
    urdfCamera.lookAt(cameraTarget.x, cameraTarget.y, cameraTarget.z + topViewOffset);
    
    // Isometric camera maintains angle but follows target (raised by isoTargetHeight)
    const isoDistance = 1.0;
    const isoAzimuth = Math.PI / 4;
    const isoElevation = Math.atan(1 / Math.sqrt(2));
    const isoTargetY = cameraTarget.y + URDF_PANEL_CONFIG.isoTargetHeight;
    urdfCameraIso.position.set(
        cameraTarget.x + isoDistance * Math.cos(isoElevation) * Math.sin(isoAzimuth),
        isoTargetY + isoDistance * Math.sin(isoElevation),
        cameraTarget.z + isoDistance * Math.cos(isoElevation) * Math.cos(isoAzimuth)
    );
    urdfCameraIso.lookAt(cameraTarget.x, isoTargetY, cameraTarget.z);
}

/**
 * Initialize the VR panel for displaying the URDF render
 */
function initUrdfPanel() {
    if (!gl) {
        vrLogFn('URDF panel: no GL context');
        return;
    }
    
    vrLogFn('Init URDF panel...');
    
    // Initialize Three.js renderer first
    if (!urdfRenderer) {
        if (!initThreeJsRenderer()) {
            return;
        }
    }
    
    // Create composite canvas for panel display
    const canvasWidth = URDF_PANEL_CONFIG.canvasWidth;
    const canvasHeight = URDF_PANEL_CONFIG.canvasHeight;
    urdfCanvas = document.createElement('canvas');
    urdfCanvas.width = canvasWidth;
    urdfCanvas.height = canvasHeight;
    urdfCtx = urdfCanvas.getContext('2d');
    
    // Create WebGL texture
    urdfPanelTexture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, urdfPanelTexture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    
    // Create position buffer for 3D quad (right side of main screen)
    const mainHeight = stereoSettings.screenScale;
    const mainWidth = mainHeight * (16/9);
    const mainRightEdge = mainWidth / 2;
    
    const panelHeight = URDF_PANEL_CONFIG.height;
    const panelWidth = URDF_PANEL_CONFIG.width;
    
    const panelLeft = mainRightEdge + URDF_PANEL_CONFIG.horizontalOffset - panelWidth;
    const panelCenterY = stereoSettings.verticalOffset + URDF_PANEL_CONFIG.verticalOffset;
    
    urdfPanelPositionBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, urdfPanelPositionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
        panelLeft, panelCenterY - panelHeight/2,
        panelLeft + panelWidth, panelCenterY - panelHeight/2,
        panelLeft, panelCenterY + panelHeight/2,
        panelLeft + panelWidth, panelCenterY + panelHeight/2,
    ]), gl.STATIC_DRAW);
    
    vrLogFn(`URDF panel pos: L=${panelLeft.toFixed(2)} Y=${panelCenterY.toFixed(2)}`);
    
    urdfPanelTexCoordBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, urdfPanelTexCoordBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
        0, 1,
        1, 1,
        0, 0,
        1, 0,
    ]), gl.STATIC_DRAW);
    
    // Do an initial render to populate the canvas
    renderThreeJsToCanvas();
    
    // Upload initial texture
    if (urdfCanvas) {
        gl.bindTexture(gl.TEXTURE_2D, urdfPanelTexture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, urdfCanvas);
    }
    
    urdfPanelInitialized = true;
    vrLogFn('URDF panel: OK');
}

// ========================================
// URDF Loading
// ========================================

/**
 * Load URDF string into the Three.js scene
 * @param {string} urdfString - The URDF XML string
 */
export function loadUrdfForPanel(urdfString) {
    if (!urdfString || urdfLoaded) return;
    
    if (!urdfRenderer && !initThreeJsRenderer()) {
        return;
    }
    
    try {
        const parser = new DOMParser();
        const urdfDoc = parser.parseFromString(urdfString, 'text/xml');
        const robotEl = urdfDoc.querySelector('robot');
        
        if (!robotEl) {
            vrLogFn('No robot element in URDF');
            return;
        }
        
        const robotName = robotEl.getAttribute('name') || 'robot';
        vrLogFn(`Loading URDF: ${robotName}`);
        
        // Clear existing robot
        if (urdfRobot) {
            urdfScene.remove(urdfRobot);
        }
        
        urdfRobot = new THREE.Group();
        urdfRobot.name = robotName;
        urdfLinks = {};
        urdfJoints = {};
        
        // Parse material definitions
        urdfMaterialDefs = {};
        urdfDoc.querySelectorAll('robot > material').forEach(matEl => {
            const matName = matEl.getAttribute('name');
            const colorEl = matEl.querySelector('color');
            if (matName && colorEl) {
                const rgba = colorEl.getAttribute('rgba').split(' ').map(parseFloat);
                urdfMaterialDefs[matName] = new THREE.Color(rgba[0], rgba[1], rgba[2]);
            }
        });
        
        // Parse links
        const linkElements = urdfDoc.querySelectorAll('link');
        const childLinks = new Set();
        
        linkElements.forEach(linkEl => {
            const linkName = linkEl.getAttribute('name');
            const visuals = linkEl.querySelectorAll('visual');
            
            const linkGroup = new THREE.Group();
            linkGroup.name = linkName;
            
            visuals.forEach(visual => {
                const geometry = visual.querySelector('geometry');
                const origin = visual.querySelector('origin');
                const material = visual.querySelector('material');
                
                if (geometry) {
                    const mesh = createGeometry(geometry, origin, material);
                    if (mesh) {
                        linkGroup.add(mesh);
                    }
                }
            });
            
            urdfLinks[linkName] = linkGroup;
        });
        
        // Parse joints to build hierarchy
        const jointElements = urdfDoc.querySelectorAll('joint');
        
        jointElements.forEach(jointEl => {
            const jointName = jointEl.getAttribute('name');
            const jointType = jointEl.getAttribute('type');
            const parent = jointEl.querySelector('parent')?.getAttribute('link');
            const child = jointEl.querySelector('child')?.getAttribute('link');
            const origin = jointEl.querySelector('origin');
            const axis = jointEl.querySelector('axis');
            
            urdfJoints[jointName] = {
                type: jointType,
                parent: parent,
                child: child,
                axis: axis ? axis.getAttribute('xyz').split(' ').map(parseFloat) : [0, 0, 1]
            };
            
            if (parent && child && urdfLinks[parent] && urdfLinks[child]) {
                childLinks.add(child);
                
                if (origin) {
                    const xyz = origin.getAttribute('xyz');
                    const rpy = origin.getAttribute('rpy');
                    
                    if (xyz) {
                        const pos = xyz.split(' ').map(parseFloat);
                        urdfLinks[child].position.set(pos[0], pos[1], pos[2]);
                    }
                    if (rpy) {
                        const rot = rpy.split(' ').map(parseFloat);
                        urdfLinks[child].rotation.order = 'ZYX';
                        urdfLinks[child].rotation.set(rot[0], rot[1], rot[2]);
                    }
                }
                
                urdfLinks[parent].add(urdfLinks[child]);
            }
        });
        
        // Add root links to robot
        Object.keys(urdfLinks).forEach(linkName => {
            if (!childLinks.has(linkName)) {
                urdfRobot.add(urdfLinks[linkName]);
            }
        });
        
        // Rotate for ROS -> Three.js coordinate conversion (Z-up to Y-up)
        urdfRobot.rotation.x = -Math.PI / 2;
        
        // Position at origin
        urdfRobot.position.set(0, 0, 0);
        
        urdfScene.add(urdfRobot);
        urdfLoaded = true;
        
        vrLogFn(`URDF loaded: ${Object.keys(urdfLinks).length} links`);
        
    } catch (error) {
        vrLogFn('Error loading URDF: ' + error.message);
        console.error('URDF load error:', error);
    }
}

/**
 * Create geometry from URDF visual element
 */
function createGeometry(geometryEl, origin, material) {
    const box = geometryEl.querySelector('box');
    const cylinder = geometryEl.querySelector('cylinder');
    const sphere = geometryEl.querySelector('sphere');
    const meshEl = geometryEl.querySelector('mesh');
    
    let geom = null;
    let mesh = null;
    
    if (box) {
        const size = box.getAttribute('size').split(' ').map(parseFloat);
        geom = new THREE.BoxGeometry(size[0], size[1], size[2]);
    } else if (cylinder) {
        const radius = parseFloat(cylinder.getAttribute('radius'));
        const length = parseFloat(cylinder.getAttribute('length'));
        geom = new THREE.CylinderGeometry(radius, radius, length, 16);
    } else if (sphere) {
        const radius = parseFloat(sphere.getAttribute('radius'));
        geom = new THREE.SphereGeometry(radius, 16, 16);
    } else if (meshEl) {
        const filename = meshEl.getAttribute('filename');
        const scaleAttr = meshEl.getAttribute('scale');
        
        if (filename && filename.startsWith('package://alfie_urdf/meshes/') && filename.endsWith('.obj')) {
            const meshName = filename.replace('package://alfie_urdf/meshes/', '');
            const meshUrl = `/meshes/${meshName}`;
            
            const placeholder = new THREE.Group();
            
            if (typeof THREE.OBJLoader !== 'undefined') {
                const loader = new THREE.OBJLoader();
                loader.load(
                    meshUrl,
                    (obj) => {
                        const matColor = getMaterialColor(material);
                        obj.traverse((child) => {
                            if (child.isMesh) {
                                child.material = new THREE.MeshStandardMaterial({
                                    color: matColor,
                                    metalness: 0.3,
                                    roughness: 0.7
                                });
                            }
                        });
                        
                        if (scaleAttr) {
                            const scaleVals = scaleAttr.split(' ').map(parseFloat);
                            obj.scale.set(scaleVals[0], scaleVals[1], scaleVals[2]);
                        }
                        
                        applyOrigin(obj, origin);
                        placeholder.add(obj);
                    },
                    undefined,
                    (error) => console.warn('Failed to load mesh:', meshUrl)
                );
            }
            
            return placeholder;
        } else {
            geom = new THREE.BoxGeometry(0.03, 0.03, 0.03);
        }
    }
    
    if (geom) {
        const color = getMaterialColor(material);
        const mat = new THREE.MeshStandardMaterial({
            color: color,
            metalness: 0.3,
            roughness: 0.7
        });
        
        mesh = new THREE.Mesh(geom, mat);
        applyOrigin(mesh, origin);
    }
    
    return mesh;
}

/**
 * Get material color from URDF material element
 */
function getMaterialColor(materialEl) {
    if (!materialEl) return 0x666666;
    
    const colorEl = materialEl.querySelector('color');
    if (colorEl) {
        const rgba = colorEl.getAttribute('rgba').split(' ').map(parseFloat);
        return new THREE.Color(rgba[0], rgba[1], rgba[2]);
    }
    
    const matName = materialEl.getAttribute('name');
    if (matName && urdfMaterialDefs[matName]) {
        return urdfMaterialDefs[matName];
    }
    
    return 0x666666;
}

/**
 * Apply origin transform to object
 */
function applyOrigin(object, origin) {
    if (!origin) return;
    
    const xyz = origin.getAttribute('xyz');
    const rpy = origin.getAttribute('rpy');
    
    if (xyz) {
        const pos = xyz.split(' ').map(parseFloat);
        object.position.set(pos[0], pos[1], pos[2]);
    }
    if (rpy) {
        const rot = rpy.split(' ').map(parseFloat);
        object.rotation.order = 'ZYX';
        object.rotation.set(rot[0], rot[1], rot[2]);
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
export function applyUrdfPanelTransform(childFrameId, transform) {
    if (!urdfLinks[childFrameId] || !transform) return;
    
    const link = urdfLinks[childFrameId];
    
    // Apply position
    if (transform.position) {
        link.position.set(transform.position.x, transform.position.y, transform.position.z);
    }
    
    // Apply rotation (quaternion)
    if (transform.quaternion) {
        link.quaternion.set(
            transform.quaternion.x,
            transform.quaternion.y,
            transform.quaternion.z,
            transform.quaternion.w
        );
    }
}

// ========================================
// Rendering
// ========================================

/**
 * Render the Three.js scene to the offscreen canvas (dual views)
 */
function renderThreeJsToCanvas() {
    if (!urdfCanvas || !urdfCtx) return;
    
    const now = performance.now();
    const frameInterval = 1000 / URDF_PANEL_CONFIG.renderFps;
    
    if (now - lastRenderTime < frameInterval) {
        return;  // Skip frame to maintain target FPS
    }
    lastRenderTime = now;
    
    const canvasWidth = URDF_PANEL_CONFIG.canvasWidth;
    const canvasHeight = URDF_PANEL_CONFIG.canvasHeight;
    const topViewHeight = Math.floor(canvasHeight * URDF_PANEL_CONFIG.topViewRatio);
    const isoViewHeight = canvasHeight - topViewHeight;
    
    // Clear canvas with transparency
    urdfCtx.clearRect(0, 0, canvasWidth, canvasHeight);
    
    // If Three.js is ready, render both views
    if (urdfRenderer && urdfScene && urdfCamera && urdfCameraIso) {
        // Top portion: Top-down view (35% of height)
        urdfRenderer.setViewport(0, isoViewHeight, canvasWidth, topViewHeight);
        urdfRenderer.setScissor(0, isoViewHeight, canvasWidth, topViewHeight);
        urdfRenderer.clear();  // Clear this viewport region
        urdfRenderer.render(urdfScene, urdfCamera);
        
        // Bottom portion: Isometric view (65% of height)
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
    
    // Draw title bar for isometric view (at split point)
    urdfCtx.fillStyle = 'rgba(20, 20, 40, 0.85)';
    urdfCtx.fillRect(0, topViewHeight, canvasWidth, titleHeight);
    urdfCtx.fillStyle = '#88ccff';
    urdfCtx.fillText('📐 Isometric', 10, topViewHeight + titleHeight / 2);
    
    // Draw status indicator on top title bar
    const statusColor = urdfLoaded ? '#44ff44' : '#ff8844';
    const statusText = urdfLoaded ? `${Object.keys(urdfLinks).length} links` : 'Loading...';
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
}

/**
 * Helper function to create rotated model matrix
 */
function createRotatedModelMatrix(baseMatrix, yawDegrees, outMatrix) {
    const yawRad = yawDegrees * Math.PI / 180;
    const cosY = Math.cos(yawRad);
    const sinY = Math.sin(yawRad);
    
    const m = baseMatrix;
    
    outMatrix[0] = m[0] * cosY + m[8] * (-sinY);
    outMatrix[1] = m[1] * cosY + m[9] * (-sinY);
    outMatrix[2] = m[2] * cosY + m[10] * (-sinY);
    outMatrix[3] = m[3] * cosY + m[11] * (-sinY);
    
    outMatrix[4] = m[4];
    outMatrix[5] = m[5];
    outMatrix[6] = m[6];
    outMatrix[7] = m[7];
    
    outMatrix[8] = m[0] * sinY + m[8] * cosY;
    outMatrix[9] = m[1] * sinY + m[9] * cosY;
    outMatrix[10] = m[2] * sinY + m[10] * cosY;
    outMatrix[11] = m[3] * sinY + m[11] * cosY;
    
    outMatrix[12] = m[12];
    outMatrix[13] = m[13];
    outMatrix[14] = m[14];
    outMatrix[15] = m[15];
}

/**
 * Draw the URDF panel in VR
 * @param {XRView} view - The XR view
 * @param {Object} viewport - The viewport dimensions
 * @param {Float32Array} modelMatrix - The head pose model matrix
 */
export function drawUrdfPanel(view, viewport, modelMatrix) {
    const frameCounter = getFrameCounterFn();
    const shaderProgram = getShaderProgramFn();
    const cachedLocations = getCachedLocationsFn();
    
    // Initialize on first call
    if (!urdfPanelInitialized) {
        initUrdfPanel();
    }
    
    if (!urdfPanelInitialized || !shaderProgram || !urdfPanelTexture || !cachedLocations) {
        if (frameCounter < 20 && frameCounter % 5 === 0) {
            vrLogFn('URDF panel not ready');
        }
        return;
    }
    
    // Try to load URDF if not yet loaded (check every 60 frames ~1 second)
    if (!urdfLoaded && frameCounter % 60 === 0) {
        const urdfStr = getUrdfStringFn();
        if (urdfStr) {
            loadUrdfForPanel(urdfStr);
        }
    }
    
    // Render Three.js to canvas
    renderThreeJsToCanvas();
    
    // Update texture
    if (urdfCanvas) {
        gl.bindTexture(gl.TEXTURE_2D, urdfPanelTexture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, urdfCanvas);
    }
    
    // Save current WebGL state
    const prevProgram = gl.getParameter(gl.CURRENT_PROGRAM);
    
    // Enable blending for transparency
    gl.enable(gl.BLEND);
    gl.blendFuncSeparate(
        gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA,
        gl.ZERO, gl.ONE
    );
    
    // Disable depth test so panel is always visible
    gl.disable(gl.DEPTH_TEST);
    
    gl.useProgram(shaderProgram);
    
    // Set up position buffer
    if (cachedLocations.position !== -1 && cachedLocations.position !== null) {
        gl.bindBuffer(gl.ARRAY_BUFFER, urdfPanelPositionBuffer);
        gl.enableVertexAttribArray(cachedLocations.position);
        gl.vertexAttribPointer(cachedLocations.position, 2, gl.FLOAT, false, 0, 0);
    }
    
    // Set up tex coord buffer
    if (cachedLocations.texCoord !== -1 && cachedLocations.texCoord !== null) {
        gl.bindBuffer(gl.ARRAY_BUFFER, urdfPanelTexCoordBuffer);
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
            createRotatedModelMatrix(modelMatrix, URDF_PANEL_CONFIG.angle, urdfPanelModelMatrix);
            gl.uniformMatrix4fv(cachedLocations.model, false, urdfPanelModelMatrix);
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
    
    if (cachedLocations.cornerRadius !== null) {
        gl.uniform1f(cachedLocations.cornerRadius, 0.08);  // More rounded corners
    }
    
    if (cachedLocations.texture !== null) {
        gl.activeTexture(gl.TEXTURE0);
        gl.bindTexture(gl.TEXTURE_2D, urdfPanelTexture);
        gl.uniform1i(cachedLocations.texture, 0);
    }
    
    // Draw the panel
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
    
    if (frameCounter === 20) {
        vrLogFn('URDF panel drawing');
    }
    
    // Restore state
    gl.disable(gl.BLEND);
    gl.enable(gl.DEPTH_TEST);
    
    if (prevProgram) {
        gl.useProgram(prevProgram);
    }
}

// ========================================
// State Getters
// ========================================

/**
 * Check if the URDF is loaded
 */
export function isUrdfPanelLoaded() {
    return urdfLoaded;
}

/**
 * Get link names
 */
export function getUrdfPanelLinkNames() {
    return Object.keys(urdfLinks);
}

// ========================================
// Cleanup
// ========================================

/**
 * Dispose of URDF panel resources
 */
export function disposeUrdfPanel() {
    if (urdfRenderer) {
        urdfRenderer.dispose();
        urdfRenderer = null;
    }
    
    if (urdfScene) {
        urdfScene.traverse((object) => {
            if (object.geometry) {
                object.geometry.dispose();
            }
            if (object.material) {
                if (Array.isArray(object.material)) {
                    object.material.forEach(m => m.dispose());
                } else {
                    object.material.dispose();
                }
            }
        });
        urdfScene = null;
    }
    
    urdfCameraIso = null;
    urdfRobot = null;
    urdfLinks = {};
    urdfJoints = {};
    urdfLoaded = false;
    urdfPanelInitialized = false;
    
    if (urdfPanelTexture && gl) {
        gl.deleteTexture(urdfPanelTexture);
        urdfPanelTexture = null;
    }
}
