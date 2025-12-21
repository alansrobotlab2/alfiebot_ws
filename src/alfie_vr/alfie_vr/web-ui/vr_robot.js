// VR Robot Module - Renders URDF robot model in WebXR using Three.js
// Integrates with existing WebXR session from stereo_vr.js

// ========================================
// Module State
// ========================================

// Three.js scene objects
let vrScene = null;
let vrRobot = null;
let vrLinks = {};
let vrJoints = {};
let vrMaterialDefs = {};
let vrURDFLoaded = false;

// Renderer (shared with XR session)
let vrRenderer = null;
let vrCamera = null;

// Lighting
let ambientLight = null;
let directionalLight = null;

// Robot position in world space (meters from origin)
// WebXR uses right-handed coordinate system: +X right, +Y up, +Z towards user
const ROBOT_POSITION = { x: 0, y: 0.5, z: -1.5 };  // 1.5m in front, 0.5m up (table height)
const ROBOT_SCALE = 1.0;  // Full scale

// Debug flag
let vrRobotDebugLogged = false;

// TF transform storage for applying updates
let tfTransforms = {};

// ========================================
// Initialization
// ========================================

/**
 * Initialize the VR robot renderer
 * @param {WebGLRenderingContext} gl - The WebGL context from the XR session
 * @param {XRSession} xrSession - The active XR session
 * @returns {boolean} - True if initialization succeeded
 */
export function initVRRobot(gl, xrSession) {
    if (typeof THREE === 'undefined') {
        console.warn('VR Robot: Three.js not loaded');
        return false;
    }
    
    console.log('Initializing VR Robot renderer...');
    
    try {
        // Create Three.js scene
        vrScene = new THREE.Scene();
        
        // Create a Three.js renderer using the existing WebGL context
        // Note: We're sharing the WebGL context, so we need to be careful
        vrRenderer = new THREE.WebGLRenderer({
            canvas: gl.canvas,
            context: gl,
            antialias: false,
            alpha: true,
            preserveDrawingBuffer: false,
        });
        
        // IMPORTANT: Don't let Three.js manage the animation loop or clear
        // We control rendering manually per-view
        vrRenderer.autoClear = false;
        vrRenderer.autoClearColor = false;
        vrRenderer.autoClearDepth = false;
        vrRenderer.autoClearStencil = false;
        
        // Don't use Three.js XR manager - we handle XR ourselves
        vrRenderer.xr.enabled = false;
        
        // Create camera (will be updated manually per-view)
        vrCamera = new THREE.PerspectiveCamera();
        vrCamera.matrixAutoUpdate = false;
        vrCamera.matrixWorldAutoUpdate = false;
        
        // Setup lighting
        setupVRLighting();
        
        console.log('VR Robot renderer initialized');
        return true;
        
    } catch (error) {
        console.error('Failed to initialize VR Robot renderer:', error);
        return false;
    }
}

/**
 * Setup lighting for the VR scene
 */
function setupVRLighting() {
    // Ambient light - soft fill
    ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    vrScene.add(ambientLight);
    
    // Directional light - main light
    directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(1, 2, 1);
    vrScene.add(directionalLight);
    
    // Hemisphere light for natural ambient
    const hemiLight = new THREE.HemisphereLight(0xffffff, 0x444444, 0.3);
    hemiLight.position.set(0, 10, 0);
    vrScene.add(hemiLight);
    
    // Add debug cube to verify rendering is working
    addDebugCube();
}

/**
 * Add a small debug cube to verify Three.js rendering in VR
 */
function addDebugCube() {
    const geometry = new THREE.BoxGeometry(0.1, 0.1, 0.1);
    const material = new THREE.MeshStandardMaterial({ 
        color: 0x00ff00,
        emissive: 0x004400,  // Slight glow to be visible without lighting
    });
    const cube = new THREE.Mesh(geometry, material);
    
    // Position the debug cube at a known location
    // 1 meter in front, at eye level (1.5m), slightly to the right
    cube.position.set(0.3, 1.5, -1.0);
    cube.name = 'debugCube';
    
    vrScene.add(cube);
    console.log('VR Robot: Debug cube added at', cube.position.x, cube.position.y, cube.position.z);
}

// ========================================
// URDF Loading
// ========================================

/**
 * Load URDF XML and create the robot model in the VR scene
 * @param {string} urdfString - The URDF XML string
 */
export function loadVRURDF(urdfString) {
    if (!urdfString || vrURDFLoaded) return;
    if (!vrScene) {
        console.warn('VR Robot: Scene not initialized');
        return;
    }
    
    try {
        const parser = new DOMParser();
        const urdfDoc = parser.parseFromString(urdfString, 'text/xml');
        const robotEl = urdfDoc.querySelector('robot');
        
        if (!robotEl) {
            console.error('VR Robot: No robot element found in URDF');
            return;
        }
        
        const robotName = robotEl.getAttribute('name') || 'robot';
        console.log(`VR Robot: Loading URDF for ${robotName}`);
        
        // Clear existing robot
        if (vrRobot) {
            vrScene.remove(vrRobot);
        }
        
        vrRobot = new THREE.Group();
        vrRobot.name = robotName;
        vrLinks = {};
        vrJoints = {};
        
        // Parse material definitions from URDF root
        vrMaterialDefs = {};
        urdfDoc.querySelectorAll('robot > material').forEach(matEl => {
            const matName = matEl.getAttribute('name');
            const colorEl = matEl.querySelector('color');
            if (matName && colorEl) {
                const rgba = colorEl.getAttribute('rgba').split(' ').map(parseFloat);
                vrMaterialDefs[matName] = new THREE.Color(rgba[0], rgba[1], rgba[2]);
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
                    const mesh = createVRGeometry(geometry, origin, material);
                    if (mesh) {
                        linkGroup.add(mesh);
                    }
                }
            });
            
            vrLinks[linkName] = linkGroup;
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
            
            vrJoints[jointName] = {
                type: jointType,
                parent: parent,
                child: child,
                axis: axis ? axis.getAttribute('xyz').split(' ').map(parseFloat) : [0, 0, 1]
            };
            
            if (parent && child && vrLinks[parent] && vrLinks[child]) {
                childLinks.add(child);
                
                if (origin) {
                    const xyz = origin.getAttribute('xyz');
                    const rpy = origin.getAttribute('rpy');
                    
                    if (xyz) {
                        const pos = xyz.split(' ').map(parseFloat);
                        vrLinks[child].position.set(pos[0], pos[1], pos[2]);
                    }
                    if (rpy) {
                        const rot = rpy.split(' ').map(parseFloat);
                        vrLinks[child].rotation.order = 'ZYX';
                        vrLinks[child].rotation.set(rot[0], rot[1], rot[2]);
                    }
                }
                
                vrLinks[parent].add(vrLinks[child]);
            }
        });
        
        // Add root links to robot
        Object.keys(vrLinks).forEach(linkName => {
            if (!childLinks.has(linkName)) {
                vrRobot.add(vrLinks[linkName]);
            }
        });
        
        // Position robot in world space
        // Rotate for ROS -> Three.js coordinate conversion (Z-up to Y-up)
        vrRobot.rotation.x = -Math.PI / 2;
        
        // Position the robot in front of user
        vrRobot.position.set(ROBOT_POSITION.x, ROBOT_POSITION.y, ROBOT_POSITION.z);
        vrRobot.scale.set(ROBOT_SCALE, ROBOT_SCALE, ROBOT_SCALE);
        
        vrScene.add(vrRobot);
        vrURDFLoaded = true;
        
        console.log(`VR Robot: URDF loaded - ${Object.keys(vrLinks).length} links, ${Object.keys(vrJoints).length} joints`);
        
    } catch (error) {
        console.error('VR Robot: Error parsing URDF:', error);
    }
}

/**
 * Create geometry from URDF visual element
 */
function createVRGeometry(geometry, origin, material) {
    const box = geometry.querySelector('box');
    const cylinder = geometry.querySelector('cylinder');
    const sphere = geometry.querySelector('sphere');
    const meshEl = geometry.querySelector('mesh');
    
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
        // Load OBJ mesh asynchronously
        const filename = meshEl.getAttribute('filename');
        const scaleAttr = meshEl.getAttribute('scale');
        
        if (filename && filename.startsWith('package://alfie_urdf/meshes/') && filename.endsWith('.obj')) {
            const meshName = filename.replace('package://alfie_urdf/meshes/', '');
            const meshUrl = `/meshes/${meshName}`;
            
            // Create placeholder group to add mesh to when loaded
            const placeholder = new THREE.Group();
            
            if (typeof THREE.OBJLoader !== 'undefined') {
                const loader = new THREE.OBJLoader();
                loader.load(
                    meshUrl,
                    (obj) => {
                        const matColor = getVRMaterialColor(material);
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
                        
                        applyVROrigin(obj, origin);
                        placeholder.add(obj);
                    },
                    undefined,
                    (error) => console.warn('VR Robot: Failed to load mesh:', meshUrl, error)
                );
            }
            
            return placeholder;
        } else {
            // Fallback placeholder for unsupported mesh formats
            geom = new THREE.BoxGeometry(0.03, 0.03, 0.03);
        }
    }
    
    if (geom) {
        const color = getVRMaterialColor(material);
        const mat = new THREE.MeshStandardMaterial({
            color: color,
            metalness: 0.3,
            roughness: 0.7
        });
        
        mesh = new THREE.Mesh(geom, mat);
        applyVROrigin(mesh, origin);
    }
    
    return mesh;
}

/**
 * Get material color from URDF material element
 */
function getVRMaterialColor(materialEl) {
    if (!materialEl) return 0x666666;
    
    const colorEl = materialEl.querySelector('color');
    if (colorEl) {
        const rgba = colorEl.getAttribute('rgba').split(' ').map(parseFloat);
        return new THREE.Color(rgba[0], rgba[1], rgba[2]);
    }
    
    const matName = materialEl.getAttribute('name');
    if (matName && vrMaterialDefs[matName]) {
        return vrMaterialDefs[matName];
    }
    
    return 0x666666;
}

/**
 * Apply origin transform to object
 */
function applyVROrigin(object, origin) {
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
export function applyVRTransform(childFrameId, transform) {
    if (!vrLinks[childFrameId] || !transform) return;
    
    const link = vrLinks[childFrameId];
    
    // Store transform for later use
    tfTransforms[childFrameId] = transform;
    
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

/**
 * Apply multiple TF transforms at once
 * @param {Array} transforms - Array of {childFrameId, position, quaternion}
 */
export function applyVRTransforms(transforms) {
    if (!transforms || !Array.isArray(transforms)) return;
    
    for (const tf of transforms) {
        applyVRTransform(tf.childFrameId, tf);
    }
}

// ========================================
// VR Rendering
// ========================================

/**
 * Render the robot model for the current XR frame
 * Call this from the main XR render loop
 * @param {XRFrame} frame - The current XR frame
 * @param {XRReferenceSpace} refSpace - The reference space
 */
export function renderVRRobot(frame, refSpace) {
    if (!vrRenderer || !vrScene || !vrURDFLoaded) {
        return;
    }
    
    // Let Three.js handle the XR camera setup through its WebXRManager
    vrRenderer.render(vrScene, vrCamera);
}

/**
 * Alternative render method that takes view matrices directly
 * For better integration with existing stereo_vr.js rendering pipeline
 * @param {XRView} view - The XR view
 * @param {WebGLFramebuffer} framebuffer - The XR framebuffer
 */
export function renderVRRobotForView(view, framebuffer, glLayer) {
    if (!vrRenderer || !vrScene || !vrURDFLoaded) {
        return;
    }
    
    const gl = vrRenderer.getContext();
    const viewport = glLayer.getViewport(view);
    
    // Debug logging (once)
    if (!vrRobotDebugLogged && vrRobot) {
        console.log('VR Robot render - position:', vrRobot.position, 'links:', Object.keys(vrLinks).length);
        console.log('VR Robot view eye:', view.eye, 'viewport:', viewport.x, viewport.y, viewport.width, viewport.height);
        vrRobotDebugLogged = true;
    }
    
    // Set up camera from XR view
    vrCamera.projectionMatrix.fromArray(view.projectionMatrix);
    vrCamera.projectionMatrixInverse.copy(vrCamera.projectionMatrix).invert();
    
    // Set camera world matrix from view transform (not inverse!)
    // view.transform is the pose of the viewer in world space
    vrCamera.matrixWorld.fromArray(view.transform.matrix);
    vrCamera.matrixWorldInverse.copy(vrCamera.matrixWorld).invert();
    
    // Update camera matrices
    vrCamera.matrixAutoUpdate = false;
    vrCamera.matrixWorldAutoUpdate = false;
    
    // Set viewport for Three.js renderer
    vrRenderer.setViewport(viewport.x, viewport.y, viewport.width, viewport.height);
    
    // Set scissor to match viewport (important for stereo rendering)
    vrRenderer.setScissor(viewport.x, viewport.y, viewport.width, viewport.height);
    vrRenderer.setScissorTest(true);
    
    // Ensure depth testing is enabled for 3D objects
    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.LEQUAL);
    
    // Reset Three.js state to ensure proper rendering
    vrRenderer.state.reset();
    
    // Render the robot scene (without clearing)
    vrRenderer.render(vrScene, vrCamera);
    
    // Disable scissor test after rendering
    vrRenderer.setScissorTest(false);
    
    // CRITICAL: Reset Three.js internal state so it doesn't interfere with subsequent WebGL calls
    vrRenderer.state.reset();
    
    // Fully reset WebGL state for our custom shaders
    // Unbind all vertex attribute arrays that Three.js may have enabled
    for (let i = 0; i < 16; i++) {
        gl.disableVertexAttribArray(i);
    }
    
    // Unbind buffers
    gl.bindBuffer(gl.ARRAY_BUFFER, null);
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, null);
    
    // Unbind textures
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, null);
    
    // Reset program
    gl.useProgram(null);
    
    // Re-enable blending with our settings
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    
    // Re-enable depth test
    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.LEQUAL);
}

// ========================================
// State Getters
// ========================================

/**
 * Check if the VR robot model is loaded
 */
export function isVRRobotLoaded() {
    return vrURDFLoaded;
}

/**
 * Get the robot's world position
 */
export function getVRRobotPosition() {
    return vrRobot ? vrRobot.position.clone() : null;
}

/**
 * Set the robot's world position
 */
export function setVRRobotPosition(x, y, z) {
    if (vrRobot) {
        vrRobot.position.set(x, y, z);
    }
}

// ========================================
// Cleanup
// ========================================

/**
 * Dispose of VR robot resources
 */
export function disposeVRRobot() {
    if (vrRenderer) {
        vrRenderer.dispose();
        vrRenderer = null;
    }
    
    if (vrScene) {
        // Dispose of all objects in scene
        vrScene.traverse((object) => {
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
        vrScene = null;
    }
    
    vrRobot = null;
    vrLinks = {};
    vrJoints = {};
    vrURDFLoaded = false;
}
