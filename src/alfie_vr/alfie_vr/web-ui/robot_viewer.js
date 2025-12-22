// Robot Viewer Module - Lightweight Three.js URDF visualization
// Uses URDF and TF data from robot_state.js instead of its own Foxglove connection

// ========================================
// Module State
// ========================================
let scene = null;
let camera = null;
let renderer = null;
let container = null;
let robot = null;
let links = {};
let joints = {};
let transforms = {};
let urdfLoaded = false;

// Camera controls
let isDragging = false;
let previousMousePosition = { x: 0, y: 0 };
let spherical = { theta: 0, phi: Math.PI / 4, radius: 1.2 };
let target = null;

// Animation
let animationId = null;
let lastFrameTime = 0;
const targetFPS = 10;
const frameInterval = 1000 / targetFPS;

// Material definitions from URDF
let materialDefs = {};

// ========================================
// Initialization
// ========================================

export function initRobotViewer(containerId) {
    container = document.getElementById(containerId);
    if (!container) {
        console.log('Robot viewer container not found:', containerId);
        return false;
    }
    
    if (typeof THREE === 'undefined') {
        console.error('Three.js not loaded');
        return false;
    }
    
    console.log('Initializing Robot Viewer...');
    
    setupScene();
    setupLighting();
    setupControls();
    animate();
    
    // Handle resize
    window.addEventListener('resize', onResize);
    new ResizeObserver(onResize).observe(container);
    
    console.log('Robot Viewer initialized');
    return true;
}

function setupScene() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1a1a);
    
    // Add small grid helper
    const gridHelper = new THREE.GridHelper(1, 10, 0x333333, 0x222222);
    scene.add(gridHelper);
    
    // Add axes helper
    const axesHelper = new THREE.AxesHelper(0.15);
    scene.add(axesHelper);
    
    // Setup camera
    const aspect = container.clientWidth / container.clientHeight;
    camera = new THREE.PerspectiveCamera(50, aspect, 0.01, 100);
    target = new THREE.Vector3(0, 0.3, 0);
    updateCameraPosition();
    
    // Setup renderer
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.shadowMap.enabled = false; // Disable shadows for performance
    container.appendChild(renderer.domElement);
}

function setupLighting() {
    // Ambient light
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);
    
    // Directional light
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.6);
    directionalLight.position.set(1, 2, 1);
    scene.add(directionalLight);
    
    // Hemisphere light for better ambient
    const hemiLight = new THREE.HemisphereLight(0xffffff, 0x444444, 0.2);
    hemiLight.position.set(0, 10, 0);
    scene.add(hemiLight);
}

function setupControls() {
    // Mouse down - start dragging
    container.addEventListener('mousedown', (e) => {
        isDragging = true;
        previousMousePosition = { x: e.clientX, y: e.clientY };
    });
    
    // Mouse up - stop dragging
    container.addEventListener('mouseup', () => {
        isDragging = false;
    });
    
    container.addEventListener('mouseleave', () => {
        isDragging = false;
    });
    
    // Mouse move - rotate camera
    container.addEventListener('mousemove', (e) => {
        if (!isDragging) return;
        
        const deltaX = e.clientX - previousMousePosition.x;
        const deltaY = e.clientY - previousMousePosition.y;
        
        spherical.theta -= deltaX * 0.01;
        spherical.phi -= deltaY * 0.01;
        spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, spherical.phi));
        
        updateCameraPosition();
        previousMousePosition = { x: e.clientX, y: e.clientY };
    });
    
    // Mouse wheel - zoom
    container.addEventListener('wheel', (e) => {
        e.preventDefault();
        spherical.radius += e.deltaY * 0.001;
        spherical.radius = Math.max(0.3, Math.min(3, spherical.radius));
        updateCameraPosition();
    }, { passive: false });
    
    // Touch support
    container.addEventListener('touchstart', (e) => {
        if (e.touches.length === 1) {
            isDragging = true;
            previousMousePosition = { x: e.touches[0].clientX, y: e.touches[0].clientY };
        }
    });
    
    container.addEventListener('touchend', () => {
        isDragging = false;
    });
    
    container.addEventListener('touchmove', (e) => {
        if (!isDragging || e.touches.length !== 1) return;
        
        const deltaX = e.touches[0].clientX - previousMousePosition.x;
        const deltaY = e.touches[0].clientY - previousMousePosition.y;
        
        spherical.theta -= deltaX * 0.01;
        spherical.phi -= deltaY * 0.01;
        spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, spherical.phi));
        
        updateCameraPosition();
        previousMousePosition = { x: e.touches[0].clientX, y: e.touches[0].clientY };
    });
}

function updateCameraPosition() {
    if (!camera || !target) return;
    
    const x = spherical.radius * Math.sin(spherical.phi) * Math.sin(spherical.theta);
    const y = spherical.radius * Math.cos(spherical.phi);
    const z = spherical.radius * Math.sin(spherical.phi) * Math.cos(spherical.theta);
    
    camera.position.set(target.x + x, target.y + y, target.z + z);
    camera.lookAt(target);
}

function onResize() {
    if (!container || !camera || !renderer) return;
    
    const width = container.clientWidth;
    const height = container.clientHeight;
    
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
}

function animate(currentTime = 0) {
    animationId = requestAnimationFrame((time) => animate(time));
    
    // Throttle rendering to target FPS
    const elapsed = currentTime - lastFrameTime;
    if (elapsed < frameInterval) return;
    lastFrameTime = currentTime - (elapsed % frameInterval);
    
    if (renderer && scene && camera) {
        renderer.render(scene, camera);
    }
}

// ========================================
// URDF Loading
// ========================================

export function loadURDF(urdfString) {
    if (!urdfString || urdfLoaded) return;
    if (!scene) {
        console.warn('Robot viewer not initialized');
        return;
    }
    
    try {
        const parser = new DOMParser();
        const urdfDoc = parser.parseFromString(urdfString, 'text/xml');
        const robotEl = urdfDoc.querySelector('robot');
        
        if (!robotEl) {
            console.error('No robot element found in URDF');
            return;
        }
        
        const robotName = robotEl.getAttribute('name') || 'robot';
        console.log(`Loading URDF for robot: ${robotName}`);
        
        // Clear existing robot
        if (robot) {
            scene.remove(robot);
        }
        
        robot = new THREE.Group();
        robot.name = robotName;
        links = {};
        joints = {};
        
        // Parse material definitions from URDF root
        materialDefs = {};
        urdfDoc.querySelectorAll('robot > material').forEach(matEl => {
            const matName = matEl.getAttribute('name');
            const colorEl = matEl.querySelector('color');
            if (matName && colorEl) {
                const rgba = colorEl.getAttribute('rgba').split(' ').map(parseFloat);
                materialDefs[matName] = new THREE.Color(rgba[0], rgba[1], rgba[2]);
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
            
            links[linkName] = linkGroup;
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
            
            joints[jointName] = {
                type: jointType,
                parent: parent,
                child: child,
                axis: axis ? axis.getAttribute('xyz').split(' ').map(parseFloat) : [0, 0, 1]
            };
            
            if (parent && child && links[parent] && links[child]) {
                childLinks.add(child);
                
                if (origin) {
                    const xyz = origin.getAttribute('xyz');
                    const rpy = origin.getAttribute('rpy');
                    
                    if (xyz) {
                        const pos = xyz.split(' ').map(parseFloat);
                        links[child].position.set(pos[0], pos[1], pos[2]);
                    }
                    if (rpy) {
                        const rot = rpy.split(' ').map(parseFloat);
                        links[child].rotation.order = 'ZYX';
                        links[child].rotation.set(rot[0], rot[1], rot[2]);
                    }
                }
                
                links[parent].add(links[child]);
            }
        });
        
        // Add root links to robot
        Object.keys(links).forEach(linkName => {
            if (!childLinks.has(linkName)) {
                robot.add(links[linkName]);
            }
        });
        
        // Rotate for ROS -> Three.js coordinate conversion (Z-up to Y-up)
        robot.rotation.x = -Math.PI / 2;
        
        scene.add(robot);
        urdfLoaded = true;
        
        console.log(`URDF loaded: ${Object.keys(links).length} links, ${Object.keys(joints).length} joints`);
        
    } catch (error) {
        console.error('Error parsing URDF:', error);
    }
}

function createGeometry(geometry, origin, material) {
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
                    (error) => console.warn('Failed to load mesh:', meshUrl, error)
                );
            }
            
            return placeholder;
        } else {
            // Fallback placeholder for unsupported mesh formats
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

function getMaterialColor(materialEl) {
    if (!materialEl) return 0x666666;
    
    const colorEl = materialEl.querySelector('color');
    if (colorEl) {
        const rgba = colorEl.getAttribute('rgba').split(' ').map(parseFloat);
        return new THREE.Color(rgba[0], rgba[1], rgba[2]);
    }
    
    const matName = materialEl.getAttribute('name');
    if (matName && materialDefs[matName]) {
        return materialDefs[matName];
    }
    
    return 0x666666;
}

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
// TF Updates
// ========================================

// Debug counter for TF updates
let tfApplyCount = 0;

export function applyTransform(childFrameId, transform) {
    if (!transform) return;
    
    // Try exact match first, then try without leading slash
    let link = links[childFrameId];
    if (!link) {
        const cleanName = childFrameId.replace(/^\//, '');
        link = links[cleanName];
    }
    
    if (!link) {
        // Log first few misses for debugging
        if (tfApplyCount < 5) {
            console.log('TF frame not found in links:', childFrameId, 'Available:', Object.keys(links).slice(0, 10));
        }
        tfApplyCount++;
        return;
    }
    
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
    
    // Log first successful update
    if (tfApplyCount === 0) {
        console.log('First TF applied to link:', childFrameId);
        tfApplyCount++;
    }
}

// ========================================
// State Getters
// ========================================

export function isLoaded() {
    return urdfLoaded;
}

export function getLinkCount() {
    return Object.keys(links).length;
}

// ========================================
// Cleanup
// ========================================

export function dispose() {
    if (animationId) {
        cancelAnimationFrame(animationId);
    }
    if (renderer) {
        renderer.dispose();
    }
}
