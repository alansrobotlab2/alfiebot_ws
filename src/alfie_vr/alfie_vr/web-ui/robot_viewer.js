// Robot Viewer Module - Lightweight Three.js URDF visualization
// Uses URDF and TF data from robot_state.js instead of its own Foxglove connection

import { buildUrdfRobot, applyLinkTransform } from './urdf_builder.js';

// ========================================
// Module State
// ========================================
let scene = null;
let camera = null;
let renderer = null;
let container = null;
let robotModel = null;  // Result of buildUrdfRobot()
let links = {};
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
        const built = buildUrdfRobot(urdfString, { log: (msg) => console.log(msg) });
        if (!built) {
            console.error('Failed to build robot from URDF');
            return;
        }
        
        // Clear existing robot
        if (robotModel) {
            robotModel.dispose();
        }
        
        robotModel = built;
        links = built.links;
        scene.add(built.robot);
        urdfLoaded = true;
        
    } catch (error) {
        console.error('Error parsing URDF:', error);
    }
}

// ========================================
// TF Updates
// ========================================

// Debug counter for TF updates
let tfApplyCount = 0;

export function applyTransform(childFrameId, transform) {
    if (!transform || !urdfLoaded) return;
    
    if (!applyLinkTransform(links, childFrameId, transform)) {
        // Log first few misses for debugging
        if (tfApplyCount < 5) {
            console.log('TF frame not found in links:', childFrameId, 'Available:', Object.keys(links).slice(0, 10));
        }
        tfApplyCount++;
        return;
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
