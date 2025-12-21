// Robot State Module
// Handles Foxglove connection, TF processing, URDF handling, and video stream

import { FoxgloveConnection, FOXGLOVE_PORT } from './foxglove_conn.js';

// ========================================
// Module State
// ========================================

// Connection state
let foxgloveConn = null;
let compressedImageSubscriptionId = null;
let robotDescriptionSubscriptionId = null;
let tfSubscriptionId = null;
let tfStaticSubscriptionId = null;

// Video state
let currentFrameBitmap = null;
let lastFrameReceivedTimestamp = 0;
let receivedFrameCount = 0;
let lastReceivedFpsTime = performance.now();
let currentVrFps = 0;

// Robot state
let urdfString = null;
let linkNames = [];
let tfCount = 0;
let lastTfTime = performance.now();
let currentTfRate = 0;

// ========================================
// Callback References
// ========================================

let setStatusFn = (text, type) => console.log('[Status]', type, text);
let showCertButtonFn = () => {};
let enableVrButtonFn = () => {};
let startPreviewFn = () => {};
let vrLogFn = (msg) => console.log('[VR]', msg);

// ========================================
// Context Setup
// ========================================

export function setRobotStateContext(context) {
    setStatusFn = context.setStatus || setStatusFn;
    showCertButtonFn = context.showCertButton || showCertButtonFn;
    enableVrButtonFn = context.enableVrButton || enableVrButtonFn;
    startPreviewFn = context.startPreview || startPreviewFn;
    vrLogFn = context.vrLog || vrLogFn;
}

// ========================================
// Getters for State Access
// ========================================

export function getFoxgloveConn() {
    return foxgloveConn;
}

export function getCurrentFrameBitmap() {
    return currentFrameBitmap;
}

export function getLastFrameReceivedTimestamp() {
    return lastFrameReceivedTimestamp;
}

export function getCurrentVrFps() {
    return currentVrFps;
}

export function getUrdfString() {
    return urdfString;
}

export function getLinkNames() {
    return linkNames;
}

export function getTfRate() {
    return currentTfRate;
}

// ========================================
// Foxglove Connection
// ========================================

export function initFoxgloveConnection() {
    setStatusFn('Connecting to stereo stream...');
    
    foxgloveConn = new FoxgloveConnection({
        reconnectInterval: 2000,
        onConnect: () => {
            setStatusFn('Connected to bridge, waiting for topics...', 'connected');
        },
        onDisconnect: (event) => {
            setStatusFn('Connection closed, reconnecting...', 'error');
            if (event.code === 1006) {
                showCertButtonFn();
            }
        },
        onAdvertise: (channels) => {
            subscribeToCompressedImage(channels);
            subscribeToRobotTopics(channels);
        }
    });
    
    foxgloveConn.connect();
}

// ========================================
// Subscription Handlers
// ========================================

function subscribeToCompressedImage(channels) {
    const topic = '/alfie/stereo_camera/image_raw/compressed';
    
    // Use the FoxgloveConnection's subscribe with custom handler
    compressedImageSubscriptionId = foxgloveConn.subscribe(topic, (messageData) => {
        const view = new DataView(messageData.buffer, messageData.byteOffset, messageData.byteLength);
        processCompressedImage(view);
    });
    
    if (compressedImageSubscriptionId) {
        setStatusFn('Subscribed to stereo stream', 'connected');
    } else {
        console.log('Compressed image topic not found in advertised channels');
        console.log('Available topics:', channels.map(c => c.topic).join(', '));
        setStatusFn('Topic not found - is camera running?', 'error');
    }
}

function subscribeToRobotTopics(channels) {
    let subscribedCount = 0;
    
    // Subscribe to robot_description
    robotDescriptionSubscriptionId = foxgloveConn.subscribe('/alfie/robot_description', (messageData) => {
        handleRobotDescriptionMessage(messageData);
    });
    if (robotDescriptionSubscriptionId) {
        console.log('Subscribed to /alfie/robot_description');
        subscribedCount++;
    }
    
    // Subscribe to TF
    tfSubscriptionId = foxgloveConn.subscribe('/alfie/tf', (messageData) => {
        handleTFMessage(messageData);
    });
    if (tfSubscriptionId) {
        console.log('Subscribed to /alfie/tf');
        subscribedCount++;
    }
    
    // Subscribe to TF static
    tfStaticSubscriptionId = foxgloveConn.subscribe('/alfie/tf_static', (messageData) => {
        handleTFMessage(messageData);
    });
    if (tfStaticSubscriptionId) {
        console.log('Subscribed to /alfie/tf_static');
        subscribedCount++;
    }
    
    console.log(`Subscribed to ${subscribedCount} robot topics`);
    updateURDFStatus('Waiting for URDF...');
}

// ========================================
// CDR Binary Message Decoders
// ========================================

// Decode robot_description (std_msgs/msg/String) CDR message
function handleRobotDescriptionMessage(data) {
    try {
        // CDR format for std_msgs/msg/String:
        // - 4 bytes: CDR encapsulation header (00 01 00 00 for little-endian)
        // - 4 bytes: string length (uint32, little-endian, includes null terminator)
        // - N bytes: string data (UTF-8)
        const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
        
        // Skip 4-byte CDR encapsulation header, then read string length
        const stringLength = view.getUint32(4, true);
        
        // Extract the string (skip 8 bytes header, exclude null terminator)
        const stringData = new Uint8Array(data.buffer, data.byteOffset + 8, stringLength - 1);
        const decoder = new TextDecoder();
        urdfString = decoder.decode(stringData);
        
        console.log('Received robot_description, length:', stringLength);
        
        if (urdfString.includes('<robot')) {
            // Parse URDF to extract link names
            parseURDFLinks(urdfString);
        } else {
            console.log('robot_description does not contain URDF XML');
            updateURDFStatus('Invalid URDF');
        }
    } catch (error) {
        console.error('Failed to decode robot_description CDR:', error);
        updateURDFStatus('Decode error');
    }
}

// Parse URDF XML to extract link names
function parseURDFLinks(urdfXml) {
    try {
        const parser = new DOMParser();
        const doc = parser.parseFromString(urdfXml, 'text/xml');
        const robotEl = doc.querySelector('robot');
        
        if (!robotEl) {
            console.error('No robot element in URDF');
            updateURDFStatus('Parse error');
            return;
        }
        
        const robotName = robotEl.getAttribute('name') || 'robot';
        const links = doc.querySelectorAll('link');
        const joints = doc.querySelectorAll('joint');
        
        linkNames = Array.from(links).map(link => link.getAttribute('name'));
        
        console.log(`Parsed URDF: ${robotName} - ${linkNames.length} links, ${joints.length} joints`);
        updateURDFStatus('Loaded');
        updateLinkCount(linkNames.length);
    } catch (error) {
        console.error('Error parsing URDF:', error);
        updateURDFStatus('Parse error');
    }
}

// Decode TF message (tf2_msgs/TFMessage) CDR format
function handleTFMessage(data) {
    try {
        let offset = 4;  // Skip CDR header
        const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
        
        const readString = () => {
            const len = view.getUint32(offset, true);
            offset += 4;
            // len includes null terminator, so read len-1 bytes
            const strBytes = new Uint8Array(data.buffer, data.byteOffset + offset, len > 0 ? len - 1 : 0);
            offset += len;
            // Align to 4 bytes after string
            if (offset % 4 !== 0) offset += 4 - (offset % 4);
            return new TextDecoder().decode(strBytes);
        };
        
        // TFMessage: TransformStamped[] transforms
        const transformCount = view.getUint32(offset, true);
        offset += 4;
        
        for (let i = 0; i < transformCount; i++) {
            // Header: stamp (sec, nanosec) + frame_id
            const stampSec = view.getUint32(offset, true); offset += 4;
            const stampNsec = view.getUint32(offset, true); offset += 4;
            const frameId = readString();
            const childFrameId = readString();
            
            // Transform: translation (Vector3 float64), rotation (Quaternion float64)
            // CDR alignment: after child_frame_id, ensure offset % 8 === 4
            if (offset % 8 === 0) offset += 4;
            const tx = view.getFloat64(offset, true); offset += 8;
            const ty = view.getFloat64(offset, true); offset += 8;
            const tz = view.getFloat64(offset, true); offset += 8;
            const qx = view.getFloat64(offset, true); offset += 8;
            const qy = view.getFloat64(offset, true); offset += 8;
            const qz = view.getFloat64(offset, true); offset += 8;
            const qw = view.getFloat64(offset, true); offset += 8;
            
            // Log first few TF messages for debugging
            if (tfCount < 3) {
                console.log(`TF: ${frameId} -> ${childFrameId}, pos: (${tx.toFixed(3)}, ${ty.toFixed(3)}, ${tz.toFixed(3)})`);
            }
        }
        
        tfCount++;
        
        // Update TF rate every 5 seconds
        const now = performance.now();
        const elapsed = (now - lastTfTime) / 1000;
        if (elapsed >= 5.0) {
            currentTfRate = tfCount / elapsed;
            tfCount = 0;
            lastTfTime = now;
            // Update desktop panel
            const tfRateEl = document.getElementById('tfRate');
            if (tfRateEl) tfRateEl.textContent = currentTfRate.toFixed(1);
        }
        
    } catch (error) {
        // Silently ignore TF decode errors after first few
        if (tfCount < 5) {
            console.warn('TF decode error:', error);
        }
    }
}

// ========================================
// UI Update Functions
// ========================================

function updateURDFStatus(status) {
    const el = document.getElementById('urdfStatus');
    if (el) el.textContent = status;
}

function updateLinkCount(count) {
    const el = document.getElementById('linkCount');
    if (el) el.textContent = count.toString();
}

// ========================================
// Image Processing
// ========================================

export async function processCompressedImage(view) {
    try {
        let offset = 4; // Skip CDR encapsulation header (4 bytes)
        
        // Helper to align offset
        const align = (n) => {
            offset = (offset + n - 1) & ~(n - 1);
        };

        // Header
        // stamp (8 bytes)
        align(4); // Time is struct of 2 int32/uint32, so 4 byte alignment
        offset += 8;
        
        // frame_id (string)
        align(4);
        const frameIdLen = view.getUint32(offset, true);
        offset += 4 + frameIdLen;
        
        // format (string)
        align(4);
        const formatLen = view.getUint32(offset, true);
        offset += 4 + formatLen;
        
        // data (uint8[])
        align(4);
        const dataLen = view.getUint32(offset, true);
        offset += 4;
        
        // Extract image data
        const imageBytes = new Uint8Array(view.buffer, view.byteOffset + offset, dataLen);
        
        // Create blob and bitmap
        const blob = new Blob([imageBytes], { type: 'image/jpeg' });
        const bitmap = await createImageBitmap(blob);
        
        // Update current frame
        if (currentFrameBitmap) {
            currentFrameBitmap.close(); // Release old bitmap
        }
        currentFrameBitmap = bitmap;
        lastFrameReceivedTimestamp = performance.now();  // Record when this frame arrived
        
        // Track received frame FPS
        receivedFrameCount++;
        const now = performance.now();
        const elapsed = now - lastReceivedFpsTime;
        if (elapsed >= 1000) {
            currentVrFps = (receivedFrameCount / elapsed) * 1000;
            receivedFrameCount = 0;
            lastReceivedFpsTime = now;
            
            // Update HTML FPS counter too
            const fpsElement = document.getElementById('fpsCounter');
            if (fpsElement) {
                fpsElement.textContent = `FPS: ${currentVrFps.toFixed(1)}`;
            }
        }
        
        // Update status if this is the first frame
        if (document.getElementById('vrButton').disabled) {
            setStatusFn('Stereo stream received', 'connected');
            enableVrButtonFn();
            document.getElementById('certButton').style.display = 'none';
            startPreviewFn();
        }
        
    } catch (error) {
        console.error('Error processing compressed image:', error);
    }
}

// ========================================
// Exports
// ========================================

export { FOXGLOVE_PORT };
