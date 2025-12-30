// Robot State Module
// Handles Foxglove connection, TF processing, URDF handling, and video stream

import { FoxgloveConnection, FOXGLOVE_PORT } from './foxglove_conn.js';

// ========================================
// Module State
// ========================================

// Connection state
let foxgloveConn = null;
let leftImageSubscriptionId = null;
let rightImageSubscriptionId = null;
let leftCenterImageSubscriptionId = null;
let rightCenterImageSubscriptionId = null;
let robotDescriptionSubscriptionId = null;
let tfSubscriptionId = null;
let tfStaticSubscriptionId = null;
let rosoutSubscriptionId = null;

// Video state - now stores left/right separately before combining
let currentFrameBitmap = null;
let leftFrameBitmap = null;
let rightFrameBitmap = null;
let leftCenterFrameBitmap = null;
let rightCenterFrameBitmap = null;
let leftFrameTimestamp = 0;
let rightFrameTimestamp = 0;
let leftCenterFrameTimestamp = 0;
let rightCenterFrameTimestamp = 0;
let lastFrameReceivedTimestamp = 0;
let receivedFrameCount = 0;
let lastReceivedFpsTime = performance.now();
let currentVrFps = 0;

// Canvas for combining left/right images into stereo
let stereoCanvas = null;
let stereoCtx = null;

// Robot state
let urdfString = null;
let linkNames = [];
let tfCount = 0;
let lastTfTime = performance.now();
let currentTfRate = 0;

// TF transform storage - stores latest transform for each frame
let tfTransforms = {};

// TF callback - allows external modules to receive TF updates
let onTfUpdateFn = null;

// Rosout message storage (ring buffer)
const MAX_ROSOUT_MESSAGES = 50;
let rosoutMessages = [];
let onRosoutMessageFn = null;

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

export function getLeftCenterFrameBitmap() {
    return leftCenterFrameBitmap;
}

export function getRightCenterFrameBitmap() {
    return rightCenterFrameBitmap;
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

export function getTfTransforms() {
    return tfTransforms;
}

export function getTfTransform(frameId) {
    return tfTransforms[frameId] || null;
}

/**
 * Set a callback to receive TF updates
 * @param {Function} callback - Called with (childFrameId, transform) for each TF update
 */
export function setTfUpdateCallback(callback) {
    onTfUpdateFn = callback;
}

/**
 * Get rosout messages array
 */
export function getRosoutMessages() {
    return rosoutMessages;
}

/**
 * Set a callback to receive rosout updates
 * @param {Function} callback - Called with (message) for each rosout message
 */
export function setRosoutCallback(callback) {
    onRosoutMessageFn = callback;
}

/**
 * Clear all rosout messages
 */
export function clearRosoutMessages() {
    rosoutMessages = [];
    if (onRosoutMessageFn) {
        onRosoutMessageFn(null);  // Signal clear
    }
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
    const leftTopic = '/alfie/stereo_camera/left_wide/image_raw/compressed';
    const rightTopic = '/alfie/stereo_camera/right_wide/image_raw/compressed';
    const leftCenterTopic = '/alfie/stereo_camera/left_center/image_raw/compressed';
    const rightCenterTopic = '/alfie/stereo_camera/right_center/image_raw/compressed';

    // Subscribe to left wide image with custom handler
    leftImageSubscriptionId = foxgloveConn.subscribe(leftTopic, (messageData) => {
        const view = new DataView(messageData.buffer, messageData.byteOffset, messageData.byteLength);
        processCompressedImageSide(view, 'left');
    });

    // Subscribe to right wide image with custom handler
    rightImageSubscriptionId = foxgloveConn.subscribe(rightTopic, (messageData) => {
        const view = new DataView(messageData.buffer, messageData.byteOffset, messageData.byteLength);
        processCompressedImageSide(view, 'right');
    });

    // Subscribe to left center (higher resolution) image
    leftCenterImageSubscriptionId = foxgloveConn.subscribe(leftCenterTopic, (messageData) => {
        const view = new DataView(messageData.buffer, messageData.byteOffset, messageData.byteLength);
        processCompressedImageSide(view, 'left_center');
    });

    // Subscribe to right center (higher resolution) image
    rightCenterImageSubscriptionId = foxgloveConn.subscribe(rightCenterTopic, (messageData) => {
        const view = new DataView(messageData.buffer, messageData.byteOffset, messageData.byteLength);
        processCompressedImageSide(view, 'right_center');
    });

    // Report subscription status
    let subscribedStreams = [];
    if (leftImageSubscriptionId) subscribedStreams.push('left_wide');
    if (rightImageSubscriptionId) subscribedStreams.push('right_wide');
    if (leftCenterImageSubscriptionId) subscribedStreams.push('left_center');
    if (rightCenterImageSubscriptionId) subscribedStreams.push('right_center');

    if (subscribedStreams.length === 4) {
        setStatusFn('Subscribed to all stereo streams (wide + center)', 'connected');
        console.log('Subscribed to all 4 camera streams:', subscribedStreams.join(', '));
    } else if (subscribedStreams.length > 0) {
        setStatusFn(`Subscribed to ${subscribedStreams.length}/4 camera streams`, 'connected');
        console.log('Subscribed to camera streams:', subscribedStreams.join(', '));
        console.log('Available topics:', channels.map(c => c.topic).join(', '));
    } else {
        console.log('Stereo image topics not found in advertised channels');
        console.log('Available topics:', channels.map(c => c.topic).join(', '));
        setStatusFn('Topics not found - is camera running?', 'error');
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
    
    // Subscribe to rosout
    rosoutSubscriptionId = foxgloveConn.subscribe('/rosout', (messageData) => {
        handleRosoutMessage(messageData);
    });
    if (rosoutSubscriptionId) {
        console.log('Subscribed to /rosout');
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
            
            // Store transform for this frame
            const transform = {
                frameId: frameId,
                childFrameId: childFrameId,
                position: { x: tx, y: ty, z: tz },
                quaternion: { x: qx, y: qy, z: qz, w: qw }
            };
            tfTransforms[childFrameId] = transform;
            
            // Call TF update callback if registered
            if (onTfUpdateFn) {
                onTfUpdateFn(childFrameId, transform);
            }
            
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

// Decode rosout message (rcl_interfaces/msg/Log) CDR format
// Log level constants: DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50
const LOG_LEVELS = {
    10: 'DEBUG',
    20: 'INFO',
    30: 'WARN',
    40: 'ERROR',
    50: 'FATAL'
};

function handleRosoutMessage(data) {
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
        
        // builtin_interfaces/Time stamp (8 bytes: sec + nanosec)
        const stampSec = view.getUint32(offset, true); offset += 4;
        const stampNsec = view.getUint32(offset, true); offset += 4;
        
        // uint8 level
        const level = view.getUint8(offset); offset += 1;
        // Align to 4 bytes for next string
        if (offset % 4 !== 0) offset += 4 - (offset % 4);
        
        // string name (logger name)
        const name = readString();
        
        // string msg
        const msg = readString();
        
        // string file
        const file = readString();
        
        // string function
        const func = readString();
        
        // uint32 line
        const line = view.getUint32(offset, true); offset += 4;
        
        // Create message object
        const logMessage = {
            timestamp: stampSec + stampNsec / 1e9,
            level: level,
            levelName: LOG_LEVELS[level] || `L${level}`,
            name: name,
            msg: msg,
            file: file,
            function: func,
            line: line
        };
        
        // Add to ring buffer
        rosoutMessages.push(logMessage);
        if (rosoutMessages.length > MAX_ROSOUT_MESSAGES) {
            rosoutMessages.shift();
        }
        
        // Call callback if registered
        if (onRosoutMessageFn) {
            onRosoutMessageFn(logMessage);
        }
        
    } catch (error) {
        console.warn('Rosout decode error:', error);
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

/**
 * Process a compressed image from either left or right camera
 * Combines both into a stereo side-by-side image when both are received
 */
async function processCompressedImageSide(view, side) {
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
        
        const now = performance.now();
        
        // Store the bitmap for the appropriate side
        if (side === 'left') {
            if (leftFrameBitmap) {
                leftFrameBitmap.close();
            }
            leftFrameBitmap = bitmap;
            leftFrameTimestamp = now;
        } else if (side === 'right') {
            if (rightFrameBitmap) {
                rightFrameBitmap.close();
            }
            rightFrameBitmap = bitmap;
            rightFrameTimestamp = now;
        } else if (side === 'left_center') {
            if (leftCenterFrameBitmap) {
                leftCenterFrameBitmap.close();
            }
            leftCenterFrameBitmap = bitmap;
            leftCenterFrameTimestamp = now;
        } else if (side === 'right_center') {
            if (rightCenterFrameBitmap) {
                rightCenterFrameBitmap.close();
            }
            rightCenterFrameBitmap = bitmap;
            rightCenterFrameTimestamp = now;
        }
        
        // Combine if we have both frames within a reasonable time window (100ms)
        // Only combine once per pair by resetting timestamps after combining
        if (leftFrameBitmap && rightFrameBitmap && 
            leftFrameTimestamp > 0 && rightFrameTimestamp > 0 &&
            Math.abs(leftFrameTimestamp - rightFrameTimestamp) < 100) {
            // Reset timestamps to prevent double-counting this pair
            leftFrameTimestamp = 0;
            rightFrameTimestamp = 0;
            combineStereoBitmaps();
        }
        
    } catch (error) {
        console.error(`Error processing compressed image (${side}):`, error);
    }
}

/**
 * Combine left and right bitmaps into a single side-by-side stereo image
 */
function combineStereoBitmaps() {
    if (!leftFrameBitmap || !rightFrameBitmap) return;
    
    const width = leftFrameBitmap.width + rightFrameBitmap.width;
    const height = Math.max(leftFrameBitmap.height, rightFrameBitmap.height);
    
    // Create or resize stereo canvas
    if (!stereoCanvas) {
        stereoCanvas = new OffscreenCanvas(width, height);
        stereoCtx = stereoCanvas.getContext('2d');
    } else if (stereoCanvas.width !== width || stereoCanvas.height !== height) {
        stereoCanvas.width = width;
        stereoCanvas.height = height;
    }
    
    // Draw left image on left side
    stereoCtx.drawImage(leftFrameBitmap, 0, 0);
    
    // Draw right image on right side
    stereoCtx.drawImage(rightFrameBitmap, leftFrameBitmap.width, 0);
    
    // Create combined bitmap
    createImageBitmap(stereoCanvas).then(combinedBitmap => {
        // Release old bitmap
        if (currentFrameBitmap) {
            currentFrameBitmap.close();
        }
        currentFrameBitmap = combinedBitmap;
        lastFrameReceivedTimestamp = performance.now();
        
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
            setStatusFn('Stereo stream received (L+R combined)', 'connected');
            enableVrButtonFn();
            document.getElementById('certButton').style.display = 'none';
            startPreviewFn();
        }
    }).catch(error => {
        console.error('Error creating combined stereo bitmap:', error);
    });
}

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
