// Robot State Module
// Handles Foxglove connection, TF processing, URDF handling, and video stream

import { FoxgloveConnection, FOXGLOVE_PORT } from './foxglove_conn.js';
import {
    parseStdString,
    parseCompressedImage,
    parseTFMessage,
    parseRosoutLog,
} from './cdr.js';

// ========================================
// Topics
// ========================================

const IMAGE_TOPICS = {
    left: '/alfie/stereo_camera/left_wide/image_raw/compressed',
    right: '/alfie/stereo_camera/right_wide/image_raw/compressed',
    left_center: '/alfie/stereo_camera/left_center/image_raw/compressed',
    right_center: '/alfie/stereo_camera/right_center/image_raw/compressed',
};
const ROBOT_DESCRIPTION_TOPIC = '/alfie/robot_description';
const TF_TOPIC = '/alfie/tf';
const TF_STATIC_TOPIC = '/alfie/tf_static';
const ROSOUT_TOPIC = '/rosout';

// ========================================
// Module State
// ========================================

// Connection state
let foxgloveConn = null;

// Video state - stores left/right separately before combining
let currentFrameBitmap = null;
let leftFrameBitmap = null;
let rightFrameBitmap = null;
let leftCenterFrameBitmap = null;
let rightCenterFrameBitmap = null;
let leftFrameTimestamp = 0;
let rightFrameTimestamp = 0;
let lastFrameReceivedTimestamp = 0;
let receivedFrameCount = 0;
let lastReceivedFpsTime = performance.now();
let currentVrFps = 0;
let firstFrameReceived = false;

// Frame sequence numbers so renderers only upload when a new frame arrived
let stereoFrameSeq = 0;   // Increments when currentFrameBitmap is replaced
let centerFrameSeq = 0;   // Increments when either center bitmap is replaced

// Per-stream decode pipeline: at most one createImageBitmap in flight per stream,
// only the latest pending message is kept, stale results are dropped.
function makeDecodeStream(side) {
    return { side, inFlight: false, pending: null, recvSeq: 0, displayedSeq: 0 };
}
const decodeStreams = {
    left: makeDecodeStream('left'),
    right: makeDecodeStream('right'),
    left_center: makeDecodeStream('left_center'),
    right_center: makeDecodeStream('right_center'),
};

// Stereo combine pipeline (same one-in-flight policy)
let combineInFlight = false;
let combinePending = false;
let combineSeq = 0;
let displayedCombineSeq = 0;

// Canvas for combining left/right images into stereo
let stereoCanvas = null;
let stereoCtx = null;

// Robot state
let urdfString = null;
let linkNames = [];
let tfCount = 0;
let lastTfTime = performance.now();
let currentTfRate = 0;
let tfRateInterval = null;
const TF_RATE_INTERVAL_MS = 5000;

// TF transform storage - stores latest transform for each frame
let tfTransforms = {};

// TF listeners - multiple modules (desktop viewer, VR URDF panel) receive TF updates
const tfListeners = new Set();

// URDF listeners - called whenever a robot_description arrives
const urdfListeners = new Set();

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

// ========================================
// Context Setup
// ========================================

export function setRobotStateContext(context) {
    setStatusFn = context.setStatus || setStatusFn;
    showCertButtonFn = context.showCertButton || showCertButtonFn;
    enableVrButtonFn = context.enableVrButton || enableVrButtonFn;
    startPreviewFn = context.startPreview || startPreviewFn;
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

export function getStereoFrameSeq() {
    return stereoFrameSeq;
}

export function getCenterFrameSeq() {
    return centerFrameSeq;
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

/**
 * Add a listener for TF updates
 * @param {Function} listener - Called with (childFrameId, transform) for each TF update
 * @returns {Function} unsubscribe function
 */
export function addTfListener(listener) {
    tfListeners.add(listener);
    return () => removeTfListener(listener);
}

export function removeTfListener(listener) {
    tfListeners.delete(listener);
}

/**
 * Call listener with the URDF string whenever robot_description arrives.
 * If a URDF is already available the listener is called immediately.
 * @returns {Function} unsubscribe function
 */
export function onUrdfAvailable(listener) {
    urdfListeners.add(listener);
    if (urdfString) {
        try {
            listener(urdfString);
        } catch (e) {
            console.error('URDF listener error:', e);
        }
    }
    return () => urdfListeners.delete(listener);
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
            reportSubscriptionStatus(channels);
        }
    });

    // Declare subscriptions once; the connection (re)subscribes whenever the
    // channels are advertised, including after reconnects
    for (const [side, topic] of Object.entries(IMAGE_TOPICS)) {
        foxgloveConn.subscribe(topic, (messageData) => onImageMessage(decodeStreams[side], messageData));
    }
    foxgloveConn.subscribe(ROBOT_DESCRIPTION_TOPIC, handleRobotDescriptionMessage);
    foxgloveConn.subscribe(TF_TOPIC, handleTFMessage);
    foxgloveConn.subscribe(TF_STATIC_TOPIC, handleTFMessage);
    foxgloveConn.subscribe(ROSOUT_TOPIC, handleRosoutMessage);

    startTfRateTracking();
    foxgloveConn.connect();
}

// ========================================
// Subscription Status
// ========================================

function reportSubscriptionStatus(channels) {
    const subscribedStreams = Object.entries(IMAGE_TOPICS)
        .filter(([, topic]) => foxgloveConn.isSubscribed(topic))
        .map(([side]) => side);

    if (subscribedStreams.length === 4) {
        setStatusFn('Subscribed to all stereo streams (wide + center)', 'connected');
    } else if (subscribedStreams.length > 0) {
        setStatusFn(`Subscribed to ${subscribedStreams.length}/4 camera streams`, 'connected');
        console.log('Subscribed to camera streams:', subscribedStreams.join(', '));
        console.log('Available topics:', channels.map(c => c.topic).join(', '));
    } else {
        console.log('Stereo image topics not found in advertised channels');
        console.log('Available topics:', channels.map(c => c.topic).join(', '));
        setStatusFn('Topics not found - is camera running?', 'error');
    }

    if (!urdfString && foxgloveConn.isSubscribed(ROBOT_DESCRIPTION_TOPIC)) {
        updateURDFStatus('Waiting for URDF...');
    }
}

// ========================================
// Message Handlers
// ========================================

// Decode robot_description (std_msgs/msg/String)
function handleRobotDescriptionMessage(data) {
    let urdf;
    try {
        urdf = parseStdString(data);
    } catch (error) {
        console.error('Failed to decode robot_description CDR:', error);
        updateURDFStatus('Decode error');
        return;
    }

    console.log('Received robot_description, length:', urdf.length);

    if (!urdf.includes('<robot')) {
        console.log('robot_description does not contain URDF XML');
        updateURDFStatus('Invalid URDF');
        return;
    }

    urdfString = urdf;
    // Parse URDF to extract link names
    parseURDFLinks(urdfString);

    for (const listener of urdfListeners) {
        try {
            listener(urdfString);
        } catch (e) {
            console.error('URDF listener error:', e);
        }
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

// Decode TF message (tf2_msgs/TFMessage)
function handleTFMessage(data) {
    let transforms;
    try {
        transforms = parseTFMessage(data);
    } catch (error) {
        // Silently ignore TF decode errors after first few
        if (tfCount < 5) {
            console.warn('TF decode error:', error);
        }
        return;
    }

    for (const transform of transforms) {
        // Store transform for this frame
        tfTransforms[transform.childFrameId] = transform;

        for (const listener of tfListeners) {
            try {
                listener(transform.childFrameId, transform);
            } catch (e) {
                console.error('TF listener error:', e);
            }
        }

        // Log first few TF messages for debugging
        if (tfCount < 3) {
            const p = transform.position;
            console.log(`TF: ${transform.frameId} -> ${transform.childFrameId}, pos: (${p.x.toFixed(3)}, ${p.y.toFixed(3)}, ${p.z.toFixed(3)})`);
        }
    }

    tfCount++;
}

// TF rate is computed on a timer so it drops to 0 when TF stops arriving.
// This is the single writer of the desktop #tfRate element.
function startTfRateTracking() {
    if (tfRateInterval) return;
    lastTfTime = performance.now();
    tfRateInterval = setInterval(() => {
        const now = performance.now();
        const elapsed = (now - lastTfTime) / 1000;
        currentTfRate = elapsed > 0 ? tfCount / elapsed : 0;
        tfCount = 0;
        lastTfTime = now;

        const tfRateEl = document.getElementById('tfRate');
        if (tfRateEl) {
            tfRateEl.textContent = currentTfRate > 0 ? `${currentTfRate.toFixed(1)} Hz` : '-- Hz';
        }
    }, TF_RATE_INTERVAL_MS);
}

// Decode rosout message (rcl_interfaces/msg/Log)
function handleRosoutMessage(data) {
    let logMessage;
    try {
        logMessage = parseRosoutLog(data);
    } catch (error) {
        console.warn('Rosout decode error:', error);
        return;
    }

    // Add to ring buffer
    rosoutMessages.push(logMessage);
    if (rosoutMessages.length > MAX_ROSOUT_MESSAGES) {
        rosoutMessages.shift();
    }

    // Call callback if registered
    if (onRosoutMessageFn) {
        onRosoutMessageFn(logMessage);
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

function onImageMessage(stream, messageData) {
    stream.recvSeq++;
    // Keep only the newest undecoded message
    stream.pending = { bytes: messageData, seq: stream.recvSeq };
    if (!stream.inFlight) {
        decodeNext(stream);
    }
}

/**
 * Decode the latest pending compressed image for one stream.
 * Combines left/right into a stereo side-by-side image when both are received.
 */
async function decodeNext(stream) {
    const job = stream.pending;
    stream.pending = null;
    if (!job) return;

    stream.inFlight = true;
    try {
        const image = parseCompressedImage(job.bytes);
        const blob = new Blob([image.data], { type: 'image/jpeg' });
        const bitmap = await createImageBitmap(blob);

        if (job.seq <= stream.displayedSeq) {
            bitmap.close();  // Older than what is already displayed
        } else {
            stream.displayedSeq = job.seq;
            storeBitmap(stream.side, bitmap);
        }
    } catch (error) {
        console.error(`Error processing compressed image (${stream.side}):`, error);
    } finally {
        stream.inFlight = false;
        if (stream.pending) {
            decodeNext(stream);
        }
    }
}

function storeBitmap(side, bitmap) {
    const now = performance.now();

    // Store the bitmap for the appropriate side, releasing the replaced one
    if (side === 'left') {
        if (leftFrameBitmap) leftFrameBitmap.close();
        leftFrameBitmap = bitmap;
        leftFrameTimestamp = now;
    } else if (side === 'right') {
        if (rightFrameBitmap) rightFrameBitmap.close();
        rightFrameBitmap = bitmap;
        rightFrameTimestamp = now;
    } else if (side === 'left_center') {
        if (leftCenterFrameBitmap) leftCenterFrameBitmap.close();
        leftCenterFrameBitmap = bitmap;
        centerFrameSeq++;
    } else if (side === 'right_center') {
        if (rightCenterFrameBitmap) rightCenterFrameBitmap.close();
        rightCenterFrameBitmap = bitmap;
        centerFrameSeq++;
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
}

/**
 * Combine left and right bitmaps into a single side-by-side stereo image
 */
function combineStereoBitmaps() {
    if (!leftFrameBitmap || !rightFrameBitmap) return;

    // One combine in flight; a newer pair re-runs the combine when it finishes
    if (combineInFlight) {
        combinePending = true;
        return;
    }

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

    // Draw left image on left side, right image on right side
    stereoCtx.drawImage(leftFrameBitmap, 0, 0);
    stereoCtx.drawImage(rightFrameBitmap, leftFrameBitmap.width, 0);

    const seq = ++combineSeq;
    combineInFlight = true;

    // Create combined bitmap (snapshot of the canvas at call time)
    createImageBitmap(stereoCanvas).then(combinedBitmap => {
        if (seq <= displayedCombineSeq) {
            combinedBitmap.close();
            return;
        }
        displayedCombineSeq = seq;

        // Release old bitmap
        if (currentFrameBitmap) {
            currentFrameBitmap.close();
        }
        currentFrameBitmap = combinedBitmap;
        stereoFrameSeq++;
        lastFrameReceivedTimestamp = performance.now();

        // Track received frame FPS
        receivedFrameCount++;
        const now = performance.now();
        const elapsed = now - lastReceivedFpsTime;
        if (elapsed >= 1000) {
            currentVrFps = (receivedFrameCount / elapsed) * 1000;
            receivedFrameCount = 0;
            lastReceivedFpsTime = now;
        }

        // Update status if this is the first frame
        if (!firstFrameReceived) {
            firstFrameReceived = true;
            setStatusFn('Stereo stream received (L+R combined)', 'connected');
            enableVrButtonFn();
            const certButton = document.getElementById('certButton');
            if (certButton) certButton.style.display = 'none';
            startPreviewFn();
        }
    }).catch(error => {
        console.error('Error creating combined stereo bitmap:', error);
    }).finally(() => {
        combineInFlight = false;
        if (combinePending) {
            combinePending = false;
            combineStereoBitmaps();
        }
    });
}

// ========================================
// Exports
// ========================================

export { FOXGLOVE_PORT };
