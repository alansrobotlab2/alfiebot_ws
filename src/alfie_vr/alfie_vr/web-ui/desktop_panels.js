// Desktop Panels Module
// Non-VR page UI: draggable robot status panel + 3D viewer, rosout log panel,
// headset battery indicator and the stereo preview image.

import { batteryState } from './state.js';
import {
    getCurrentFrameBitmap,
    getStereoFrameSeq,
    onUrdfAvailable,
    addTfListener,
    getTfTransforms,
    setRosoutCallback,
    clearRosoutMessages,
} from './robot_state.js';
import {
    initRobotViewer,
    loadURDF,
    applyTransform,
    isLoaded as isRobotViewerLoaded,
} from './robot_viewer.js';

const MAX_DESKTOP_ROSOUT_MESSAGES = 50;

// ========================================
// Stereo Preview (non-VR mode)
// ========================================

let previewRunning = false;

/**
 * Show preview of stereo stream until an XR session starts.
 * @param {Function} isXrActive - Returns true while an XR session is running
 */
export function startPreview(isXrActive) {
    if (previewRunning) return;
    previewRunning = true;

    const preview = document.getElementById('preview');

    // Create canvas for preview
    const canvas = document.createElement('canvas');
    const ctx = canvas.getContext('2d');
    let lastSeq = -1;

    function updatePreview() {
        if (isXrActive()) {
            previewRunning = false;
            return;
        }

        // Only re-encode when a new stereo frame arrived
        const bitmap = getCurrentFrameBitmap();
        const seq = getStereoFrameSeq();
        if (bitmap && seq !== lastSeq) {
            lastSeq = seq;
            if (canvas.width !== bitmap.width || canvas.height !== bitmap.height) {
                canvas.width = bitmap.width;
                canvas.height = bitmap.height;
            }
            ctx.drawImage(bitmap, 0, 0);
            preview.src = canvas.toDataURL('image/jpeg', 0.8);
            preview.classList.remove('hidden');
        }
        requestAnimationFrame(updatePreview);
    }

    updatePreview();
}

// ========================================
// Battery level monitoring
// ========================================

export function initBatteryMonitor() {
    if ('getBattery' in navigator) {
        navigator.getBattery().then((battery) => {
            updateBatteryDisplay(battery);
            battery.addEventListener('levelchange', () => updateBatteryDisplay(battery));
            battery.addEventListener('chargingchange', () => updateBatteryDisplay(battery));
        });
    } else {
        document.getElementById('battery').textContent = '🔋 N/A';
    }
}

function updateBatteryDisplay(battery) {
    const batteryEl = document.getElementById('battery');
    const level = Math.round(battery.level * 100);
    const icon = battery.charging ? '⚡' : '🔋';
    batteryEl.textContent = `${icon} ${level}%`;
    batteryEl.className = '';
    if (battery.charging) {
        batteryEl.classList.add('charging');
    } else if (level <= 20) {
        batteryEl.classList.add('low');
    }

    // Store for VR overlay
    batteryState.level = level;
    batteryState.charging = battery.charging;
}

// ========================================
// Robot Panel Functions
// ========================================

export function initRobotPanel() {
    const panel = document.getElementById('robotPanel');
    const toggle = document.getElementById('robotPanelToggle');
    const header = document.getElementById('robotPanelHeader');
    const resizeHandle = document.getElementById('robotPanelResize');
    const viewerContainer = document.getElementById('robotViewerContainer');

    if (!panel || !toggle) return;

    // Toggle collapse/expand
    toggle.addEventListener('click', () => {
        panel.classList.toggle('collapsed');
        toggle.textContent = panel.classList.contains('collapsed') ? '+' : '−';
    });

    // Make panel draggable
    let isDragging = false;
    let offsetX = 0;
    let offsetY = 0;

    header.addEventListener('mousedown', (e) => {
        if (e.target === toggle) return;
        isDragging = true;
        offsetX = e.clientX - panel.offsetLeft;
        offsetY = e.clientY - panel.offsetTop;
        panel.style.transition = 'none';
    });

    document.addEventListener('mousemove', (e) => {
        if (!isDragging) return;
        panel.style.left = (e.clientX - offsetX) + 'px';
        panel.style.top = (e.clientY - offsetY) + 'px';
    });

    document.addEventListener('mouseup', () => {
        isDragging = false;
        panel.style.transition = '';
    });

    // Make panel resizable
    let isResizing = false;
    let startWidth = 0;
    let startHeight = 0;
    let startX = 0;
    let startY = 0;

    if (resizeHandle) {
        resizeHandle.addEventListener('mousedown', (e) => {
            e.preventDefault();
            e.stopPropagation();
            isResizing = true;
            startX = e.clientX;
            startY = e.clientY;
            startWidth = panel.offsetWidth;
            startHeight = panel.offsetHeight;
            panel.classList.add('resizing');
        });

        document.addEventListener('mousemove', (e) => {
            if (!isResizing) return;

            const newWidth = Math.max(220, startWidth + (e.clientX - startX));
            const newHeight = Math.max(150, startHeight + (e.clientY - startY));

            panel.style.width = newWidth + 'px';
            panel.style.height = newHeight + 'px';

            // Adjust 3D viewer height to fill available space
            if (viewerContainer) {
                const contentPadding = 24; // 12px top + 12px bottom
                const statusRowsHeight = 120; // Approximate height of status rows
                const headerHeight = header.offsetHeight;
                const viewerHeight = newHeight - headerHeight - contentPadding - statusRowsHeight;
                viewerContainer.style.height = Math.max(100, viewerHeight) + 'px';
            }
        });

        document.addEventListener('mouseup', () => {
            if (isResizing) {
                isResizing = false;
                panel.classList.remove('resizing');
                // Trigger resize event for Three.js canvas to update
                window.dispatchEvent(new Event('resize'));
            }
        });
    }

    console.log('Robot panel initialized');
}

// Initialize 3D robot viewer and load the URDF when robot_description arrives
export function initRobotViewer3D() {
    // Initialize the Three.js viewer (a WebGL failure here must not block teleop init)
    let viewerReady = false;
    try {
        viewerReady = initRobotViewer('robotViewerContainer');
    } catch (error) {
        console.error('Robot 3D viewer failed to initialize:', error);
    }
    if (!viewerReady) {
        console.log('Robot 3D viewer initialization deferred (container or Three.js not ready)');
        return;
    }

    // TF updates for the desktop viewer (no-op until the URDF is loaded)
    addTfListener(applyTransform);

    // Event-driven: fires immediately if the URDF is already known, otherwise on arrival
    onUrdfAvailable((urdf) => {
        if (isRobotViewerLoaded()) return;
        console.log('URDF available, loading into 3D viewer...');
        loadURDF(urdf);
        if (isRobotViewerLoaded()) {
            // Apply the latest known transforms (TF static only arrives once)
            const transforms = getTfTransforms();
            for (const frameId of Object.keys(transforms)) {
                applyTransform(frameId, transforms[frameId]);
            }
        }
    });
}

// ========================================
// Rosout Log Panel Functions
// ========================================

export function initRosoutPanel() {
    const panel = document.getElementById('rosoutPanel');
    const toggle = document.getElementById('rosoutPanelToggle');
    const header = document.getElementById('rosoutPanelHeader');
    const clearBtn = document.getElementById('rosoutPanelClear');
    const messagesDiv = document.getElementById('rosoutMessages');

    if (!panel || !toggle || !messagesDiv) {
        console.warn('Rosout panel elements not found');
        return;
    }

    // Toggle collapse/expand
    toggle.addEventListener('click', () => {
        panel.classList.toggle('collapsed');
        toggle.textContent = panel.classList.contains('collapsed') ? '+' : '−';
    });

    // Clear button
    if (clearBtn) {
        clearBtn.addEventListener('click', () => {
            clearRosoutMessages();
            messagesDiv.replaceChildren();
        });
    }

    // Make panel draggable by header
    let isDragging = false;
    let offsetX = 0;
    let offsetY = 0;

    header.addEventListener('mousedown', (e) => {
        if (e.target === toggle || e.target === clearBtn) return;
        isDragging = true;
        offsetX = e.clientX - panel.offsetLeft;
        offsetY = e.clientY - panel.offsetTop;
    });

    document.addEventListener('mousemove', (e) => {
        if (isDragging) {
            panel.style.left = (e.clientX - offsetX) + 'px';
            panel.style.top = (e.clientY - offsetY) + 'px';
            panel.style.bottom = 'auto';
            panel.style.right = 'auto';
        }
    });

    document.addEventListener('mouseup', () => {
        isDragging = false;
    });

    // Set up callback to receive rosout messages
    setRosoutCallback((logMessage) => {
        if (!logMessage) {
            // Clear signal
            messagesDiv.replaceChildren();
            return;
        }
        addRosoutMessageToPanel(logMessage, messagesDiv);
    });

    console.log('Rosout panel initialized');
}

function makeSpan(className, text) {
    const span = document.createElement('span');
    span.className = className;
    span.textContent = text;  // textContent: ROS-sourced strings are never parsed as HTML
    return span;
}

function addRosoutMessageToPanel(logMessage, messagesDiv) {
    // Format timestamp
    const date = new Date(logMessage.timestamp * 1000);
    const timeStr = date.toLocaleTimeString('en-US', {
        hour12: false,
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit'
    });

    // Create message element
    const msgEl = document.createElement('div');
    msgEl.className = `rosout-msg ${String(logMessage.levelName).toLowerCase()}`;
    msgEl.append(
        makeSpan('rosout-time', timeStr), ' ',
        makeSpan('rosout-level', `[${logMessage.levelName}]`), ' ',
        makeSpan('rosout-name', logMessage.name), ' ',
        makeSpan('rosout-text', logMessage.msg)
    );

    messagesDiv.appendChild(msgEl);

    // Auto-scroll to bottom
    messagesDiv.scrollTop = messagesDiv.scrollHeight;

    // Limit displayed messages
    while (messagesDiv.children.length > MAX_DESKTOP_ROSOUT_MESSAGES) {
        messagesDiv.removeChild(messagesDiv.firstElementChild);
    }
}
