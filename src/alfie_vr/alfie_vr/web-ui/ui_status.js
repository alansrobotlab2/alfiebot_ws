// UI Status Module
// Shared status line and on-screen VR debug log (visible in headset via dom-overlay)

const MAX_VR_LOG_LINES = 15;
const vrLogLines = [];

// Status display
export function setStatus(text, type = '') {
    const status = document.getElementById('status');
    if (!status) return;
    status.textContent = text;
    status.className = type;
}

// VR Debug logging - shows on screen in headset
// Uses textContent so ROS-sourced strings (URDF names, errors) can never inject markup
export function vrLog(message) {
    console.log(message);
    vrLogLines.push(new Date().toLocaleTimeString() + ': ' + message);
    if (vrLogLines.length > MAX_VR_LOG_LINES) vrLogLines.shift();  // Keep last 15 lines
    const debugDiv = document.getElementById('vrDebug');
    if (debugDiv) {
        debugDiv.style.whiteSpace = 'pre-line';
        debugDiv.textContent = vrLogLines.join('\n');
    }
}
