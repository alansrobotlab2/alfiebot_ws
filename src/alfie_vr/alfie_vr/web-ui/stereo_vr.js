        // WebXR Stereo Vision for Meta Quest 3
        // Displays side-by-side stereo video as immersive VR
        // With VR Controller Tracking support
        //
        // Entry point (loaded by index.html). Wires the modules together:
        //   xr_session.js      - XR session lifecycle, render loop, input + safety releases
        //   video_renderer.js  - stereo video shaders / quads / textures
        //   vr_overlays.js     - head-locked VR panels (state, stream, bearing, rosout, URDF)
        //   desktop_panels.js  - desktop drag panels, rosout log, battery, preview
        //   robot_state.js     - Foxglove connection, video decode, TF, URDF, rosout
        //   controller_tracking.js - controller WebSocket protocol

        import { initControllerWebSocket } from './controller_tracking.js';
        import {
            FOXGLOVE_PORT,
            setRobotStateContext,
            initFoxgloveConnection,
            getLeftCenterFrameBitmap,
            getRightCenterFrameBitmap,
        } from './robot_state.js';
        import { initXR, isXrActive, setStreamReady } from './xr_session.js';
        import {
            startPreview,
            initBatteryMonitor,
            initRobotPanel,
            initRobotViewer3D,
            initRosoutPanel,
        } from './desktop_panels.js';
        import { setStatus } from './ui_status.js';

        // Re-export center camera frame getters for external access
        export {
            FOXGLOVE_PORT,
            getLeftCenterFrameBitmap,
            getRightCenterFrameBitmap,
        };

        // Show certificate button for user to accept SSL cert
        function showCertButton() {
            const btn = document.getElementById('certButton');
            btn.style.display = 'block';
            btn.onclick = () => {
                // Open the stereo signaling URL in a new tab so user can accept cert
                window.open(`https://${window.location.hostname}:${FOXGLOVE_PORT}/`, '_blank');
                setStatus('Accept the certificate in the new tab, then reload this page', 'error');
            };
        }

        // Set up robot state context with callbacks
        setRobotStateContext({
            setStatus: setStatus,
            showCertButton: showCertButton,
            enableVrButton: setStreamReady,
            startPreview: () => startPreview(isXrActive),
        });

        // Initialize on page load
        window.addEventListener('DOMContentLoaded', async () => {
            console.log('Stereo VR Vision - Initializing...');
            initBatteryMonitor();
            initRobotPanel();  // Initialize robot status panel
            initRosoutPanel();  // Initialize rosout log panel
            initRobotViewer3D();  // Initialize 3D robot viewer
            initControllerWebSocket();  // Initialize controller tracking WebSocket
            initFoxgloveConnection();
            await initXR({
                onSessionEnded: () => startPreview(isXrActive),
            });
        });
