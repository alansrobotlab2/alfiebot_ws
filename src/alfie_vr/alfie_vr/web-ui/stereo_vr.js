        // WebXR Stereo Vision for Meta Quest 3
        // Displays side-by-side stereo video as immersive VR
        // With VR Controller Tracking support
        
        import { FoxgloveConnection, FOXGLOVE_PORT } from './foxglove_conn.js';
        
        const CONTROLLER_WS_PORT = 8442;
        
        let foxgloveConn = null;
        let compressedImageSubscriptionId = null;
        let robotDescriptionSubscriptionId = null;
        let tfSubscriptionId = null;
        let tfStaticSubscriptionId = null;
        let currentFrameBitmap = null;
        
        // Robot state tracking
        let urdfString = null;
        let linkNames = [];
        let tfCount = 0;
        let lastTfTime = performance.now();
        let currentTfRate = 0;  // Hz - updated every 5 seconds
        let xrSession = null;
        let xrRefSpace = null;
        let gl = null;
        let leftTexture = null;
        let rightTexture = null;
        let videoElement = null;
        let leftCanvas, rightCanvas, leftCtx, rightCtx;
        let retryCount = 0;
        const MAX_RETRIES = 3;
        
        // Controller WebSocket connection
        let controllerWS = null;
        let controllerWSRetryCount = 0;
        const CONTROLLER_WS_MAX_RETRIES = 10;
        
        // Controller tracking state
        let leftController = {
            hand: 'left',
            position: null,
            rotation: null,
            quaternion: null,
            gripActive: false,
            trigger: 0,
            thumbstick: { x: 0, y: 0 },
            buttons: { squeeze: false, thumbstick: false, x: false, y: false, menu: false }
        };
        
        let rightController = {
            hand: 'right',
            position: null,
            rotation: null,
            quaternion: null,
            gripActive: false,
            trigger: 0,
            thumbstick: { x: 0, y: 0 },
            buttons: { squeeze: false, thumbstick: false, a: false, b: false, menu: false }
        };
        
        let headsetPose = {
            position: null,
            rotation: null,
            quaternion: null
        };
        
        // Grip state tracking for relative rotation
        let leftGripInitialQuaternion = null;
        let rightGripInitialQuaternion = null;
        let leftGripDown = false;
        let rightGripDown = false;
        
        // ========================================
        // Controller WebSocket Functions
        // ========================================
        
        function initControllerWebSocket() {
            const wsUrl = `wss://${window.location.hostname}:${CONTROLLER_WS_PORT}`;
            console.log(`Connecting to controller WebSocket: ${wsUrl}`);
            
            try {
                controllerWS = new WebSocket(wsUrl);
                
                controllerWS.onopen = () => {
                    console.log('Controller WebSocket connected');
                    controllerWSRetryCount = 0;
                    vrLog('Controller WS: Connected');
                };
                
                controllerWS.onerror = (error) => {
                    console.error('Controller WebSocket error:', error);
                    vrLog('Controller WS: Error');
                };
                
                controllerWS.onclose = (event) => {
                    console.log(`Controller WebSocket closed. Code: ${event.code}, Reason: ${event.reason}`);
                    controllerWS = null;
                    vrLog('Controller WS: Disconnected');
                    
                    // Retry connection
                    if (controllerWSRetryCount < CONTROLLER_WS_MAX_RETRIES) {
                        controllerWSRetryCount++;
                        setTimeout(initControllerWebSocket, 3000);
                    }
                };
                
                controllerWS.onmessage = (event) => {
                    // Handle any messages from the server if needed
                    console.log('Controller WS message:', event.data);
                };
            } catch (error) {
                console.error('Failed to create controller WebSocket:', error);
                if (controllerWSRetryCount < CONTROLLER_WS_MAX_RETRIES) {
                    controllerWSRetryCount++;
                    setTimeout(initControllerWebSocket, 3000);
                }
            }
        }
        
        function sendControllerData() {
            if (!controllerWS || controllerWS.readyState !== WebSocket.OPEN) return;
            
            const hasValidLeft = leftController.position !== null;
            const hasValidRight = rightController.position !== null;
            const hasValidHeadset = headsetPose.position !== null;
            
            if (hasValidLeft || hasValidRight || hasValidHeadset) {
                const data = {
                    timestamp: Date.now(),
                    leftController: leftController,
                    rightController: rightController,
                    headset: headsetPose
                };
                controllerWS.send(JSON.stringify(data));
            }
        }
        
        // Quaternion to Euler angles conversion (in degrees)
        function quaternionToEuler(q) {
            // Roll (x-axis rotation)
            const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
            const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
            const roll = Math.atan2(sinr_cosp, cosr_cosp);
            
            // Pitch (y-axis rotation)
            const sinp = 2 * (q.w * q.y - q.z * q.x);
            let pitch;
            if (Math.abs(sinp) >= 1) {
                pitch = Math.sign(sinp) * Math.PI / 2; // Use 90 degrees if out of range
            } else {
                pitch = Math.asin(sinp);
            }
            
            // Yaw (z-axis rotation)
            const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            const yaw = Math.atan2(siny_cosp, cosy_cosp);
            
            return {
                x: roll * 180 / Math.PI,
                y: pitch * 180 / Math.PI,
                z: yaw * 180 / Math.PI
            };
        }
        
        // Process XR input sources to extract controller data
        function processInputSources(frame, refSpace) {
            if (!xrSession || !xrSession.inputSources) return;
            
            for (const inputSource of xrSession.inputSources) {
                if (inputSource.targetRayMode !== 'tracked-pointer') continue;
                
                const handedness = inputSource.handedness; // 'left' or 'right'
                const controller = handedness === 'left' ? leftController : rightController;
                
                // Get grip pose (hand position/rotation)
                const gripSpace = inputSource.gripSpace;
                if (gripSpace) {
                    const gripPose = frame.getPose(gripSpace, refSpace);
                    if (gripPose) {
                        const pos = gripPose.transform.position;
                        const ori = gripPose.transform.orientation;
                        
                        controller.position = { x: pos.x, y: pos.y, z: pos.z };
                        controller.quaternion = { x: ori.x, y: ori.y, z: ori.z, w: ori.w };
                        controller.rotation = quaternionToEuler(controller.quaternion);
                    }
                }
                
                // Get gamepad data (buttons, triggers, thumbsticks)
                const gamepad = inputSource.gamepad;
                if (gamepad) {
                    // Trigger (index 0)
                    if (gamepad.buttons[0]) {
                        controller.trigger = gamepad.buttons[0].value || 0;
                    }
                    
                    // Squeeze/Grip (index 1)
                    const isGripPressed = gamepad.buttons[1]?.pressed || false;
                    
                    // Track grip state changes
                    if (handedness === 'left') {
                        if (isGripPressed && !leftGripDown) {
                            leftGripDown = true;
                            leftGripInitialQuaternion = controller.quaternion ? { ...controller.quaternion } : null;
                        } else if (!isGripPressed && leftGripDown) {
                            leftGripDown = false;
                            leftGripInitialQuaternion = null;
                            // Send grip release message
                            if (controllerWS && controllerWS.readyState === WebSocket.OPEN) {
                                controllerWS.send(JSON.stringify({ hand: 'left', gripReleased: true }));
                            }
                        }
                        controller.gripActive = leftGripDown;
                    } else {
                        if (isGripPressed && !rightGripDown) {
                            rightGripDown = true;
                            rightGripInitialQuaternion = controller.quaternion ? { ...controller.quaternion } : null;
                        } else if (!isGripPressed && rightGripDown) {
                            rightGripDown = false;
                            rightGripInitialQuaternion = null;
                            // Send grip release message
                            if (controllerWS && controllerWS.readyState === WebSocket.OPEN) {
                                controllerWS.send(JSON.stringify({ hand: 'right', gripReleased: true }));
                            }
                        }
                        controller.gripActive = rightGripDown;
                    }
                    
                    // Thumbstick axes (axes 2, 3)
                    controller.thumbstick = {
                        x: gamepad.axes[2] || 0,
                        y: gamepad.axes[3] || 0
                    };
                    
                    // Buttons
                    if (handedness === 'left') {
                        controller.buttons = {
                            squeeze: isGripPressed,
                            thumbstick: !!gamepad.buttons[3]?.pressed,
                            x: !!gamepad.buttons[4]?.pressed,
                            y: !!gamepad.buttons[5]?.pressed,
                            menu: !!gamepad.buttons[6]?.pressed
                        };
                    } else {
                        controller.buttons = {
                            squeeze: isGripPressed,
                            thumbstick: !!gamepad.buttons[3]?.pressed,
                            a: !!gamepad.buttons[4]?.pressed,
                            b: !!gamepad.buttons[5]?.pressed,
                            menu: !!gamepad.buttons[6]?.pressed
                        };
                    }
                }
            }
        }
        
        // Process headset pose
        function processHeadsetPose(pose) {
            if (!pose || !pose.transform) return;
            
            const pos = pose.transform.position;
            const ori = pose.transform.orientation;
            
            headsetPose.position = { x: pos.x, y: pos.y, z: pos.z };
            headsetPose.quaternion = { x: ori.x, y: ori.y, z: ori.z, w: ori.w };
            headsetPose.rotation = quaternionToEuler(headsetPose.quaternion);
        }
        
        // Use thumbsticks to adjust stereo settings in VR
        function processThumbstickAdjustments() {
            // Left thumbstick Y: vertical offset - DISABLED to prevent conflict with robot control
            /*
            if (Math.abs(leftController.thumbstick?.y || 0) > 0.1) {
                stereoSettings.verticalOffset += leftController.thumbstick.y * 0.005;
                stereoSettings.verticalOffset = Math.max(-1, Math.min(1, stereoSettings.verticalOffset));
                reinitQuadBuffers();
            }
            */
            
            // Right thumbstick X: IPD offset - DISABLED
            /*
            if (Math.abs(rightController.thumbstick?.x || 0) > 0.1) {
                stereoSettings.ipdOffset += rightController.thumbstick.x * 0.0005;
                stereoSettings.ipdOffset = Math.max(-0.05, Math.min(0.05, stereoSettings.ipdOffset));
            }
            */
            
            // Triggers: screen distance - DISABLED
            /*
            if ((leftController.trigger || 0) > 0.5) {
                stereoSettings.screenDistance -= 0.02;
                stereoSettings.screenDistance = Math.max(0.5, stereoSettings.screenDistance);
            }
            if ((rightController.trigger || 0) > 0.5) {
                stereoSettings.screenDistance += 0.02;
                stereoSettings.screenDistance = Math.max(0.5, Math.min(5, stereoSettings.screenDistance));
            }
            */
        }
        
        // ========================================
        // End Controller WebSocket Functions
        // ========================================
        
        // Advanced optimization: Use video element directly as texture source
        let useDirectVideoTexture = false;  // TEMPORARILY DISABLED - use canvas mode for debugging
        let videoTexture = null;  // Single texture for entire video
        
        // FPS overlay for immersive VR mode
        let fpsOverlayCanvas = null;
        let fpsOverlayCtx = null;
        let fpsOverlayTexture = null;
        let currentVrFps = 0;  // This now tracks RECEIVED frame rate
        let fpsOverlayShader = null;
        let fpsOverlayPositionBuffer = null;
        let fpsOverlayTexCoordBuffer = null;
        let fpsOverlayCachedLocations = null;
        
        // Battery state for VR overlay
        let currentBatteryLevel = null;
        let currentBatteryCharging = false;
        
        // Status panel for VR mode (head-locked, right side of 3D view)
        let statusPanelCanvas = null;
        let statusPanelCtx = null;
        let statusPanelTexture = null;
        let statusPanelPositionBuffer = null;
        let statusPanelTexCoordBuffer = null;
        let statusPanelInitialized = false;
        
        // Left status panel for VR mode (head-locked, left side of 3D view)
        let leftStatusPanelCanvas = null;
        let leftStatusPanelCtx = null;
        let leftStatusPanelTexture = null;
        let leftStatusPanelPositionBuffer = null;
        let leftStatusPanelTexCoordBuffer = null;
        let leftStatusPanelInitialized = false;
        
        // Received frame FPS tracking
        let receivedFrameCount = 0;
        let lastReceivedFpsTime = performance.now();
        let lastFrameReceivedTimestamp = 0;  // When the current frame was received
        
        // Latency tracking
        let lastFrameTime = 0;
        let frameCount = 0;
        let totalLatency = 0;
        
        // Stereo adjustment settings (optimized defaults)
        let stereoSettings = {
            verticalOffset: -0.17,  // Meters - positive = up
            ipdOffset: -0.018,      // Meters - adjustment to convergence
            screenDistance: 0.6,    // Meters - distance to virtual screen
            screenScale: 0.5        // Multiplier for screen size
        };
        
        // Reinitialize quad buffers when size changes - now invalidates hash to trigger update
        function reinitQuadBuffers() {
            // Invalidate the hash so updatePositionBufferIfNeeded() will rebuild
            lastSettingsHash = '';
        }
        
        // Status display
        function setStatus(text, type = '') {
            const status = document.getElementById('status');
            status.textContent = text;
            status.className = type;
        }
        
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
        
        // VR Debug logging - shows on screen in headset
        function vrLog(message) {
            console.log(message);
            const debugDiv = document.getElementById('vrDebug');
            if (debugDiv) {
                const lines = debugDiv.innerHTML.split('<br>');
                lines.push(new Date().toLocaleTimeString() + ': ' + message);
                if (lines.length > 15) lines.shift();  // Keep last 15 lines
                debugDiv.innerHTML = lines.join('<br>');
            }
        }
        
        // Initialize Foxglove WebSocket connection using FoxgloveConnection class
        function initFoxgloveConnection() {
            setStatus('Connecting to stereo stream...');
            
            foxgloveConn = new FoxgloveConnection({
                reconnectInterval: 2000,
                onConnect: () => {
                    setStatus('Connected to bridge, waiting for topics...', 'connected');
                },
                onDisconnect: (event) => {
                    setStatus('Connection closed, reconnecting...', 'error');
                    if (event.code === 1006) {
                        showCertButton();
                    }
                },
                onAdvertise: (channels) => {
                    subscribeToCompressedImage(channels);
                    subscribeToRobotTopics(channels);
                }
            });
            
            foxgloveConn.connect();
        }

        function subscribeToCompressedImage(channels) {
            const topic = '/alfie/stereo_camera/image_raw/compressed';
            
            // Use the FoxgloveConnection's subscribe with custom handler
            compressedImageSubscriptionId = foxgloveConn.subscribe(topic, (messageData) => {
                const view = new DataView(messageData.buffer, messageData.byteOffset, messageData.byteLength);
                processCompressedImage(view);
            });
            
            if (compressedImageSubscriptionId) {
                setStatus('Subscribed to stereo stream', 'connected');
            } else {
                console.log('Compressed image topic not found in advertised channels');
                console.log('Available topics:', channels.map(c => c.topic).join(', '));
                setStatus('Topic not found - is camera running?', 'error');
            }
        }

        // ========================================
        // Robot Topic Subscriptions (3.3)
        // ========================================
        
        function subscribeToRobotTopics(channels) {
            let subscribedCount = 0;
            
            // Subscribe to robot_description (3.3.1)
            robotDescriptionSubscriptionId = foxgloveConn.subscribe('/alfie/robot_description', (messageData) => {
                handleRobotDescriptionMessage(messageData);
            });
            if (robotDescriptionSubscriptionId) {
                console.log('Subscribed to /alfie/robot_description');
                subscribedCount++;
            }
            
            // Subscribe to TF (3.3.2)
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
        // CDR Binary Message Decoders (3.3.3)
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
                
                // Update TF rate every 5 seconds (3.4.1)
                const now = performance.now();
                const elapsed = (now - lastTfTime) / 1000;
                if (elapsed >= 5.0) {
                    currentTfRate = tfCount / elapsed;
                    tfCount = 0;
                    lastTfTime = now;
                    // Update desktop panel (3.4.2)
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
        
        // Get current TF rate (for VR status panel)
        function getTfRate() {
            return currentTfRate;
        }
        
        // Update UI elements
        function updateURDFStatus(status) {
            const el = document.getElementById('urdfStatus');
            if (el) el.textContent = status;
        }
        
        function updateLinkCount(count) {
            const el = document.getElementById('linkCount');
            if (el) el.textContent = count.toString();
        }

        async function processCompressedImage(view) {
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
                    setStatus('Stereo stream received', 'connected');
                    document.getElementById('vrButton').disabled = false;
                    document.getElementById('certButton').style.display = 'none';
                    startPreview();
                }
                
            } catch (error) {
                console.error('Error processing compressed image:', error);
            }
        }
        
        // Show preview of stereo stream (non-VR mode)
        function startPreview() {
            const preview = document.getElementById('preview');
            
            // Create canvas for preview
            const canvas = document.createElement('canvas');
            const ctx = canvas.getContext('2d');
            
            let lastTime = performance.now();
            let frames = 0;

            function updatePreview() {
                // Calculate FPS
                const now = performance.now();
                frames++;
                if (now - lastTime >= 1000) {
                    const fps = (frames / (now - lastTime)) * 1000;
                    const fpsElement = document.getElementById('fpsCounter');
                    if (fpsElement) {
                        fpsElement.textContent = `FPS: ${fps.toFixed(1)}`;
                    }
                    lastTime = now;
                    frames = 0;
                }

                if (currentFrameBitmap) {
                    if (canvas.width !== currentFrameBitmap.width || canvas.height !== currentFrameBitmap.height) {
                        canvas.width = currentFrameBitmap.width;
                        canvas.height = currentFrameBitmap.height;
                    }
                    ctx.drawImage(currentFrameBitmap, 0, 0);
                    preview.src = canvas.toDataURL('image/jpeg', 0.8);
                    preview.classList.remove('hidden');
                }
                if (!xrSession) {
                    requestAnimationFrame(updatePreview);
                }
            }
            
            updatePreview();
        }
        
        // Initialize WebXR for VR display
        async function initXR() {
            if (!navigator.xr) {
                setStatus('WebXR not supported', 'error');
                return;
            }
            
            const supported = await navigator.xr.isSessionSupported('immersive-ar');
            if (!supported) {
                setStatus('Immersive AR not supported', 'error');
                document.getElementById('vrButton').disabled = true;
                return;
            }
            
            document.getElementById('vrButton').addEventListener('click', startVRSession);
        }
        
        // Start immersive VR session
        async function startVRSession() {
            try {
                setStatus('Starting VR session...');
                vrLog('Starting VR session...');
                
                // Request immersive AR session with local-floor reference space and passthrough
                xrSession = await navigator.xr.requestSession('immersive-ar', {
                    requiredFeatures: ['local-floor'],
                    optionalFeatures: ['hand-tracking', 'dom-overlay'],
                    domOverlay: { root: document.body }
                });
                
                vrLog('XR session created');
                
                // Show debug overlay in VR
                document.getElementById('vrDebug').style.display = 'block';
                
                xrSession.addEventListener('end', onSessionEnd);
                
                // Set up WebGL context for XR - optimized for performance
                const canvas = document.createElement('canvas');
                const glOptions = { 
                    xrCompatible: true,
                    antialias: false,           // Disable AA for performance
                    depth: true,                // Enable depth buffer for WebXR
                    stencil: false,             // No stencil needed
                    powerPreference: 'high-performance',
                    preserveDrawingBuffer: false
                };
                gl = canvas.getContext('webgl2', glOptions);
                
                if (!gl) {
                    gl = canvas.getContext('webgl', glOptions);
                }
                
                if (!gl) {
                    throw new Error('Failed to create WebGL context');
                }
                
                console.log('WebGL context created:', gl.getParameter(gl.VERSION));
                vrLog('WebGL: ' + gl.getParameter(gl.VERSION));
                
                await xrSession.updateRenderState({
                    baseLayer: new XRWebGLLayer(xrSession, gl, {
                        antialias: false,       // Faster rendering
                        depth: true,            // Enable depth buffer
                        stencil: false,         // No stencil needed
                        alpha: true,            // Enable alpha for proper blending
                        framebufferScaleFactor: 1.0  // Native resolution
                    })
                });
                
                // Enable depth testing and blending
                gl.enable(gl.DEPTH_TEST);
                gl.depthFunc(gl.LEQUAL);
                gl.enable(gl.BLEND);
                gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
                
                console.log('Render state configured');
                vrLog('Depth/blend enabled');
                
                xrRefSpace = await xrSession.requestReferenceSpace('local-floor');
                vrLog('Got reference space');
                
                // Create textures for left and right eye views
                leftTexture = gl.createTexture();
                rightTexture = gl.createTexture();
                vrLog('Textures created');
                
                // Set up canvases for extracting left/right from video
                // Use willReadFrequently for GPU-accelerated canvas operations
                leftCanvas = document.getElementById('leftCanvas');
                rightCanvas = document.getElementById('rightCanvas');
                leftCtx = leftCanvas.getContext('2d', { willReadFrequently: false, desynchronized: true });
                rightCtx = rightCanvas.getContext('2d', { willReadFrequently: false, desynchronized: true });
                
                console.log('WebXR initialized successfully');
                console.log('Using direct video texture mode:', useDirectVideoTexture);
                vrLog('Canvas mode: ' + (!useDirectVideoTexture ? 'YES' : 'NO'));
                
                vrLog('Video ready: ' + (currentFrameBitmap ? 'YES' : 'NO'));
                if (currentFrameBitmap) {
                    vrLog('Video: ' + currentFrameBitmap.width + 'x' + currentFrameBitmap.height);
                }
                
                document.getElementById('preview').classList.add('hidden');
                document.getElementById('vrButton').textContent = '🛑 Exit AR';
                document.getElementById('vrButton').onclick = () => xrSession.end();
                
                setStatus('AR Active - Stereo 3D Vision (95% opacity)', 'connected');
                
                // Start XR render loop
                xrSession.requestAnimationFrame(onXRFrame);
                
            } catch (err) {
                console.error('VR session error:', err);
                setStatus('VR error: ' + err.message, 'error');
            }
        }
        
        // Pre-allocated view matrix to avoid per-frame allocation
        const viewMatrixBuffer = new Float32Array(16);
        
        // Initialize textures once with proper parameters
        function initTexture(texture) {
            gl.bindTexture(gl.TEXTURE_2D, texture);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
        }
        
        let texturesInitialized = false;
        let lastVideoWidth = 0;
        let lastVideoHeight = 0;
        
        // XR render loop - optimized for low latency
        let frameCounter = 0;

        function onXRFrame(time, frame) {
            if (!xrSession) return;
            
            xrSession.requestAnimationFrame(onXRFrame);
            
            const pose = frame.getViewerPose(xrRefSpace);
            if (!pose) {
                if (frameCounter % 60 === 0) {
                    console.log('No pose available');
                    vrLog('No pose');
                }
                frameCounter++;
                return;
            }
            
            if (frameCounter === 5) {
                vrLog('Pose OK, views: ' + pose.views.length);
            }
            
            // Process controller and headset tracking
            processInputSources(frame, xrRefSpace);
            processHeadsetPose(pose);
            processThumbstickAdjustments();
            
            // Send controller data to WebSocket (throttled - every 3 frames)
            if (frameCounter % 3 === 0) {
                sendControllerData();
            }
            
            const glLayer = xrSession.renderState.baseLayer;
            gl.bindFramebuffer(gl.FRAMEBUFFER, glLayer.framebuffer);
            
            // Clear with 95% opacity black to dim passthrough (5% passthrough visible)
            gl.clearColor(0.0, 0.0, 0.0, 0.95);  // 95% opacity for immersive AR dimming
            gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
            
            if (!currentFrameBitmap) {
                if (frameCounter % 60 === 0) {
                    console.log('Waiting for video frame');
                    vrLog('Wait video');
                }
                frameCounter++;
                // Still render FPS overlay and status panels while waiting for video
                for (const view of pose.views) {
                    const viewport = glLayer.getViewport(view);
                    gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
                    drawFpsOverlay(view, viewport);
                    drawStatusPanel(view, viewport, pose.transform.matrix);
                    drawLeftStatusPanel(view, viewport, pose.transform.matrix);
                }
                return;
            }
            
            frameCounter++;
            if (frameCounter === 10) {
                console.log('Video ready, rendering. Size:', currentFrameBitmap.width, 'x', currentFrameBitmap.height);
                console.log('Pose views:', pose.views.length);
                vrLog('Rendering video: ' + currentFrameBitmap.width + 'x' + currentFrameBitmap.height);
            }
            
            // ULTRA-LOW LATENCY MODE: Use video element directly as texture
            // This completely bypasses canvas operations and their associated latency
            if (useDirectVideoTexture) {
                if (!videoTexture) {
                    videoTexture = gl.createTexture();
                    gl.bindTexture(gl.TEXTURE_2D, videoTexture);
                    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
                    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
                    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
                    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
                    console.log('Created video texture for direct mode');
                }
                
                // Upload video texture once per frame (shared by both eyes)
                gl.bindTexture(gl.TEXTURE_2D, videoTexture);
                try {
                    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, currentFrameBitmap);
                    
                    if (frameCounter === 10) {
                        console.log('Video texture upload successful, size:', currentFrameBitmap.width, 'x', currentFrameBitmap.height);
                    }
                } catch (e) {
                    console.error('Failed to upload video texture:', e);
                    useDirectVideoTexture = false;  // Fall back to canvas mode
                    return;
                }
                
                // Render both eyes using shader-based splitting
                for (const view of pose.views) {
                    const viewport = glLayer.getViewport(view);
                    gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
                    
                    // Use the same video texture for both eyes, shader will split it
                    gl.bindTexture(gl.TEXTURE_2D, videoTexture);
                    drawTexturedQuad(view, viewport, true, pose.transform.matrix);  // true = use direct video mode
                    
                    // Draw FPS overlay in upper left corner
                    drawFpsOverlay(view, viewport);
                    
                    // Draw status panels (head-locked)
                    drawStatusPanel(view, viewport, pose.transform.matrix);
                    drawLeftStatusPanel(view, viewport, pose.transform.matrix);
                }
            } else {
                // FALLBACK: Use canvas-based splitting (higher latency)
                const videoWidth = currentFrameBitmap.width;
                const videoHeight = currentFrameBitmap.height;
                const halfWidth = videoWidth / 2;
                
                if (frameCounter === 10) {
                    console.log('Using canvas-based rendering mode');
                    console.log('Video dimensions:', videoWidth, 'x', videoHeight);
                    vrLog('Canvas render mode');
                }
                
                if (!texturesInitialized) {
                    initTexture(leftTexture);
                    initTexture(rightTexture);
                    texturesInitialized = true;
                    console.log('Initialized left/right textures');
                    vrLog('Init L/R textures');
                }
                
                if (leftCanvas.width !== halfWidth || leftCanvas.height !== videoHeight) {
                    leftCanvas.width = halfWidth;
                    leftCanvas.height = videoHeight;
                    rightCanvas.width = halfWidth;
                    rightCanvas.height = videoHeight;
                    if (frameCounter < 20) {
                        console.log('Canvas size set to:', halfWidth, 'x', videoHeight);
                        vrLog('Canvas: ' + halfWidth + 'x' + videoHeight);
                    }
                }
                
                // Draw left and right halves to canvases
                leftCtx.drawImage(currentFrameBitmap, 0, 0, halfWidth, videoHeight, 0, 0, halfWidth, videoHeight);
                rightCtx.drawImage(currentFrameBitmap, halfWidth, 0, halfWidth, videoHeight, 0, 0, halfWidth, videoHeight);
                
                // Calculate frame age (how old is the current frame)
                const frameAge = performance.now() - lastFrameReceivedTimestamp;
                const lagColor = frameAge > 150 ? '#ff4444' : '#00ff00';  // Red if > 150ms, green otherwise
                
                // FPS color coding: green >= 25, yellow 15-25, red < 15
                let fpsColor;
                if (currentVrFps >= 25) {
                    fpsColor = '#00ff00';  // Green
                } else if (currentVrFps >= 15) {
                    fpsColor = '#ffff00';  // Yellow
                } else {
                    fpsColor = '#ff4444';  // Red
                }
                
                // Get headset rotation values (default to 0 if not available)
                // WebXR coordinate system with right-hand rule:
                // X-axis = right, Y-axis = up, Z-axis = back
                // Pitch = rotation around X (nodding), Yaw = rotation around Y (turning), Roll = rotation around Z (tilting)
                const pitch = headsetPose.rotation ? Math.round(headsetPose.rotation.x) : 0;
                const yaw = headsetPose.rotation ? Math.round(headsetPose.rotation.y) : 0;
                const roll = headsetPose.rotation ? Math.round(headsetPose.rotation.z) : 0;
                
                // Upload to WebGL textures (no overlay - using floating panels instead)
                gl.bindTexture(gl.TEXTURE_2D, leftTexture);
                gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, leftCanvas);
                
                gl.bindTexture(gl.TEXTURE_2D, rightTexture);
                gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, rightCanvas);
                
                if (frameCounter === 11) {
                    console.log('Textures uploaded, rendering to eyes');
                    vrLog('Drawing to eyes...');
                }
                
                // Render to both eyes
                for (const view of pose.views) {
                    const viewport = glLayer.getViewport(view);
                    gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
                    
                    if (frameCounter === 12) {
                        console.log(`Rendering ${view.eye} eye, viewport:`, viewport.width, 'x', viewport.height);
                        vrLog(view.eye + ' eye: ' + viewport.width + 'x' + viewport.height);
                    }
                    
                    const texture = view.eye === 'left' ? leftTexture : rightTexture;
                    gl.bindTexture(gl.TEXTURE_2D, texture);
                    drawTexturedQuad(view, viewport, false, pose.transform.matrix);  // false = use canvas mode
                    
                    // Draw FPS overlay in upper left corner
                    drawFpsOverlay(view, viewport);
                    
                    // Draw status panels (head-locked)
                    drawStatusPanel(view, viewport, pose.transform.matrix);
                    drawLeftStatusPanel(view, viewport, pose.transform.matrix);
                }
            }
        }
        
        // ========================================
        // FPS Overlay Functions for Immersive VR
        // ========================================
        
        function initFpsOverlay() {
            if (!gl) {
                vrLog('FPS overlay: no GL context');
                return;
            }
            
            vrLog('Init FPS overlay...');
            
            // Create canvas for FPS text (larger for better visibility)
            fpsOverlayCanvas = document.createElement('canvas');
            fpsOverlayCanvas.width = 256;
            fpsOverlayCanvas.height = 64;
            fpsOverlayCtx = fpsOverlayCanvas.getContext('2d');
            
            // Create texture
            fpsOverlayTexture = gl.createTexture();
            gl.bindTexture(gl.TEXTURE_2D, fpsOverlayTexture);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
            
            // Initial update
            updateFpsOverlayCanvas();
            
            // Initialize overlay shader
            initFpsOverlayShader();
            
            vrLog('FPS overlay: ' + (fpsOverlayShader ? 'OK' : 'FAIL'));
        }
        
        function updateFpsOverlayCanvas() {
            if (!fpsOverlayCtx) return;
            
            // Clear canvas
            fpsOverlayCtx.clearRect(0, 0, fpsOverlayCanvas.width, fpsOverlayCanvas.height);
            
            // Bright red background for high visibility (testing)
            fpsOverlayCtx.fillStyle = 'rgba(200, 0, 0, 0.8)';
            fpsOverlayCtx.fillRect(0, 0, fpsOverlayCanvas.width, fpsOverlayCanvas.height);
            
            // White border
            fpsOverlayCtx.strokeStyle = '#ffffff';
            fpsOverlayCtx.lineWidth = 3;
            fpsOverlayCtx.strokeRect(2, 2, fpsOverlayCanvas.width - 4, fpsOverlayCanvas.height - 4);
            
            // Draw FPS text in white
            fpsOverlayCtx.fillStyle = '#ffffff';
            fpsOverlayCtx.font = 'bold 24px monospace';
            fpsOverlayCtx.textBaseline = 'middle';
            fpsOverlayCtx.textAlign = 'center';
            
            // Build display text with FPS and battery
            let displayText = `FPS: ${currentVrFps.toFixed(1)}`;
            if (currentBatteryLevel !== null) {
                const batteryIcon = currentBatteryCharging ? 'CHG' : 'BAT';
                displayText += ` | ${batteryIcon}: ${currentBatteryLevel}%`;
            }
            fpsOverlayCtx.fillText(displayText, fpsOverlayCanvas.width / 2, fpsOverlayCanvas.height / 2);
            
            // Update texture if GL context is available
            if (gl && fpsOverlayTexture) {
                gl.bindTexture(gl.TEXTURE_2D, fpsOverlayTexture);
                gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, fpsOverlayCanvas);
            }
        }
        
        function initFpsOverlayShader() {
            if (!gl) return;
            
            // Simple 2D overlay shader (screen-space, no 3D transforms)
            const vertexShader = gl.createShader(gl.VERTEX_SHADER);
            gl.shaderSource(vertexShader, `
                attribute vec2 a_position;
                attribute vec2 a_texCoord;
                varying vec2 v_texCoord;
                
                void main() {
                    gl_Position = vec4(a_position, 0.0, 1.0);
                    v_texCoord = a_texCoord;
                }
            `);
            gl.compileShader(vertexShader);
            
            if (!gl.getShaderParameter(vertexShader, gl.COMPILE_STATUS)) {
                console.error('FPS overlay vertex shader error:', gl.getShaderInfoLog(vertexShader));
                return;
            }
            
            const fragmentShader = gl.createShader(gl.FRAGMENT_SHADER);
            gl.shaderSource(fragmentShader, `
                precision mediump float;
                varying vec2 v_texCoord;
                uniform sampler2D u_texture;
                
                void main() {
                    gl_FragColor = texture2D(u_texture, v_texCoord);
                }
            `);
            gl.compileShader(fragmentShader);
            
            if (!gl.getShaderParameter(fragmentShader, gl.COMPILE_STATUS)) {
                console.error('FPS overlay fragment shader error:', gl.getShaderInfoLog(fragmentShader));
                return;
            }
            
            fpsOverlayShader = gl.createProgram();
            gl.attachShader(fpsOverlayShader, vertexShader);
            gl.attachShader(fpsOverlayShader, fragmentShader);
            gl.linkProgram(fpsOverlayShader);
            
            if (!gl.getProgramParameter(fpsOverlayShader, gl.LINK_STATUS)) {
                vrLog('FPS shader link FAIL');
                fpsOverlayShader = null;
                return;
            }
            
            vrLog('FPS shader linked');
            
            // Create buffers for overlay quad (upper left corner, screen space coords)
            // Screen space: -1 to 1, so upper left is around (-1, 1)
            // Make it LARGE for testing visibility
            const overlayWidth = 0.5;    // 25% of screen width (increased)
            const overlayHeight = 0.15;  // 7.5% of screen height (increased)
            const marginX = -0.95;       // Left margin
            const marginY = 0.95;        // Top margin
            
            fpsOverlayPositionBuffer = gl.createBuffer();
            gl.bindBuffer(gl.ARRAY_BUFFER, fpsOverlayPositionBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
                marginX, marginY - overlayHeight,
                marginX + overlayWidth, marginY - overlayHeight,
                marginX, marginY,
                marginX + overlayWidth, marginY,
            ]), gl.STATIC_DRAW);
            
            console.log('FPS overlay position buffer created, bounds:', marginX, marginY - overlayHeight, 'to', marginX + overlayWidth, marginY);
            
            fpsOverlayTexCoordBuffer = gl.createBuffer();
            gl.bindBuffer(gl.ARRAY_BUFFER, fpsOverlayTexCoordBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
                0, 1,
                1, 1,
                0, 0,
                1, 0,
            ]), gl.STATIC_DRAW);
            
            // Cache locations
            fpsOverlayCachedLocations = {
                position: gl.getAttribLocation(fpsOverlayShader, 'a_position'),
                texCoord: gl.getAttribLocation(fpsOverlayShader, 'a_texCoord'),
                texture: gl.getUniformLocation(fpsOverlayShader, 'u_texture')
            };
        }
        
        function drawFpsOverlay(view, viewport) {
            // Initialize on first call
            if (!fpsOverlayCanvas) {
                initFpsOverlay();
            }
            
            // Check if initialization succeeded
            if (!fpsOverlayShader || !fpsOverlayTexture || !fpsOverlayCachedLocations) {
                if (frameCounter < 20 && frameCounter % 5 === 0) {
                    vrLog('FPS ovl not ready');
                }
                return;
            }
            
            if (frameCounter === 15) {
                vrLog('Drawing FPS overlay');
            }
            
            // Save current WebGL state
            const prevProgram = gl.getParameter(gl.CURRENT_PROGRAM);
            
            // Enable blending for transparency
            gl.enable(gl.BLEND);
            gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
            
            // Disable depth test so overlay is always visible
            gl.disable(gl.DEPTH_TEST);
            
            gl.useProgram(fpsOverlayShader);
            
            // Disable any previously enabled vertex attrib arrays that might interfere
            for (let i = 0; i < 8; i++) {
                gl.disableVertexAttribArray(i);
            }
            
            // Bind position buffer
            gl.bindBuffer(gl.ARRAY_BUFFER, fpsOverlayPositionBuffer);
            gl.enableVertexAttribArray(fpsOverlayCachedLocations.position);
            gl.vertexAttribPointer(fpsOverlayCachedLocations.position, 2, gl.FLOAT, false, 0, 0);
            
            // Bind texcoord buffer
            gl.bindBuffer(gl.ARRAY_BUFFER, fpsOverlayTexCoordBuffer);
            gl.enableVertexAttribArray(fpsOverlayCachedLocations.texCoord);
            gl.vertexAttribPointer(fpsOverlayCachedLocations.texCoord, 2, gl.FLOAT, false, 0, 0);
            
            // Bind texture
            gl.activeTexture(gl.TEXTURE0);
            gl.bindTexture(gl.TEXTURE_2D, fpsOverlayTexture);
            gl.uniform1i(fpsOverlayCachedLocations.texture, 0);
            
            // Draw overlay
            gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
            
            if (frameCounter === 15) {
                console.log('FPS overlay draw complete');
            }
            
            // Restore state
            gl.disable(gl.BLEND);
            gl.enable(gl.DEPTH_TEST);
            
            // Restore previous program
            if (prevProgram) {
                gl.useProgram(prevProgram);
            }
        }
        
        // ========================================
        // End FPS Overlay Functions
        // ========================================
        
        // ========================================
        // Status Panel Functions (Head-Locked 3D Quad)
        // ========================================
        
        // Helper function to draw a rounded rectangle
        function drawRoundedRect(ctx, x, y, width, height, radius) {
            ctx.beginPath();
            ctx.moveTo(x + radius, y);
            ctx.lineTo(x + width - radius, y);
            ctx.arcTo(x + width, y, x + width, y + radius, radius);
            ctx.lineTo(x + width, y + height - radius);
            ctx.arcTo(x + width, y + height, x + width - radius, y + height, radius);
            ctx.lineTo(x + radius, y + height);
            ctx.arcTo(x, y + height, x, y + height - radius, radius);
            ctx.lineTo(x, y + radius);
            ctx.arcTo(x, y, x + radius, y, radius);
            ctx.closePath();
            ctx.fill();
        }
        
        // Pre-allocated buffers for panel rotation matrices
        const rightPanelModelMatrix = new Float32Array(16);
        const leftPanelModelMatrix = new Float32Array(16);
        
        // Helper function to create a rotated model matrix (yaw rotation around Y axis)
        function createRotatedModelMatrix(baseMatrix, yawDegrees, outMatrix) {
            const yawRad = yawDegrees * Math.PI / 180;
            const cosY = Math.cos(yawRad);
            const sinY = Math.sin(yawRad);
            
            // Rotation matrix around Y axis
            // [cosY  0  sinY  0]
            // [0     1  0     0]
            // [-sinY 0  cosY  0]
            // [0     0  0     1]
            
            // Multiply baseMatrix * rotationMatrix
            const m = baseMatrix;
            
            outMatrix[0] = m[0] * cosY + m[2] * (-sinY);
            outMatrix[1] = m[1] * cosY + m[3] * (-sinY);
            outMatrix[2] = m[0] * sinY + m[2] * cosY;
            outMatrix[3] = m[3];
            
            outMatrix[4] = m[4] * cosY + m[6] * (-sinY);
            outMatrix[5] = m[5] * cosY + m[7] * (-sinY);
            outMatrix[6] = m[4] * sinY + m[6] * cosY;
            outMatrix[7] = m[7];
            
            outMatrix[8] = m[8] * cosY + m[10] * (-sinY);
            outMatrix[9] = m[9] * cosY + m[11] * (-sinY);
            outMatrix[10] = m[8] * sinY + m[10] * cosY;
            outMatrix[11] = m[11];
            
            outMatrix[12] = m[12] * cosY + m[14] * (-sinY);
            outMatrix[13] = m[13] * cosY + m[15] * (-sinY);
            outMatrix[14] = m[12] * sinY + m[14] * cosY;
            outMatrix[15] = m[15];
        }
        
        function initStatusPanel() {
            if (!gl) {
                vrLog('Status panel: no GL context');
                return;
            }
            
            vrLog('Init status panel...');
            
            // Create canvas for status text
            statusPanelCanvas = document.createElement('canvas');
            statusPanelCanvas.width = 256;
            statusPanelCanvas.height = 96;
            statusPanelCtx = statusPanelCanvas.getContext('2d');
            
            // Create texture
            statusPanelTexture = gl.createTexture();
            gl.bindTexture(gl.TEXTURE_2D, statusPanelTexture);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
            
            // Create position buffer for 3D quad
            // Position with right edge aligned to right edge of main view
            const mainHeight = stereoSettings.screenScale;
            const mainWidth = mainHeight * (16/9);
            const mainRightEdge = mainWidth / 2;
            
            // Status panel size
            const panelHeight = 0.09;  // 9cm tall in world space
            const panelWidth = 0.20;   // 20cm wide
            
            // Position: nudged left to stay inside 3D view, vertically centered
            const panelLeft = mainRightEdge - panelWidth - 0.175;  // 5cm inward from right edge
            const panelCenterY = stereoSettings.verticalOffset + 0.30;  // Raised significantly to compensate for rotation
            
            statusPanelPositionBuffer = gl.createBuffer();
            gl.bindBuffer(gl.ARRAY_BUFFER, statusPanelPositionBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
                panelLeft, panelCenterY - panelHeight/2,
                panelLeft + panelWidth, panelCenterY - panelHeight/2,
                panelLeft, panelCenterY + panelHeight/2,
                panelLeft + panelWidth, panelCenterY + panelHeight/2,
            ]), gl.STATIC_DRAW);
            
            statusPanelTexCoordBuffer = gl.createBuffer();
            gl.bindBuffer(gl.ARRAY_BUFFER, statusPanelTexCoordBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
                0, 1,
                1, 1,
                0, 0,
                1, 0,
            ]), gl.STATIC_DRAW);
            
            // Initial update
            updateStatusPanelCanvas();
            
            statusPanelInitialized = true;
            vrLog('Status panel: OK');
        }
        
        function updateStatusPanelCanvas() {
            if (!statusPanelCtx) return;
            
            const canvas = statusPanelCanvas;
            const ctx = statusPanelCtx;
            
            // Clear canvas
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            
            // Semi-transparent dark background with rounded corners (50% opaque)
            ctx.fillStyle = 'rgba(20, 20, 30, 0.5)';
            drawRoundedRect(ctx, 0, 0, canvas.width, canvas.height, 12);
            
            // Title
            ctx.fillStyle = '#88ccff';
            ctx.font = 'bold 14px monospace';
            ctx.textBaseline = 'top';
            ctx.fillText('Robot Status', 10, 8);
            
            // Foxglove connection status
            const foxgloveStatus = foxgloveConn && foxgloveConn.isConnected ? 'Connected' : 'Disconnected';
            const foxgloveColor = foxgloveConn && foxgloveConn.isConnected ? '#44ff44' : '#ff4444';
            ctx.fillStyle = '#cccccc';
            ctx.font = '11px monospace';
            ctx.fillText('Foxglove:', 10, 28);
            ctx.fillStyle = foxgloveColor;
            ctx.fillText(foxgloveStatus, 80, 28);
            
            // TF Rate
            const tfRate = getTfRate();
            const tfColor = tfRate > 10 ? '#44ff44' : (tfRate > 0 ? '#ffff00' : '#ff4444');
            ctx.fillStyle = '#cccccc';
            ctx.fillText('TF Rate:', 10, 43);
            ctx.fillStyle = tfColor;
            ctx.fillText(`${tfRate.toFixed(1)} Hz`, 80, 43);
            
            // Link count
            ctx.fillStyle = '#cccccc';
            ctx.fillText('Links:', 10, 58);
            ctx.fillStyle = '#ffffff';
            ctx.fillText(`${linkNames.length}`, 80, 58);
            
            // URDF status
            ctx.fillStyle = '#cccccc';
            ctx.fillText('URDF:', 10, 73);
            ctx.fillStyle = urdfString ? '#44ff44' : '#888888';
            ctx.fillText(urdfString ? 'Loaded' : 'Waiting...', 80, 73);
            
            // Update texture
            if (gl && statusPanelTexture) {
                gl.bindTexture(gl.TEXTURE_2D, statusPanelTexture);
                gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, canvas);
            }
        }
        
        function drawStatusPanel(view, viewport, modelMatrix) {
            // Initialize on first call
            if (!statusPanelInitialized) {
                initStatusPanel();
            }
            
            if (!statusPanelInitialized || !shaderProgram || !statusPanelTexture || !cachedLocations) {
                if (frameCounter < 20 && frameCounter % 5 === 0) {
                    vrLog(`Status panel skip: init=${statusPanelInitialized}, shader=${!!shaderProgram}, tex=${!!statusPanelTexture}, locs=${!!cachedLocations}`);
                }
                return;
            }
            
            if (frameCounter === 15) {
                vrLog('Drawing status panel');
            }
            
            // Update canvas content periodically (every 30 frames ~ 0.5 sec at 60fps)
            if (frameCounter % 30 === 0) {
                updateStatusPanelCanvas();
            }
            
            // Save current state
            const prevProgram = gl.getParameter(gl.CURRENT_PROGRAM);
            
            // Enable blending for transparency
            // Use blendFuncSeparate to preserve destination alpha (video's alpha=1.0)
            // This prevents AR passthrough from showing through the panel
            gl.enable(gl.BLEND);
            gl.blendFuncSeparate(
                gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA,  // RGB: standard alpha blend
                gl.ZERO, gl.ONE                        // Alpha: preserve destination alpha
            );
            
            // Disable depth test so panel is always visible on top
            gl.disable(gl.DEPTH_TEST);
            
            // Use the main 3D shader (same as video quad)
            gl.useProgram(shaderProgram);
            
            // Set up position buffer (status panel geometry)
            if (cachedLocations.position !== -1 && cachedLocations.position !== null) {
                gl.bindBuffer(gl.ARRAY_BUFFER, statusPanelPositionBuffer);
                gl.enableVertexAttribArray(cachedLocations.position);
                gl.vertexAttribPointer(cachedLocations.position, 2, gl.FLOAT, false, 0, 0);
            }
            
            // Set up tex coord buffer
            if (cachedLocations.texCoord !== -1 && cachedLocations.texCoord !== null) {
                gl.bindBuffer(gl.ARRAY_BUFFER, statusPanelTexCoordBuffer);
                gl.enableVertexAttribArray(cachedLocations.texCoord);
                gl.vertexAttribPointer(cachedLocations.texCoord, 2, gl.FLOAT, false, 0, 0);
            }
            
            // Set uniforms
            if (cachedLocations.projection !== null) {
                gl.uniformMatrix4fv(cachedLocations.projection, false, view.projectionMatrix);
            }
            
            if (cachedLocations.view !== null) {
                invertMatrix(view.transform.matrix, viewMatrixBuffer);
                gl.uniformMatrix4fv(cachedLocations.view, false, viewMatrixBuffer);
            }
            
            if (cachedLocations.model !== null) {
                // Apply yaw rotation to face viewer (Quest 3 coordinate system)
                if (modelMatrix) {
                    createRotatedModelMatrix(modelMatrix, 15, rightPanelModelMatrix);
                    gl.uniformMatrix4fv(cachedLocations.model, false, rightPanelModelMatrix);
                } else {
                    gl.uniformMatrix4fv(cachedLocations.model, false, cachedLocations.identityMatrix);
                }
            }
            
            // Position at same distance as main view so transparency works
            if (cachedLocations.distance !== null) {
                gl.uniform1f(cachedLocations.distance, stereoSettings.screenDistance);
            }
            
            if (cachedLocations.ipdOffset !== null) {
                gl.uniform1f(cachedLocations.ipdOffset, stereoSettings.ipdOffset);  // Match video IPD
            }
            
            if (cachedLocations.isLeftEye !== null) {
                gl.uniform1f(cachedLocations.isLeftEye, view.eye === 'left' ? 1.0 : 0.0);
            }
            
            if (cachedLocations.useDirectVideo !== null) {
                gl.uniform1f(cachedLocations.useDirectVideo, 0.0);  // Not using video splitting
            }
            
            if (cachedLocations.texture !== null) {
                gl.activeTexture(gl.TEXTURE0);
                gl.bindTexture(gl.TEXTURE_2D, statusPanelTexture);
                gl.uniform1i(cachedLocations.texture, 0);
            }
            
            // Draw the panel
            gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
            
            if (frameCounter === 15) {
                vrLog('Status panel draw complete');
            }
            
            // Restore state
            gl.disable(gl.BLEND);
            gl.enable(gl.DEPTH_TEST);
            
            if (prevProgram) {
                gl.useProgram(prevProgram);
            }
        }
        
        // ========================================
        // End Status Panel Functions
        // ========================================
        
        // ========================================
        // Left Status Panel Functions (Head-Locked 3D Quad)
        // ========================================
        
        function initLeftStatusPanel() {
            if (!gl) {
                vrLog('Left status panel: no GL context');
                return;
            }
            
            vrLog('Init left status panel...');
            
            // Create canvas for status text (taller for more info)
            leftStatusPanelCanvas = document.createElement('canvas');
            leftStatusPanelCanvas.width = 256;
            leftStatusPanelCanvas.height = 144;
            leftStatusPanelCtx = leftStatusPanelCanvas.getContext('2d');
            
            // Create texture
            leftStatusPanelTexture = gl.createTexture();
            gl.bindTexture(gl.TEXTURE_2D, leftStatusPanelTexture);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
            
            // Create position buffer for 3D quad
            // Position with left edge aligned to left edge of main view
            const mainHeight = stereoSettings.screenScale;
            const mainWidth = mainHeight * (16/9);
            const mainLeftEdge = -mainWidth / 2;
            
            // Left status panel size (taller than right panel for more info)
            const panelHeight = 0.135;  // 13.5cm tall in world space
            const panelWidth = 0.20;   // 20cm wide
            
            // Position: left edge aligned with main view left edge, vertically centered
            const panelLeft = mainLeftEdge + 0.24;  // Left edges align
            const panelCenterY = stereoSettings.verticalOffset - 0.175;  // Lowered significantly to compensate for rotation
            
            leftStatusPanelPositionBuffer = gl.createBuffer();
            gl.bindBuffer(gl.ARRAY_BUFFER, leftStatusPanelPositionBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
                panelLeft, panelCenterY - panelHeight/2,
                panelLeft + panelWidth, panelCenterY - panelHeight/2,
                panelLeft, panelCenterY + panelHeight/2,
                panelLeft + panelWidth, panelCenterY + panelHeight/2,
            ]), gl.STATIC_DRAW);
            
            leftStatusPanelTexCoordBuffer = gl.createBuffer();
            gl.bindBuffer(gl.ARRAY_BUFFER, leftStatusPanelTexCoordBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
                0, 1,
                1, 1,
                0, 0,
                1, 0,
            ]), gl.STATIC_DRAW);
            
            // Initial update
            updateLeftStatusPanelCanvas();
            
            leftStatusPanelInitialized = true;
            vrLog('Left status panel: OK');
        }
        
        function updateLeftStatusPanelCanvas() {
            if (!leftStatusPanelCtx) return;
            
            const canvas = leftStatusPanelCanvas;
            const ctx = leftStatusPanelCtx;
            
            // Clear canvas
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            
            // Semi-transparent dark background with rounded corners (50% opaque)
            ctx.fillStyle = 'rgba(20, 20, 30, 0.5)';
            drawRoundedRect(ctx, 0, 0, canvas.width, canvas.height, 12);
            
            // Title
            ctx.fillStyle = '#88ccff';
            ctx.font = 'bold 14px monospace';
            ctx.textBaseline = 'top';
            ctx.fillText('Stream Info', 10, 8);
            
            // FPS
            ctx.fillStyle = '#cccccc';
            ctx.font = '11px monospace';
            ctx.fillText('FPS:', 10, 28);
            const fpsColor = currentVrFps >= 25 ? '#44ff44' : (currentVrFps >= 15 ? '#ffff00' : '#ff4444');
            ctx.fillStyle = fpsColor;
            ctx.fillText(`${currentVrFps.toFixed(1)}`, 80, 28);
            
            // Frame latency
            const frameAge = performance.now() - lastFrameReceivedTimestamp;
            const lagColor = frameAge > 150 ? '#ff4444' : '#44ff44';
            ctx.fillStyle = '#cccccc';
            ctx.fillText('Latency:', 10, 43);
            ctx.fillStyle = lagColor;
            ctx.fillText(`${frameAge.toFixed(0)} ms`, 80, 43);
            
            // Battery status
            ctx.fillStyle = '#cccccc';
            ctx.fillText('Battery:', 10, 58);
            if (currentBatteryLevel !== null) {
                const batteryColor = currentBatteryLevel > 20 ? '#44ff44' : '#ff4444';
                ctx.fillStyle = batteryColor;
                const chargingIcon = currentBatteryCharging ? ' ⚡' : '';
                ctx.fillText(`${currentBatteryLevel}%${chargingIcon}`, 80, 58);
            } else {
                ctx.fillStyle = '#888888';
                ctx.fillText('N/A', 80, 58);
            }
            
            // Controller WS status
            ctx.fillStyle = '#cccccc';
            ctx.fillText('Ctrl WS:', 10, 73);
            const ctrlStatus = controllerWS && controllerWS.readyState === WebSocket.OPEN ? 'Connected' : 'Disconnected';
            const ctrlColor = controllerWS && controllerWS.readyState === WebSocket.OPEN ? '#44ff44' : '#ff4444';
            ctx.fillStyle = ctrlColor;
            ctx.fillText(ctrlStatus, 80, 73);
            
            // Headset orientation (pitch/roll/yaw)
            const pitch = headsetPose.rotation ? Math.round(headsetPose.rotation.x) : 0;
            const roll = headsetPose.rotation ? Math.round(headsetPose.rotation.z) : 0;
            const yaw = headsetPose.rotation ? Math.round(headsetPose.rotation.y) : 0;
            
            ctx.fillStyle = '#88ccff';
            ctx.fillText('Pitch:', 10, 88);
            ctx.fillText(`${pitch}°`, 80, 88);
            
            ctx.fillText('Roll:', 10, 103);
            ctx.fillText(`${roll}°`, 80, 103);
            
            // Yaw: red if outside +/- 10 degrees
            ctx.fillText('Yaw:', 10, 118);
            ctx.fillStyle = (Math.abs(yaw) > 10) ? '#ff4444' : '#88ccff';
            ctx.fillText(`${yaw}°`, 80, 118);
            
            // Update texture
            if (gl && leftStatusPanelTexture) {
                gl.bindTexture(gl.TEXTURE_2D, leftStatusPanelTexture);
                gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, canvas);
            }
        }
        
        function drawLeftStatusPanel(view, viewport, modelMatrix) {
            // Initialize on first call
            if (!leftStatusPanelInitialized) {
                initLeftStatusPanel();
            }
            
            if (!leftStatusPanelInitialized || !shaderProgram || !leftStatusPanelTexture || !cachedLocations) {
                return;
            }
            
            // Update canvas content periodically (every 30 frames ~ 0.5 sec at 60fps)
            if (frameCounter % 30 === 0) {
                updateLeftStatusPanelCanvas();
            }
            
            // Save current state
            const prevProgram = gl.getParameter(gl.CURRENT_PROGRAM);
            
            // Enable blending for transparency
            // Use blendFuncSeparate to preserve destination alpha (video's alpha=1.0)
            // This prevents AR passthrough from showing through the panel
            gl.enable(gl.BLEND);
            gl.blendFuncSeparate(
                gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA,  // RGB: standard alpha blend
                gl.ZERO, gl.ONE                        // Alpha: preserve destination alpha
            );
            
            // Disable depth test so panel is always visible on top
            gl.disable(gl.DEPTH_TEST);
            
            // Use the main 3D shader (same as video quad)
            gl.useProgram(shaderProgram);
            
            // Set up position buffer (left status panel geometry)
            if (cachedLocations.position !== -1 && cachedLocations.position !== null) {
                gl.bindBuffer(gl.ARRAY_BUFFER, leftStatusPanelPositionBuffer);
                gl.enableVertexAttribArray(cachedLocations.position);
                gl.vertexAttribPointer(cachedLocations.position, 2, gl.FLOAT, false, 0, 0);
            }
            
            // Set up tex coord buffer
            if (cachedLocations.texCoord !== -1 && cachedLocations.texCoord !== null) {
                gl.bindBuffer(gl.ARRAY_BUFFER, leftStatusPanelTexCoordBuffer);
                gl.enableVertexAttribArray(cachedLocations.texCoord);
                gl.vertexAttribPointer(cachedLocations.texCoord, 2, gl.FLOAT, false, 0, 0);
            }
            
            // Set uniforms
            if (cachedLocations.projection !== null) {
                gl.uniformMatrix4fv(cachedLocations.projection, false, view.projectionMatrix);
            }
            
            if (cachedLocations.view !== null) {
                invertMatrix(view.transform.matrix, viewMatrixBuffer);
                gl.uniformMatrix4fv(cachedLocations.view, false, viewMatrixBuffer);
            }
            
            if (cachedLocations.model !== null) {
                // Apply yaw rotation to face viewer (Quest 3 coordinate system)
                if (modelMatrix) {
                    createRotatedModelMatrix(modelMatrix, -15, leftPanelModelMatrix);
                    gl.uniformMatrix4fv(cachedLocations.model, false, leftPanelModelMatrix);
                } else {
                    gl.uniformMatrix4fv(cachedLocations.model, false, cachedLocations.identityMatrix);
                }
            }
            
            // Position at same distance as main view so transparency works
            if (cachedLocations.distance !== null) {
                gl.uniform1f(cachedLocations.distance, stereoSettings.screenDistance);
            }
            
            if (cachedLocations.ipdOffset !== null) {
                gl.uniform1f(cachedLocations.ipdOffset, stereoSettings.ipdOffset);  // Match video IPD
            }
            
            if (cachedLocations.isLeftEye !== null) {
                gl.uniform1f(cachedLocations.isLeftEye, view.eye === 'left' ? 1.0 : 0.0);
            }
            
            if (cachedLocations.useDirectVideo !== null) {
                gl.uniform1f(cachedLocations.useDirectVideo, 0.0);  // Not using video splitting
            }
            
            if (cachedLocations.texture !== null) {
                gl.activeTexture(gl.TEXTURE0);
                gl.bindTexture(gl.TEXTURE_2D, leftStatusPanelTexture);
                gl.uniform1i(cachedLocations.texture, 0);
            }
            
            // Draw the panel
            gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
            
            // Restore state
            gl.disable(gl.BLEND);
            gl.enable(gl.DEPTH_TEST);
            
            if (prevProgram) {
                gl.useProgram(prevProgram);
            }
        }
        
        // ========================================
        // End Left Status Panel Functions
        // ========================================

        // Simple shader program for rendering video texture
        let shaderProgram = null;
        let positionBuffer = null;
        let texCoordBuffer = null;
        
        // Cached shader locations for performance
        let cachedLocations = null;
        let lastSettingsHash = '';
        
        // Video frame callback for optimal sync
        let lastVideoTime = -1;
        let videoFrameCallbackId = null;
        let newVideoFrameAvailable = false;
        
        function initShaders() {
            const vertexShader = gl.createShader(gl.VERTEX_SHADER);
            gl.shaderSource(vertexShader, `
                attribute vec2 a_position;
                attribute vec2 a_texCoord;
                varying vec2 v_texCoord;
                varying float v_isLeftEye;
                uniform mat4 u_projection;
                uniform mat4 u_view;
                uniform mat4 u_model;
                uniform float u_distance;
                uniform float u_ipdOffset;
                uniform float u_isLeftEye;
                
                void main() {
                    // Apply IPD offset (inward for each eye to adjust convergence)
                    float xOffset = u_ipdOffset * (u_isLeftEye > 0.5 ? 1.0 : -1.0);
                    vec4 pos = vec4(a_position.x + xOffset, a_position.y, -u_distance, 1.0);
                    gl_Position = u_projection * u_view * u_model * pos;
                    v_texCoord = a_texCoord;
                    v_isLeftEye = u_isLeftEye;
                }
            `);
            gl.compileShader(vertexShader);
            
            // Check vertex shader compilation
            if (!gl.getShaderParameter(vertexShader, gl.COMPILE_STATUS)) {
                const err = gl.getShaderInfoLog(vertexShader);
                console.error('Vertex shader compilation error:', err);
                vrLog('VTX SHADER ERR: ' + err.substring(0, 50));
                useDirectVideoTexture = false;  // Fall back to canvas mode
            }
            
            // OPTIMIZED: Fragment shader that can handle both modes
            const fragmentShader = gl.createShader(gl.FRAGMENT_SHADER);
            gl.shaderSource(fragmentShader, `
                precision mediump float;
                varying vec2 v_texCoord;
                varying float v_isLeftEye;
                uniform sampler2D u_texture;
                uniform float u_useDirectVideo;
                
                void main() {
                    vec2 texCoord = v_texCoord;
                    
                    // If using direct video texture, remap coordinates for left/right half
                    if (u_useDirectVideo > 0.5) {
                        // Split side-by-side: left eye uses 0.0-0.5, right uses 0.5-1.0
                        if (v_isLeftEye > 0.5) {
                            texCoord.x = texCoord.x * 0.5;  // Left half
                        } else {
                            texCoord.x = texCoord.x * 0.5 + 0.5;  // Right half
                        }
                    }
                    
                    gl_FragColor = texture2D(u_texture, texCoord);
                }
            `);
            gl.compileShader(fragmentShader);
            
            // Check fragment shader compilation
            if (!gl.getShaderParameter(fragmentShader, gl.COMPILE_STATUS)) {
                const err = gl.getShaderInfoLog(fragmentShader);
                console.error('Fragment shader compilation error:', err);
                vrLog('FRAG SHADER ERR: ' + err.substring(0, 50));
                useDirectVideoTexture = false;  // Fall back to canvas mode
            }
            
            shaderProgram = gl.createProgram();
            gl.attachShader(shaderProgram, vertexShader);
            gl.attachShader(shaderProgram, fragmentShader);
            gl.linkProgram(shaderProgram);
            
            // Check program linking
            if (!gl.getProgramParameter(shaderProgram, gl.LINK_STATUS)) {
                const err = gl.getProgramInfoLog(shaderProgram);
                console.error('Shader program linking error:', err);
                vrLog('LINK ERR: ' + err.substring(0, 50));
                useDirectVideoTexture = false;  // Fall back to canvas mode
                shaderProgram = null;
                return;
            }
            
            vrLog('Shaders compiled OK');
            
            // Create quad vertices (screen-sized quad positioned in front of viewer)
            const aspect = 16 / 9;  // Assume 16:9 aspect ratio per eye
            const height = stereoSettings.screenScale;
            const width = height * aspect;
            const yOffset = stereoSettings.verticalOffset;
            
            positionBuffer = gl.createBuffer();
            gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
                -width/2, -height/2 + yOffset,
                 width/2, -height/2 + yOffset,
                -width/2,  height/2 + yOffset,
                 width/2,  height/2 + yOffset,
            ]), gl.STATIC_DRAW);
            
            texCoordBuffer = gl.createBuffer();
            gl.bindBuffer(gl.ARRAY_BUFFER, texCoordBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
                0, 1,
                1, 1,
                0, 0,
                1, 0,
            ]), gl.STATIC_DRAW);
            
            // Cache all uniform and attribute locations (avoid per-frame lookups)
            cachedLocations = {
                position: gl.getAttribLocation(shaderProgram, 'a_position'),
                texCoord: gl.getAttribLocation(shaderProgram, 'a_texCoord'),
                projection: gl.getUniformLocation(shaderProgram, 'u_projection'),
                view: gl.getUniformLocation(shaderProgram, 'u_view'),
                model: gl.getUniformLocation(shaderProgram, 'u_model'),
                texture: gl.getUniformLocation(shaderProgram, 'u_texture'),
                distance: gl.getUniformLocation(shaderProgram, 'u_distance'),
                ipdOffset: gl.getUniformLocation(shaderProgram, 'u_ipdOffset'),
                isLeftEye: gl.getUniformLocation(shaderProgram, 'u_isLeftEye'),
                useDirectVideo: gl.getUniformLocation(shaderProgram, 'u_useDirectVideo')
            };
            
            console.log('Shader locations:', cachedLocations);
            
            // Verify all locations are valid
            let invalidLocs = [];
            for (const [name, loc] of Object.entries(cachedLocations)) {
                if ((name === 'position' || name === 'texCoord') && (loc === null || loc === -1)) {
                    invalidLocs.push(name + '(attr)');
                    console.warn(`Failed to get attribute location for: ${name}`);
                } else if (name !== 'position' && name !== 'texCoord' && loc === null) {
                    invalidLocs.push(name);
                    console.warn(`Failed to get uniform location for: ${name}`);
                }
            }
            if (invalidLocs.length > 0) {
                vrLog('Missing: ' + invalidLocs.join(', '));
            } else {
                vrLog('All shader locs OK');
            }
            
            // Identity model matrix (never changes)
            cachedLocations.identityMatrix = new Float32Array([
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1
            ]);
        }
        
        // Check if position buffer needs rebuild (only when settings change)
        function updatePositionBufferIfNeeded() {
            const hash = `${stereoSettings.screenScale}_${stereoSettings.verticalOffset}`;
            if (hash !== lastSettingsHash && positionBuffer) {
                lastSettingsHash = hash;
                const aspect = 16 / 9;
                const height = stereoSettings.screenScale;
                const width = height * aspect;
                const yOffset = stereoSettings.verticalOffset;
                
                gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
                gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
                    -width/2, -height/2 + yOffset,
                     width/2, -height/2 + yOffset,
                    -width/2,  height/2 + yOffset,
                     width/2,  height/2 + yOffset,
                ]), gl.STATIC_DRAW);
            }
        }
        
        function drawTexturedQuad(view, viewport, useDirectVideo = false, modelMatrix = null) {
            if (!shaderProgram) {
                initShaders();
            }
            
            if (!shaderProgram) {
                console.error('Shader program not initialized!');
                vrLog('ERROR: No shader!');
                return;
            }
            
            gl.useProgram(shaderProgram);
            
            // Only rebuild position buffer when settings actually change
            updatePositionBufferIfNeeded();
            
            if (!positionBuffer || !texCoordBuffer) {
                vrLog('ERROR: No buffers!');
                return;
            }
            
            // Set up attributes using cached locations - only if valid
            if (cachedLocations.position !== -1 && cachedLocations.position !== null) {
                gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
                gl.enableVertexAttribArray(cachedLocations.position);
                gl.vertexAttribPointer(cachedLocations.position, 2, gl.FLOAT, false, 0, 0);
            }
            
            if (cachedLocations.texCoord !== -1 && cachedLocations.texCoord !== null) {
                gl.bindBuffer(gl.ARRAY_BUFFER, texCoordBuffer);
                gl.enableVertexAttribArray(cachedLocations.texCoord);
                gl.vertexAttribPointer(cachedLocations.texCoord, 2, gl.FLOAT, false, 0, 0);
            }
            
            // Set uniforms using cached locations - only if valid
            if (cachedLocations.projection !== null) {
                gl.uniformMatrix4fv(cachedLocations.projection, false, view.projectionMatrix);
            }
            
            if (cachedLocations.view !== null) {
                // Invert view transform (view.transform is viewer pose, we need inverse)
                invertMatrix(view.transform.matrix, viewMatrixBuffer);
                gl.uniformMatrix4fv(cachedLocations.view, false, viewMatrixBuffer);
            }
            
            if (cachedLocations.model !== null) {
                // Model matrix - use provided matrix (head pose) or identity
                gl.uniformMatrix4fv(cachedLocations.model, false, modelMatrix || cachedLocations.identityMatrix);
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
                gl.uniform1f(cachedLocations.useDirectVideo, useDirectVideo ? 1.0 : 0.0);
            }
            
            if (cachedLocations.texture !== null) {
                gl.uniform1i(cachedLocations.texture, 0);
            }
            
            // Check for GL errors
            const err = gl.getError();
            if (err !== gl.NO_ERROR) {
                if (frameCounter < 20) {
                    console.error('WebGL error in drawTexturedQuad:', err);
                    vrLog('GL ERROR: ' + err);
                }
                return;  // Don't draw if there's an error
            }
            
            // Draw quad
            gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
            
            if (frameCounter === 15) {
                vrLog('Draw quad OK');
            }
        }
        
        // Matrix inversion helper
        function invertMatrix(m, out) {
            const m00 = m[0], m01 = m[1], m02 = m[2], m03 = m[3];
            const m10 = m[4], m11 = m[5], m12 = m[6], m13 = m[7];
            const m20 = m[8], m21 = m[9], m22 = m[10], m23 = m[11];
            const m30 = m[12], m31 = m[13], m32 = m[14], m33 = m[15];
            
            const tmp0 = m22 * m33 - m32 * m23;
            const tmp1 = m21 * m33 - m31 * m23;
            const tmp2 = m21 * m32 - m31 * m22;
            const tmp3 = m20 * m33 - m30 * m23;
            const tmp4 = m20 * m32 - m30 * m22;
            const tmp5 = m20 * m31 - m30 * m21;
            
            const t0 = m11 * tmp0 - m12 * tmp1 + m13 * tmp2;
            const t1 = -(m10 * tmp0 - m12 * tmp3 + m13 * tmp4);
            const t2 = m10 * tmp1 - m11 * tmp3 + m13 * tmp5;
            const t3 = -(m10 * tmp2 - m11 * tmp4 + m12 * tmp5);
            
            const det = 1.0 / (m00 * t0 + m01 * t1 + m02 * t2 + m03 * t3);
            
            out[0] = t0 * det;
            out[1] = (-(m01 * tmp0 - m02 * tmp1 + m03 * tmp2)) * det;
            out[2] = (m01 * (m12 * m33 - m32 * m13) - m02 * (m11 * m33 - m31 * m13) + m03 * (m11 * m32 - m31 * m12)) * det;
            out[3] = (-(m01 * (m12 * m23 - m22 * m13) - m02 * (m11 * m23 - m21 * m13) + m03 * (m11 * m22 - m21 * m12))) * det;
            out[4] = t1 * det;
            out[5] = (m00 * tmp0 - m02 * tmp3 + m03 * tmp4) * det;
            out[6] = (-(m00 * (m12 * m33 - m32 * m13) - m02 * (m10 * m33 - m30 * m13) + m03 * (m10 * m32 - m30 * m12))) * det;
            out[7] = (m00 * (m12 * m23 - m22 * m13) - m02 * (m10 * m23 - m20 * m13) + m03 * (m10 * m22 - m20 * m12)) * det;
            out[8] = t2 * det;
            out[9] = (-(m00 * tmp1 - m01 * tmp3 + m03 * tmp5)) * det;
            out[10] = (m00 * (m11 * m33 - m31 * m13) - m01 * (m10 * m33 - m30 * m13) + m03 * (m10 * m31 - m30 * m11)) * det;
            out[11] = (-(m00 * (m11 * m23 - m21 * m13) - m01 * (m10 * m23 - m20 * m13) + m03 * (m10 * m21 - m20 * m11))) * det;
            out[12] = t3 * det;
            out[13] = (m00 * tmp2 - m01 * tmp4 + m02 * tmp5) * det;
            out[14] = (-(m00 * (m11 * m32 - m31 * m12) - m01 * (m10 * m32 - m30 * m12) + m02 * (m10 * m31 - m30 * m11))) * det;
            out[15] = (m00 * (m11 * m22 - m21 * m12) - m01 * (m10 * m22 - m20 * m12) + m02 * (m10 * m21 - m20 * m11)) * det;
        }
        
        function onSessionEnd() {
            xrSession = null;
            gl = null;
            setStatus('AR session ended', '');
            document.getElementById('vrButton').textContent = '🥽 Enter AR';
            document.getElementById('vrButton').onclick = startVRSession;
            document.getElementById('preview').classList.remove('hidden');
            startPreview();
        }
        
        // Battery level monitoring
        function initBatteryMonitor() {
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
            currentBatteryLevel = level;
            currentBatteryCharging = battery.charging;
        }
        
        // ========================================
        // Robot Panel Functions
        // ========================================
        
        function initRobotPanel() {
            const panel = document.getElementById('robotPanel');
            const toggle = document.getElementById('robotPanelToggle');
            const header = document.getElementById('robotPanelHeader');
            
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
            
            console.log('Robot panel initialized');
        }
        
        // Initialize on page load
        window.addEventListener('DOMContentLoaded', async () => {
            console.log('Stereo VR Vision - Initializing...');
            initBatteryMonitor();
            initRobotPanel();  // Initialize robot status panel
            initControllerWebSocket();  // Initialize controller tracking WebSocket
            initFoxgloveConnection();
            await initXR();
        });