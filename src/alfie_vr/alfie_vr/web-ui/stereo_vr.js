        // WebXR Stereo Vision for Meta Quest 3
        // Displays side-by-side stereo video as immersive VR
        // With VR Controller Tracking support
        
        const SIGNALING_PORT = 8084;
        const CONTROLLER_WS_PORT = 8442;
        
        let pc = null;
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
        
        function updateControllerWSStatus(connected) {
            const statusDot = document.getElementById('controllerWSStatus');
            const statusText = document.getElementById('controllerWSStatusText');
            if (statusDot) {
                statusDot.style.color = connected ? '#4CAF50' : '#f44336';
            }
            if (statusText) {
                statusText.textContent = connected ? 'Connected' : 'Disconnected';
            }
        }
        
        function initControllerWebSocket() {
            const wsUrl = `wss://${window.location.hostname}:${CONTROLLER_WS_PORT}`;
            console.log(`Connecting to controller WebSocket: ${wsUrl}`);
            
            try {
                controllerWS = new WebSocket(wsUrl);
                
                controllerWS.onopen = () => {
                    console.log('Controller WebSocket connected');
                    controllerWSRetryCount = 0;
                    vrLog('Controller WS: Connected');
                    updateControllerWSStatus(true);
                };
                
                controllerWS.onerror = (error) => {
                    console.error('Controller WebSocket error:', error);
                    vrLog('Controller WS: Error');
                    updateControllerWSStatus(false);
                };
                
                controllerWS.onclose = (event) => {
                    console.log(`Controller WebSocket closed. Code: ${event.code}, Reason: ${event.reason}`);
                    controllerWS = null;
                    vrLog('Controller WS: Disconnected');
                    updateControllerWSStatus(false);
                    
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
                updateControllerWSStatus(false);
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
        
        // Latency tracking
        let lastFrameTime = 0;
        let frameCount = 0;
        let totalLatency = 0;
        
        // Stereo adjustment settings
        let stereoSettings = {
            verticalOffset: 0,      // Meters - positive = up
            ipdOffset: 0,           // Meters - adjustment to convergence
            screenDistance: 2.0,    // Meters - distance to virtual screen
            screenScale: 1.5        // Multiplier for screen size
        };
        
        // Load saved settings
        function loadSettings() {
            try {
                const saved = localStorage.getItem('stereoVRSettings');
                if (saved) {
                    stereoSettings = { ...stereoSettings, ...JSON.parse(saved) };
                }
            } catch (e) {}
            updateControlsFromSettings();
        }
        
        // Save settings to localStorage
        window.saveSettings = function() {
            localStorage.setItem('stereoVRSettings', JSON.stringify(stereoSettings));
            setStatus('Settings saved!', 'connected');
            setTimeout(() => setStatus('VR Active - Stereo 3D Vision', 'connected'), 1000);
        };
        
        // Reset settings to defaults
        window.resetSettings = function() {
            stereoSettings = {
                verticalOffset: 0,
                ipdOffset: 0,
                screenDistance: 2.0,
                screenScale: 1.5
            };
            updateControlsFromSettings();
            reinitQuadBuffers();
        };
        
        // Update HTML controls to match settings
        function updateControlsFromSettings() {
            document.getElementById('verticalOffset').value = stereoSettings.verticalOffset;
            document.getElementById('verticalOffsetVal').textContent = stereoSettings.verticalOffset.toFixed(2);
            document.getElementById('ipdOffset').value = stereoSettings.ipdOffset;
            document.getElementById('ipdOffsetVal').textContent = stereoSettings.ipdOffset.toFixed(3);
            document.getElementById('screenDistance').value = stereoSettings.screenDistance;
            document.getElementById('screenDistanceVal').textContent = stereoSettings.screenDistance.toFixed(1);
            document.getElementById('screenScale').value = stereoSettings.screenScale;
            document.getElementById('screenScaleVal').textContent = stereoSettings.screenScale.toFixed(1);
        }
        
        // Set up control event listeners
        function setupControls() {
            document.getElementById('verticalOffset').addEventListener('input', (e) => {
                stereoSettings.verticalOffset = parseFloat(e.target.value);
                document.getElementById('verticalOffsetVal').textContent = stereoSettings.verticalOffset.toFixed(2);
                reinitQuadBuffers();
            });
            document.getElementById('ipdOffset').addEventListener('input', (e) => {
                stereoSettings.ipdOffset = parseFloat(e.target.value);
                document.getElementById('ipdOffsetVal').textContent = stereoSettings.ipdOffset.toFixed(3);
            });
            document.getElementById('screenDistance').addEventListener('input', (e) => {
                stereoSettings.screenDistance = parseFloat(e.target.value);
                document.getElementById('screenDistanceVal').textContent = stereoSettings.screenDistance.toFixed(1);
            });
            document.getElementById('screenScale').addEventListener('input', (e) => {
                stereoSettings.screenScale = parseFloat(e.target.value);
                document.getElementById('screenScaleVal').textContent = stereoSettings.screenScale.toFixed(1);
                reinitQuadBuffers();
            });
        }
        
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
                window.open(`https://${window.location.hostname}:${SIGNALING_PORT}/`, '_blank');
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
        
        // Initialize WebRTC connection to stereo stream
        async function initWebRTC() {
            setStatus('Connecting to stereo stream...');
            
            const signalingUrl = `https://${window.location.hostname}:${SIGNALING_PORT}/offer`;
            
            pc = new RTCPeerConnection({
                iceServers: [{ urls: 'stun:stun.l.google.com:19302' }],
                // Ultra-low latency configuration
                bundlePolicy: 'max-bundle',
                rtcpMuxPolicy: 'require',
                iceTransportPolicy: 'all'
            });
            
            pc.ontrack = (event) => {
                console.log('Track received:', event.track.kind);
                if (event.track.kind === 'video') {
                    videoElement = document.getElementById('stereoVideo');
                    videoElement.srcObject = event.streams[0];
                    // ULTRA-low latency settings
                    videoElement.playsInline = true;
                    videoElement.muted = true;
                    
                    // Disable all buffering for minimum latency
                    if ('getSettings' in event.track) {
                        const settings = event.track.getSettings();
                        console.log('Video track settings:', settings);
                    }
                    
                    // Track video latency
                    if ('requestVideoFrameCallback' in videoElement) {
                        const measureLatency = (now, metadata) => {
                            const currentTime = performance.now();
                            if (lastFrameTime > 0) {
                                const frameDelta = currentTime - lastFrameTime;
                                frameCount++;
                                
                                // Calculate presentation to render latency
                                if (metadata.presentationTime && metadata.expectedDisplayTime) {
                                    const presentationLatency = (metadata.expectedDisplayTime - metadata.presentationTime) * 1000;
                                    totalLatency += presentationLatency;
                                }
                                
                                if (frameCount % 50 === 0) {
                                    const avgLatency = totalLatency / frameCount;
                                    console.log(
                                        `[Client] Frame interval: ${frameDelta.toFixed(1)}ms ` +
                                        `(${(1000/frameDelta).toFixed(1)} fps) | ` +
                                        `Presentation latency: ${avgLatency.toFixed(1)}ms`
                                    );
                                }
                            }
                            lastFrameTime = currentTime;
                            videoElement.requestVideoFrameCallback(measureLatency);
                        };
                        videoElement.requestVideoFrameCallback(measureLatency);
                    }
                    
                    videoElement.play().then(() => {
                        setStatus('Stereo stream connected - Ultra-low latency mode', 'connected');
                        document.getElementById('vrButton').disabled = false;
                        document.getElementById('certButton').style.display = 'none';
                        startPreview();
                    }).catch(err => {
                        console.error('Video play error:', err);
                        setStatus('Video play error: ' + err.message, 'error');
                    });
                }
            };
            
            pc.oniceconnectionstatechange = () => {
                console.log('ICE state:', pc.iceConnectionState);
                if (pc.iceConnectionState === 'failed' || pc.iceConnectionState === 'disconnected') {
                    setStatus('Connection lost, reconnecting...', 'error');
                    setTimeout(initWebRTC, 2000);
                }
            };
            
            // Add transceiver for video with low-latency hint
            pc.addTransceiver('video', { 
                direction: 'recvonly'
            });
            
            try {
                const offer = await pc.createOffer({
                    offerToReceiveVideo: true,
                    offerToReceiveAudio: false
                });
                
                // Modify SDP for ultra-low latency before setting
                let sdp = offer.sdp;
                
                // Add receive-only bandwidth constraints and latency hints
                sdp = sdp.replace(/(m=video.*\r\n)/g, 
                    '$1b=AS:3000\r\n' +  // 3 Mbps receive bandwidth
                    'b=TIAS:3000000\r\n'
                );
                
                // Request minimal latency in WebRTC receive pipeline
                sdp = sdp.replace(/(a=rtcp-fb:.*\r\n)/g, 
                    '$1a=x-google-flag:conference\r\n'  // Hint for low-latency mode
                );
                
                offer.sdp = sdp;
                await pc.setLocalDescription(offer);
                
                // Wait for ICE gathering
                await new Promise((resolve) => {
                    if (pc.iceGatheringState === 'complete') {
                        resolve();
                    } else {
                        const checkState = () => {
                            if (pc.iceGatheringState === 'complete') {
                                pc.removeEventListener('icegatheringstatechange', checkState);
                                resolve();
                            }
                        };
                        pc.addEventListener('icegatheringstatechange', checkState);
                        // Timeout after 3 seconds
                        setTimeout(resolve, 3000);
                    }
                });
                
                const response = await fetch(signalingUrl, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        sdp: pc.localDescription.sdp,
                        type: pc.localDescription.type
                    })
                });
                
                if (!response.ok) {
                    throw new Error(`Signaling failed: ${response.status}`);
                }
                
                const answer = await response.json();
                await pc.setRemoteDescription(new RTCSessionDescription(answer));
                
                console.log('WebRTC connection established');
                retryCount = 0;  // Reset retry count on success
                
            } catch (err) {
                console.error('WebRTC error:', err);
                retryCount++;
                
                // Check if it's likely a certificate error
                if (err.message.includes('fetch') || err.message.includes('Failed') || err.name === 'TypeError') {
                    setStatus('Connection error - SSL certificate may need to be accepted', 'error');
                    showCertButton();
                    
                    if (retryCount <= MAX_RETRIES) {
                        setTimeout(initWebRTC, 5000);
                    }
                } else {
                    setStatus('Connection error: ' + err.message, 'error');
                    setTimeout(initWebRTC, 3000);
                }
            }
        }
        
        // Show preview of stereo stream (non-VR mode)
        function startPreview() {
            const video = document.getElementById('stereoVideo');
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

                if (video.readyState >= 2) {
                    canvas.width = video.videoWidth;
                    canvas.height = video.videoHeight;
                    ctx.drawImage(video, 0, 0);
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
            
            const supported = await navigator.xr.isSessionSupported('immersive-vr');
            if (!supported) {
                setStatus('Immersive VR not supported', 'error');
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
                
                // Request immersive VR session with local-floor reference space
                xrSession = await navigator.xr.requestSession('immersive-vr', {
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
                
                const video = document.getElementById('stereoVideo');
                vrLog('Video ready: ' + (video.readyState >= 2 ? 'YES' : 'NO'));
                if (video.readyState >= 2) {
                    vrLog('Video: ' + video.videoWidth + 'x' + video.videoHeight);
                }
                
                document.getElementById('preview').classList.add('hidden');
                document.getElementById('vrButton').textContent = '🛑 Exit VR';
                document.getElementById('vrButton').onclick = () => xrSession.end();
                
                setStatus('VR Active - Stereo 3D Vision', 'connected');
                
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
        let lastFpsTime = 0;
        let fpsFrameCount = 0;

        function onXRFrame(time, frame) {
            if (!xrSession) return;
            
            xrSession.requestAnimationFrame(onXRFrame);

            // Calculate FPS
            if (lastFpsTime === 0) lastFpsTime = time;
            fpsFrameCount++;
            const elapsed = time - lastFpsTime;
            if (elapsed >= 1000) {
                const fps = (fpsFrameCount / elapsed) * 1000;
                const fpsElement = document.getElementById('fpsCounter');
                if (fpsElement) {
                    fpsElement.textContent = `FPS: ${fps.toFixed(1)}`;
                }
                lastFpsTime = time;
                fpsFrameCount = 0;
            }
            
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
            
            // Clear to a visible color to test if rendering is working at all
            gl.clearColor(0.1, 0.1, 0.1, 1.0);  // Dark gray instead of black
            gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
            
            const video = document.getElementById('stereoVideo');
            if (video.readyState < 2) {
                if (frameCounter % 60 === 0) {
                    console.log('Waiting for video readyState:', video.readyState);
                    vrLog('Wait video: ' + video.readyState);
                }
                frameCounter++;
                // Still render something so user knows VR is working
                return;
            }
            
            frameCounter++;
            if (frameCounter === 10) {
                console.log('Video ready, rendering. Size:', video.videoWidth, 'x', video.videoHeight);
                console.log('Pose views:', pose.views.length);
                vrLog('Rendering video: ' + video.videoWidth + 'x' + video.videoHeight);
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
                    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, video);
                    
                    if (frameCounter === 10) {
                        console.log('Video texture upload successful, size:', video.videoWidth, 'x', video.videoHeight);
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
                }
            } else {
                // FALLBACK: Use canvas-based splitting (higher latency)
                const videoWidth = video.videoWidth;
                const videoHeight = video.videoHeight;
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
                leftCtx.drawImage(video, 0, 0, halfWidth, videoHeight, 0, 0, halfWidth, videoHeight);
                rightCtx.drawImage(video, halfWidth, 0, halfWidth, videoHeight, 0, 0, halfWidth, videoHeight);
                
                // Upload to WebGL textures
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
                }
            }
        }
        
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
            setStatus('VR session ended', '');
            document.getElementById('vrButton').textContent = '🥽 Enter VR';
            document.getElementById('vrButton').onclick = startVRSession;
            document.getElementById('preview').classList.remove('hidden');
            startPreview();
        }
        
        // Make settings panel draggable
        function initDraggable() {
            const controls = document.getElementById('controls');
            const header = controls.querySelector('h3');
            let isDragging = false;
            let offsetX, offsetY;
            
            header.addEventListener('mousedown', (e) => {
                isDragging = true;
                offsetX = e.clientX - controls.offsetLeft;
                offsetY = e.clientY - controls.offsetTop;
                controls.style.right = 'auto';
                controls.style.left = controls.offsetLeft + 'px';
            });
            
            document.addEventListener('mousemove', (e) => {
                if (!isDragging) return;
                e.preventDefault();
                let newX = e.clientX - offsetX;
                let newY = e.clientY - offsetY;
                
                // Keep within viewport bounds
                newX = Math.max(0, Math.min(newX, window.innerWidth - controls.offsetWidth));
                newY = Math.max(0, Math.min(newY, window.innerHeight - controls.offsetHeight));
                
                controls.style.left = newX + 'px';
                controls.style.top = newY + 'px';
            });
            
            document.addEventListener('mouseup', () => {
                isDragging = false;
            });
            
            // Touch support for VR headset
            header.addEventListener('touchstart', (e) => {
                isDragging = true;
                const touch = e.touches[0];
                offsetX = touch.clientX - controls.offsetLeft;
                offsetY = touch.clientY - controls.offsetTop;
                controls.style.right = 'auto';
                controls.style.left = controls.offsetLeft + 'px';
            });
            
            document.addEventListener('touchmove', (e) => {
                if (!isDragging) return;
                e.preventDefault();
                const touch = e.touches[0];
                let newX = touch.clientX - offsetX;
                let newY = touch.clientY - offsetY;
                
                newX = Math.max(0, Math.min(newX, window.innerWidth - controls.offsetWidth));
                newY = Math.max(0, Math.min(newY, window.innerHeight - controls.offsetHeight));
                
                controls.style.left = newX + 'px';
                controls.style.top = newY + 'px';
            }, { passive: false });
            
            document.addEventListener('touchend', () => {
                isDragging = false;
            });
        }
        
        // Initialize on page load
        window.addEventListener('DOMContentLoaded', async () => {
            console.log('Stereo VR Vision - Initializing...');
            loadSettings();
            setupControls();
            initDraggable();
            initControllerWebSocket();  // Initialize controller tracking WebSocket
            await initWebRTC();
            await initXR();
        });