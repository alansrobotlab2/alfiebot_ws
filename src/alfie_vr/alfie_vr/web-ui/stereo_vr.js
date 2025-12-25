        // WebXR Stereo Vision for Meta Quest 3
        // Displays side-by-side stereo video as immersive VR
        // With VR Controller Tracking support
        
        import {
            CONFIG,
            stereoSettings,
            connectionState,
            subscriptions,
            xrState,
            videoState,
            robotState,
            leftController,
            rightController,
            headsetPose,
            fpsOverlayState,
            batteryState,
            statePanelState,
            streamPanelState,
            frameTracking,
            shaderState,
            preAllocatedBuffers,
            invalidateSettingsHash,
            passthroughMode,
        } from './state.js';
        import {
            initControllerWebSocket,
            sendControllerData,
            processInputSources,
            processHeadsetPose,
            processThumbstickAdjustments,
            setVrLogFunction,
        } from './controller_tracking.js';
        import {
            setOverlayContext,
            updateGlContext,
            drawStatePanel,
            drawStreamPanel,
            drawRosoutPanel,
            drawBearingPanel,
            setRosoutMessagesCallback,
        } from './vr_overlays.js';
        import {
            FOXGLOVE_PORT,
            setRobotStateContext,
            initFoxgloveConnection,
            getFoxgloveConn,
            getCurrentFrameBitmap,
            getLastFrameReceivedTimestamp,
            getCurrentVrFps,
            getUrdfString,
            getLinkNames,
            getTfRate,
            setTfUpdateCallback,
            setRosoutCallback,
            getRosoutMessages,
            clearRosoutMessages,
        } from './robot_state.js';
        import {
            initRobotViewer,
            loadURDF,
            applyTransform,
            isLoaded as isRobotViewerLoaded,
        } from './robot_viewer.js';
        import {
            initVRRobot,
            loadVRURDF,
            renderVRRobotForView,
            isVRRobotLoaded,
            disposeVRRobot,
            applyVRTransform,
        } from './vr_robot.js';
        import {
            setUrdfPanelContext,
            updateUrdfPanelGlContext,
            loadUrdfForPanel,
            drawUrdfPanel,
            applyUrdfPanelTransform,
            disposeUrdfPanel,
            isUrdfPanelLoaded,
        } from './vr_urdf_panel.js';
        
        // ========================================
        // Local Aliases for Backward Compatibility
        // These reference the shared state objects
        // ========================================
        
        // Video/texture aliases (local to this module)
        let videoElement = null;
        let leftTexture = null;
        let rightTexture = null;
        let leftCanvas = null, rightCanvas = null, leftCtx = null, rightCtx = null;
        let useDirectVideoTexture = false;
        let videoTexture = null;
        let texturesInitialized = false;
        let lastVideoWidth = 0;
        let lastVideoHeight = 0;
        let lastVideoTime = -1;
        let videoFrameCallbackId = null;
        let newVideoFrameAvailable = false;
        
        // XR session aliases
        let xrSession = null;
        let xrRefSpace = null;
        let gl = null;
        
        // Battery state (local - not from robot_state)
        let currentBatteryLevel = null;
        let currentBatteryCharging = false;
        
        // Frame tracking aliases (local render loop state)
        let lastFrameTime = 0;
        let frameCount = 0;
        let totalLatency = 0;
        
        // Shader state aliases
        let shaderProgram = null;
        let positionBuffer = null;
        let texCoordBuffer = null;
        let cachedLocations = null;
        let lastSettingsHash = '';
        
        // Pre-allocated buffers
        const viewMatrixBuffer = preAllocatedBuffers.viewMatrix;
        
        // Retry tracking
        let retryCount = 0;
        const MAX_RETRIES = CONFIG.MAX_RETRIES;
        
        // ========================================
        // End Local Aliases
        // ========================================
        
        // Reinitialize quad buffers when size changes - now invalidates hash to trigger update
        function reinitQuadBuffers() {
            invalidateSettingsHash();
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
        
        // Share vrLog with controller tracking module
        setVrLogFunction(vrLog);
        
        // Set up robot state context with callbacks
        setRobotStateContext({
            setStatus: setStatus,
            showCertButton: showCertButton,
            enableVrButton: () => document.getElementById('vrButton').disabled = false,
            startPreview: startPreview,
            vrLog: vrLog,
        });
        
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

                if (getCurrentFrameBitmap()) {
                    const bitmap = getCurrentFrameBitmap();
                    if (canvas.width !== bitmap.width || canvas.height !== bitmap.height) {
                        canvas.width = bitmap.width;
                        canvas.height = bitmap.height;
                    }
                    ctx.drawImage(bitmap, 0, 0);
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
                
                vrLog('Video ready: ' + (getCurrentFrameBitmap() ? 'YES' : 'NO'));
                if (getCurrentFrameBitmap()) {
                    vrLog('Video: ' + getCurrentFrameBitmap().width + 'x' + getCurrentFrameBitmap().height);
                }
                
                document.getElementById('preview').classList.add('hidden');
                document.getElementById('vrButton').textContent = '🛑 Exit AR';
                document.getElementById('vrButton').onclick = () => xrSession.end();
                
                setStatus('AR Active - Stereo 3D Vision (95% opacity)', 'connected');
                
                // Set up overlay context with callbacks
                setOverlayContext({
                    gl: gl,
                    vrLog: vrLog,
                    getFrameCounter: () => frameCounter,
                    getCurrentVrFps: getCurrentVrFps,
                    getCurrentBatteryLevel: () => currentBatteryLevel,
                    getCurrentBatteryCharging: () => currentBatteryCharging,
                    getLastFrameReceivedTimestamp: getLastFrameReceivedTimestamp,
                    getFoxgloveConn: getFoxgloveConn,
                    getTfRate: getTfRate,
                    getLinkNamesLength: () => getLinkNames().length,
                    getUrdfString: getUrdfString,
                    getShaderProgram: () => shaderProgram,
                    getCachedLocations: () => cachedLocations,
                    invertMatrix: invertMatrix,
                    viewMatrixBuffer: viewMatrixBuffer,
                });
                
                // Set up rosout messages callback for VR overlay
                setRosoutMessagesCallback(getRosoutMessages);
                
                updateGlContext(gl);
                
                // Set up URDF panel context for offscreen Three.js rendering
                setUrdfPanelContext({
                    gl: gl,
                    vrLog: vrLog,
                    getFrameCounter: () => frameCounter,
                    getShaderProgram: () => shaderProgram,
                    getCachedLocations: () => cachedLocations,
                    invertMatrix: invertMatrix,
                    viewMatrixBuffer: viewMatrixBuffer,
                    getUrdfString: getUrdfString,  // Pass URDF getter so panel can check for updates
                });
                updateUrdfPanelGlContext(gl);
                
                // Load URDF for the panel if already available
                const urdf = getUrdfString();
                if (urdf) {
                    loadUrdfForPanel(urdf);
                    vrLog('URDF Panel: loaded');
                }
                
                // Set up TF update callback to update both 2D viewer and VR panel
                setTfUpdateCallback((childFrameId, transform) => {
                    if (isUrdfPanelLoaded()) {
                        applyUrdfPanelTransform(childFrameId, transform);
                    }
                });
                vrLog('URDF Panel TF callback set');
                
                // TODO: VR Robot disabled - Three.js WebGL state conflicts with custom shaders
                // Initialize VR Robot renderer for 3D robot visualization
                // const vrRobotReady = initVRRobot(gl, xrSession);
                // if (vrRobotReady) {
                //     vrLog('VR Robot initialized');
                //     // Load URDF if already available
                //     const urdf = getUrdfString();
                //     if (urdf) {
                //         loadVRURDF(urdf);
                //         vrLog('VR Robot URDF loaded');
                //     }
                //     // Set up TF update callback to update robot pose
                //     setTfUpdateCallback((childFrameId, transform) => {
                //         if (isVRRobotLoaded()) {
                //             applyVRTransform(childFrameId, transform);
                //         }
                //     });
                //     vrLog('VR Robot TF callback set');
                // } else {
                //     vrLog('VR Robot init skipped');
                // }
                
                // Start XR render loop
                xrSession.requestAnimationFrame(onXRFrame);
                
            } catch (err) {
                console.error('VR session error:', err);
                setStatus('VR error: ' + err.message, 'error');
            }
        }
        
        // Initialize textures once with proper parameters
        function initTexture(texture) {
            gl.bindTexture(gl.TEXTURE_2D, texture);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
        }
        
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
            processInputSources(frame, xrRefSpace, xrSession);
            processHeadsetPose(pose);
            processThumbstickAdjustments();
            
            // Send controller data to WebSocket (throttled - every 3 frames)
            if (frameCounter % 3 === 0) {
                sendControllerData();
            }
            
            const glLayer = xrSession.renderState.baseLayer;
            gl.bindFramebuffer(gl.FRAMEBUFFER, glLayer.framebuffer);
            
            // Check passthrough mode - if active, use 0% opacity (full passthrough) and skip rendering
            if (passthroughMode.active) {
                gl.clearColor(0.0, 0.0, 0.0, 0.0);  // 0% opacity - full passthrough
                gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
                frameCounter++;
                return;  // Skip all rendering - panels, video, everything
            }
            
            // Clear with 95% opacity black to dim passthrough (5% passthrough visible)
            gl.clearColor(0.0, 0.0, 0.0, 0.95);  // 95% opacity for immersive AR dimming
            gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
            
            if (!getCurrentFrameBitmap()) {
                if (frameCounter % 60 === 0) {
                    console.log('Waiting for video frame');
                    vrLog('Wait video');
                }
                frameCounter++;
                // Still render status panels while waiting for video
                for (const view of pose.views) {
                    const viewport = glLayer.getViewport(view);
                    gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
                    drawStatePanel(view, viewport, pose.transform.matrix);
                    drawStreamPanel(view, viewport, pose.transform.matrix);
                    drawBearingPanel(view, viewport, pose.transform.matrix);
                    drawRosoutPanel(view, viewport, pose.transform.matrix);
                    drawUrdfPanel(view, viewport, pose.transform.matrix);
                }
                return;
            }
            
            // Get current frame for rendering
            const currentFrameBitmap = getCurrentFrameBitmap();
            
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
                    
                    // TODO: VR robot rendering disabled - Three.js breaks WebGL state
                    // Need to use a different approach (custom WebGL shaders for robot)
                    // if (isVRRobotLoaded()) {
                    //     if (frameCounter === 15) {
                    //         vrLog('Rendering VR robot');
                    //     }
                    //     renderVRRobotForView(view, glLayer.framebuffer, glLayer);
                    // } else if (frameCounter === 15) {
                    //     vrLog('VR robot not loaded yet');
                    // }
                    
                    // Draw status panels (head-locked)
                    drawStatePanel(view, viewport, pose.transform.matrix);
                    drawStreamPanel(view, viewport, pose.transform.matrix);
                    drawBearingPanel(view, viewport, pose.transform.matrix);
                    drawRosoutPanel(view, viewport, pose.transform.matrix);
                    drawUrdfPanel(view, viewport, pose.transform.matrix);
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
                const frameBitmap = getCurrentFrameBitmap();
                leftCtx.drawImage(frameBitmap, 0, 0, halfWidth, videoHeight, 0, 0, halfWidth, videoHeight);
                rightCtx.drawImage(frameBitmap, halfWidth, 0, halfWidth, videoHeight, 0, 0, halfWidth, videoHeight);
                
                // Calculate frame age (how old is the current frame)
                const frameAge = performance.now() - getLastFrameReceivedTimestamp();
                const lagColor = frameAge > 150 ? '#ff4444' : '#00ff00';  // Red if > 150ms, green otherwise
                
                // FPS color coding: green >= 25, yellow 15-25, red < 15
                const vrFps = getCurrentVrFps();
                let fpsColor;
                if (vrFps >= 25) {
                    fpsColor = '#00ff00';  // Green
                } else if (vrFps >= 15) {
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
                    
                    // TODO: VR robot rendering disabled - Three.js breaks WebGL state
                    // Need to use a different approach (custom WebGL shaders for robot)
                    // if (isVRRobotLoaded()) {
                    //     if (frameCounter === 15) {
                    //         vrLog('Rendering VR robot (canvas mode)');
                    //     }
                    //     renderVRRobotForView(view, glLayer.framebuffer, glLayer);
                    // } else if (frameCounter === 15) {
                    //     vrLog('VR robot not loaded yet (canvas mode)');
                    // }
                    
                    // Draw status panels (head-locked)
                    drawStatePanel(view, viewport, pose.transform.matrix);
                    drawStreamPanel(view, viewport, pose.transform.matrix);
                    drawBearingPanel(view, viewport, pose.transform.matrix);
                    drawRosoutPanel(view, viewport, pose.transform.matrix);
                    drawUrdfPanel(view, viewport, pose.transform.matrix);
                }
            }
        }
        
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
            
            // OPTIMIZED: Fragment shader that can handle both modes with rounded corners
            const fragmentShader = gl.createShader(gl.FRAGMENT_SHADER);
            gl.shaderSource(fragmentShader, `
                precision mediump float;
                varying vec2 v_texCoord;
                varying float v_isLeftEye;
                uniform sampler2D u_texture;
                uniform float u_useDirectVideo;
                uniform float u_cornerRadius;
                
                // Signed distance function for rounded rectangle
                float roundedBoxSDF(vec2 centerPos, vec2 halfSize, float radius) {
                    vec2 q = abs(centerPos) - halfSize + radius;
                    return min(max(q.x, q.y), 0.0) + length(max(q, 0.0)) - radius;
                }
                
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
                    
                    // Apply rounded corners using SDF
                    // Convert tex coords to centered coordinates (-0.5 to 0.5)
                    vec2 centered = v_texCoord - 0.5;
                    float dist = roundedBoxSDF(centered, vec2(0.5), u_cornerRadius);
                    
                    // Discard pixels outside rounded rectangle
                    if (dist > 0.0) {
                        discard;
                    }
                    
                    // Smooth edge with anti-aliasing (optional - slight softness at edges)
                    float alpha = 1.0 - smoothstep(-0.005, 0.0, dist);
                    
                    vec4 color = texture2D(u_texture, texCoord);
                    gl_FragColor = vec4(color.rgb, color.a * alpha);
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
                useDirectVideo: gl.getUniformLocation(shaderProgram, 'u_useDirectVideo'),
                cornerRadius: gl.getUniformLocation(shaderProgram, 'u_cornerRadius')
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
            
            if (cachedLocations.cornerRadius !== null) {
                gl.uniform1f(cachedLocations.cornerRadius, stereoSettings.cornerRadius);
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
            
            // Disable blending for video quad to ensure full opacity (no passthrough behind video)
            gl.disable(gl.BLEND);
            
            // Draw quad
            gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
            
            // Re-enable blending for other overlays
            gl.enable(gl.BLEND);
            
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
            
            // Clear TF callback and dispose VR robot resources
            setTfUpdateCallback(null);
            disposeVRRobot();
            disposeUrdfPanel();
            
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
        
        // ========================================
        // Rosout Log Panel Functions
        // ========================================
        
        function initRosoutPanel() {
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
                    messagesDiv.innerHTML = '';
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
                    messagesDiv.innerHTML = '';
                    return;
                }
                addRosoutMessageToPanel(logMessage, messagesDiv);
            });
            
            console.log('Rosout panel initialized');
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
            
            // Get level class
            const levelClass = logMessage.levelName.toLowerCase();
            
            // Create message element
            const msgEl = document.createElement('div');
            msgEl.className = `rosout-msg ${levelClass}`;
            msgEl.innerHTML = `
                <span class="rosout-time">${timeStr}</span>
                <span class="rosout-level">[${logMessage.levelName}]</span>
                <span class="rosout-name">${logMessage.name}</span>
                <span class="rosout-text">${escapeHtml(logMessage.msg)}</span>
            `;
            
            messagesDiv.appendChild(msgEl);
            
            // Auto-scroll to bottom
            messagesDiv.scrollTop = messagesDiv.scrollHeight;
            
            // Limit displayed messages
            while (messagesDiv.children.length > 50) {
                messagesDiv.removeChild(messagesDiv.firstChild);
            }
        }
        
        function escapeHtml(text) {
            const div = document.createElement('div');
            div.textContent = text;
            return div.innerHTML;
        }
        
        // Initialize 3D robot viewer and watch for URDF data
        function initRobotViewer3D() {
            // Initialize the Three.js viewer
            const viewerReady = initRobotViewer('robotViewerContainer');
            if (!viewerReady) {
                console.log('Robot 3D viewer initialization deferred (container or Three.js not ready)');
                return;
            }
            
            // Poll for URDF string from robot_state.js and load when available
            const checkForURDF = setInterval(() => {
                const urdf = getUrdfString();
                if (urdf) {
                    // Load into desktop viewer
                    if (!isRobotViewerLoaded()) {
                        console.log('URDF available, loading into 3D viewer...');
                        loadURDF(urdf);
                        
                        // Set up TF callback to update robot pose in desktop viewer
                        setTfUpdateCallback((childFrameId, transform) => {
                            if (isRobotViewerLoaded()) {
                                applyTransform(childFrameId, transform);
                            }
                        });
                        console.log('TF callback set for 3D robot viewer');
                    }
                    // Also load into VR robot if VR session is active
                    if (!isVRRobotLoaded() && xrSession) {
                        console.log('URDF available, loading into VR robot...');
                        loadVRURDF(urdf);
                    }
                    // Stop polling once both are loaded (or VR not active)
                    if (isRobotViewerLoaded() && (!xrSession || isVRRobotLoaded())) {
                        clearInterval(checkForURDF);
                    }
                }
            }, 500);
            
            // Stop polling after 60 seconds
            setTimeout(() => clearInterval(checkForURDF), 60000);
        }
        
        // Initialize on page load
        window.addEventListener('DOMContentLoaded', async () => {
            console.log('Stereo VR Vision - Initializing...');
            initBatteryMonitor();
            initRobotPanel();  // Initialize robot status panel
            initRosoutPanel();  // Initialize rosout log panel
            initRobotViewer3D();  // Initialize 3D robot viewer
            initControllerWebSocket();  // Initialize controller tracking WebSocket
            initFoxgloveConnection();
            await initXR();
        });