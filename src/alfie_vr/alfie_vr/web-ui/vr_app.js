// Wait for A-Frame scene to load

// VR Robot Viewer Component - displays robot state in VR
AFRAME.registerComponent('vr-robot-viewer', {
  init: function () {
    console.log('VR Robot Viewer component initialized');
    
    this.robotModel = document.querySelector('#vrRobotModel');
    this.statusText = document.querySelector('#vrRobotStatus');
    this.foxgloveStatusText = document.querySelector('#vrFoxgloveStatus');
    this.tfCountText = document.querySelector('#vrTFCount');
    this.links = {};
    this.joints = {};
    this.foxgloveClient = null;
    this.tfSubscriptionId = null;
    this.robotDescriptionSubscriptionId = null;
    this.urdfLoaded = false;
    this.connectionAttempts = 0;
    
    // Throttling for performance - limit updates to 10 FPS
    this.targetFPS = 10;
    this.updateInterval = 1000 / this.targetFPS;
    this.lastTFUpdateTime = 0;
    this.lastJointUpdateTime = 0;
    this.pendingTFData = null;
    this.pendingJointData = null;
    this.tfUpdateCount = 0;
    
    // Connect to Foxglove Bridge using WebSocket via nginx proxy
    // Port 8082 is nginx with TLS, proxying to foxglove_bridge on 8765
    const foxgloveUrl = `wss://${window.location.hostname}:8082`;
    this.connectToFoxglove(foxgloveUrl);
  },
  
  connectToFoxglove: function(url) {
    this.connectionAttempts++;
    console.log(`VR Viewer: Connecting to Foxglove at ${url} (attempt ${this.connectionAttempts})`);
    this.updateStatus(`Connecting... #${this.connectionAttempts}`);
    this.updateFoxgloveStatus(false);
    
    try {
      // Use the Foxglove SDK WebSocket subprotocol (required for foxglove_bridge 3.x)
      this.foxgloveClient = new WebSocket(url, ['foxglove.sdk.v1']);
      this.foxgloveClient.binaryType = 'arraybuffer';
      
      this.foxgloveClient.onopen = () => {
        console.log('VR Viewer: Connected to Foxglove');
        this.updateStatus('Connected');
        this.updateFoxgloveStatus(true);
        this.connectionAttempts = 0;
      };
      
      this.foxgloveClient.onmessage = (event) => {
        this.handleMessage(event.data);
      };
      
      this.foxgloveClient.onerror = (error) => {
        console.error('VR Viewer: Foxglove error', error);
        this.updateStatus('Error');
        this.updateFoxgloveStatus(false);
      };
      
      this.foxgloveClient.onclose = (event) => {
        console.log('VR Viewer: Foxglove disconnected. Code:', event.code, 'Reason:', event.reason);
        this.updateStatus('Disconnected');
        this.updateFoxgloveStatus(false);
        
        // Retry in 5 seconds
        setTimeout(() => this.connectToFoxglove(url), 5000);
      };
    } catch (error) {
      console.error('VR Viewer: Failed to connect', error);
      this.updateStatus('Exception');
      this.updateFoxgloveStatus(false);
    }
  },
  
  updateFoxgloveStatus: function(connected) {
    if (this.foxgloveStatusText) {
      this.foxgloveStatusText.setAttribute('value', connected ? 'Connected' : 'Disconnected');
      this.foxgloveStatusText.setAttribute('color', connected ? '#4CAF50' : '#f44336');
    }
  },
  
  updateTFCount: function() {
    if (this.tfCountText) {
      this.tfCountText.setAttribute('value', this.tfUpdateCount.toString());
    }
  },
  
  handleMessage: function(data) {
    try {
      if (typeof data === 'string') {
        const message = JSON.parse(data);
        console.log('VR: JSON message received, op:', message.op);
        
        if (message.op === 'serverInfo') {
          console.log('VR: Server info received:', message.name, 'capabilities:', message.capabilities);
        } else if (message.op === 'advertise') {
          console.log('VR: Advertise received, channels:', message.channels?.length);
          this.subscribeToTopics(message.channels);
        } else if (message.op === 'message') {
          this.handleTopicMessage(message);
        }
      } else if (data instanceof ArrayBuffer) {
        this.handleBinaryMessage(data);
      } else {
        console.log('VR: Unknown message type:', typeof data);
      }
    } catch (error) {
      // Ignore parse errors for binary messages
    }
  },
  
  handleBinaryMessage: function(data) {
    // Foxglove binary message format
    const view = new DataView(data);
    const opCode = view.getUint8(0);
    
    if (opCode === 1) {  // Message data
      const subscriptionId = view.getUint32(1, true);
      const timestamp = view.getBigUint64(5, true);
      const messageData = new Uint8Array(data, 13);
      
      // Handle robot_description (CDR-encoded std_msgs/String)
      if (subscriptionId === this.robotDescriptionSubscriptionId) {
        this.handleRobotDescriptionBinary(messageData);
      }
      // Handle TF (CDR-encoded tf2_msgs/TFMessage)
      else if (subscriptionId === this.tfSubscriptionId) {
        this.handleTFBinary(messageData);
      }
    }
  },
  
  handleTFBinary: function(data) {
    try {
      // Simple CDR decoder for TFMessage
      let offset = 4;  // Skip CDR header
      const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
      
      const readString = () => {
        const len = view.getUint32(offset, true);
        offset += 4;
        const strBytes = new Uint8Array(data.buffer, data.byteOffset + offset, len - 1);
        offset += len;
        if (offset % 4 !== 0) offset += 4 - (offset % 4);
        return new TextDecoder().decode(strBytes);
      };
      
      const align = (n) => { if (offset % n !== 0) offset += n - (offset % n); };
      
      // TFMessage: TransformStamped[] transforms
      const transformCount = view.getUint32(offset, true); offset += 4;
      
      for (let i = 0; i < transformCount; i++) {
        // Header
        const stampSec = view.getUint32(offset, true); offset += 4;
        const stampNsec = view.getUint32(offset, true); offset += 4;
        const frameId = readString();
        const childFrameId = readString();
        
        // Transform: translation (Vector3), rotation (Quaternion)
        // CDR alignment quirk: after child_frame_id, ensure offset % 8 === 4
        if (offset % 8 === 0) offset += 4;
        const tx = view.getFloat64(offset, true); offset += 8;
        const ty = view.getFloat64(offset, true); offset += 8;
        const tz = view.getFloat64(offset, true); offset += 8;
        const qx = view.getFloat64(offset, true); offset += 8;
        const qy = view.getFloat64(offset, true); offset += 8;
        const qz = view.getFloat64(offset, true); offset += 8;
        const qw = view.getFloat64(offset, true); offset += 8;
        
        // Apply transform to corresponding link
        const link = this.links[childFrameId];
        if (link && link.object3D) {
          link.object3D.position.set(tx, ty, tz);
          link.object3D.quaternion.set(qx, qy, qz, qw);
        }
      }
    } catch (error) {
      // Silently ignore TF decode errors
    }
  },
  
  handleRobotDescriptionBinary: function(data) {
    try {
      // CDR format for ROS2 std_msgs/msg/String:
      // - 4 bytes: CDR encapsulation header (00 01 00 00 for little-endian)
      // - 4 bytes: string length (uint32, little-endian, includes null terminator)
      // - N bytes: string data (UTF-8)
      // - 1 byte: null terminator
      const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
      
      // Skip 4-byte CDR encapsulation header, then read string length
      const stringLength = view.getUint32(4, true);
      
      // Extract string (skip 8 bytes header, exclude null terminator)
      const stringData = new Uint8Array(data.buffer, data.byteOffset + 8, stringLength - 1);
      const decoder = new TextDecoder();
      const urdfString = decoder.decode(stringData);
      
      console.log('VR: Decoded robot_description CDR, length:', stringLength);
      
      if (urdfString.includes('<robot')) {
        console.log('VR: Found URDF, parsing...');
        this.parseURDF(urdfString);
      } else {
        console.log('VR: No <robot> tag found in data');
      }
    } catch (error) {
      console.error('VR: Failed to parse robot_description CDR:', error);
    }
  },
  
  subscribeToTopics: function(channels) {
    console.log('VR: subscribeToTopics called with', channels?.length, 'channels');
    const subscriptions = [];
    
    for (const channel of channels) {
      if (channel.topic === '/alfie/tf') {
        this.tfSubscriptionId = subscriptions.length + 1;
        subscriptions.push({ id: this.tfSubscriptionId, channelId: channel.id });
        console.log('VR: Found /alfie/tf, subId:', this.tfSubscriptionId);
      }
      if (channel.topic === '/alfie/robot_description') {
        this.robotDescriptionSubscriptionId = subscriptions.length + 1;
        subscriptions.push({ id: this.robotDescriptionSubscriptionId, channelId: channel.id });
        console.log('VR: Found /alfie/robot_description, subId:', this.robotDescriptionSubscriptionId);
      }
    }
    
    console.log('VR: Total subscriptions:', subscriptions.length);
    
    if (subscriptions.length > 0) {
      this.foxgloveClient.send(JSON.stringify({ op: 'subscribe', subscriptions }));
      this.updateStatus(`Subscribed (${subscriptions.length})`);
      
      if (!this.robotDescriptionSubscriptionId) {
        console.log('VR: No robot_description found, loading placeholder');
        this.loadPlaceholderRobot();
      } else {
        console.log('VR: Waiting for robot_description binary message...');
      }
    } else {
      console.log('VR: No topics matched for subscription');
      this.updateStatus('No topics found');
    }
  },
  
  handleTopicMessage: function(message) {
    if (message.topic === '/alfie/tf') {
      this.processTFData(message.message);
    } else if (message.topic === '/alfie/joint_states') {
      this.processJointState(message.message);
    } else if (message.topic === '/alfie/robot_description') {
      this.processRobotDescription(message.message);
    }
  },
  
  processTFData: function(tfData) {
    if (!tfData || !tfData.transforms) return;
    
    // Throttle TF updates for performance
    const now = performance.now();
    if (now - this.lastTFUpdateTime < this.updateInterval) {
      // Store for next update cycle
      this.pendingTFData = tfData;
      return;
    }
    this.lastTFUpdateTime = now;
    this.pendingTFData = null;
    
    for (const transform of tfData.transforms) {
      const childFrame = transform.child_frame_id;
      const t = transform.transform.translation;
      const r = transform.transform.rotation;
      
      const linkEl = this.links[childFrame];
      if (linkEl) {
        linkEl.object3D.position.set(t.x, t.y, t.z);
        linkEl.object3D.quaternion.set(r.x, r.y, r.z, r.w);
      }
    }
    
    this.tfUpdateCount++;
    // Update TF count display every 10 updates (throttled for performance)
    if (this.tfUpdateCount % 10 === 0) {
      this.updateTFCount();
    }
  },
  
  processJointState: function(jointData) {
    if (!jointData || !jointData.name || !jointData.position) return;
    
    // Throttle joint updates for performance
    const now = performance.now();
    if (now - this.lastJointUpdateTime < this.updateInterval) {
      this.pendingJointData = jointData;
      return;
    }
    this.lastJointUpdateTime = now;
    this.pendingJointData = null;
    
    for (let i = 0; i < jointData.name.length; i++) {
      const jointName = jointData.name[i];
      const position = jointData.position[i];
      
      const joint = this.joints[jointName];
      if (joint && joint.el) {
        const axis = joint.axis || [0, 0, 1];
        // Apply rotation around joint axis
        if (joint.type === 'revolute' || joint.type === 'continuous') {
          joint.el.object3D.rotation.set(
            axis[0] * position,
            axis[1] * position,
            axis[2] * position
          );
        }
      }
    }
  },
  
  processRobotDescription: function(descData) {
    if (this.urdfLoaded) return;
    
    let urdfString = null;
    if (typeof descData === 'string') {
      urdfString = descData;
    } else if (descData.data) {
      urdfString = descData.data;
    }
    
    if (urdfString && urdfString.includes('<robot')) {
      this.parseURDF(urdfString);
    }
  },
  
  parseURDF: function(urdfString) {
    if (this.urdfLoaded) return;
    
    try {
      const parser = new DOMParser();
      const urdfDoc = parser.parseFromString(urdfString, 'text/xml');
      const robotEl = urdfDoc.querySelector('robot');
      
      if (!robotEl) return;
      
      // Clear existing model
      while (this.robotModel.firstChild) {
        this.robotModel.removeChild(this.robotModel.firstChild);
      }
      
      // Parse material definitions from URDF root
      const materialDefs = {};
      urdfDoc.querySelectorAll('robot > material').forEach(matEl => {
        const matName = matEl.getAttribute('name');
        const colorEl = matEl.querySelector('color');
        if (matName && colorEl) {
          const rgba = colorEl.getAttribute('rgba').split(' ').map(parseFloat);
          const r = Math.round(rgba[0] * 255).toString(16).padStart(2, '0');
          const g = Math.round(rgba[1] * 255).toString(16).padStart(2, '0');
          const b = Math.round(rgba[2] * 255).toString(16).padStart(2, '0');
          materialDefs[matName] = `#${r}${g}${b}`;
        }
      });
      console.log('VR: Parsed material definitions:', Object.keys(materialDefs));
      
      // Helper function to get material color as hex string
      const getMaterialColor = (materialEl) => {
        if (!materialEl) return '#666666';
        
        // First check for inline color
        const colorEl = materialEl.querySelector('color');
        if (colorEl) {
          const rgba = colorEl.getAttribute('rgba').split(' ').map(parseFloat);
          const r = Math.round(rgba[0] * 255).toString(16).padStart(2, '0');
          const g = Math.round(rgba[1] * 255).toString(16).padStart(2, '0');
          const b = Math.round(rgba[2] * 255).toString(16).padStart(2, '0');
          return `#${r}${g}${b}`;
        }
        
        // Otherwise look up by name
        const matName = materialEl.getAttribute('name');
        if (matName && materialDefs[matName]) {
          return materialDefs[matName];
        }
        
        return '#666666';
      };
      
      // Parse links and create A-Frame entities
      const linkElements = urdfDoc.querySelectorAll('link');
      linkElements.forEach(linkEl => {
        const linkName = linkEl.getAttribute('name');
        const visuals = linkEl.querySelectorAll('visual');  // Get ALL visuals
        
        // Create parent entity for the link
        const linkEntity = document.createElement('a-entity');
        linkEntity.setAttribute('id', `vr-link-${linkName}`);
        
        // Process each visual element in this link
        visuals.forEach((visual, visualIndex) => {
          const geometry = visual.querySelector('geometry');
          const material = visual.querySelector('material');
          const origin = visual.querySelector('origin');
          
          if (geometry) {
            // Create a child entity for this visual
            const entity = document.createElement('a-entity');
            
            const box = geometry.querySelector('box');
            const cylinder = geometry.querySelector('cylinder');
            const sphere = geometry.querySelector('sphere');
            const mesh = geometry.querySelector('mesh');
            
            let geomStr = '';
            if (box) {
              const size = box.getAttribute('size').split(' ');
              geomStr = `primitive: box; width: ${size[0]}; height: ${size[1]}; depth: ${size[2]}`;
              entity.setAttribute('geometry', geomStr);
            } else if (cylinder) {
              const r = cylinder.getAttribute('radius');
              const h = cylinder.getAttribute('length');
              geomStr = `primitive: cylinder; radius: ${r}; height: ${h}`;
              entity.setAttribute('geometry', geomStr);
            } else if (sphere) {
              const r = sphere.getAttribute('radius');
              geomStr = `primitive: sphere; radius: ${r}`;
              entity.setAttribute('geometry', geomStr);
            } else if (mesh) {
              // Load OBJ mesh
              const filename = mesh.getAttribute('filename');
              const scale = mesh.getAttribute('scale');
              
              if (filename && filename.startsWith('package://alfie_urdf/meshes/')) {
                // Convert package:// URL to web server URL
                const meshName = filename.replace('package://alfie_urdf/meshes/', '');
                const meshUrl = `/meshes/${meshName}`;
                
                // Use A-Frame's obj-model component
                entity.setAttribute('obj-model', `obj: url(${meshUrl})`);
                
                // Apply scale if specified
                if (scale) {
                  const scaleVals = scale.split(' ').map(parseFloat);
                  entity.setAttribute('scale', `${scaleVals[0]} ${scaleVals[1]} ${scaleVals[2]}`);
                }
                
                console.log('VR: Loading mesh:', meshUrl);
              } else {
                // Fallback placeholder for unknown mesh paths
                entity.setAttribute('geometry', 'primitive: box; width: 0.03; height: 0.03; depth: 0.03');
              }
            } else {
              // Placeholder for unknown geometry
              geomStr = 'primitive: box; width: 0.03; height: 0.03; depth: 0.03';
              entity.setAttribute('geometry', geomStr);
            }
            
            // Material color - use helper to resolve named materials
            const color = getMaterialColor(material);
            entity.setAttribute('material', `color: ${color}; metalness: 0.3; roughness: 0.7`);
            
            // Origin offset for this visual
            if (origin) {
              const xyz = origin.getAttribute('xyz');
              const rpy = origin.getAttribute('rpy');
              if (xyz) entity.setAttribute('position', xyz.replace(/ /g, ' '));
              if (rpy) {
                const rot = rpy.split(' ').map(r => THREE.MathUtils.radToDeg(parseFloat(r)));
                entity.setAttribute('rotation', `${rot[0]} ${rot[1]} ${rot[2]}`);
                // Set Euler order for URDF fixed-axis RPY
                entity.addEventListener('loaded', () => {
                  if (entity.object3D) entity.object3D.rotation.order = 'ZYX';
                });
              }
            }
            
            // Add visual entity to link entity
            linkEntity.appendChild(entity);
          }
        });  // End of visuals.forEach
        
        this.links[linkName] = linkEntity;
        // Don't add to robotModel yet - we'll build hierarchy from joints
      });
      
      // Parse joints and build parent-child hierarchy
      const jointElements = urdfDoc.querySelectorAll('joint');
      const childLinks = new Set();  // Track which links are children
      
      jointElements.forEach(jointEl => {
        const jointName = jointEl.getAttribute('name');
        const jointType = jointEl.getAttribute('type');
        const parent = jointEl.querySelector('parent')?.getAttribute('link');
        const child = jointEl.querySelector('child')?.getAttribute('link');
        const axis = jointEl.querySelector('axis');
        const origin = jointEl.querySelector('origin');
        
        if (parent && child && this.links[parent] && this.links[child]) {
          childLinks.add(child);
          
          const axisVec = axis ? axis.getAttribute('xyz').split(' ').map(parseFloat) : [0, 0, 1];
          
          this.joints[jointName] = {
            el: this.links[child],
            type: jointType,
            axis: axisVec
          };
          
          // Apply joint origin transform to child link
          if (origin) {
            const xyz = origin.getAttribute('xyz');
            const rpy = origin.getAttribute('rpy');
            if (xyz) {
              const pos = xyz.split(' ').map(parseFloat);
              this.links[child].setAttribute('position', `${pos[0]} ${pos[1]} ${pos[2]}`);
            }
            if (rpy) {
              const rot = rpy.split(' ').map(r => THREE.MathUtils.radToDeg(parseFloat(r)));
              this.links[child].setAttribute('rotation', `${rot[0]} ${rot[1]} ${rot[2]}`);
              // Set Euler order for URDF fixed-axis RPY
              const childLink = this.links[child];
              childLink.addEventListener('loaded', () => {
                if (childLink.object3D) childLink.object3D.rotation.order = 'ZYX';
              });
            }
          }
          
          // Add child link as child of parent link (makes transforms accumulate)
          this.links[parent].appendChild(this.links[child]);
        }
      });
      
      // Add root links (those that aren't children of any joint) to the robot model
      Object.keys(this.links).forEach(linkName => {
        if (!childLinks.has(linkName)) {
          this.robotModel.appendChild(this.links[linkName]);
          console.log('VR: Root link:', linkName);
        }
      });
      
      // Rotate robot model to convert from ROS (Z-up) to A-Frame (Y-up)
      this.robotModel.setAttribute('rotation', '-90 0 0');
      
      this.urdfLoaded = true;
      this.updateStatus(`Loaded ${Object.keys(this.links).length} links`);
      console.log('VR Viewer: URDF loaded');
      
    } catch (error) {
      console.error('VR Viewer: URDF parse error', error);
      this.updateStatus('URDF error');
    }
  },
  
  loadPlaceholderRobot: function() {
    console.log('VR Viewer: Loading placeholder robot');
    
    const parts = [
      { name: 'base', geom: 'primitive: box; width: 0.15; height: 0.1; depth: 0.2', pos: '0 0.05 0', color: '#333333' },
      { name: 'back', geom: 'primitive: box; width: 0.14; height: 0.4; depth: 0.08', pos: '0 0.35 -0.04', color: '#444444' },
      { name: 'head', geom: 'primitive: sphere; radius: 0.08', pos: '0 0.85 0', color: '#222222' },
      { name: 'left_upper', geom: 'primitive: cylinder; radius: 0.02; height: 0.2', pos: '0.12 0.6 0.08', color: '#555555' },
      { name: 'left_lower', geom: 'primitive: cylinder; radius: 0.018; height: 0.18', pos: '0.12 0.45 0.18', color: '#555555' },
      { name: 'right_upper', geom: 'primitive: cylinder; radius: 0.02; height: 0.2', pos: '-0.12 0.6 0.08', color: '#555555' },
      { name: 'right_lower', geom: 'primitive: cylinder; radius: 0.018; height: 0.18', pos: '-0.12 0.45 0.18', color: '#555555' },
    ];
    
    parts.forEach(part => {
      const entity = document.createElement('a-entity');
      entity.setAttribute('geometry', part.geom);
      entity.setAttribute('material', `color: ${part.color}; metalness: 0.3; roughness: 0.7`);
      entity.setAttribute('position', part.pos);
      this.links[part.name] = entity;
      this.robotModel.appendChild(entity);
    });
    
    this.updateStatus('Placeholder loaded');
  },
  
  updateStatus: function(message) {
    if (this.statusText) {
      this.statusText.setAttribute('value', message);
    }
  }
});

AFRAME.registerComponent('controller-updater', {
  init: function () {
    console.log("Controller updater component initialized.");
    // Controllers are enabled

    this.leftHand = document.querySelector('#leftHand');
    this.rightHand = document.querySelector('#rightHand');
    this.leftHandInfoText = document.querySelector('#leftHandInfo');
    this.rightHandInfoText = document.querySelector('#rightHandInfo');
    
    // Add headset tracking
    this.headset = document.querySelector('#headset');
    this.headsetInfoText = document.querySelector('#headsetInfo');

    // --- WebSocket Setup ---
    this.websocket = null;
    this.leftGripDown = false;
    this.rightGripDown = false;
    this.leftTriggerDown = false;
    this.rightTriggerDown = false;

    // --- Status reporting ---
    this.lastStatusUpdate = 0;
    this.statusUpdateInterval = 5000; // 5 seconds

    // --- Relative rotation tracking ---
    this.leftGripInitialRotation = null;
    this.rightGripInitialRotation = null;
    this.leftRelativeRotation = { x: 0, y: 0, z: 0 };
    this.rightRelativeRotation = { x: 0, y: 0, z: 0 };

    // --- Quaternion-based Z-axis rotation tracking ---
    this.leftGripInitialQuaternion = null;
    this.rightGripInitialQuaternion = null;
    this.leftZAxisRotation = 0;
    this.rightZAxisRotation = 0;

    // --- Get hostname dynamically ---
    const serverHostname = window.location.hostname;
    const websocketPort = 8442; // Make sure this matches controller_server.py
    const websocketUrl = `wss://${serverHostname}:${websocketPort}`;
    console.log(`Attempting WebSocket connection to: ${websocketUrl}`);

    // --- Video Stream Setup (WebRTC - hardware accelerated, low latency) ---
    this.videoElement = document.getElementById('webrtcVideo');
    this.videoCanvas = document.getElementById('videoCanvas');
    this.videoContext = this.videoCanvas.getContext('2d');
    this.videoScreen = document.querySelector('#videoScreen');
    this.peerConnection = null;
    this.webrtcConnected = false;
    
    const webrtcPort = 8083;
    const webrtcUrl = `https://${serverHostname}:${webrtcPort}/offer`;
    
    // Initialize WebRTC connection
    this.initWebRTC = async () => {
      console.log('Initializing WebRTC video stream...');
      
      try {
        // Create peer connection with hardware-accelerated codec preferences
        const config = {
          iceServers: [], // Direct connection, no STUN/TURN needed on LAN
          sdpSemantics: 'unified-plan'
        };
        this.peerConnection = new RTCPeerConnection(config);
        
        // Handle incoming video track
        this.peerConnection.ontrack = (event) => {
          console.log('WebRTC: Received video track');
          if (event.track.kind === 'video') {
            this.videoElement.srcObject = event.streams[0];
            this.videoElement.play().catch(e => console.log('Video autoplay blocked:', e));
            this.webrtcConnected = true;
            
            // Update A-Frame video screen to use video element
            if (this.videoScreen) {
              this.videoScreen.setAttribute('visible', 'true');
              // Use video element as texture source
              this.videoScreen.setAttribute('material', 'src: #webrtcVideo; shader: flat');
            }
          }
        };
        
        this.peerConnection.oniceconnectionstatechange = () => {
          console.log('WebRTC ICE state:', this.peerConnection.iceConnectionState);
          if (this.peerConnection.iceConnectionState === 'disconnected' || 
              this.peerConnection.iceConnectionState === 'failed') {
            this.webrtcConnected = false;
            if (this.videoScreen) {
              this.videoScreen.setAttribute('visible', 'false');
            }
            // Retry connection after 3 seconds
            setTimeout(() => this.initWebRTC(), 3000);
          }
        };
        
        // Add transceiver for receiving video
        this.peerConnection.addTransceiver('video', { direction: 'recvonly' });
        
        // Create offer
        const offer = await this.peerConnection.createOffer();
        await this.peerConnection.setLocalDescription(offer);
        
        // Send offer to signaling server
        console.log('WebRTC: Sending offer to', webrtcUrl);
        const response = await fetch(webrtcUrl, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            sdp: this.peerConnection.localDescription.sdp,
            type: this.peerConnection.localDescription.type
          })
        });
        
        if (!response.ok) {
          throw new Error(`Signaling failed: ${response.status}`);
        }
        
        const answer = await response.json();
        console.log('WebRTC: Received answer');
        await this.peerConnection.setRemoteDescription(new RTCSessionDescription(answer));
        
        console.log('WebRTC: Connection established!');
        
      } catch (error) {
        console.error('WebRTC initialization failed:', error);
        console.log('Will retry in 5 seconds...');
        // Retry connection
        setTimeout(() => this.initWebRTC(), 5000);
      }
    };
    
    // Start WebRTC connection
    this.initWebRTC();

    // !!! IMPORTANT: Replace 'YOUR_LAPTOP_IP' with the actual IP address of your laptop !!!
    // const websocketUrl = 'ws://YOUR_LAPTOP_IP:8442';
    try {
      this.websocket = new WebSocket(websocketUrl);
      this.websocket.onopen = (event) => {
        console.log(`WebSocket connected to ${websocketUrl}`);
        this.reportVRStatus(true);
      };
      this.websocket.onerror = (event) => {
        // More detailed error logging
        console.error(`WebSocket Error: Event type: ${event.type}`, event);
        this.reportVRStatus(false);
      };
      this.websocket.onclose = (event) => {
        console.log(`WebSocket disconnected from ${websocketUrl}. Clean close: ${event.wasClean}, Code: ${event.code}, Reason: '${event.reason}'`);
        // Attempt to log specific error if available (might be limited by browser security)
        if (!event.wasClean) {
          console.error('WebSocket closed unexpectedly.');
        }
        this.websocket = null; // Clear the reference
        this.reportVRStatus(false);
      };
      this.websocket.onmessage = (event) => {
        console.log(`WebSocket message received: ${event.data}`); // Log any messages from server
      };
    } catch (error) {
        console.error(`Failed to create WebSocket connection to ${websocketUrl}:`, error);
        this.reportVRStatus(false);
    }
    // --- End WebSocket Setup ---

    // --- VR Status Reporting Function ---
    this.reportVRStatus = (connected) => {
      // Update global status if available (for desktop interface)
      if (typeof updateStatus === 'function') {
        updateStatus({ vrConnected: connected });
      }
      
      // Also try to notify parent window if in iframe
      try {
        if (window.parent && window.parent !== window) {
          window.parent.postMessage({
            type: 'vr_status',
            connected: connected
          }, '*');
        }
      } catch (e) {
        // Ignore cross-origin errors
      }
    };

    if (!this.leftHand || !this.rightHand || !this.leftHandInfoText || !this.rightHandInfoText) {
      console.error("Controller or text entities not found!");
      // Check which specific elements are missing
      if (!this.leftHand) console.error("Left hand entity not found");
      if (!this.rightHand) console.error("Right hand entity not found");
      if (!this.leftHandInfoText) console.error("Left hand info text not found");
      if (!this.rightHandInfoText) console.error("Right hand info text not found");
      return;
    }

    // Apply initial rotation to combined text elements
    const textRotation = '-90 0 0'; // Rotate -90 degrees around X-axis
    if (this.leftHandInfoText) this.leftHandInfoText.setAttribute('rotation', textRotation);
    if (this.rightHandInfoText) this.rightHandInfoText.setAttribute('rotation', textRotation);

    // --- Create axis indicators ---
    this.createAxisIndicators();

    // --- Helper function to send grip release message ---
    this.sendGripRelease = (hand) => {
      if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
        const releaseMessage = {
          hand: hand,
          gripReleased: true
        };
        this.websocket.send(JSON.stringify(releaseMessage));
        console.log(`Sent grip release for ${hand} hand`);
      }
    };

    // --- Helper function to send trigger release message ---
    this.sendTriggerRelease = (hand) => {
      if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
        const releaseMessage = {
          hand: hand,
          triggerReleased: true
        };
        this.websocket.send(JSON.stringify(releaseMessage));
        console.log(`Sent trigger release for ${hand} hand`);
      }
    };

    // --- Helper function to calculate relative rotation ---
    this.calculateRelativeRotation = (currentRotation, initialRotation) => {
      return {
        x: currentRotation.x - initialRotation.x,
        y: currentRotation.y - initialRotation.y,
        z: currentRotation.z - initialRotation.z
      };
    };

    // --- Helper function to calculate Z-axis rotation from quaternions ---
    this.calculateZAxisRotation = (currentQuaternion, initialQuaternion) => {
      // Calculate relative quaternion (from initial to current)
      const relativeQuat = new THREE.Quaternion();
      relativeQuat.multiplyQuaternions(currentQuaternion, initialQuaternion.clone().invert());
      
      // Get the controller's current forward direction (local Z-axis in world space)
      const forwardDirection = new THREE.Vector3(0, 0, 1);
      forwardDirection.applyQuaternion(currentQuaternion);
      
      // Convert relative quaternion to axis-angle representation
      const angle = 2 * Math.acos(Math.abs(relativeQuat.w));
      
      // Handle case where there's no rotation (avoid division by zero)
      if (angle < 0.0001) {
        return 0;
      }
      
      // Get the rotation axis
      const sinHalfAngle = Math.sqrt(1 - relativeQuat.w * relativeQuat.w);
      const rotationAxis = new THREE.Vector3(
        relativeQuat.x / sinHalfAngle,
        relativeQuat.y / sinHalfAngle,
        relativeQuat.z / sinHalfAngle
      );
      
      // Project the rotation axis onto the forward direction to get the component
      // of rotation around the forward axis
      const projectedComponent = rotationAxis.dot(forwardDirection);
      
      // The rotation around the forward axis is the angle times the projection
      const forwardRotation = angle * projectedComponent;
      
      // Convert to degrees and handle the sign properly
      let degrees = THREE.MathUtils.radToDeg(forwardRotation);
      
      // Normalize to -180 to +180 range to avoid sudden jumps
      while (degrees > 180) degrees -= 360;
      while (degrees < -180) degrees += 360;
      
      return degrees;
    };

    // --- Modify Event Listeners ---
    this.leftHand.addEventListener('triggerdown', (evt) => {
        console.log('Left Trigger Pressed');
        this.leftTriggerDown = true;
    });
    this.leftHand.addEventListener('triggerup', (evt) => {
        console.log('Left Trigger Released');
        this.leftTriggerDown = false;
        this.sendTriggerRelease('left'); // Send trigger release message
    });
    this.leftHand.addEventListener('gripdown', (evt) => {
        console.log('Left Grip Pressed');
        this.leftGripDown = true; // Set grip state
        
        // Store initial rotation for relative tracking
        if (this.leftHand.object3D.visible) {
          const leftRotEuler = this.leftHand.object3D.rotation;
          this.leftGripInitialRotation = {
            x: THREE.MathUtils.radToDeg(leftRotEuler.x),
            y: THREE.MathUtils.radToDeg(leftRotEuler.y),
            z: THREE.MathUtils.radToDeg(leftRotEuler.z)
          };
          
          // Store initial quaternion for Z-axis rotation tracking
          this.leftGripInitialQuaternion = this.leftHand.object3D.quaternion.clone();
          
          console.log('Left grip initial rotation:', this.leftGripInitialRotation);
          console.log('Left grip initial quaternion:', this.leftGripInitialQuaternion);
        }
    });
    this.leftHand.addEventListener('gripup', (evt) => { // Add gripup listener
        console.log('Left Grip Released');
        this.leftGripDown = false; // Reset grip state
        this.leftGripInitialRotation = null; // Reset initial rotation
        this.leftGripInitialQuaternion = null; // Reset initial quaternion
        this.leftRelativeRotation = { x: 0, y: 0, z: 0 }; // Reset relative rotation
        this.leftZAxisRotation = 0; // Reset Z-axis rotation
        this.sendGripRelease('left'); // Send grip release message
    });

    this.rightHand.addEventListener('triggerdown', (evt) => {
        console.log('Right Trigger Pressed');
        this.rightTriggerDown = true;
    });
    this.rightHand.addEventListener('triggerup', (evt) => {
        console.log('Right Trigger Released');
        this.rightTriggerDown = false;
        this.sendTriggerRelease('right'); // Send trigger release message
    });
    this.rightHand.addEventListener('gripdown', (evt) => {
        console.log('Right Grip Pressed');
        this.rightGripDown = true; // Set grip state
        
        // Store initial rotation for relative tracking
        if (this.rightHand.object3D.visible) {
          const rightRotEuler = this.rightHand.object3D.rotation;
          this.rightGripInitialRotation = {
            x: THREE.MathUtils.radToDeg(rightRotEuler.x),
            y: THREE.MathUtils.radToDeg(rightRotEuler.y),
            z: THREE.MathUtils.radToDeg(rightRotEuler.z)
          };
          
          // Store initial quaternion for Z-axis rotation tracking
          this.rightGripInitialQuaternion = this.rightHand.object3D.quaternion.clone();
          
          console.log('Right grip initial rotation:', this.rightGripInitialRotation);
          console.log('Right grip initial quaternion:', this.rightGripInitialQuaternion);
        }
    });
    this.rightHand.addEventListener('gripup', (evt) => { // Add gripup listener
        console.log('Right Grip Released');
        this.rightGripDown = false; // Reset grip state
        this.rightGripInitialRotation = null; // Reset initial rotation
        this.rightGripInitialQuaternion = null; // Reset initial quaternion
        this.rightRelativeRotation = { x: 0, y: 0, z: 0 }; // Reset relative rotation
        this.rightZAxisRotation = 0; // Reset Z-axis rotation
        this.sendGripRelease('right'); // Send grip release message
    });
    // --- End Modify Event Listeners ---

  },

  createAxisIndicators: function() {
    // Create XYZ axis indicators for both controllers
    
    // Left Controller Axes
    // X-axis (Red)
    const leftXAxis = document.createElement('a-cylinder');
    leftXAxis.setAttribute('id', 'leftXAxis');
    leftXAxis.setAttribute('height', '0.08');
    leftXAxis.setAttribute('radius', '0.003');
    leftXAxis.setAttribute('color', '#ff0000'); // Red for X
    leftXAxis.setAttribute('position', '0.04 0 0');
    leftXAxis.setAttribute('rotation', '0 0 90'); // Rotate to point along X-axis
    this.leftHand.appendChild(leftXAxis);

    const leftXTip = document.createElement('a-cone');
    leftXTip.setAttribute('height', '0.015');
    leftXTip.setAttribute('radius-bottom', '0.008');
    leftXTip.setAttribute('radius-top', '0');
    leftXTip.setAttribute('color', '#ff0000');
    leftXTip.setAttribute('position', '0.055 0 0');
    leftXTip.setAttribute('rotation', '0 0 90');
    this.leftHand.appendChild(leftXTip);

    // Y-axis (Green) - Up
    const leftYAxis = document.createElement('a-cylinder');
    leftYAxis.setAttribute('id', 'leftYAxis');
    leftYAxis.setAttribute('height', '0.08');
    leftYAxis.setAttribute('radius', '0.003');
    leftYAxis.setAttribute('color', '#00ff00'); // Green for Y
    leftYAxis.setAttribute('position', '0 0.04 0');
    leftYAxis.setAttribute('rotation', '0 0 0'); // Default up orientation
    this.leftHand.appendChild(leftYAxis);

    const leftYTip = document.createElement('a-cone');
    leftYTip.setAttribute('height', '0.015');
    leftYTip.setAttribute('radius-bottom', '0.008');
    leftYTip.setAttribute('radius-top', '0');
    leftYTip.setAttribute('color', '#00ff00');
    leftYTip.setAttribute('position', '0 0.055 0');
    this.leftHand.appendChild(leftYTip);

    // Z-axis (Blue) - Forward
    const leftZAxis = document.createElement('a-cylinder');
    leftZAxis.setAttribute('id', 'leftZAxis');
    leftZAxis.setAttribute('height', '0.08');
    leftZAxis.setAttribute('radius', '0.003');
    leftZAxis.setAttribute('color', '#0000ff'); // Blue for Z
    leftZAxis.setAttribute('position', '0 0 0.04');
    leftZAxis.setAttribute('rotation', '90 0 0'); // Rotate to point along Z-axis
    this.leftHand.appendChild(leftZAxis);

    const leftZTip = document.createElement('a-cone');
    leftZTip.setAttribute('height', '0.015');
    leftZTip.setAttribute('radius-bottom', '0.008');
    leftZTip.setAttribute('radius-top', '0');
    leftZTip.setAttribute('color', '#0000ff');
    leftZTip.setAttribute('position', '0 0 0.055');
    leftZTip.setAttribute('rotation', '90 0 0');
    this.leftHand.appendChild(leftZTip);

    // Right Controller Axes
    // X-axis (Red)
    const rightXAxis = document.createElement('a-cylinder');
    rightXAxis.setAttribute('id', 'rightXAxis');
    rightXAxis.setAttribute('height', '0.08');
    rightXAxis.setAttribute('radius', '0.003');
    rightXAxis.setAttribute('color', '#ff0000'); // Red for X
    rightXAxis.setAttribute('position', '0.04 0 0');
    rightXAxis.setAttribute('rotation', '0 0 90'); // Rotate to point along X-axis
    this.rightHand.appendChild(rightXAxis);

    const rightXTip = document.createElement('a-cone');
    rightXTip.setAttribute('height', '0.015');
    rightXTip.setAttribute('radius-bottom', '0.008');
    rightXTip.setAttribute('radius-top', '0');
    rightXTip.setAttribute('color', '#ff0000');
    rightXTip.setAttribute('position', '0.055 0 0');
    rightXTip.setAttribute('rotation', '0 0 90');
    this.rightHand.appendChild(rightXTip);

    // Y-axis (Green) - Up
    const rightYAxis = document.createElement('a-cylinder');
    rightYAxis.setAttribute('id', 'rightYAxis');
    rightYAxis.setAttribute('height', '0.08');
    rightYAxis.setAttribute('radius', '0.003');
    rightYAxis.setAttribute('color', '#00ff00'); // Green for Y
    rightYAxis.setAttribute('position', '0 0.04 0');
    rightYAxis.setAttribute('rotation', '0 0 0'); // Default up orientation
    this.rightHand.appendChild(rightYAxis);

    const rightYTip = document.createElement('a-cone');
    rightYTip.setAttribute('height', '0.015');
    rightYTip.setAttribute('radius-bottom', '0.008');
    rightYTip.setAttribute('radius-top', '0');
    rightYTip.setAttribute('color', '#00ff00');
    rightYTip.setAttribute('position', '0 0.055 0');
    this.rightHand.appendChild(rightYTip);

    // Z-axis (Blue) - Forward
    const rightZAxis = document.createElement('a-cylinder');
    rightZAxis.setAttribute('id', 'rightZAxis');
    rightZAxis.setAttribute('height', '0.08');
    rightZAxis.setAttribute('radius', '0.003');
    rightZAxis.setAttribute('color', '#0000ff'); // Blue for Z
    rightZAxis.setAttribute('position', '0 0 0.04');
    rightZAxis.setAttribute('rotation', '90 0 0'); // Rotate to point along Z-axis
    this.rightHand.appendChild(rightZAxis);

    const rightZTip = document.createElement('a-cone');
    rightZTip.setAttribute('height', '0.015');
    rightZTip.setAttribute('radius-bottom', '0.008');
    rightZTip.setAttribute('radius-top', '0');
    rightZTip.setAttribute('color', '#0000ff');
    rightZTip.setAttribute('position', '0 0 0.055');
    rightZTip.setAttribute('rotation', '90 0 0');
    this.rightHand.appendChild(rightZTip);

    console.log('XYZ axis indicators created for both controllers (RGB for XYZ)');
  },

  tick: function () {
    // Update controller text if controllers are visible
    if (!this.leftHand || !this.rightHand) return; // Added safety check

    // --- BEGIN DETAILED LOGGING ---
    if (this.leftHand.object3D) {
      // console.log(`Left Hand Raw - Visible: ${this.leftHand.object3D.visible}, Pos: ${this.leftHand.object3D.position.x.toFixed(2)},${this.leftHand.object3D.position.y.toFixed(2)},${this.leftHand.object3D.position.z.toFixed(2)}`);
    }
    if (this.rightHand.object3D) {
      // console.log(`Right Hand Raw - Visible: ${this.rightHand.object3D.visible}, Pos: ${this.rightHand.object3D.position.x.toFixed(2)},${this.rightHand.object3D.position.y.toFixed(2)},${this.rightHand.object3D.position.z.toFixed(2)}`);
    }
    // --- END DETAILED LOGGING ---

    // Collect data from both controllers
    const leftController = {
        hand: 'left',
        position: null,
        rotation: null,
        gripActive: false,
        trigger: 0
    };
    
    const rightController = {
        hand: 'right',
        position: null,
        rotation: null,
        gripActive: false,
        trigger: 0
    };
    
    // Collect headset data
    const headset = {
        position: null,
        rotation: null,
        quaternion: null
    };

    // Update Left Hand Text & Collect Data
    // 移除object3D.visible检查，确保即使控制器不可见也能收集数据
    if (this.leftHand && this.leftHand.object3D) {
        const leftPos = this.leftHand.object3D.position;
        const leftRotEuler = this.leftHand.object3D.rotation; // Euler angles in radians
        // Convert to degrees without offset
        const leftRotX = THREE.MathUtils.radToDeg(leftRotEuler.x);
        const leftRotY = THREE.MathUtils.radToDeg(leftRotEuler.y);
        const leftRotZ = THREE.MathUtils.radToDeg(leftRotEuler.z);

        // 添加调试信息 (commented out to reduce lag)
        // console.log(`Left Hand - Visible: ${this.leftHand.object3D.visible}, Pos: ${leftPos.x.toFixed(2)},${leftPos.y.toFixed(2)},${leftPos.z.toFixed(2)}`);

        // Calculate relative rotation if grip is held
        if (this.leftGripDown && this.leftGripInitialRotation) {
          this.leftRelativeRotation = this.calculateRelativeRotation(
            { x: leftRotX, y: leftRotY, z: leftRotZ },
            this.leftGripInitialRotation
          );
          
          // Calculate Z-axis rotation using quaternions
          if (this.leftGripInitialQuaternion) {
            this.leftZAxisRotation = this.calculateZAxisRotation(
              this.leftHand.object3D.quaternion,
              this.leftGripInitialQuaternion
            );
          }
          
          // console.log('Left relative rotation:', this.leftRelativeRotation);
          // console.log('Left Z-axis rotation:', this.leftZAxisRotation.toFixed(1), 'degrees');
        }

        // Create display text including relative rotation when grip is held
        let combinedLeftText = `Pos: ${leftPos.x.toFixed(2)} ${leftPos.y.toFixed(2)} ${leftPos.z.toFixed(2)}\\nRot: ${leftRotX.toFixed(0)} ${leftRotY.toFixed(0)} ${leftRotZ.toFixed(0)}`;
        if (this.leftGripDown && this.leftGripInitialRotation) {
          combinedLeftText += `\\nZ-Rot: ${this.leftZAxisRotation.toFixed(1)}°`;
        }

        if (this.leftHandInfoText) {
            this.leftHandInfoText.setAttribute('value', combinedLeftText);
        }

        // Collect left controller data
        leftController.position = { x: leftPos.x, y: leftPos.y, z: leftPos.z };
        leftController.rotation = { x: leftRotX, y: leftRotY, z: leftRotZ };
        leftController.quaternion = { 
          x: this.leftHand.object3D.quaternion.x, 
          y: this.leftHand.object3D.quaternion.y, 
          z: this.leftHand.object3D.quaternion.z, 
          w: this.leftHand.object3D.quaternion.w 
        };
        // Get raw analog trigger value from gamepad
        let leftTriggerValue = 0;
        if (this.leftHand && this.leftHand.components && this.leftHand.components['tracked-controls']) {
            const leftGamepad = this.leftHand.components['tracked-controls'].controller?.gamepad;
            if (leftGamepad && leftGamepad.buttons[0]) {
                leftTriggerValue = leftGamepad.buttons[0].value || 0;
            }
        }
        leftController.trigger = leftTriggerValue;
        leftController.gripActive = this.leftGripDown;
        // 采集左手柄的摇杆和按钮信息
        if (this.leftHand && this.leftHand.components && this.leftHand.components['tracked-controls']) {
            const leftGamepad = this.leftHand.components['tracked-controls'].controller?.gamepad;
            if (leftGamepad) {
                // Log all button states for debugging (commented out to reduce lag)
                // if (leftGamepad.buttons) {
                //     for (let i = 0; i < leftGamepad.buttons.length; i++) {
                //         if (leftGamepad.buttons[i].pressed) {
                //             console.log(`Left Hand Button ${i} pressed`);
                //         }
                //     }
                // }
                
                // 摇杆
                leftController.thumbstick = {
                    x: leftGamepad.axes[2] || 0,
                    y: leftGamepad.axes[3] || 0
                };
                // 侧边按钮 - Meta Quest 3 uses different indices
                // Button 4 = A, Button 5 = B for Quest controllers
                leftController.buttons = {
                    squeeze: !!leftGamepad.buttons[1]?.pressed,
                    thumbstick: !!leftGamepad.buttons[3]?.pressed,
                    x: !!leftGamepad.buttons[4]?.pressed,  // X button (left controller)
                    y: !!leftGamepad.buttons[5]?.pressed,  // Y button (left controller)
                    menu: !!leftGamepad.buttons[6]?.pressed
                };
            }
        }
    } else {
        // console.log('Left hand object not available');
    }

    // Update Right Hand Text & Collect Data
    // 移除object3D.visible检查，确保即使控制器不可见也能收集数据
    if (this.rightHand && this.rightHand.object3D) {
        const rightPos = this.rightHand.object3D.position;
        const rightRotEuler = this.rightHand.object3D.rotation; // Euler angles in radians
        // Convert to degrees without offset
        const rightRotX = THREE.MathUtils.radToDeg(rightRotEuler.x);
        const rightRotY = THREE.MathUtils.radToDeg(rightRotEuler.y);
        const rightRotZ = THREE.MathUtils.radToDeg(rightRotEuler.z);

        // 添加调试信息 (commented out to reduce lag)
        // console.log(`Right Hand - Visible: ${this.rightHand.object3D.visible}, Pos: ${rightPos.x.toFixed(2)},${rightPos.y.toFixed(2)},${rightPos.z.toFixed(2)}`);

        // Calculate relative rotation if grip is held
        if (this.rightGripDown && this.rightGripInitialRotation) {
          this.rightRelativeRotation = this.calculateRelativeRotation(
            { x: rightRotX, y: rightRotY, z: rightRotZ },
            this.rightGripInitialRotation
          );
          
          // Calculate Z-axis rotation using quaternions
          if (this.rightGripInitialQuaternion) {
            this.rightZAxisRotation = this.calculateZAxisRotation(
              this.rightHand.object3D.quaternion,
              this.rightGripInitialQuaternion
            );
          }
          
          // console.log('Right relative rotation:', this.rightRelativeRotation);
          // console.log('Right Z-axis rotation:', this.rightZAxisRotation.toFixed(1), 'degrees');
        }

        // Create display text including relative rotation when grip is held
        let combinedRightText = `Pos: ${rightPos.x.toFixed(2)} ${rightPos.y.toFixed(2)} ${rightPos.z.toFixed(2)}\\nRot: ${rightRotX.toFixed(0)} ${rightRotY.toFixed(0)} ${rightRotZ.toFixed(0)}`;
        if (this.rightGripDown && this.rightGripInitialRotation) {
          combinedRightText += `\\nZ-Rot: ${this.rightZAxisRotation.toFixed(1)}°`;
        }

        if (this.rightHandInfoText) {
            this.rightHandInfoText.setAttribute('value', combinedRightText);
        }

        // Collect right controller data
        rightController.position = { x: rightPos.x, y: rightPos.y, z: rightPos.z };
        rightController.rotation = { x: rightRotX, y: rightRotY, z: rightRotZ };
        rightController.quaternion = { 
          x: this.rightHand.object3D.quaternion.x, 
          y: this.rightHand.object3D.quaternion.y, 
          z: this.rightHand.object3D.quaternion.z, 
          w: this.rightHand.object3D.quaternion.w 
        };
        // Get raw analog trigger value from gamepad
        let rightTriggerValue = 0;
        if (this.rightHand && this.rightHand.components && this.rightHand.components['tracked-controls']) {
            const rightGamepad = this.rightHand.components['tracked-controls'].controller?.gamepad;
            if (rightGamepad && rightGamepad.buttons[0]) {
                rightTriggerValue = rightGamepad.buttons[0].value || 0;
            }
        }
        rightController.trigger = rightTriggerValue;
        rightController.gripActive = this.rightGripDown;
        
        // 采集右手柄的摇杆和按钮信息
        if (this.rightHand && this.rightHand.components && this.rightHand.components['tracked-controls']) {
            const rightGamepad = this.rightHand.components['tracked-controls'].controller?.gamepad;
            if (rightGamepad) {
                // Log all button states for debugging (commented out to reduce lag)
                // if (rightGamepad.buttons) {
                //     for (let i = 0; i < rightGamepad.buttons.length; i++) {
                //         if (rightGamepad.buttons[i].pressed) {
                //             console.log(`Right Hand Button ${i} pressed`);
                //         }
                //     }
                // }
                
                // 摇杆
                rightController.thumbstick = {
                    x: rightGamepad.axes[2] || 0,
                    y: rightGamepad.axes[3] || 0
                };
                // 侧边按钮 - Meta Quest 3 uses different indices
                // Button 4 = A, Button 5 = B for Quest controllers
                rightController.buttons = {
                    squeeze: !!rightGamepad.buttons[1]?.pressed,
                    thumbstick: !!rightGamepad.buttons[3]?.pressed,  
                    a: !!rightGamepad.buttons[4]?.pressed,  // A button (right controller)
                    b: !!rightGamepad.buttons[5]?.pressed,  // B button (right controller)

                    menu: !!rightGamepad.buttons[6]?.pressed
                };
            }
        }
    } else {
        // console.log('Right hand object not available');
    }

    // Collect headset data
    if (this.headset && this.headset.object3D) {
        const headsetPos = this.headset.object3D.position;
        const headsetRotEuler = this.headset.object3D.rotation;
        const headsetRotX = THREE.MathUtils.radToDeg(headsetRotEuler.x);
        const headsetRotY = THREE.MathUtils.radToDeg(headsetRotEuler.y);
        const headsetRotZ = THREE.MathUtils.radToDeg(headsetRotEuler.z);

        // Format rotation values with leading zeros and space for positive numbers
        const formatRotation = (val) => {
            const rounded = Math.round(val);
            const absValue = Math.abs(rounded).toString().padStart(3, '0');
            return rounded >= 0 ? ' ' + absValue : '-' + absValue;
        };
        
        // Update headset info text
        const headsetText = `Rot: ${formatRotation(headsetRotX)} ${formatRotation(headsetRotY)} ${formatRotation(headsetRotZ)}`;
        if (this.headsetInfoText) {
            this.headsetInfoText.setAttribute('value', headsetText);
        }

        // Collect headset data
        headset.position = { x: headsetPos.x, y: headsetPos.y, z: headsetPos.z };
        headset.rotation = { x: headsetRotX, y: headsetRotY, z: headsetRotZ };
        headset.quaternion = { 
          x: this.headset.object3D.quaternion.x, 
          y: this.headset.object3D.quaternion.y, 
          z: this.headset.object3D.quaternion.z, 
          w: this.headset.object3D.quaternion.w 
        };
        
        // console.log(`Headset - Pos: ${headsetPos.x.toFixed(2)},${headsetPos.y.toFixed(2)},${headsetPos.z.toFixed(2)}`);
    } else {
        // console.log('Headset object not available');
    }

    // Send combined packet if WebSocket is open and at least one controller has valid data
    if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
        // 修改发送条件：只要有位置数据就发送，不检查是否为(0,0,0)
        const hasValidLeft = leftController.position !== null;
        const hasValidRight = rightController.position !== null;
        const hasValidHeadset = headset.position !== null;
        
        if (hasValidLeft || hasValidRight || hasValidHeadset) {
            const dualControllerData = {
                timestamp: Date.now(),
                leftController: leftController,
                rightController: rightController,
                headset: headset
            };
            this.websocket.send(JSON.stringify(dualControllerData));
            
            // 添加调试信息 (commented out to reduce lag)
            // console.log('Sending VR data:', {
            //     left: hasValidLeft ? 'valid' : 'invalid',
            //     right: hasValidRight ? 'valid' : 'invalid',
            //     headset: hasValidHeadset ? 'valid' : 'invalid',
            //     leftPos: leftController.position,
            //     rightPos: rightController.position,
            //     headsetPos: headset.position
            // });
        }
    }
  }
});


// Add the component to the scene after it's loaded
document.addEventListener('DOMContentLoaded', (event) => {
    const scene = document.querySelector('a-scene');

    if (scene) {
        // Listen for controller connection events
        scene.addEventListener('controllerconnected', (evt) => {
            console.log('Controller CONNECTED:', evt.detail.name, evt.detail.component.data.hand);
        });
        scene.addEventListener('controllerdisconnected', (evt) => {
            console.log('Controller DISCONNECTED:', evt.detail.name, evt.detail.component.data.hand);
        });

        // Add controller-updater component when scene is loaded (A-Frame manages session)
        if (scene.hasLoaded) {
            scene.setAttribute('controller-updater', '');
            console.log("controller-updater component added immediately.");
        } else {
            scene.addEventListener('loaded', () => {
                scene.setAttribute('controller-updater', '');
                console.log("controller-updater component added after scene loaded.");
            });
        }
    } else {
        console.error('A-Frame scene not found!');
    }

    // Add controller tracking button logic
    addControllerTrackingButton();
});

function addControllerTrackingButton() {
    if (navigator.xr) {
        navigator.xr.isSessionSupported('immersive-ar').then((supported) => {
            if (supported) {
                // Create Start Controller Tracking button
                const startButton = document.createElement('button');
                startButton.id = 'start-tracking-button';
                startButton.textContent = 'Start Controller Tracking';
                startButton.style.position = 'fixed';
                startButton.style.top = '50%';
                startButton.style.left = '50%';
                startButton.style.transform = 'translate(-50%, -50%)';
                startButton.style.padding = '20px 40px';
                startButton.style.fontSize = '20px';
                startButton.style.fontWeight = 'bold';
                startButton.style.backgroundColor = '#4CAF50';
                startButton.style.color = 'white';
                startButton.style.border = 'none';
                startButton.style.borderRadius = '8px';
                startButton.style.cursor = 'pointer';
                startButton.style.zIndex = '9999';
                startButton.style.boxShadow = '0 4px 8px rgba(0,0,0,0.3)';
                startButton.style.transition = 'all 0.3s ease';

                // Hover effects
                startButton.addEventListener('mouseenter', () => {
                    startButton.style.backgroundColor = '#45a049';
                    startButton.style.transform = 'translate(-50%, -50%) scale(1.05)';
                });
                startButton.addEventListener('mouseleave', () => {
                    startButton.style.backgroundColor = '#4CAF50';
                    startButton.style.transform = 'translate(-50%, -50%) scale(1)';
                });

                startButton.onclick = () => {
                    console.log('Start Controller Tracking button clicked. Requesting session via A-Frame...');
                    const sceneEl = document.querySelector('a-scene');
                    if (sceneEl) {
                        // Use A-Frame's enterVR to handle session start
                        sceneEl.enterVR(true).catch((err) => {
                            console.error('A-Frame failed to enter VR/AR:', err);
                            alert(`Failed to start AR session via A-Frame: ${err.message}`);
                        });
                    } else {
                         console.error('A-Frame scene not found for enterVR call!');
                    }
                };

                document.body.appendChild(startButton);
                console.log('Official "Start Controller Tracking" button added.');

                // Listen for VR session events to hide/show start button and dimmer
                const sceneEl = document.querySelector('a-scene');
                const dimmer = document.querySelector('#passthroughDimmer');
                if (sceneEl) {
                    sceneEl.addEventListener('enter-vr', () => {
                        console.log('Entered VR - hiding start button, showing dimmer');
                        startButton.style.display = 'none';
                        if (dimmer) dimmer.setAttribute('visible', 'true');
                    });

                    sceneEl.addEventListener('exit-vr', () => {
                        console.log('Exited VR - showing start button, hiding dimmer');
                        startButton.style.display = 'block';
                        if (dimmer) dimmer.setAttribute('visible', 'false');
                    });
                }

            } else {
                console.warn('immersive-ar session not supported by this browser/device.');
            }
        }).catch((err) => {
            console.error('Error checking immersive-ar support:', err);
        });
    } else {
        console.warn('WebXR not supported by this browser.');
    }
} 