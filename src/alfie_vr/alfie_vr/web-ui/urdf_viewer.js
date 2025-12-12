// URDF Viewer Component - Connects to Foxglove Bridge to visualize robot state
// Uses @foxglove/ws-protocol for WebSocket communication

class URDFViewer {
  constructor(containerId, options = {}) {
    this.container = document.getElementById(containerId);
    this.options = {
      // Port 8082 is nginx proxy that terminates TLS and forwards to foxglove_bridge on 8765
      foxgloveUrl: options.foxgloveUrl || `wss://${window.location.hostname}:8082`,
      urdfUrl: options.urdfUrl || '/urdf/alfiebot.urdf',
      meshBasePath: options.meshBasePath || '/meshes/',
      ...options
    };
    
    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;
    this.robot = null;
    this.links = {};
    this.joints = {};
    this.transforms = {};
    this.foxgloveClient = null;
    this.tfSubscriptionId = null;
    this.tfStaticSubscriptionId = null;
    this.jointStateSubscriptionId = null;
    this.robotDescriptionSubscriptionId = null;
    this.urdfLoaded = false;
    this.tfUpdateCount = 0;
    this.isConnected = false;
    
    // TF rate tracking
    this.tfRateCount = 0;
    this.tfRateHz = 0;
    this.lastRateUpdate = performance.now();
    
    // Frame rate limiting (10 FPS = 100ms between frames)
    this.targetFPS = 10;
    this.frameInterval = 1000 / this.targetFPS;
    this.lastFrameTime = 0;
    
    this.init();
  }

  init() {
    this.setupScene();
    this.setupLighting();
    this.setupControls();
    this.connectToFoxglove();
    this.animate();
    
    // Start rate update interval (every 5 seconds)
    setInterval(() => this.updateTFRate(), 5000);
    
    // Handle container resize
    window.addEventListener('resize', () => this.onResize());
    new ResizeObserver(() => this.onResize()).observe(this.container);
  }

  setupScene() {
    // Create Three.js scene
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x303030);
    
    // Add grid helper
    const gridHelper = new THREE.GridHelper(2, 20, 0x444444, 0x333333);
    this.scene.add(gridHelper);
    
    // Add axes helper
    const axesHelper = new THREE.AxesHelper(0.3);
    this.scene.add(axesHelper);
    
    // Setup camera
    const aspect = this.container.clientWidth / this.container.clientHeight;
    this.camera = new THREE.PerspectiveCamera(50, aspect, 0.01, 100);
    this.camera.position.set(0.8, 0.6, 0.8);
    this.camera.lookAt(0, 0.4, 0);
    
    // Setup renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.container.appendChild(this.renderer.domElement);
  }

  setupLighting() {
    // Ambient light
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
    this.scene.add(ambientLight);
    
    // Directional light
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(1, 2, 1);
    directionalLight.castShadow = true;
    this.scene.add(directionalLight);
    
    // Hemisphere light for better ambient
    const hemiLight = new THREE.HemisphereLight(0xffffff, 0x444444, 0.3);
    hemiLight.position.set(0, 10, 0);
    this.scene.add(hemiLight);
  }

  setupControls() {
    // Simple mouse orbit controls implementation
    this.isDragging = false;
    this.previousMousePosition = { x: 0, y: 0 };
    this.spherical = { theta: 0, phi: Math.PI / 4, radius: 1.5 };
    this.target = new THREE.Vector3(0, 0.4, 0);
    
    const container = this.container;
    
    // Mouse down - start dragging
    container.addEventListener('mousedown', (e) => {
      this.isDragging = true;
      this.previousMousePosition = { x: e.clientX, y: e.clientY };
    });
    
    // Mouse up - stop dragging
    container.addEventListener('mouseup', () => {
      this.isDragging = false;
    });
    
    container.addEventListener('mouseleave', () => {
      this.isDragging = false;
    });
    
    // Mouse move - rotate camera
    container.addEventListener('mousemove', (e) => {
      if (!this.isDragging) return;
      
      const deltaX = e.clientX - this.previousMousePosition.x;
      const deltaY = e.clientY - this.previousMousePosition.y;
      
      // Rotate around target
      this.spherical.theta -= deltaX * 0.01;
      this.spherical.phi -= deltaY * 0.01;
      
      // Clamp phi to avoid flipping
      this.spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, this.spherical.phi));
      
      this.updateCameraPosition();
      this.previousMousePosition = { x: e.clientX, y: e.clientY };
    });
    
    // Mouse wheel - zoom
    container.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.spherical.radius += e.deltaY * 0.001;
      this.spherical.radius = Math.max(0.3, Math.min(5, this.spherical.radius));
      this.updateCameraPosition();
    }, { passive: false });
    
    // Touch support for mobile
    container.addEventListener('touchstart', (e) => {
      if (e.touches.length === 1) {
        this.isDragging = true;
        this.previousMousePosition = { x: e.touches[0].clientX, y: e.touches[0].clientY };
      }
    });
    
    container.addEventListener('touchend', () => {
      this.isDragging = false;
    });
    
    container.addEventListener('touchmove', (e) => {
      if (!this.isDragging || e.touches.length !== 1) return;
      
      const deltaX = e.touches[0].clientX - this.previousMousePosition.x;
      const deltaY = e.touches[0].clientY - this.previousMousePosition.y;
      
      this.spherical.theta -= deltaX * 0.01;
      this.spherical.phi -= deltaY * 0.01;
      this.spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, this.spherical.phi));
      
      this.updateCameraPosition();
      this.previousMousePosition = { x: e.touches[0].clientX, y: e.touches[0].clientY };
    });
    
    // Initialize camera position
    this.updateCameraPosition();
  }
  
  updateCameraPosition() {
    // Convert spherical coordinates to Cartesian
    const x = this.spherical.radius * Math.sin(this.spherical.phi) * Math.sin(this.spherical.theta);
    const y = this.spherical.radius * Math.cos(this.spherical.phi);
    const z = this.spherical.radius * Math.sin(this.spherical.phi) * Math.cos(this.spherical.theta);
    
    this.camera.position.set(
      this.target.x + x,
      this.target.y + y,
      this.target.z + z
    );
    this.camera.lookAt(this.target);
  }

  async connectToFoxglove() {
    console.log(`Connecting to Foxglove Bridge at ${this.options.foxgloveUrl}...`);
    this.updateStatus('Connecting to Foxglove...');
    
    try {
      // Create WebSocket connection to Foxglove Bridge with the SDK subprotocol (required for foxglove_bridge 3.x)
      this.foxgloveClient = new WebSocket(this.options.foxgloveUrl, ['foxglove.sdk.v1']);
      
      this.foxgloveClient.binaryType = 'arraybuffer';
      
      this.foxgloveClient.onopen = () => {
        console.log('Connected to Foxglove Bridge');
        this.updateStatus('Connected to Foxglove');
        this.isConnected = true;
        this.updateConnectionStatus(true);
        this.hideCertWarning();
        // Foxglove will automatically send serverInfo and advertise messages
      };
      
      this.foxgloveClient.onmessage = (event) => {
        this.handleFoxgloveMessage(event.data);
      };
      
      this.foxgloveClient.onerror = (error) => {
        console.error('Foxglove WebSocket error:', error);
        this.updateStatus('Connection error - check certificate');
        this.isConnected = false;
        this.updateConnectionStatus(false);
        this.showCertWarning();
      };
      
      this.foxgloveClient.onclose = (event) => {
        console.log('Foxglove WebSocket closed. Code:', event.code, 'Reason:', event.reason);
        this.updateStatus('Foxglove disconnected');
        this.isConnected = false;
        this.updateConnectionStatus(false);
        
        // Show certificate warning if connection failed immediately (likely cert issue)
        if (event.code === 1006) {
          this.showCertWarning();
        }
        
        // Load placeholder if we don't have a model yet
        if (!this.urdfLoaded) {
          this.loadRobotModel();
        }
        
        // Attempt reconnection after 5 seconds (but not if it's a protocol error)
        if (event.code !== 1002 && event.code !== 1003) {
          setTimeout(() => this.connectToFoxglove(), 5000);
        }
      };
      
    } catch (error) {
      console.error('Failed to connect to Foxglove:', error);
      this.updateStatus('Connection failed');
    }
  }

  sendServerInfo() {
    // Foxglove Bridge sends serverInfo automatically after connection
    // We just need to wait for the 'advertise' message to subscribe
  }

  handleFoxgloveMessage(data) {
    try {
      // Handle both binary and text messages
      if (typeof data === 'string') {
        const message = JSON.parse(data);
        this.handleJsonMessage(message);
      } else if (data instanceof ArrayBuffer) {
        this.handleBinaryMessage(data);
      }
    } catch (error) {
      console.error('Error handling Foxglove message:', error);
    }
  }

  handleJsonMessage(message) {
    switch (message.op) {
      case 'serverInfo':
        console.log('Foxglove server info:', message);
        break;
        
      case 'advertise':
        console.log('Available channels:', message.channels);
        this.subscribeToTopics(message.channels);
        break;
        
      case 'status':
        console.log('Foxglove status:', message);
        break;
        
      case 'message':
        this.handleTopicMessage(message);
        break;
    }
  }

  handleBinaryMessage(data) {
    // Foxglove binary message format
    const view = new DataView(data);
    const opCode = view.getUint8(0);
    
    if (opCode === 1) {  // Message data
      const subscriptionId = view.getUint32(1, true);
      const timestamp = view.getBigUint64(5, true);
      const messageData = new Uint8Array(data, 13);
      
      // Decode based on subscription
      if (subscriptionId === this.tfSubscriptionId || subscriptionId === this.tfStaticSubscriptionId) {
        this.handleTFMessage(messageData);
      } else if (subscriptionId === this.jointStateSubscriptionId) {
        this.handleJointStateMessage(messageData);
      } else if (subscriptionId === this.robotDescriptionSubscriptionId) {
        this.handleRobotDescriptionMessage(messageData);
      }
    }
  }

  subscribeToTopics(channels) {
    const subscriptions = [];
    
    for (const channel of channels) {
      // Subscribe to TF topic (dynamic transforms)
      if (channel.topic === '/alfie/tf') {
        this.tfSubscriptionId = subscriptions.length + 1;
        subscriptions.push({
          id: this.tfSubscriptionId,
          channelId: channel.id
        });
        console.log(`Subscribing to TF: ${channel.topic}`);
      }
      
      // Subscribe to TF static topic (static transforms for fixed joints)
      if (channel.topic === '/alfie/tf_static') {
        this.tfStaticSubscriptionId = subscriptions.length + 1;
        subscriptions.push({
          id: this.tfStaticSubscriptionId,
          channelId: channel.id
        });
        console.log(`Subscribing to TF static: ${channel.topic}`);
      }
      
      // Subscribe to joint states
      if (channel.topic === '/alfie/joint_states') {
        this.jointStateSubscriptionId = subscriptions.length + 1;
        subscriptions.push({
          id: this.jointStateSubscriptionId,
          channelId: channel.id
        });
        console.log(`Subscribing to joint states: ${channel.topic}`);
      }
      
      // Subscribe to robot description for URDF
      if (channel.topic === '/alfie/robot_description') {
        this.robotDescriptionSubscriptionId = subscriptions.length + 1;
        subscriptions.push({
          id: this.robotDescriptionSubscriptionId,
          channelId: channel.id
        });
        console.log(`Subscribing to robot description: ${channel.topic}`);
      }
    }
    
    if (subscriptions.length > 0) {
      const subscribeMsg = {
        op: 'subscribe',
        subscriptions: subscriptions
      };
      this.foxgloveClient.send(JSON.stringify(subscribeMsg));
      this.updateStatus(`Subscribed to ${subscriptions.length} topics`);
      
      // Load placeholder model if robot_description not available
      if (!this.robotDescriptionSubscriptionId) {
        console.log('No robot_description topic found, loading placeholder model');
        this.loadRobotModel();
      } else {
        this.updateStatus('Waiting for URDF from robot_description...');
      }
    }
  }

  handleTopicMessage(message) {
    // Handle JSON-encoded topic messages
    if (message.topic === '/alfie/tf') {
      this.processTFData(message.message);
    } else if (message.topic === '/alfie/joint_states') {
      this.processJointStateData(message.message);
    } else if (message.topic === '/alfie/robot_description') {
      this.processRobotDescription(message.message);
    }
  }

  handleRobotDescriptionMessage(data) {
    try {
      // CDR format for ROS2 std_msgs/msg/String:
      // - 4 bytes: CDR encapsulation header (00 01 00 00 for little-endian)
      // - 4 bytes: string length (uint32, little-endian, includes null terminator)
      // - N bytes: string data (UTF-8)
      // - 1 byte: null terminator
      
      const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
      
      // Skip 4-byte CDR encapsulation header, then read string length
      const stringLength = view.getUint32(4, true);
      
      // Extract the string (skip 8 bytes header, exclude null terminator)
      const stringData = new Uint8Array(data.buffer, data.byteOffset + 8, stringLength - 1);
      const decoder = new TextDecoder();
      const urdfString = decoder.decode(stringData);
      
      console.log('Decoded robot_description CDR message, length:', stringLength);
      
      if (urdfString.includes('<robot')) {
        console.log('Found URDF in robot_description');
        this.parseAndLoadURDF(urdfString);
      } else {
        console.log('robot_description does not contain URDF XML');
      }
    } catch (error) {
      console.error('Failed to parse robot description CDR:', error);
      // Fallback: try parsing as plain text or JSON
      try {
        const decoder = new TextDecoder();
        const text = decoder.decode(data);
        if (text.includes('<robot')) {
          this.parseAndLoadURDF(text);
        } else {
          const jsonData = JSON.parse(text);
          this.processRobotDescription(jsonData);
        }
      } catch (e) {
        console.error('Fallback parsing also failed:', e);
      }
    }
  }

  processRobotDescription(descData) {
    if (this.urdfLoaded) return; // Only load once
    
    let urdfString = null;
    
    // Handle different message formats
    if (typeof descData === 'string') {
      urdfString = descData;
    } else if (descData.data) {
      urdfString = descData.data;
    } else if (descData.robot_description) {
      urdfString = descData.robot_description;
    }
    
    if (urdfString && urdfString.includes('<robot')) {
      console.log('Received URDF from robot_description topic');
      this.parseAndLoadURDF(urdfString);
    }
  }

  parseAndLoadURDF(urdfString) {
    if (this.urdfLoaded) return;
    
    try {
      const parser = new DOMParser();
      const urdfDoc = parser.parseFromString(urdfString, 'text/xml');
      const robotEl = urdfDoc.querySelector('robot');
      
      if (!robotEl) {
        console.error('No robot element found in URDF');
        return;
      }
      
      const robotName = robotEl.getAttribute('name') || 'robot';
      console.log(`Parsing URDF for robot: ${robotName}`);
      
      // Clear existing robot
      if (this.robot) {
        this.scene.remove(this.robot);
      }
      
      this.robot = new THREE.Group();
      this.robot.name = robotName;
      this.links = {};
      this.joints = {};
      
      // Parse material definitions from URDF root
      const materialDefs = {};
      urdfDoc.querySelectorAll('robot > material').forEach(matEl => {
        const matName = matEl.getAttribute('name');
        const colorEl = matEl.querySelector('color');
        if (matName && colorEl) {
          const rgba = colorEl.getAttribute('rgba').split(' ').map(parseFloat);
          materialDefs[matName] = new THREE.Color(rgba[0], rgba[1], rgba[2]);
        }
      });
      console.log('Parsed material definitions:', Object.keys(materialDefs));
      
      // Helper function to get material color
      const getMaterialColor = (materialEl) => {
        if (!materialEl) return 0x666666;
        
        // First check for inline color
        const colorEl = materialEl.querySelector('color');
        if (colorEl) {
          const rgba = colorEl.getAttribute('rgba').split(' ').map(parseFloat);
          return new THREE.Color(rgba[0], rgba[1], rgba[2]);
        }
        
        // Otherwise look up by name
        const matName = materialEl.getAttribute('name');
        if (matName && materialDefs[matName]) {
          return materialDefs[matName];
        }
        
        return 0x666666;
      };
      
      // Parse links
      const linkElements = urdfDoc.querySelectorAll('link');
      linkElements.forEach(linkEl => {
        const linkName = linkEl.getAttribute('name');
        const visuals = linkEl.querySelectorAll('visual');  // Get ALL visuals
        
        const linkGroup = new THREE.Group();
        linkGroup.name = linkName;
        
        // Process each visual element in this link
        visuals.forEach(visual => {
          const geometry = visual.querySelector('geometry');
          const origin = visual.querySelector('origin');
          const material = visual.querySelector('material');
          
          let mesh = null;
          
          if (geometry) {
            const box = geometry.querySelector('box');
            const cylinder = geometry.querySelector('cylinder');
            const sphere = geometry.querySelector('sphere');
            const meshEl = geometry.querySelector('mesh');
            
            let geom = null;
            
            if (box) {
              const size = box.getAttribute('size').split(' ').map(parseFloat);
              geom = new THREE.BoxGeometry(size[0], size[1], size[2]);
            } else if (cylinder) {
              const radius = parseFloat(cylinder.getAttribute('radius'));
              const length = parseFloat(cylinder.getAttribute('length'));
              geom = new THREE.CylinderGeometry(radius, radius, length, 16);
            } else if (sphere) {
              const radius = parseFloat(sphere.getAttribute('radius'));
              geom = new THREE.SphereGeometry(radius, 16, 16);
            } else if (meshEl) {
              // Load OBJ mesh file
              const filename = meshEl.getAttribute('filename');
              const scaleAttr = meshEl.getAttribute('scale');
              
              if (filename && filename.startsWith('package://alfie_urdf/meshes/') && filename.endsWith('.obj')) {
                const meshName = filename.replace('package://alfie_urdf/meshes/', '');
                const meshUrl = `/meshes/${meshName}`;
                
                // Load OBJ asynchronously
                const loader = new THREE.OBJLoader();
                loader.load(
                  meshUrl,
                  (obj) => {
                    // Apply material color to all children
                    const matColor = getMaterialColor(material);
                    
                    obj.traverse((child) => {
                      if (child.isMesh) {
                        child.material = new THREE.MeshStandardMaterial({
                          color: matColor,
                          metalness: 0.3,
                          roughness: 0.7
                        });
                        child.castShadow = true;
                        child.receiveShadow = true;
                      }
                    });
                    
                    // Apply scale if specified
                    if (scaleAttr) {
                      const scaleVals = scaleAttr.split(' ').map(parseFloat);
                      obj.scale.set(scaleVals[0], scaleVals[1], scaleVals[2]);
                    }
                    
                    // Apply origin transform
                    if (origin) {
                      const xyz = origin.getAttribute('xyz');
                      const rpy = origin.getAttribute('rpy');
                      if (xyz) {
                        const pos = xyz.split(' ').map(parseFloat);
                        obj.position.set(pos[0], pos[1], pos[2]);
                      }
                      if (rpy) {
                        const rot = rpy.split(' ').map(parseFloat);
                        // URDF uses fixed-axis RPY (roll, pitch, yaw = X, Y, Z)
                        obj.rotation.order = 'ZYX';  // Fixed-axis XYZ = intrinsic ZYX
                        obj.rotation.set(rot[0], rot[1], rot[2]);
                      }
                    }
                    
                    linkGroup.add(obj);
                  },
                  undefined,
                  (error) => {
                    console.warn('Failed to load mesh:', meshUrl, error);
                  }
                );
                // Don't create placeholder - mesh will be added async
                geom = null;
              } else {
                // Fallback placeholder for unsupported mesh formats
                geom = new THREE.BoxGeometry(0.03, 0.03, 0.03);
              }
            }
            
            if (geom) {
              const color = getMaterialColor(material);
              
              const mat = new THREE.MeshStandardMaterial({
                color: color,
                metalness: 0.3,
                roughness: 0.7
              });
              
              mesh = new THREE.Mesh(geom, mat);
              mesh.castShadow = true;
              mesh.receiveShadow = true;
              
              // Apply origin transform
              if (origin) {
                const xyz = origin.getAttribute('xyz');
                const rpy = origin.getAttribute('rpy');
                
                if (xyz) {
                  const pos = xyz.split(' ').map(parseFloat);
                  mesh.position.set(pos[0], pos[1], pos[2]);
                }
                if (rpy) {
                  const rot = rpy.split(' ').map(parseFloat);
                  // URDF uses fixed-axis RPY (roll, pitch, yaw = X, Y, Z)
                  mesh.rotation.order = 'ZYX';  // Fixed-axis XYZ = intrinsic ZYX
                  mesh.rotation.set(rot[0], rot[1], rot[2]);
                }
              }
              
              linkGroup.add(mesh);
            }
          }
        });  // End of visuals.forEach
        
        this.links[linkName] = linkGroup;
        console.log('Parsed link:', linkName, 'children:', linkGroup.children.length);
        // Don't add to robot yet - we'll build hierarchy from joints
      });
      
      // Parse joints to build hierarchy
      const jointElements = urdfDoc.querySelectorAll('joint');
      const childLinks = new Set();  // Track which links are children
      
      jointElements.forEach(jointEl => {
        const jointName = jointEl.getAttribute('name');
        const jointType = jointEl.getAttribute('type');
        const parent = jointEl.querySelector('parent')?.getAttribute('link');
        const child = jointEl.querySelector('child')?.getAttribute('link');
        const origin = jointEl.querySelector('origin');
        const axis = jointEl.querySelector('axis');
        
        this.joints[jointName] = {
          type: jointType,
          parent: parent,
          child: child,
          axis: axis ? axis.getAttribute('xyz').split(' ').map(parseFloat) : [0, 0, 1]
        };
        
        // Build parent-child hierarchy
        if (parent && child && this.links[parent] && this.links[child]) {
          childLinks.add(child);
          
          // Apply joint origin transform to child link
          if (origin) {
            const xyz = origin.getAttribute('xyz');
            const rpy = origin.getAttribute('rpy');
            
            if (xyz) {
              const pos = xyz.split(' ').map(parseFloat);
              this.links[child].position.set(pos[0], pos[1], pos[2]);
            }
            if (rpy) {
              const rot = rpy.split(' ').map(parseFloat);
              // URDF uses fixed-axis RPY (roll, pitch, yaw = X, Y, Z)
              this.links[child].rotation.order = 'ZYX';  // Fixed-axis XYZ = intrinsic ZYX
              this.links[child].rotation.set(rot[0], rot[1], rot[2]);
            }
          }
          
          // Add child link as child of parent link (this makes transforms accumulate)
          this.links[parent].add(this.links[child]);
        }
      });
      
      // For TF-based updates, we use a FLAT structure (not hierarchical)
      // Each link is added directly to the robot group and positioned via TF
      // This allows TF transforms to work correctly
      Object.keys(this.links).forEach(linkName => {
        // Add ALL links directly to robot (flat structure for TF)
        if (!childLinks.has(linkName)) {
          // Root links go directly under robot
          this.robot.add(this.links[linkName]);
          console.log('Root link:', linkName);
        }
        // Child links are already added to parents in hierarchy
        // The hierarchy is maintained for initial positioning from URDF
        // TF updates will override positions as transforms arrive
      });
      
      // Rotate to convert from ROS (Z-up) to Three.js (Y-up) coordinate system
      this.robot.rotation.x = -Math.PI / 2;
      
      this.scene.add(this.robot);
      this.urdfLoaded = true;
      this.updateStatus(`URDF loaded: ${Object.keys(this.links).length} links`);
      console.log(`URDF loaded with ${Object.keys(this.links).length} links and ${Object.keys(this.joints).length} joints`);
      
    } catch (error) {
      console.error('Error parsing URDF:', error);
      this.updateStatus('URDF parse error');
    }
  }

  handleTFMessage(data) {
    // Decode TF message (tf2_msgs/TFMessage) in CDR format
    try {
      let offset = 4;  // Skip CDR header
      const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
      
      const readString = () => {
        const len = view.getUint32(offset, true);
        offset += 4;
        // len includes null terminator, so read len-1 bytes for string content
        const strBytes = new Uint8Array(data.buffer, data.byteOffset + offset, len > 0 ? len - 1 : 0);
        offset += len;
        // Align to 4 bytes after string
        if (offset % 4 !== 0) offset += 4 - (offset % 4);
        return new TextDecoder().decode(strBytes);
      };
      
      const align = (n) => { if (offset % n !== 0) offset += n - (offset % n); };
      
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
        // CDR alignment quirk: after child_frame_id, ensure offset % 8 === 4
        // (Transform struct has internal padding that results in this pattern)
        if (offset % 8 === 0) offset += 4;
        const tx = view.getFloat64(offset, true); offset += 8;
        const ty = view.getFloat64(offset, true); offset += 8;
        const tz = view.getFloat64(offset, true); offset += 8;
        const qx = view.getFloat64(offset, true); offset += 8;
        const qy = view.getFloat64(offset, true); offset += 8;
        const qz = view.getFloat64(offset, true); offset += 8;
        const qw = view.getFloat64(offset, true); offset += 8;
        
        // Store transform
        this.transforms[childFrameId] = {
          parent: frameId,
          position: new THREE.Vector3(tx, ty, tz),
          quaternion: new THREE.Quaternion(qx, qy, qz, qw)
        };
        
        if (this.tfUpdateCount < 3) {
          console.log('TF received:', frameId, '->', childFrameId, 'pos:', tx.toFixed(3), ty.toFixed(3), tz.toFixed(3));
        }
        
        // Apply transform to corresponding link
        this.applyTransformToLink(childFrameId);
      }
      
      // Update TF count and rate tracking
      this.tfUpdateCount++;
      this.tfRateCount++;  // Count every update for accurate rate
      
    } catch (error) {
      // Log TF decode errors for debugging
      console.warn('TF decode error:', error, 'at offset approx', error.offset || 'unknown');
    }
  }

  processTFData(tfData) {
    // JSON format fallback (for non-CDR messages)
    if (!tfData || !tfData.transforms) return;
    
    for (const transform of tfData.transforms) {
      const childFrame = transform.child_frame_id;
      const translation = transform.transform.translation;
      const rotation = transform.transform.rotation;
      
      this.transforms[childFrame] = {
        parent: transform.header?.frame_id || '',
        position: new THREE.Vector3(translation.x, translation.y, translation.z),
        quaternion: new THREE.Quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
      };
      
      // Apply transform to corresponding link
      this.applyTransformToLink(childFrame);
    }
    
    // Update TF count and rate tracking
    this.tfUpdateCount++;
    this.tfRateCount++;  // Count every update for accurate rate
  }

  handleJointStateMessage(data) {
    try {
      const decoder = new TextDecoder();
      const jsonStr = decoder.decode(data);
      const jointData = JSON.parse(jsonStr);
      this.processJointStateData(jointData);
    } catch (error) {
      // Binary format
    }
  }

  processJointStateData(jointData) {
    if (!jointData || !jointData.name || !jointData.position) return;
    
    for (let i = 0; i < jointData.name.length; i++) {
      const jointName = jointData.name[i];
      const position = jointData.position[i];
      
      this.applyJointPosition(jointName, position);
    }
  }

  applyTransformToLink(frameName) {
    // Try exact match first, then common variations
    let link = this.links[frameName];
    if (!link) {
      // Try without leading slash
      const cleanName = frameName.replace(/^\//, '');
      link = this.links[cleanName];
    }
    
    if (link && this.transforms[frameName]) {
      const tf = this.transforms[frameName];
      
      // Apply the transform from TF
      // TF gives us the transform from parent to child frame
      link.position.copy(tf.position);
      link.quaternion.copy(tf.quaternion);
    } else if (this.tfUpdateCount < 5) {
      // Log first few misses for debugging
      console.log('TF frame not found in links:', frameName, 'Available:', Object.keys(this.links).slice(0, 10));
    }
  }

  applyJointPosition(jointName, position) {
    // Find the joint in the robot model and apply rotation
    if (!this.robot) return;
    
    this.robot.traverse((child) => {
      if (child.userData && child.userData.jointName === jointName) {
        // Apply rotation based on joint axis
        const axis = child.userData.axis || new THREE.Vector3(0, 0, 1);
        child.setRotationFromAxisAngle(axis, position);
      }
    });
  }

  async loadRobotModel() {
    console.log('Loading robot model...');
    this.updateStatus('Loading robot model...');
    
    // Create a simple robot representation using primitives
    // In production, you'd parse the actual URDF and load meshes
    this.robot = new THREE.Group();
    this.robot.name = 'alfiebot';
    
    // Create placeholder links based on URDF structure
    const linkData = [
      { name: 'base_link', geometry: new THREE.BoxGeometry(0.15, 0.1, 0.2), color: 0x333333, position: [0, 0.05, 0] },
      { name: 'back_link', geometry: new THREE.BoxGeometry(0.14, 0.4, 0.08), color: 0x444444, position: [0, 0.35, -0.04] },
      { name: 'head_link', geometry: new THREE.SphereGeometry(0.08, 16, 16), color: 0x222222, position: [0, 0.85, 0] },
      // Left arm
      { name: 'left_shoulder_link', geometry: new THREE.BoxGeometry(0.05, 0.05, 0.05), color: 0x666666, position: [0.12, 0.7, 0] },
      { name: 'left_arm_upper_link', geometry: new THREE.CylinderGeometry(0.02, 0.02, 0.2), color: 0x555555, position: [0.12, 0.6, 0.08] },
      { name: 'left_arm_lower_link', geometry: new THREE.CylinderGeometry(0.018, 0.018, 0.18), color: 0x555555, position: [0.12, 0.45, 0.18] },
      { name: 'left_hand_link', geometry: new THREE.BoxGeometry(0.04, 0.03, 0.06), color: 0x777777, position: [0.12, 0.38, 0.25] },
      // Right arm
      { name: 'right_shoulder_link', geometry: new THREE.BoxGeometry(0.05, 0.05, 0.05), color: 0x666666, position: [-0.12, 0.7, 0] },
      { name: 'right_arm_upper_link', geometry: new THREE.CylinderGeometry(0.02, 0.02, 0.2), color: 0x555555, position: [-0.12, 0.6, 0.08] },
      { name: 'right_arm_lower_link', geometry: new THREE.CylinderGeometry(0.018, 0.018, 0.18), color: 0x555555, position: [-0.12, 0.45, 0.18] },
      { name: 'right_hand_link', geometry: new THREE.BoxGeometry(0.04, 0.03, 0.06), color: 0x777777, position: [-0.12, 0.38, 0.25] },
      // Wheels
      { name: 'mecanum_fl_link', geometry: new THREE.CylinderGeometry(0.03, 0.03, 0.02, 16), color: 0x222222, position: [0.06, 0.02, 0.07], rotation: [Math.PI/2, 0, 0] },
      { name: 'mecanum_fr_link', geometry: new THREE.CylinderGeometry(0.03, 0.03, 0.02, 16), color: 0x222222, position: [-0.06, 0.02, 0.07], rotation: [Math.PI/2, 0, 0] },
      { name: 'mecanum_rl_link', geometry: new THREE.CylinderGeometry(0.03, 0.03, 0.02, 16), color: 0x222222, position: [0.06, 0.02, -0.07], rotation: [Math.PI/2, 0, 0] },
      { name: 'mecanum_rr_link', geometry: new THREE.CylinderGeometry(0.03, 0.03, 0.02, 16), color: 0x222222, position: [-0.06, 0.02, -0.07], rotation: [Math.PI/2, 0, 0] },
    ];
    
    for (const data of linkData) {
      const material = new THREE.MeshStandardMaterial({ 
        color: data.color,
        metalness: 0.3,
        roughness: 0.7
      });
      const mesh = new THREE.Mesh(data.geometry, material);
      mesh.name = data.name;
      mesh.position.set(...data.position);
      if (data.rotation) {
        mesh.rotation.set(...data.rotation);
      }
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      
      this.links[data.name] = mesh;
      this.robot.add(mesh);
    }
    
    this.scene.add(this.robot);
    this.updateStatus('Robot model loaded');
    console.log('Robot model loaded with', Object.keys(this.links).length, 'links');
  }

  onResize() {
    if (!this.container || !this.camera || !this.renderer) return;
    
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;
    
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  animate(currentTime = 0) {
    requestAnimationFrame((time) => this.animate(time));
    
    // Throttle rendering to target FPS
    const elapsed = currentTime - this.lastFrameTime;
    if (elapsed < this.frameInterval) return;
    this.lastFrameTime = currentTime - (elapsed % this.frameInterval);
    
    if (this.renderer && this.scene && this.camera) {
      this.renderer.render(this.scene, this.camera);
    }
  }

  updateStatus(message) {
    const statusEl = document.getElementById('urdfViewerStatus');
    if (statusEl) {
      statusEl.textContent = message;
    }
    console.log('URDF Viewer:', message);
  }

  updateConnectionStatus(connected) {
    const statusEl = document.getElementById('foxgloveStatus');
    if (statusEl) {
      statusEl.textContent = connected ? 'Connected' : 'Disconnected';
      statusEl.className = 'info-value ' + (connected ? 'connected' : 'disconnected');
    }
  }

  updateTFCount() {
    // Legacy method - rate tracking now happens in TF handlers
    // UI updated by updateTFRate every 5 seconds
  }
  
  updateTFRate() {
    const now = performance.now();
    const elapsed = (now - this.lastRateUpdate) / 1000; // seconds
    
    if (elapsed > 0) {
      this.tfRateHz = this.tfRateCount / elapsed;
    }
    
    this.tfRateCount = 0;
    this.lastRateUpdate = now;
    
    const countEl = document.getElementById('tfUpdateCount');
    if (countEl) {
      countEl.textContent = this.tfRateHz.toFixed(1) + ' Hz';
    }
  }

  dispose() {
    if (this.foxgloveClient) {
      this.foxgloveClient.close();
    }
    if (this.renderer) {
      this.renderer.dispose();
    }
    if (this.controls) {
      this.controls.dispose();
    }
  }
  
  showCertWarning() {
    const warningEl = document.getElementById('certWarning');
    const linkEl = document.getElementById('acceptCertLink');
    if (warningEl && linkEl) {
      // Set the link to the Foxglove Bridge HTTPS endpoint
      const foxgloveHttps = `https://${window.location.hostname}:8765`;
      linkEl.href = foxgloveHttps;
      warningEl.style.display = 'block';
    }
  }
  
  hideCertWarning() {
    const warningEl = document.getElementById('certWarning');
    if (warningEl) {
      warningEl.style.display = 'none';
    }
  }
}

// Initialize URDF viewer when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
  const container = document.getElementById('urdfViewerContainer');
  if (container) {
    // Use wss:// via nginx proxy (port 8082) for secure WebSocket connection
    // nginx terminates TLS and proxies to foxglove_bridge on port 8765
    const foxgloveUrl = `wss://${window.location.hostname}:8082`;
    
    window.urdfViewer = new URDFViewer('urdfViewerContainer', {
      foxgloveUrl: foxgloveUrl
    });
    
    console.log('URDF Viewer initialized with URL:', foxgloveUrl);
  }
  
  // Setup panel toggle functionality
  const toggleBtn = document.getElementById('toggleViewerBtn');
  const resetBtn = document.getElementById('resetViewBtn');
  const panel = document.getElementById('robotViewerPanel');
  
  if (toggleBtn && panel) {
    toggleBtn.addEventListener('click', () => {
      panel.classList.toggle('minimized');
      toggleBtn.textContent = panel.classList.contains('minimized') ? '+' : '−';
      toggleBtn.title = panel.classList.contains('minimized') ? 'Expand' : 'Minimize';
      
      // Trigger resize after animation
      setTimeout(() => {
        if (window.urdfViewer) {
          window.urdfViewer.onResize();
        }
      }, 300);
    });
  }
  
  if (resetBtn) {
    resetBtn.addEventListener('click', () => {
      if (window.urdfViewer && window.urdfViewer.controls) {
        window.urdfViewer.camera.position.set(0.8, 0.6, 0.8);
        window.urdfViewer.controls.target.set(0, 0.4, 0);
        window.urdfViewer.controls.update();
      }
    });
  }
});

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
  module.exports = URDFViewer;
}