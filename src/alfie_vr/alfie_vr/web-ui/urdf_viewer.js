// URDF Viewer Component - Connects to Foxglove Bridge to visualize robot state
// Uses @foxglove/ws-protocol for WebSocket communication

class URDFViewer {
  constructor(containerId, options = {}) {
    this.container = document.getElementById(containerId);
    this.options = {
      foxgloveUrl: options.foxgloveUrl || `wss://${window.location.hostname}:8765`,
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
    this.transforms = {};
    this.foxgloveClient = null;
    this.tfSubscriptionId = null;
    this.jointStateSubscriptionId = null;
    this.robotDescriptionSubscriptionId = null;
    this.urdfLoaded = false;
    this.tfUpdateCount = 0;
    this.isConnected = false;
    
    this.init();
  }

  init() {
    this.setupScene();
    this.setupLighting();
    this.setupControls();
    this.connectToFoxglove();
    this.animate();
    
    // Handle container resize
    window.addEventListener('resize', () => this.onResize());
    new ResizeObserver(() => this.onResize()).observe(this.container);
  }

  setupScene() {
    // Create Three.js scene
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x1a1a2e);
    
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
    // Use OrbitControls if available
    if (typeof THREE.OrbitControls !== 'undefined') {
      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.05;
      this.controls.target.set(0, 0.4, 0);
      this.controls.update();
    }
  }

  async connectToFoxglove() {
    console.log(`Connecting to Foxglove Bridge at ${this.options.foxgloveUrl}...`);
    this.updateStatus('Connecting to Foxglove...');
    
    try {
      // Create WebSocket connection to Foxglove Bridge with the required subprotocol
      this.foxgloveClient = new WebSocket(this.options.foxgloveUrl, ['foxglove.websocket.v1']);
      
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
      if (subscriptionId === this.tfSubscriptionId) {
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
      // Subscribe to TF topic
      if (channel.topic === '/alfie/tf') {
        this.tfSubscriptionId = subscriptions.length + 1;
        subscriptions.push({
          id: this.tfSubscriptionId,
          channelId: channel.id
        });
        console.log(`Subscribing to TF: ${channel.topic}`);
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
      const decoder = new TextDecoder();
      const jsonStr = decoder.decode(data);
      const descData = JSON.parse(jsonStr);
      this.processRobotDescription(descData);
    } catch (error) {
      // Try parsing as plain URDF string
      try {
        const decoder = new TextDecoder();
        const urdfString = decoder.decode(data);
        if (urdfString.includes('<robot')) {
          this.parseAndLoadURDF(urdfString);
        }
      } catch (e) {
        console.error('Failed to parse robot description:', e);
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
      
      // Parse links
      const linkElements = urdfDoc.querySelectorAll('link');
      linkElements.forEach(linkEl => {
        const linkName = linkEl.getAttribute('name');
        const visual = linkEl.querySelector('visual');
        
        const linkGroup = new THREE.Group();
        linkGroup.name = linkName;
        
        if (visual) {
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
              // For mesh files, create a placeholder box
              geom = new THREE.BoxGeometry(0.05, 0.05, 0.05);
            }
            
            if (geom) {
              let color = 0x666666;
              if (material) {
                const colorEl = material.querySelector('color');
                if (colorEl) {
                  const rgba = colorEl.getAttribute('rgba').split(' ').map(parseFloat);
                  color = new THREE.Color(rgba[0], rgba[1], rgba[2]);
                }
              }
              
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
                  mesh.rotation.set(rot[0], rot[1], rot[2]);
                }
              }
              
              linkGroup.add(mesh);
            }
          }
        }
        
        this.links[linkName] = linkGroup;
        this.robot.add(linkGroup);
      });
      
      // Parse joints to build hierarchy
      const jointElements = urdfDoc.querySelectorAll('joint');
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
        
        // Apply origin to child link
        if (child && this.links[child] && origin) {
          const xyz = origin.getAttribute('xyz');
          const rpy = origin.getAttribute('rpy');
          
          if (xyz) {
            const pos = xyz.split(' ').map(parseFloat);
            this.links[child].position.set(pos[0], pos[1], pos[2]);
          }
          if (rpy) {
            const rot = rpy.split(' ').map(parseFloat);
            this.links[child].rotation.set(rot[0], rot[1], rot[2]);
          }
        }
      });
      
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
    // Decode TF message (tf2_msgs/TFMessage)
    // This is simplified - full implementation would use ROS message deserialization
    try {
      const decoder = new TextDecoder();
      const jsonStr = decoder.decode(data);
      const tfData = JSON.parse(jsonStr);
      this.processTFData(tfData);
    } catch (error) {
      // Binary format - would need proper ROS CDR deserialization
    }
  }

  processTFData(tfData) {
    if (!tfData || !tfData.transforms) return;
    
    for (const transform of tfData.transforms) {
      const childFrame = transform.child_frame_id;
      const translation = transform.transform.translation;
      const rotation = transform.transform.rotation;
      
      this.transforms[childFrame] = {
        position: new THREE.Vector3(translation.x, translation.y, translation.z),
        quaternion: new THREE.Quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
      };
      
      // Apply transform to corresponding link
      this.applyTransformToLink(childFrame);
    }
    
    // Update TF count
    this.tfUpdateCount++;
    this.updateTFCount();
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
    const linkName = frameName.replace('_link', '').replace('/', '');
    const link = this.links[linkName] || this.links[frameName];
    
    if (link && this.transforms[frameName]) {
      const tf = this.transforms[frameName];
      link.position.copy(tf.position);
      link.quaternion.copy(tf.quaternion);
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

  animate() {
    requestAnimationFrame(() => this.animate());
    
    if (this.controls) {
      this.controls.update();
    }
    
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
    const countEl = document.getElementById('tfUpdateCount');
    if (countEl) {
      countEl.textContent = this.tfUpdateCount.toString();
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
    // Use wss:// for secure WebSocket connection to Foxglove Bridge
    const foxgloveUrl = `wss://${window.location.hostname}:8765`;
    
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
