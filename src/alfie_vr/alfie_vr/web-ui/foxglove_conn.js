// Foxglove WebSocket Connection Manager
// Reusable module for connecting to Foxglove Bridge
// Based on patterns from urdf_viewer.js and vr_app.js

const FOXGLOVE_PORT = 8082;

class FoxgloveConnection {
    constructor(options = {}) {
        this.options = {
            // Port 8082 is nginx proxy that terminates TLS and forwards to foxglove_bridge on 8765
            url: options.url || `wss://${window.location.hostname}:${FOXGLOVE_PORT}`,
            reconnectInterval: options.reconnectInterval || 5000,
            statusElementId: options.statusElementId || 'foxgloveStatus',
            onConnect: options.onConnect || null,
            onDisconnect: options.onDisconnect || null,
            onAdvertise: options.onAdvertise || null,
            onMessage: options.onMessage || null,
            onBinaryMessage: options.onBinaryMessage || null,
            ...options
        };

        this.client = null;
        this.isConnected = false;
        this.subscriptions = new Map(); // channelId -> { id, topic, handler }
        this.nextSubscriptionId = 1;
        this.channels = []; // Available channels from advertise message
        
        // TF rate tracking
        this.tfRateCount = 0;
        this.tfRateHz = 0;
        this.lastRateUpdate = performance.now();
        this.tfRateInterval = null;
    }

    // Connect to Foxglove Bridge
    connect() {
        console.log(`Connecting to Foxglove Bridge at ${this.options.url}...`);
        this.updateStatusIndicator('Connecting...', 'connecting');

        try {
            // Create WebSocket connection with SDK subprotocol (required for foxglove_bridge 3.x)
            this.client = new WebSocket(this.options.url, ['foxglove.sdk.v1']);
            this.client.binaryType = 'arraybuffer';

            this.client.onopen = () => {
                console.log('Connected to Foxglove Bridge');
                this.isConnected = true;
                this.updateStatusIndicator('Connected', 'connected');
                this.startTFRateTracking();
                
                if (this.options.onConnect) {
                    this.options.onConnect();
                }
            };

            this.client.onmessage = (event) => {
                this.handleMessage(event.data);
            };

            this.client.onerror = (error) => {
                console.error('Foxglove WebSocket error:', error);
                this.updateStatusIndicator('Error - check certificate', 'error');
            };

            this.client.onclose = (event) => {
                console.log('Foxglove WebSocket closed. Code:', event.code, 'Reason:', event.reason);
                this.isConnected = false;
                this.updateStatusIndicator('Disconnected', 'disconnected');
                this.stopTFRateTracking();

                if (this.options.onDisconnect) {
                    this.options.onDisconnect(event);
                }

                // Attempt reconnection (but not for protocol errors)
                if (event.code !== 1002 && event.code !== 1003) {
                    setTimeout(() => this.connect(), this.options.reconnectInterval);
                }
            };

        } catch (error) {
            console.error('Failed to connect to Foxglove:', error);
            this.updateStatusIndicator('Connection failed', 'error');
            setTimeout(() => this.connect(), this.options.reconnectInterval);
        }
    }

    // Handle incoming WebSocket messages
    handleMessage(data) {
        try {
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

    // Handle JSON protocol messages
    handleJsonMessage(message) {
        switch (message.op) {
            case 'serverInfo':
                console.log('Foxglove server info:', message.name, message.capabilities);
                break;

            case 'advertise':
                console.log('Available channels:', message.channels?.length);
                this.channels = message.channels || [];
                if (this.options.onAdvertise) {
                    this.options.onAdvertise(this.channels);
                }
                break;

            case 'status':
                console.log('Foxglove status:', message);
                break;

            case 'message':
                if (this.options.onMessage) {
                    this.options.onMessage(message);
                }
                break;
        }
    }

    // Handle binary messages (topic data)
    handleBinaryMessage(data) {
        const view = new DataView(data);
        const opCode = view.getUint8(0);

        if (opCode === 1) { // Message data
            const subscriptionId = view.getUint32(1, true);
            const timestamp = view.getBigUint64(5, true);
            const messageData = new Uint8Array(data, 13);

            // Find subscription handler
            for (const [channelId, sub] of this.subscriptions) {
                if (sub.id === subscriptionId) {
                    // Track TF rate for TF topics
                    if (sub.topic.includes('/tf')) {
                        this.tfRateCount++;
                    }
                    
                    if (sub.handler) {
                        sub.handler(messageData, timestamp, sub.topic);
                    }
                    break;
                }
            }

            // Also call generic binary handler if provided
            if (this.options.onBinaryMessage) {
                this.options.onBinaryMessage(subscriptionId, messageData, timestamp);
            }
        }
    }

    // Subscribe to a topic
    subscribe(topic, handler = null) {
        const channel = this.channels.find(c => c.topic === topic);
        if (!channel) {
            console.warn(`Topic not found: ${topic}`);
            return null;
        }

        const subscriptionId = this.nextSubscriptionId++;
        this.subscriptions.set(channel.id, {
            id: subscriptionId,
            channelId: channel.id,
            topic: topic,
            handler: handler
        });

        const subscribeMsg = {
            op: 'subscribe',
            subscriptions: [{ id: subscriptionId, channelId: channel.id }]
        };

        this.client.send(JSON.stringify(subscribeMsg));
        console.log(`Subscribed to ${topic} (channelId: ${channel.id}, subId: ${subscriptionId})`);
        
        return subscriptionId;
    }

    // Subscribe to multiple topics at once
    subscribeMultiple(topicsWithHandlers) {
        const subscriptions = [];
        
        for (const { topic, handler } of topicsWithHandlers) {
            const channel = this.channels.find(c => c.topic === topic);
            if (channel) {
                const subscriptionId = this.nextSubscriptionId++;
                this.subscriptions.set(channel.id, {
                    id: subscriptionId,
                    channelId: channel.id,
                    topic: topic,
                    handler: handler
                });
                subscriptions.push({ id: subscriptionId, channelId: channel.id });
                console.log(`Subscribing to ${topic} (channelId: ${channel.id}, subId: ${subscriptionId})`);
            } else {
                console.warn(`Topic not found: ${topic}`);
            }
        }

        if (subscriptions.length > 0) {
            this.client.send(JSON.stringify({ op: 'subscribe', subscriptions }));
            console.log(`Subscribed to ${subscriptions.length} topics`);
        }

        return subscriptions.length;
    }

    // Find a channel by topic pattern
    findChannel(topicPattern) {
        if (typeof topicPattern === 'string') {
            return this.channels.find(c => c.topic === topicPattern);
        } else if (topicPattern instanceof RegExp) {
            return this.channels.find(c => topicPattern.test(c.topic));
        }
        return null;
    }

    // Get all channels matching a pattern
    findChannels(topicPattern) {
        if (typeof topicPattern === 'string') {
            return this.channels.filter(c => c.topic.includes(topicPattern));
        } else if (topicPattern instanceof RegExp) {
            return this.channels.filter(c => topicPattern.test(c.topic));
        }
        return [];
    }

    // Update the status indicator element
    updateStatusIndicator(text, state = 'disconnected') {
        const element = document.getElementById(this.options.statusElementId);
        if (element) {
            element.textContent = text;
            element.className = `robot-status-value foxglove-${state}`;
        }
    }

    // TF Rate tracking
    startTFRateTracking() {
        this.tfRateInterval = setInterval(() => {
            const now = performance.now();
            const elapsed = (now - this.lastRateUpdate) / 1000;
            this.tfRateHz = this.tfRateCount / elapsed;
            this.tfRateCount = 0;
            this.lastRateUpdate = now;
            
            this.updateTFRateDisplay();
        }, 5000); // Update every 5 seconds
    }

    stopTFRateTracking() {
        if (this.tfRateInterval) {
            clearInterval(this.tfRateInterval);
            this.tfRateInterval = null;
        }
        this.updateTFRateDisplay(0);
    }

    updateTFRateDisplay(rate = null) {
        const tfRateElement = document.getElementById('tfRate');
        if (tfRateElement) {
            const displayRate = rate !== null ? rate : this.tfRateHz;
            tfRateElement.textContent = displayRate > 0 ? `${displayRate.toFixed(1)} Hz` : '-- Hz';
        }
    }

    // Disconnect from Foxglove
    disconnect() {
        if (this.client) {
            this.client.close();
            this.client = null;
        }
        this.isConnected = false;
        this.stopTFRateTracking();
    }

    // Send a message to the server
    send(message) {
        if (this.client && this.isConnected) {
            if (typeof message === 'object') {
                this.client.send(JSON.stringify(message));
            } else {
                this.client.send(message);
            }
        }
    }

    // Get Foxglove URL for certificate acceptance
    getCertUrl() {
        return `https://${window.location.hostname}:${FOXGLOVE_PORT}/`;
    }
}

// Export for ES modules
export { FoxgloveConnection, FOXGLOVE_PORT };
