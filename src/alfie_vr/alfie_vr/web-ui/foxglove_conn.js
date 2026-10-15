// Foxglove WebSocket Connection Manager
// Reusable module for connecting to Foxglove Bridge
//
// Subscriptions are declared by topic and persist across reconnects: whenever a
// matching channel is advertised (initially, later, or after a reconnect) the
// subscription is (re)sent automatically.

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
            ...options
        };

        this.client = null;
        this.isConnected = false;
        this.serverCapabilities = [];
        this.reconnectTimer = null;

        // Desired subscriptions (persist across reconnects): topic -> handler
        this.topicHandlers = new Map();

        // Per-connection state (cleared on close)
        this.channelsById = new Map();    // channelId -> channel
        this.activeBySubId = new Map();   // subscriptionId -> { id, channelId, topic }
        this.activeByTopic = new Map();   // topic -> subscriptionId
        this.nextSubscriptionId = 1;
    }

    // Available channels from advertise messages (merged)
    get channels() {
        return Array.from(this.channelsById.values());
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
                this.client = null;
                this.resetConnectionState();
                this.updateStatusIndicator('Disconnected', 'disconnected');

                if (this.options.onDisconnect) {
                    this.options.onDisconnect(event);
                }

                // Attempt reconnection (but not for protocol errors)
                if (event.code !== 1002 && event.code !== 1003) {
                    this.scheduleReconnect();
                }
            };

        } catch (error) {
            console.error('Failed to connect to Foxglove:', error);
            this.updateStatusIndicator('Connection failed', 'error');
            this.scheduleReconnect();
        }
    }

    scheduleReconnect() {
        if (this.reconnectTimer) return;
        this.reconnectTimer = setTimeout(() => {
            this.reconnectTimer = null;
            this.connect();
        }, this.options.reconnectInterval);
    }

    // Channels and subscription ids are only valid for one connection
    resetConnectionState() {
        this.channelsById.clear();
        this.activeBySubId.clear();
        this.activeByTopic.clear();
        this.nextSubscriptionId = 1;
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
                this.serverCapabilities = message.capabilities || [];
                break;

            case 'advertise': {
                // Advertise messages are incremental - merge, don't replace
                const channels = message.channels || [];
                for (const channel of channels) {
                    this.channelsById.set(channel.id, channel);
                }
                console.log(`Advertised channels: +${channels.length} (total ${this.channelsById.size})`);
                this.subscribePending();
                if (this.options.onAdvertise) {
                    this.options.onAdvertise(this.channels);
                }
                break;
            }

            case 'unadvertise': {
                const channelIds = message.channelIds || [];
                for (const channelId of channelIds) {
                    const channel = this.channelsById.get(channelId);
                    this.channelsById.delete(channelId);
                    // Server drops subscriptions on removed channels
                    for (const [subId, sub] of this.activeBySubId) {
                        if (sub.channelId === channelId) {
                            this.activeBySubId.delete(subId);
                            this.activeByTopic.delete(sub.topic);
                        }
                    }
                    if (channel) console.log(`Unadvertised ${channel.topic} (channelId: ${channelId})`);
                }
                break;
            }

            case 'status':
                console.log('Foxglove status:', message);
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

            const sub = this.activeBySubId.get(subscriptionId);
            if (!sub) return;
            const handler = this.topicHandlers.get(sub.topic);
            if (handler) {
                handler(messageData, timestamp, sub.topic);
            }
        }
    }

    // Declare a subscription by topic. It is sent now if the channel is known,
    // otherwise as soon as it is advertised (including after reconnects).
    // Returns true if currently subscribed.
    subscribe(topic, handler = null) {
        this.topicHandlers.set(topic, handler);
        this.subscribePending();
        return this.isSubscribed(topic);
    }

    isSubscribed(topic) {
        return this.activeByTopic.has(topic);
    }

    // Send subscribe for every desired topic that has a channel but no active subscription
    subscribePending() {
        if (!this.client || !this.isConnected || this.client.readyState !== WebSocket.OPEN) return;

        const subscriptions = [];
        for (const topic of this.topicHandlers.keys()) {
            if (this.activeByTopic.has(topic)) continue;
            const channel = this.findChannel(topic);
            if (!channel) continue;

            const subscriptionId = this.nextSubscriptionId++;
            this.activeBySubId.set(subscriptionId, { id: subscriptionId, channelId: channel.id, topic: topic });
            this.activeByTopic.set(topic, subscriptionId);
            subscriptions.push({ id: subscriptionId, channelId: channel.id });
            console.log(`Subscribing to ${topic} (channelId: ${channel.id}, subId: ${subscriptionId})`);
        }

        if (subscriptions.length > 0) {
            this.client.send(JSON.stringify({ op: 'subscribe', subscriptions }));
        }
    }

    // Find a channel by exact topic name
    findChannel(topic) {
        for (const channel of this.channelsById.values()) {
            if (channel.topic === topic) return channel;
        }
        return null;
    }

    // Update the status indicator element
    updateStatusIndicator(text, state = 'disconnected') {
        const element = document.getElementById(this.options.statusElementId);
        if (element) {
            element.textContent = text;
            element.className = `robot-status-value foxglove-${state}`;
        }
    }

    // Disconnect from Foxglove
    disconnect() {
        if (this.reconnectTimer) {
            clearTimeout(this.reconnectTimer);
            this.reconnectTimer = null;
        }
        if (this.client) {
            this.client.onclose = null;
            this.client.close();
            this.client = null;
        }
        this.isConnected = false;
        this.resetConnectionState();
    }
}

// Export for ES modules
export { FoxgloveConnection, FOXGLOVE_PORT };
