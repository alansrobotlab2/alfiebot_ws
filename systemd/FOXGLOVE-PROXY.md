# Foxglove Bridge TLS/WSS Proxy

This document describes how to set up nginx as a TLS termination proxy for the Foxglove Bridge WebSocket server, enabling secure WSS connections from browsers and Foxglove Studio.

## Overview

The Foxglove Bridge (`ros2 launch foxglove_bridge foxglove_bridge_launch.xml`) runs a WebSocket server on `ws://localhost:8765` without TLS encryption. However, when accessing Foxglove from:

- **HTTPS web pages** (like the VR interface)
- **Remote browsers** over the network
- **Foxglove Studio** requiring secure connections

...you need WSS (WebSocket Secure) connections. This proxy configuration uses nginx to:

1. Listen on port **8082** with TLS enabled
2. Terminate TLS and decrypt incoming WSS connections
3. Proxy requests to the non-TLS Foxglove Bridge on port **8765**
4. Handle WebSocket protocol upgrades correctly

```
┌─────────────────────┐        ┌─────────────────────┐        ┌─────────────────────┐
│   Foxglove Studio   │        │     nginx proxy     │        │  foxglove_bridge    │
│   or Browser        │  WSS   │  (TLS termination)  │   WS   │   (ROS2 node)       │
│                     │ ──────►│   :8082 (SSL)       │──────► │   :8765 (no TLS)    │
└─────────────────────┘        └─────────────────────┘        └─────────────────────┘
```

## Why This Is Needed

### Mixed Content Policy
Modern browsers block insecure WebSocket (`ws://`) connections from HTTPS pages. If your VR web interface is served over HTTPS, it **cannot** connect to `ws://robot:8765` directly.

### Self-Signed Certificate Sharing
By using the same SSL certificates as the VR web server (`alfie_vr`), users only need to accept one self-signed certificate for both the web interface and the Foxglove connection.

## Prerequisites

1. **nginx installed:**
   ```bash
   sudo apt update
   sudo apt install nginx
   ```

2. **SSL certificates exist** (shared with alfie_vr):
   ```
   /home/alfie/alfiebot_ws/src/alfie_vr/alfie_vr/cert.pem
   /home/alfie/alfiebot_ws/src/alfie_vr/alfie_vr/key.pem
   ```

3. **Foxglove Bridge running** on port 8765:
   ```bash
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml
   ```

## Installation

### 1. Copy the nginx configuration

```bash
sudo cp /home/alfie/alfiebot_ws/systemd/foxglove-proxy.conf /etc/nginx/sites-available/foxglove-proxy
```

### 2. Enable the site

```bash
sudo ln -s /etc/nginx/sites-available/foxglove-proxy /etc/nginx/sites-enabled/
```

### 3. Test the configuration

```bash
sudo nginx -t
```

You should see:
```
nginx: the configuration file /etc/nginx/nginx.conf syntax is ok
nginx: configuration file /etc/nginx/nginx.conf test is successful
```

### 4. Reload nginx

```bash
sudo systemctl reload nginx
```

Or restart if needed:
```bash
sudo systemctl restart nginx
```

### 5. Verify nginx is running

```bash
sudo systemctl status nginx
```

## Usage

### Connecting from Foxglove Studio

1. Open Foxglove Studio
2. Click "Open connection"
3. Select "Foxglove WebSocket"
4. Enter: `wss://ROBOT_IP:8082`
5. Accept the self-signed certificate if prompted

### Connecting from Browser JavaScript

```javascript
const ws = new WebSocket('wss://robot:8082');
ws.onopen = () => console.log('Connected to Foxglove Bridge');
ws.onmessage = (event) => console.log('Received:', event.data);
```

### Accepting Self-Signed Certificate

Before the WebSocket connection will work, you need to accept the self-signed certificate in your browser:

1. Navigate to `https://ROBOT_IP:8082/health` in your browser
2. Accept the security warning / proceed to the site
3. You should see: `Foxglove proxy OK - certificate accepted!`
4. Now WSS connections will work from that browser

## Configuration Details

The proxy configuration (`foxglove-proxy.conf`) includes:

### TLS Settings
```nginx
listen 8082 ssl;
ssl_certificate /path/to/cert.pem;
ssl_certificate_key /path/to/key.pem;
```

### WebSocket Upgrade Headers
```nginx
proxy_set_header Upgrade $http_upgrade;
proxy_set_header Connection "upgrade";
```
These headers are **required** for the HTTP→WebSocket protocol upgrade.

### Foxglove Protocol Header
```nginx
proxy_set_header Sec-WebSocket-Protocol $http_sec_websocket_protocol;
```
Foxglove uses a specific WebSocket subprotocol that must be forwarded.

### Long-Running Connection Timeouts
```nginx
proxy_read_timeout 86400s;
proxy_send_timeout 86400s;
```
24-hour timeout to prevent nginx from closing idle connections.

### No Buffering
```nginx
proxy_buffering off;
```
WebSocket traffic should not be buffered for real-time performance.

## Troubleshooting

### Connection Refused

**Check foxglove_bridge is running:**
```bash
ros2 node list | grep foxglove
```

**Check it's listening on port 8765:**
```bash
ss -tlnp | grep 8765
```

### 502 Bad Gateway

nginx cannot reach the upstream server (foxglove_bridge):
1. Verify foxglove_bridge is running
2. Check port 8765 is correct in the config
3. Look at nginx error logs: `sudo tail -f /var/log/nginx/error.log`

### SSL Certificate Errors

**Check certificates exist:**
```bash
ls -la /home/alfie/alfiebot_ws/src/alfie_vr/alfie_vr/*.pem
```

**Generate new certificates if needed:**
```bash
cd /home/alfie/alfiebot_ws/src/alfie_vr/alfie_vr/
openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365 -nodes
```

### WebSocket Connection Fails After Certificate Accepted

Check if the Sec-WebSocket-Protocol header is being forwarded correctly. Some Foxglove versions require this header.

### Checking nginx Logs

**Access log:**
```bash
sudo tail -f /var/log/nginx/access.log
```

**Error log:**
```bash
sudo tail -f /var/log/nginx/error.log
```

## Port Summary

| Port | Protocol | Service | TLS |
|------|----------|---------|-----|
| 8765 | WS | foxglove_bridge (internal) | No |
| 8082 | WSS | nginx proxy (external) | Yes |

## Firewall Configuration

If using ufw, allow the proxy port:

```bash
sudo ufw allow 8082/tcp comment 'Foxglove WSS Proxy'
```

## Uninstallation

To remove the proxy configuration:

```bash
sudo rm /etc/nginx/sites-enabled/foxglove-proxy
sudo rm /etc/nginx/sites-available/foxglove-proxy
sudo systemctl reload nginx
```

## Related Files

- `foxglove-proxy.conf` - nginx site configuration
- `alfiebot.service` - Main robot service (launches foxglove_bridge via bringup)
- SSL certificates in `src/alfie_vr/alfie_vr/`

## See Also

- [Foxglove Bridge Documentation](https://docs.foxglove.dev/docs/connecting-to-data/ros-foxglove-bridge/)
- [nginx WebSocket Proxying](https://nginx.org/en/docs/http/websocket.html)
- Main project README in repository root
