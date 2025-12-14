#!/usr/bin/env python3
"""
VR Monitor - Independent VR control information monitoring script
Can call AlfieVR's VR functionality from other folders and read/print VR control information
"""

import os
import sys
import asyncio
import json
import logging
import threading
import http.server
import ssl
import socket
from pathlib import Path
from typing import Optional

# Set the absolute path to the alfievr folder
ALFIEVR_PATH = "/home/alfie/alfiebot_ws/src/alfie_vr/alfie_vr"
# Path to URDF meshes
MESHES_PATH = "/home/alfie/alfiebot_ws/src/alfie_urdf/meshes"

def setup_alfievr_environment():
    """Setup alfievr environment"""
    # Add alfievr path to Python path
    if ALFIEVR_PATH not in sys.path:
        sys.path.insert(0, ALFIEVR_PATH)
    
    # Set working directory
    os.chdir(ALFIEVR_PATH)
    
    # Set environment variables
    os.environ['PYTHONPATH'] = f"{ALFIEVR_PATH}:{os.environ.get('PYTHONPATH', '')}"

def get_local_ip():
    """Get the local IP address of this machine."""
    try:
        # Connect to a remote address to determine the local IP
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        try:
            # Fallback: get hostname IP
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            # Final fallback
            return "localhost"

def import_alfievr_modules():
    """Import alfievr modules"""
    try:
        from alfievr.config import AlfieVRConfig
        from alfievr.inputs.vr_ws_server import VRWebSocketServer
        from alfievr.inputs.base import ControlGoal, ControlMode
        return AlfieVRConfig, VRWebSocketServer, ControlGoal, ControlMode
    except ImportError as e:
        print(f"Error importing alfievr modules: {e}")
        print(f"Make sure ALFIEVR_PATH is correct: {ALFIEVR_PATH}")
        return None, None, None, None

class SimpleAPIHandler(http.server.BaseHTTPRequestHandler):
    """Simplified HTTP request handler, only provides basic web services"""
    
    def end_headers(self):
        """Add CORS headers to all responses."""
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        try:
            super().end_headers()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, ssl.SSLError):
            pass
    
    def do_OPTIONS(self):
        """Handle preflight CORS requests."""
        self.send_response(200)
        self.end_headers()
    
    def log_message(self, format, *args):
        """Override to reduce HTTP request logging noise."""
        pass  # Disable default HTTP logging
    
    def do_GET(self):
        """Handle GET requests."""
        if self.path == '/' or self.path == '/index.html':
            # Serve main page from web-ui directory
            self.serve_file('web-ui/index.html', 'text/html')
        elif self.path == '/foxglove-cert':
            # Serve a page to help accept the Foxglove Bridge certificate
            self.serve_foxglove_cert_page()
        elif self.path.endswith('.css'):
            # Serve CSS files from web-ui directory
            self.serve_file(f'web-ui{self.path}', 'text/css')
        elif self.path.endswith('.js'):
            # Serve JS files from web-ui directory
            self.serve_file(f'web-ui{self.path}', 'application/javascript')
        elif self.path.endswith('.ico'):
            self.serve_file(self.path[1:], 'image/x-icon')
        elif self.path.endswith(('.jpg', '.jpeg', '.png', '.gif')):
            # Serve image files from web-ui directory
            content_type = 'image/jpeg' if self.path.endswith(('.jpg', '.jpeg')) else 'image/png' if self.path.endswith('.png') else 'image/gif'
            self.serve_file(f'web-ui{self.path}', content_type)
        elif self.path.startswith('/meshes/') and self.path.endswith('.obj'):
            # Serve OBJ mesh files from alfie_urdf meshes directory
            mesh_name = self.path[8:]  # Remove '/meshes/' prefix
            self.serve_mesh_file(mesh_name)
        elif self.path.startswith('/meshes/') and self.path.endswith('.mtl'):
            # Serve MTL material files from alfie_urdf meshes directory
            mtl_name = self.path[8:]  # Remove '/meshes/' prefix
            self.serve_mesh_file(mtl_name, 'text/plain')
        else:
            self.send_error(404, "Not found")
    
    def serve_foxglove_cert_page(self):
        """Serve a page that helps accept the Foxglove Bridge certificate."""
        # Get the hostname from the request
        host = self.headers.get('Host', 'localhost:8443').split(':')[0]
        foxglove_url = f"https://{host}:8765"
        
        html = f"""<!DOCTYPE html>
<html>
<head>
    <title>Accept Foxglove Certificate</title>
    <style>
        body {{ font-family: Arial, sans-serif; background: #1a1a2e; color: #fff; padding: 40px; text-align: center; }}
        .container {{ max-width: 600px; margin: 0 auto; }}
        h1 {{ color: #4ecdc4; }}
        .status {{ padding: 20px; margin: 20px 0; border-radius: 10px; }}
        .pending {{ background: #f39c12; color: #000; }}
        .success {{ background: #27ae60; }}
        .error {{ background: #e74c3c; }}
        button {{ background: #4ecdc4; border: none; padding: 15px 30px; font-size: 18px; border-radius: 5px; cursor: pointer; margin: 10px; }}
        button:hover {{ background: #45b7aa; }}
        .url {{ font-family: monospace; background: #333; padding: 10px; border-radius: 5px; }}
        iframe {{ display: none; }}
    </style>
</head>
<body>
    <div class="container">
        <h1>🔐 Foxglove Certificate Setup</h1>
        <p>The VR robot viewer needs to connect to Foxglove Bridge securely.</p>
        <p class="url">{foxglove_url}</p>
        
        <div id="status" class="status pending">
            Click the button below to accept the certificate
        </div>
        
        <button onclick="openFoxglove()">Open Foxglove URL</button>
        <button onclick="window.location.href='/'">Back to VR</button>
        
        <p style="margin-top: 30px; font-size: 14px; color: #888;">
            A new tab will open. You may see a security warning - click "Advanced" and "Proceed" to accept the certificate.
            Then close that tab and return here.
        </p>
    </div>
    
    <script>
        function openFoxglove() {{
            window.open('{foxglove_url}', '_blank');
            document.getElementById('status').className = 'status success';
            document.getElementById('status').innerHTML = 'Tab opened! Accept the certificate warning, then return to VR.';
        }}
    </script>
</body>
</html>"""
        
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.end_headers()
        self.wfile.write(html.encode('utf-8'))
    
    def serve_file(self, filename, content_type):
        """Serve a file with the given content type."""
        try:
            # Get the web root path from the server
            web_root = getattr(self.server, 'web_root_path', ALFIEVR_PATH)
            file_path = os.path.join(web_root, filename)
            
            if os.path.exists(file_path):
                with open(file_path, 'rb') as f:
                    content = f.read()
                
                self.send_response(200)
                self.send_header('Content-Type', content_type)
                # Disable caching for development - always fetch fresh files
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Expires', '0')
                self.end_headers()
                self.wfile.write(content)
            else:
                self.send_error(404, f"File not found: {filename}")
        except Exception as e:
            print(f"Error serving file {filename}: {e}")
            self.send_error(500, "Internal server error")
    
    def serve_mesh_file(self, filename, content_type='model/obj'):
        """Serve a mesh file from the URDF meshes directory."""
        try:
            # Security: only allow specific extensions and no path traversal
            if '..' in filename or filename.startswith('/'):
                self.send_error(403, "Forbidden")
                return
            
            file_path = os.path.join(MESHES_PATH, filename)
            
            if os.path.exists(file_path):
                with open(file_path, 'rb') as f:
                    content = f.read()
                
                self.send_response(200)
                self.send_header('Content-Type', content_type)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(content)
            else:
                self.send_error(404, f"Mesh not found: {filename}")
        except Exception as e:
            print(f"Error serving mesh {filename}: {e}")
            self.send_error(500, "Internal server error")

class SimpleHTTPSServer:
    """Simplified HTTPS server for providing web interface"""
    
    def __init__(self, config):
        self.config = config
        self.httpd = None
        self.server_thread = None
        self.web_root_path = ALFIEVR_PATH
    
    async def start(self):
        """Start the HTTPS server."""
        try:
            # Create server
            self.httpd = http.server.HTTPServer((self.config.host_ip, self.config.https_port), SimpleAPIHandler)
            
            # Set web root path for file serving
            self.httpd.web_root_path = self.web_root_path
            
            # Setup SSL
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.load_cert_chain('cert.pem', 'key.pem')
            self.httpd.socket = context.wrap_socket(self.httpd.socket, server_side=True)
            
            # Start server in a separate thread
            self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
            self.server_thread.start()
            
            print(f"🌐 HTTPS server started on {self.config.host_ip}:{self.config.https_port}")
            
        except Exception as e:
            print(f"❌ Failed to start HTTPS server: {e}")
            raise
    
    async def stop(self):
        """Stop the HTTPS server."""
        if self.httpd:
            self.httpd.shutdown()
            if self.server_thread:
                self.server_thread.join(timeout=5)
            print("🌐 HTTPS server stopped")

class VRMonitor:
    """VR control information monitor"""
    
    def __init__(self, debug_logs: bool = False):
        self.config = None
        self.vr_server = None
        self.https_server = None
        self.is_running = False
        self.latest_goal = None  # Store the latest goal
        self.left_goal = None    # Store left and right controller goals separately
        self.right_goal = None
        self.headset_goal = None  # Add headset goal
        self._goal_lock = threading.Lock()  # Add thread lock
        self.debug_logs = debug_logs  # Debug logging flag
    
    def initialize(self):
        """Initialize VR monitor"""
        print("🔧 Initializing AlfieVR Monitor...")
        
        # Setup environment
        setup_alfievr_environment()
        
        # Import modules
        AlfieVRConfig, VRWebSocketServer, ControlGoal, ControlMode = import_alfievr_modules()
        if AlfieVRConfig is None:
            print("❌ Failed to import alfievr modules")
            return False
        
        # Create configuration
        self.config = AlfieVRConfig()
        self.config.enable_vr = True
        self.config.enable_keyboard = False
        self.config.enable_https = True  # Enable HTTPS server, VR requires web interface
        
        # Create command queue
        self.command_queue = asyncio.Queue()
        
        # Create VR server (print-only mode)
        try:
            self.vr_server = VRWebSocketServer(
                command_queue=self.command_queue,
                config=self.config,
                print_only=False,  # Changed to False to send data to queue
                debug_logs=self.debug_logs  # Pass debug logging flag
            )
        except Exception as e:
            print(f"❌ Failed to create VR WebSocket server: {e}")
            return False
        
        # Create HTTPS server
        try:
            self.https_server = SimpleHTTPSServer(self.config)
        except Exception as e:
            print(f"❌ Failed to create HTTPS server: {e}")
            return False
        
        print("✅ AlfieVR Monitor initialized successfully")
        
        return True
    
    async def start_monitoring(self):
        """Start monitoring VR control information"""
        print("🚀 Starting VR Monitor...")
        
        if not self.initialize():
            print("❌ Failed to initialize VR monitor")
            return
        
        try:
            # Start HTTPS server
            await self.https_server.start()
            
            # Start VR server
            await self.vr_server.start()
            
            self.is_running = True
            print("✅ VR Monitor is now running")
            
            # Display connection information
            host_display = get_local_ip() if self.config.host_ip == "0.0.0.0" else self.config.host_ip
            print(f"📱 Open your VR headset browser and navigate to:")
            print(f"   https://{host_display}:{self.config.https_port}")
            print("🎯 Press Ctrl+C to stop monitoring")
            print()
            
            # Monitor command queue
            await self.monitor_commands()
            
        except KeyboardInterrupt:
            print("\n⏹️  Stopping VR monitor...")
        except Exception as e:
            print(f"❌ Error in VR monitor: {e}")
            import traceback
            print(f"Traceback: {traceback.format_exc()}")
        finally:
            await self.stop_monitoring()
    
    async def monitor_commands(self):
        """Monitor commands from VR controllers"""
        print("📊 Monitoring VR control commands...")
        
        # Rate limiting: 100Hz max (0.01 second between updates)
        min_interval = 0.01
        last_update_time = 0.0
        
        while self.is_running:
            try:
                # Wait for command with 1-second timeout
                goal = await asyncio.wait_for(self.command_queue.get(), timeout=1.0)
                
                # Always save goal regardless of rate limiting
                # (rate limiting should only affect logging/printing, not data storage)
                with self._goal_lock:
                    if goal.arm == "left":
                        self.left_goal = goal
                    elif goal.arm == "right":
                        self.right_goal = goal
                    elif goal.arm == "headset":  # Add headset data processing
                        self.headset_goal = goal
                    
                    # Maintain backward compatibility, save latest goal
                    self.latest_goal = goal
                
                # Rate limit logging/printing to 100Hz
                current_time = asyncio.get_event_loop().time()
                time_since_last = current_time - last_update_time
                
                if time_since_last < min_interval:
                    # Skip logging if too soon, but data is already saved above
                    continue
                
                last_update_time = current_time
                
            except asyncio.TimeoutError:
                # Timeout, continue loop
                continue
            except Exception as e:
                print(f"❌ Error processing command: {e}")
                import traceback
                print(f"Traceback: {traceback.format_exc()}")
    
    def print_control_goal(self, goal):
        """Print control goal information"""
        print(f"\n🎮 Control Goal Received:")
        print(f"   Timestamp: {asyncio.get_event_loop().time():.3f}")
        print(f"   Arm: {goal.arm}")
        print(f"   Mode: {goal.mode}")
        
        if goal.target_position is not None:
            if isinstance(goal.target_position, (list, tuple)):
                pos = goal.target_position
                print(f"   Target Position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
            else:
                print(f"   Target Position: {goal.target_position}")
        
        if goal.wrist_roll_deg is not None:
            print(f"   Wrist Roll: {goal.wrist_roll_deg:.1f}°")
        
        if goal.wrist_flex_deg is not None:
            print(f"   Wrist Flex: {goal.wrist_flex_deg:.1f}°")
        
        if goal.gripper_closed is not None:
            print(f"   Gripper: {'CLOSED' if goal.gripper_closed else 'OPEN'}")
        
        if goal.metadata:
            print(f"   Metadata: {goal.metadata}")
        
        print("-" * 30)
    
    def get_latest_goal_nowait(self, arm=None):
        """Return the latest VR control goal if available, else None.
        
        Args:
            arm: If specified ("left" or "right"), return that arm's goal.
                 If None, return a dict containing both left and right goals.
        """
        with self._goal_lock:
            if arm == "left":
                return self.left_goal
            elif arm == "right":
                return self.right_goal
            elif arm == "headset":  # Add headset data retrieval
                return self.headset_goal
            else:
                # Return dictionary containing both left and right controllers
                dual_goals = {
                    "left": self.left_goal,
                    "right": self.right_goal,
                    "headset": self.headset_goal,  # Add headset data
                    "has_left": self.left_goal is not None,
                    "has_right": self.right_goal is not None,
                    "has_headset": self.headset_goal is not None  # Add headset status
                }
                
                return dual_goals
    
    def get_left_goal_nowait(self):
        """Return the latest left arm goal if available, else None."""
        return self.get_latest_goal_nowait("left")
    
    def get_right_goal_nowait(self):
        """Return the latest right arm goal if available, else None."""
        return self.get_latest_goal_nowait("right")
    
    async def stop_monitoring(self):
        """Stop monitoring"""
        self.is_running = False
        
        if self.vr_server:
            await self.vr_server.stop()
        
        if self.https_server:
            await self.https_server.stop()
        
        print("✅ VR Monitor stopped")

def main():
    """Main function"""
    print("🎮 AlfieVR Monitor - AlfieVR VR Control Information Monitor")
    print("=" * 60)
    
    # Check AlfieVR path
    if not os.path.exists(ALFIEVR_PATH):
        print(f"❌ AlfieVR path does not exist: {ALFIEVR_PATH}")
        print("Please update ALFIEVR_PATH in the script")
        return
    
    # Create monitor
    monitor = VRMonitor()
    
    # Run monitoring
    try:
        asyncio.run(monitor.start_monitoring())
    except KeyboardInterrupt:
        print("\n👋 AlfieVR Monitor stopped by user")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")

if __name__ == "__main__":
    main() 