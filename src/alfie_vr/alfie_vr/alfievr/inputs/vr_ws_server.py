"""
VR WebSocket server for receiving controller data from web browsers.
Adapted from the original vr_robot_teleop.py script.
"""

import asyncio
import json
import ssl
import websockets
import numpy as np
import math
import logging
from typing import Dict, Optional, Set
from scipy.spatial.transform import Rotation as R

from .base import BaseInputProvider, ControlGoal, ControlMode
from ..config import AlfieVRConfig

logger = logging.getLogger(__name__)


class VRControllerState:
    """State tracking for a VR controller."""
    
    def __init__(self, hand: str):
        self.hand = hand
        self.grip_active = False
        self.trigger_active = False
        
        # Position tracking for relative movement
        self.origin_position = None
        self.origin_rotation = None
        
        # Quaternion-based rotation tracking (more stable than Euler)
        self.origin_quaternion = None
        self.accumulated_rotation_quat = None  # Accumulated rotation as quaternion
        
        # Rotation tracking for wrist control
        self.z_axis_rotation = 0.0  # For wrist_roll
        self.x_axis_rotation = 0.0  # For wrist_flex (pitch)
        
        # Position tracking
        self.current_position = None
        
        # Rotation tracking
        self.origin_wrist_angle = 0.0
    
    def reset_grip(self):
        """Reset grip state but preserve trigger state."""
        self.grip_active = False
        self.origin_position = None
        self.origin_rotation = None
        self.origin_quaternion = None
        self.accumulated_rotation_quat = None
        self.z_axis_rotation = 0.0
        self.x_axis_rotation = 0.0
    
    def reset_origin(self):
        """Reset origin position and rotation for auto-control mode."""
        self.origin_position = None
        self.origin_rotation = None
        self.origin_quaternion = None
        self.accumulated_rotation_quat = None
        self.z_axis_rotation = 0.0
        self.x_axis_rotation = 0.0


class VRWebSocketServer(BaseInputProvider):
    """WebSocket server for VR controller input."""
    
    def __init__(self, command_queue: asyncio.Queue, config: AlfieVRConfig, print_only: bool = False, debug_logs: bool = False):
        super().__init__(command_queue)
        self.config = config
        self.clients: Set = set()
        self.server = None
        self.print_only = print_only  # New flag for print-only mode
        self.debug_logs = debug_logs  # Debug logging flag
        
        # Controller states
        self.left_controller = VRControllerState("left")
        self.right_controller = VRControllerState("right")
        
        # Robot state tracking (for relative position calculation)
        self.left_arm_origin_position = None
        self.right_arm_origin_position = None
        
        # Event loop reference for thread-safe broadcasts
        self._event_loop = None
    
    def debug_print(self, msg: str):
        """Print debug message only if debug_logs is enabled."""
        if self.debug_logs:
            print(msg)
    
    def setup_ssl(self) -> Optional[ssl.SSLContext]:
        """Setup SSL context for WebSocket server."""
        # Automatically generate SSL certificates if they don't exist
        if not self.config.ssl_files_exist:
            logger.info("SSL certificates not found for WebSocket server, attempting to generate them...")
            if not self.config.ensure_ssl_certificates():
                logger.error("Failed to generate SSL certificates for WebSocket server")
                return None
        
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_context.minimum_version = ssl.TLSVersion.TLSv1_2
        ssl_context.maximum_version = ssl.TLSVersion.TLSv1_3
        # Broad cipher support for Android Chrome and Meta Quest browser
        ssl_context.set_ciphers(
            "ECDHE+AESGCM:ECDHE+CHACHA20:DHE+AESGCM:DHE+CHACHA20:"
            "ECDH+AESGCM:DH+AESGCM:ECDH+AES:DH+AES:RSA+AESGCM:RSA+AES:!aNULL:!eNULL:!MD5"
        )
        try:
            ssl_context.load_cert_chain(certfile=self.config.certfile, keyfile=self.config.keyfile)
            logger.info("SSL certificate and key loaded successfully for WebSocket server (TLS 1.2-1.3, mobile-compatible)")
            return ssl_context
        except ssl.SSLError as e:
            logger.error(f"Error loading SSL cert/key: {e}")
            return None
    
    async def start(self):
        """Start the WebSocket server."""
        if not self.config.enable_vr:
            logger.info("VR WebSocket server disabled in configuration")
            return
        
        ssl_context = self.setup_ssl()
        if ssl_context is None:
            logger.error("Failed to setup SSL for WebSocket server")
            return
        
        host = self.config.host_ip
        port = self.config.websocket_port
        
        # Store reference to the running event loop for thread-safe broadcasts
        self._event_loop = asyncio.get_running_loop()
        
        try:
            self.server = await websockets.serve(
                self.websocket_handler, 
                host, 
                port, 
                ssl=ssl_context
            )
            self.is_running = True
            logger.info(f"VR WebSocket server running on wss://{host}:{port}")
        except Exception as e:
            logger.error(f"Failed to start WebSocket server: {e}")
    
    async def stop(self):
        """Stop the WebSocket server."""
        self.is_running = False
        if self.server:
            self.server.close()
            await self.server.wait_closed()
            logger.info("VR WebSocket server stopped")
    
    async def broadcast_message(self, message: dict):
        """Broadcast a message to all connected VR clients."""
        if not self.clients:
            logger.debug("No clients connected for broadcast")
            return
        
        message_json = json.dumps(message)
        disconnected = set()
        
        logger.info(f"Broadcasting to {len(self.clients)} clients: {message}")
        
        for client in self.clients:
            try:
                await client.send(message_json)
                logger.debug(f"Sent message to client: {client.remote_address}")
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(client)
            except Exception as e:
                logger.error(f"Error broadcasting to client: {e}")
                disconnected.add(client)
        
        # Clean up disconnected clients
        self.clients -= disconnected
    
    def broadcast_message_sync(self, message: dict):
        """Synchronously broadcast a message (thread-safe, schedules on the server's event loop)."""
        if not hasattr(self, '_event_loop') or self._event_loop is None:
            logger.warning("Event loop not set, cannot broadcast message")
            return
        
        try:
            # Schedule the coroutine on the VR server's event loop (thread-safe)
            asyncio.run_coroutine_threadsafe(self.broadcast_message(message), self._event_loop)
            logger.debug(f"Scheduled broadcast: {message}")
        except Exception as e:
            logger.error(f"Failed to schedule broadcast: {e}")
    
    async def websocket_handler(self, websocket, path=None):
        """Handle WebSocket connections from VR controllers."""
        client_address = websocket.remote_address
        logger.info(f"VR client connected: {client_address}")
        self.clients.add(websocket)
        
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self.process_controller_data(data)
                except json.JSONDecodeError:
                    logger.warning(f"Received non-JSON message: {message}")
                except Exception as e:
                    logger.error(f"Error processing VR data: {e}")
                    # Add more context for debugging
                    logger.error(f"Data that caused error: {data}")
                    import traceback
                    logger.error(f"Traceback: {traceback.format_exc()}")
        
        except websockets.exceptions.ConnectionClosedOK:
            logger.info(f"VR client {client_address} disconnected normally")
        except websockets.exceptions.ConnectionClosedError as e:
            logger.warning(f"VR client {client_address} disconnected with error: {e}")
        except Exception as e:
            logger.error(f"Unexpected error with VR client {client_address}: {e}")
        finally:
            self.clients.discard(websocket)
            # Handle grip releases when client disconnects
            await self.handle_grip_release('left')
            await self.handle_grip_release('right')
            logger.info(f"VR client {client_address} cleanup complete")
    
    async def process_controller_data(self, data: Dict):
        """Process incoming VR controller data."""
        # Check for thumbstick or button activity, only print when there is activity
        has_thumbstick_or_button_activity = False
        thumbstick_info = []
        button_info = []
        
        # Check thumbstick and button status for left and right controllers
        for hand in ['leftController', 'rightController']:
            if hand in data:
                controller_data = data[hand]
                hand_name = hand.replace('Controller', '').upper()
                
                # Check thumbstick
                if 'thumbstick' in controller_data:
                    thumbstick = controller_data['thumbstick']
                    x = thumbstick.get('x', 0)
                    y = thumbstick.get('y', 0)
                    # Only print when thumbstick has actual input (threshold 0.1)
                    if abs(x) > 0.1 or abs(y) > 0.1:
                        has_thumbstick_or_button_activity = True
                        thumbstick_info.append(f"[{hand_name}] Thumbstick: x={x:.2f}, y={y:.2f}")
                
                # Check buttons
                if 'buttons' in controller_data:
                    buttons = controller_data['buttons']
                    pressed_buttons = []
                    for button_name, is_pressed in buttons.items():
                        if is_pressed:
                            has_thumbstick_or_button_activity = True
                            pressed_buttons.append(button_name)
                    
                    if pressed_buttons:
                        button_info.append(f"[{hand_name}] Buttons: {', '.join(pressed_buttons)}")
        
        # Only print when there is activity
        if has_thumbstick_or_button_activity:
            self.debug_print(f"[VR_WS] Activity detected:")
            for info in thumbstick_info:
                self.debug_print(f"  {info}")
            for info in button_info:
                self.debug_print(f"  {info}")
        
        # Process headset data if available
        if 'headset' in data:
            headset_data = data['headset']
            if headset_data and headset_data.get('position'):
                pos = headset_data['position']
                rot = headset_data.get('rotation', {})
                quat = headset_data.get('quaternion', {})
                
                # Filter out invalid (0, 0, 0) positions - these are jitter from VR tracking gaps
                px, py, pz = pos.get('x', 0), pos.get('y', 0), pos.get('z', 0)
                if abs(px) < 0.001 and abs(py) < 0.001 and abs(pz) < 0.001:
                    # Skip this frame - invalid headset data
                    pass
                else:
                    self.debug_print(f"[VR_WS] Headset - Position: [{px:.3f}, {py:.3f}, {pz:.3f}], "
                          f"Rotation: [{rot.get('x', 0):.1f}, {rot.get('y', 0):.1f}, {rot.get('z', 0):.1f}]")
                    
                    # Create headset ControlGoal
                    headset_position = np.array([px, py, pz])
                    headset_goal = ControlGoal(
                        arm="headset",
                        mode=ControlMode.POSITION_CONTROL,
                        target_position=headset_position,
                        wrist_roll_deg=rot.get('y', 0),  # Yaw rotation
                        wrist_flex_deg=rot.get('x', 0),   # Pitch rotation
                        metadata={
                            "source": "vr_headset",
                            "relative_position": False,
                            "vr_position": headset_position.tolist(),
                            "rotation": rot,
                            "quaternion": quat
                        }
                    )
                    await self.send_goal(headset_goal)
        
        # Process controller data
        if 'leftController' in data:
            await self.process_single_controller('left', data['leftController'])
        
        if 'rightController' in data:
            await self.process_single_controller('right', data['rightController'])
    
    async def process_single_controller(self, hand: str, data: Dict):
        """Process data for a single controller."""
        position = data.get('position', {})
        rotation = data.get('rotation', {})
        quaternion = data.get('quaternion', {})  # Get quaternion data directly
        grip_active = data.get('gripActive', False)
        trigger = data.get('trigger', 0)
        thumbstick = data.get('thumbstick', {})
        buttons = data.get('buttons', {})  # Get buttons dictionary
        
        controller = self.left_controller if hand == 'left' else self.right_controller
        
        # Handle trigger for gripper control
        trigger_active = trigger > 0.5
        
        # Print trigger value if it's being squeezed (for debugging)
        if trigger > 0.01:
            self.debug_print(f"[VR_WS] {hand.upper()} trigger: {trigger:.3f} (active: {trigger_active})")
        
        # Always send gripper control with analog trigger value
        # Reverse behavior: gripper open by default, closes when trigger pressed
        gripper_goal = ControlGoal(
            arm=hand,
            gripper_closed=not trigger_active,  # Inverted: closed when trigger NOT active
            metadata={
                "source": "vr_trigger",
                "trigger": trigger,  # Raw analog value 0.0 to 1.0
                "trigger_active": trigger_active,
                "thumbstick": thumbstick,
                "grip_active": grip_active,
                "buttons": buttons
            }
        )
        await self.send_goal(gripper_goal)
        
        # Log trigger state changes
        if trigger_active != controller.trigger_active:
            controller.trigger_active = trigger_active
            self.debug_print(f"[VR_WS] 🤏 {hand.upper()} trigger: {trigger:.2f} - gripper {'OPENED' if trigger_active else 'CLOSED'}")
            logger.info(f"🤏 {hand.upper()} trigger: {trigger:.2f} - gripper {'OPENED' if trigger_active else 'CLOSED'}")
        
        # Modified: directly respond to controller position, no need to press squeeze button
        # Check if there is position data
        if position and all(k in position for k in ['x', 'y', 'z']):
            px, py, pz = position.get('x', 0), position.get('y', 0), position.get('z', 0)
            
            # Filter out invalid (0, 0, 0) positions - these are jitter from VR tracking gaps
            if abs(px) < 0.001 and abs(py) < 0.001 and abs(pz) < 0.001:
                # Skip this frame - invalid controller data
                return
            
            # If origin is not yet set, set current position as origin
            if controller.origin_position is None:
                controller.origin_position = np.array([px, py, pz])
                
                # Set quaternion origin
                if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                    controller.origin_quaternion = np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
                else:
                    controller.origin_quaternion = self.euler_to_quaternion(rotation) if rotation else None
                
                controller.accumulated_rotation_quat = controller.origin_quaternion
                controller.z_axis_rotation = 0.0
                controller.x_axis_rotation = 0.0
                
                # Send reset signal
                reset_goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,
                    target_position=None,
                    metadata={
                        "source": f"vr_auto_reset_{hand}",
                        "reset_target_to_current": True,
                        "trigger": trigger,
                        "trigger_active": trigger_active,
                        "thumbstick": thumbstick,
                        "grip_active": grip_active,
                        "buttons": buttons
                    }
                )
                await self.send_goal(reset_goal)
                logger.info(f"🎯 {hand.upper()} auto-activated - controlling {hand} arm")
            
            # Calculate target position - switch to absolute position control
            position_array = np.array([position.get('x', 0), position.get('y', 0), position.get('z', 0)])
            
            # Use absolute position of VR controller directly, apply scaling
            absolute_position = position_array * self.config.vr_to_robot_scale
            
            # Calculate wrist rotation
            if controller.origin_quaternion is not None:
                if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                    current_quat = np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
                    self.update_quaternion_rotation_direct(controller, current_quat)
                else:
                    self.update_quaternion_rotation(controller, rotation)
                
                controller.z_axis_rotation = self.extract_roll_from_quaternion(controller.accumulated_rotation_quat, controller.origin_quaternion, hand)
                controller.x_axis_rotation = self.extract_pitch_from_quaternion(controller.accumulated_rotation_quat, controller.origin_quaternion, hand)
            
            # Create absolute position control goal
            # Note: Negation is applied here to match robot coordinate conventions
            # Handedness mirroring is handled in the extraction functions
            goal = ControlGoal(
                arm=hand,
                mode=ControlMode.POSITION_CONTROL,
                target_position=absolute_position,  # Absolute position
                wrist_roll_deg=-controller.z_axis_rotation,
                wrist_flex_deg=-controller.x_axis_rotation,
                metadata={
                    "source": "vr_absolute_position",
                    "relative_position": False,  # Mark as absolute position
                    "vr_position": position_array.tolist(),
                    "scaled_position": absolute_position.tolist(),
                    "trigger": trigger,
                    "trigger_active": trigger_active,
                    "thumbstick": thumbstick,
                    "grip_active": grip_active,  # Grip button state for delta control
                    "buttons": buttons
                }
            )
            await self.send_goal(goal)
        
        # Preserve original squeeze button logic as backup (optional)
        # If you want to completely remove squeeze button control, you can comment out the code below
        """
        # Handle grip button for arm movement control (original logic)
        if grip_active:
            if not controller.grip_active:
                print_pose()
                # Grip just activated - set origin and reset target position
                controller.grip_active = True
                # Convert position dict to numpy array for proper subtraction later
                controller.origin_position = np.array([position.get('x', 0), position.get('y', 0), position.get('z', 0)])
                
                # Use quaternion data directly if available, otherwise fall back to Euler conversion
                if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                    controller.origin_quaternion = np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
                    controller.origin_rotation = controller.origin_quaternion  # Store for compatibility
                else:
                    # Fallback to Euler angle conversion
                    controller.origin_quaternion = self.euler_to_quaternion(rotation) if rotation else None
                    controller.origin_rotation = controller.origin_quaternion
                
                controller.accumulated_rotation_quat = controller.origin_quaternion
                controller.z_axis_rotation = 0.0
                controller.x_axis_rotation = 0.0
                
                # Send reset signal to control loop to reset target position to current robot position
                reset_goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,  # Keep in position control
                    target_position=None,  # Special signal
                    metadata={
                        "source": f"vr_grip_reset_{hand}",
                        "reset_target_to_current": True,  # Signal to reset target to current position
                        "trigger": trigger,
                        "trigger_active": trigger_active,
                        "thumbstick": thumbstick
                    }
                )
                await self.send_goal(reset_goal)
                
                logger.info(f"🔒 {hand.upper()} grip activated - controlling {hand} arm (target reset to current position)")
            
            # Compute target position
            if controller.origin_position is not None:
                # Convert position dict to numpy array for proper subtraction
                position_array = np.array([position.get('x', 0), position.get('y', 0), position.get('z', 0)])
                
                # Ensure origin_position is a numpy array
                if isinstance(controller.origin_position, dict):
                    # If origin_position is still a dict, convert it to numpy array
                    logger.warning(f"origin_position was dict, converting to numpy array for {hand} controller")
                    controller.origin_position = np.array([controller.origin_position.get('x', 0), controller.origin_position.get('y', 0), controller.origin_position.get('z', 0)])
                elif not isinstance(controller.origin_position, np.ndarray):
                    # If origin_position is neither dict nor numpy array, log warning and skip
                    logger.warning(f"origin_position is {type(controller.origin_position)}, skipping position calculation for {hand} controller")
                    return
                
                relative_delta = (position_array - controller.origin_position) * self.config.vr_to_robot_scale
                
                # Calculate Z-axis rotation for wrist_roll control
                # Calculate X-axis rotation for wrist_flex control
                if controller.origin_quaternion is not None:
                    # Update quaternion-based rotation tracking
                    if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                        # Use quaternion data directly
                        current_quat = np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
                        self.update_quaternion_rotation_direct(controller, current_quat)
                    else:
                        # Fallback to Euler angle conversion
                        self.update_quaternion_rotation(controller, rotation)
                    
                    # Get accumulated rotations from quaternion
                    controller.z_axis_rotation = self.extract_roll_from_quaternion(controller.accumulated_rotation_quat, controller.origin_quaternion, hand)
                    controller.x_axis_rotation = self.extract_pitch_from_quaternion(controller.accumulated_rotation_quat, controller.origin_quaternion, hand)
                
                # Create position control goal
                # Note: We send relative position here, the control loop will handle
                # adding it to the robot's current position
                goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,
                    target_position=relative_delta,  # Relative position delta
                    wrist_roll_deg=-controller.z_axis_rotation,
                    wrist_flex_deg=-controller.x_axis_rotation,
                    metadata={
                        "source": "vr_grip",
                        "relative_position": True,
                        "origin_position": controller.origin_position.tolist(),
                        "trigger": trigger,
                        "trigger_active": trigger_active,
                        "thumbstick": thumbstick
                    }
                )
                await self.send_goal(goal)
        """
    
    async def handle_grip_release(self, hand: str):
        """Handle grip release for a controller."""
        if hand == 'left':
            controller = self.left_controller
        elif hand == 'right':
            controller = self.right_controller
        else:
            return
        
        if controller.grip_active:
            controller.reset_grip()
            
            # Send idle goal to stop arm control
            goal = ControlGoal(
                arm=hand,
                mode=ControlMode.IDLE,
                metadata={
                    "source": "vr_grip_release",
                    "trigger": 0.0,
                    "trigger_active": False,
                    "thumbstick": {}
                }
            )
            await self.send_goal(goal)
            
            logger.info(f"🔓 {hand.upper()} grip released - arm control stopped")
    
    async def handle_trigger_release(self, hand: str):
        """Handle trigger release for a controller."""
        controller = self.left_controller if hand == 'left' else self.right_controller
        
        if controller.trigger_active:
            controller.trigger_active = False
            
            # Send gripper closed goal - reversed behavior: gripper closes when trigger released
            goal = ControlGoal(
                arm=hand,
                gripper_closed=True,  # Close gripper when trigger released
                metadata={
                    "source": "vr_trigger_release",
                    "trigger": 0.0,
                    "trigger_active": False,
                    "thumbstick": {}
                }
            )
            await self.send_goal(goal)
            
            logger.info(f"🤏 {hand.upper()} gripper CLOSED (trigger released)")
    
    def euler_to_quaternion(self, euler_deg: Dict[str, float]) -> np.ndarray:
        """Convert Euler angles in degrees to quaternion [x, y, z, w]."""
        euler_rad = [math.radians(euler_deg['x']), math.radians(euler_deg['y']), math.radians(euler_deg['z'])]
        rotation = R.from_euler('xyz', euler_rad)
        return rotation.as_quat()
    
    def update_quaternion_rotation(self, controller: VRControllerState, current_euler: dict):
        """Update quaternion-based rotation tracking."""
        if not current_euler:
            return
        
        # Convert current Euler to quaternion
        current_quat = self.euler_to_quaternion(current_euler)
        
        # Store current quaternion for accumulated rotation calculation
        controller.accumulated_rotation_quat = current_quat
    
    def update_quaternion_rotation_direct(self, controller: VRControllerState, current_quat: np.ndarray):
        """Update quaternion-based rotation tracking using quaternion data directly."""
        if current_quat is None:
            return
        
        # Store current quaternion for accumulated rotation calculation
        controller.accumulated_rotation_quat = current_quat
    
    def extract_roll_from_quaternion(self, current_quat: np.ndarray, origin_quat: np.ndarray, hand: str = 'right') -> float:
        """Extract roll (wrist twist) from relative quaternion rotation.

        Args:
            current_quat: Current quaternion orientation
            origin_quat: Origin quaternion orientation
            hand: 'left' or 'right' controller handedness
        """
        if current_quat is None or origin_quat is None:
            return 0.0

        try:
            # Calculate relative rotation quaternion (from origin to current)
            origin_rotation = R.from_quat(origin_quat)
            current_rotation = R.from_quat(current_quat)
            relative_rotation = current_rotation * origin_rotation.inv()

            # Use Euler angles for proper decomposition
            # 'zyx' order: first rotate around Z (roll), then Y (pitch), then X
            # This gives us independent axis rotations
            euler = relative_rotation.as_euler('zyx', degrees=True)
            # euler[0] = Z rotation (roll/twist around controller's pointing axis)
            roll_deg = -euler[0]  # Negate for correct direction

            return roll_deg
        except Exception as e:
            logger.warning(f"Error extracting roll from quaternion: {e}")
            return 0.0

    def extract_pitch_from_quaternion(self, current_quat: np.ndarray, origin_quat: np.ndarray, hand: str = 'right') -> float:
        """Extract pitch (wrist flex up/down) from relative quaternion rotation.

        Args:
            current_quat: Current quaternion orientation
            origin_quat: Origin quaternion orientation
            hand: 'left' or 'right' controller handedness
        """
        if current_quat is None or origin_quat is None:
            return 0.0

        try:
            # Calculate relative rotation quaternion (from origin to current)
            origin_rotation = R.from_quat(origin_quat)
            current_rotation = R.from_quat(current_quat)
            relative_rotation = current_rotation * origin_rotation.inv()

            # Use Euler angles for proper decomposition
            # 'zyx' order: first rotate around Z (roll), then Y (pitch), then X
            euler = relative_rotation.as_euler('zyx', degrees=True)
            # euler[2] = X rotation (pitch/tilt up-down)
            pitch_deg = euler[2]

            return pitch_deg
        except Exception as e:
            logger.warning(f"Error extracting pitch from quaternion: {e}")
            return 0.0
    
    async def send_goal(self, goal: ControlGoal):
        """Send a control goal to the command queue or print it if in print-only mode."""
        if self.print_only:
            # Print the ControlGoal in a formatted way
            print(f"\n🎮 ControlGoal:")
            print(f"   Arm: {goal.arm}")
            print(f"   Mode: {goal.mode}")
            if goal.target_position is not None:
                print(f"   Target Position: [{goal.target_position[0]:.3f}, {goal.target_position[1]:.3f}, {goal.target_position[2]:.3f}]")
            if goal.wrist_roll_deg is not None:
                print(f"   Wrist Roll: {goal.wrist_roll_deg:.1f}°")
            if goal.wrist_flex_deg is not None:
                print(f"   Wrist Flex: {goal.wrist_flex_deg:.1f}°")
            if goal.gripper_closed is not None:
                print(f"   Gripper: {'CLOSED' if goal.gripper_closed else 'OPEN'}")
            if goal.metadata:
                print(f"   Metadata: {goal.metadata}")
            print()
        else:
            # Use the parent class method to send to queue
            await super().send_goal(goal) 