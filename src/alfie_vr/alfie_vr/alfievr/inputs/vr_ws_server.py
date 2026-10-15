"""
VR WebSocket server for receiving controller data from web browsers.
Adapted from the original vr_robot_teleop.py script.

Client -> server messages (JSON):
  Pose frame:  {seq, timestamp, headset?, leftController?, rightController?}
               A device key is omitted when that device is not tracked this
               frame; the consumer treats a device as lost once its goals go stale.
  Release:     {"type": "release_all", "reason": str}
               {"type": "release_hand", "hand": "left"|"right", "reason": str}
"""

import asyncio
import json
import ssl
import websockets
import numpy as np
import logging
from typing import Dict, Optional, Set
from scipy.spatial.transform import Rotation as R

from .base import BaseInputProvider, ControlGoal
from ..config import AlfieVRConfig

logger = logging.getLogger(__name__)

QUAT_KEYS = ('x', 'y', 'z', 'w')


def _quat(d) -> Optional[np.ndarray]:
    """Return [x, y, z, w] from a dict, or None if missing/degenerate."""
    if not d or not all(k in d for k in QUAT_KEYS):
        return None
    q = np.array([float(d[k]) for k in QUAT_KEYS])
    if not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-6:
        return None
    return q


def _position(d) -> Optional[np.ndarray]:
    """Return [x, y, z] from a dict, or None if missing, non-finite or the
    (0, 0, 0) placeholder WebXR reports during tracking gaps."""
    if not d or not all(k in d for k in ('x', 'y', 'z')):
        return None
    p = np.array([float(d['x']), float(d['y']), float(d['z'])])
    if not np.all(np.isfinite(p)) or np.all(np.abs(p) < 0.001):
        return None
    return p


class VRControllerState:
    """Rotation origin for one controller (wrist angles are relative to it)."""

    def __init__(self, hand: str):
        self.hand = hand
        self.origin_quaternion = None

    def reset(self):
        self.origin_quaternion = None


class VRWebSocketServer(BaseInputProvider):
    """WebSocket server for VR controller input."""

    def __init__(self, command_queue: asyncio.Queue, config: AlfieVRConfig, debug_logs: bool = False):
        super().__init__(command_queue)
        self.config = config
        self.clients: Set = set()
        self.server = None
        self.debug_logs = debug_logs

        self.controllers = {
            'left': VRControllerState('left'),
            'right': VRControllerState('right'),
        }

        # Event loop reference for thread-safe broadcasts
        self._event_loop = None

    def debug_print(self, msg: str):
        """Print debug message only if debug_logs is enabled."""
        if self.debug_logs:
            print(msg)

    def setup_ssl(self) -> Optional[ssl.SSLContext]:
        """Setup SSL context for WebSocket server."""
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
            raise RuntimeError("Failed to setup SSL for WebSocket server")

        self._event_loop = asyncio.get_running_loop()
        self.server = await websockets.serve(
            self.websocket_handler,
            self.config.host_ip,
            self.config.websocket_port,
            ssl=ssl_context,
        )
        self.is_running = True
        logger.info(f"VR WebSocket server running on wss://{self.config.host_ip}:{self.config.websocket_port}")

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
            return

        message_json = json.dumps(message)
        disconnected = set()
        for client in list(self.clients):
            try:
                await client.send(message_json)
            except Exception:
                disconnected.add(client)
        self.clients -= disconnected

    def broadcast_message_sync(self, message: dict):
        """Thread-safe broadcast: schedules on the server's event loop."""
        if self._event_loop is None:
            logger.warning("Event loop not set, cannot broadcast message")
            return
        asyncio.run_coroutine_threadsafe(self.broadcast_message(message), self._event_loop)

    async def websocket_handler(self, websocket, path=None):
        """Handle WebSocket connections from VR controllers."""
        client_address = websocket.remote_address
        logger.info(f"VR client connected: {client_address}")
        self.clients.add(websocket)

        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                except json.JSONDecodeError:
                    logger.warning("Received non-JSON message")
                    continue
                if not isinstance(data, dict):
                    continue
                try:
                    await self.process_message(data)
                except Exception as e:
                    logger.error(f"Error processing VR data: {e}", exc_info=True)
        except websockets.exceptions.ConnectionClosed as e:
            logger.info(f"VR client {client_address} disconnected: {e}")
        finally:
            self.clients.discard(websocket)
            # Only release if no other client is still driving.
            if not self.clients:
                await self.send_release(['left', 'right', 'headset'], 'websocket_closed')
            logger.info(f"VR client {client_address} cleanup complete")

    async def send_release(self, hands, reason: str):
        """Tell the consumer to drop control of `hands` immediately."""
        for hand in hands:
            if hand in self.controllers:
                self.controllers[hand].reset()
        await self.send_goal(ControlGoal(arm='release', metadata={'hands': list(hands), 'reason': reason}))

    async def process_message(self, data: Dict):
        """Dispatch one client message."""
        msg_type = data.get('type')
        if msg_type == 'release_all':
            await self.send_release(['left', 'right', 'headset'], data.get('reason', 'client'))
            return
        if msg_type == 'release_hand':
            hand = data.get('hand')
            if hand in self.controllers:
                await self.send_release([hand], data.get('reason', 'client'))
            return

        if 'headset' in data:
            await self.process_headset(data['headset'])
        if 'leftController' in data:
            await self.process_single_controller('left', data['leftController'])
        if 'rightController' in data:
            await self.process_single_controller('right', data['rightController'])

    async def process_headset(self, headset_data: Dict):
        """Forward the headset pose (orientation drives the robot head)."""
        if not headset_data:
            return
        position = _position(headset_data.get('position'))
        if position is None:
            return  # tracking gap
        await self.send_goal(ControlGoal(
            arm="headset",
            target_position=position,
            metadata={
                "source": "vr_headset",
                "rotation": headset_data.get('rotation') or {},
                "quaternion": headset_data.get('quaternion') or {},
            },
        ))

    async def process_single_controller(self, hand: str, data: Dict):
        """Forward one controller's pose, wrist angles and buttons as a single goal."""
        if not data:
            return
        controller = self.controllers[hand]
        position = _position(data.get('position'))
        quat = _quat(data.get('quaternion'))
        if quat is None and data.get('rotation'):
            r = data['rotation']
            quat = R.from_euler('xyz', [r.get('x', 0), r.get('y', 0), r.get('z', 0)], degrees=True).as_quat()

        wrist_roll_deg = wrist_flex_deg = None
        if quat is not None:
            if controller.origin_quaternion is None:
                controller.origin_quaternion = quat
            roll, flex = self.wrist_angles(quat, controller.origin_quaternion)
            # Negation matches robot wrist conventions.
            wrist_roll_deg, wrist_flex_deg = -roll, -flex

        trigger = float(data.get('trigger', 0) or 0)
        await self.send_goal(ControlGoal(
            arm=hand,
            target_position=position,  # None during a tracking gap: buttons still apply
            wrist_roll_deg=wrist_roll_deg,
            wrist_flex_deg=wrist_flex_deg,
            metadata={
                "source": "vr_controller",
                "trigger": trigger,
                "thumbstick": data.get('thumbstick') or {},
                "grip_active": bool(data.get('gripActive', False)),
                "buttons": data.get('buttons') or {},
            },
        ))

    @staticmethod
    def wrist_angles(current_quat: np.ndarray, origin_quat: np.ndarray):
        """Roll and pitch (degrees) of the controller relative to its origin pose.

        The relative rotation is taken in the controller's own frame
        (origin^-1 * current) so roll stays about the controller's pointing axis
        (Z) and pitch about its X axis regardless of which way the operator faces.
        """
        relative = R.from_quat(origin_quat).inv() * R.from_quat(current_quat)
        z, _y, x = relative.as_euler('zyx', degrees=True)
        return -z, x
