"""
Servo Calibration TUI Application using curses.

Layout:
                    HEAD
              [yaw] [pitch] [roll]
              
    LEFT ARM                RIGHT ARM
    [shoulder yaw]          [shoulder yaw]
    [shoulder1 pitch]       [shoulder1 pitch]
    [shoulder2 pitch]       [shoulder2 pitch]
    [elbow pitch]           [elbow pitch]
    [wrist pitch]           [wrist pitch]
    [wrist roll]            [wrist roll]
    [hand]                  [hand]

Navigation: Tab/Arrow keys
Input: Type value when field selected, Enter to confirm
q: Quit
"""

import curses
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from alfie_msgs.msg import RobotLowCmd, ServoCmd
from alfie_msgs.srv import GDBServoService


# Servo configuration: (name, bus, servo_id, servo_index_in_RobotLowCmd)
# servo_index is 0-14 for the 15-servo RobotLowCmd.servo_cmd array
# Left arm: 0-5, Right arm: 6-11, Head: 12-14
SERVO_CONFIG = {
    # Head (top of screen)
    'head_yaw':      {'bus': 0, 'id': 8,  'idx': 12, 'label': 'Head Yaw'},
    'head_pitch':    {'bus': 0, 'id': 9,  'idx': 13, 'label': 'Head Pitch'},
    'head_roll':     {'bus': 0, 'id': 10, 'idx': 14, 'label': 'Head Roll'},
    
    # Left arm (left side of screen) - bus 1
    'left_shoulder_yaw':      {'bus': 1, 'id': 1, 'idx': 0, 'label': 'L Shoulder Yaw'},
    'left_shoulder1_pitch':   {'bus': 1, 'id': 2, 'idx': 1, 'label': 'L Shoulder1 Pitch'},
    'left_shoulder2_pitch':   {'bus': 1, 'id': 3, 'idx': 2, 'label': 'L Shoulder2 Pitch'},
    'left_elbow_pitch':       {'bus': 1, 'id': 4, 'idx': 3, 'label': 'L Elbow Pitch'},
    'left_wrist_pitch':       {'bus': 1, 'id': 5, 'idx': 4, 'label': 'L Wrist Pitch'},
    'left_wrist_roll':        {'bus': 1, 'id': 6, 'idx': 5, 'label': 'L Wrist Roll'},
    'left_hand':              {'bus': 1, 'id': 7, 'idx': 6, 'label': 'L Hand'},
    
    # Right arm (right side of screen) - bus 0
    'right_shoulder_yaw':     {'bus': 0, 'id': 1, 'idx': 7,  'label': 'R Shoulder Yaw'},
    'right_shoulder1_pitch':  {'bus': 0, 'id': 2, 'idx': 8,  'label': 'R Shoulder1 Pitch'},
    'right_shoulder2_pitch':  {'bus': 0, 'id': 3, 'idx': 9,  'label': 'R Shoulder2 Pitch'},
    'right_elbow_pitch':      {'bus': 0, 'id': 4, 'idx': 10, 'label': 'R Elbow Pitch'},
    'right_wrist_pitch':      {'bus': 0, 'id': 5, 'idx': 11, 'label': 'R Wrist Pitch'},
    'right_wrist_roll':       {'bus': 0, 'id': 6, 'idx': 12, 'label': 'R Wrist Roll'},
    'right_hand':             {'bus': 0, 'id': 7, 'idx': 13, 'label': 'R Hand'},
}

# Navigation order for Tab/Arrow keys - arranged to match visual layout
NAV_ORDER = [
    # Head row (left to right)
    'head_yaw', 'head_pitch', 'head_roll',
    # Left arm column (top to bottom)
    'left_shoulder_yaw', 'left_shoulder1_pitch', 'left_shoulder2_pitch',
    'left_elbow_pitch', 'left_wrist_pitch', 'left_wrist_roll', 'left_hand',
    # Right arm column (top to bottom)
    'right_shoulder_yaw', 'right_shoulder1_pitch', 'right_shoulder2_pitch',
    'right_elbow_pitch', 'right_wrist_pitch', 'right_wrist_roll', 'right_hand',
]

# Memory addresses
ADDR_POSITION_CORRECTION = 31  # EPROM: position offset (-2047 to +2047)
ADDR_LOCK_MARK = 55  # EEPROM lock: 0=unlocked, 1=locked (actually 48 for lock)


class ServoCalibApp:
    """TUI application for servo calibration using curses."""
    
    def __init__(self, node: Node):
        """Initialize the calibration app.
        
        Args:
            node: ROS2 node instance
        """
        self.node = node
        self.logger = node.get_logger()
        
        # Create service clients for servo communication
        self.gdb0 = node.create_client(GDBServoService, "/alfie/low/gdb0servoservice")
        self.gdb1 = node.create_client(GDBServoService, "/alfie/low/gdb1servoservice")
        
        # Create publisher for RobotLowCmd
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.robot_cmd_pub = node.create_publisher(RobotLowCmd, '/alfie/robotlowcmd', qos)
        
        # Current offset values for each servo (read from servo EEPROM)
        self.offset_values = {name: 0 for name in SERVO_CONFIG}
        
        # Currently selected field
        self.selected_idx = 0
        
        # Input buffer for typing values
        self.input_buffer = ""
        self.editing = False
        
        # Status message
        self.status_msg = "Initializing..."
        
        # Timer for publishing robotlowcmd at 100Hz
        self.cmd_timer = node.create_timer(0.01, self._publish_zero_position)
        
    def _wait_for_services(self):
        """Wait for servo services to become available."""
        self.status_msg = "Waiting for servo services..."
        
        timeout = 1.0
        if not self.gdb0.wait_for_service(timeout_sec=timeout):
            self.logger.warning("GDB0 service not available")
        if not self.gdb1.wait_for_service(timeout_sec=timeout):
            self.logger.warning("GDB1 service not available")
            
        self.status_msg = "Services ready"
        
    def _get_client(self, bus: int):
        """Get the appropriate service client for a bus."""
        return self.gdb0 if bus == 0 else self.gdb1
    
    def _call_service(self, bus: int, servo_id: int, operation: str, 
                      address: int, value: int = 0):
        """Call servo service synchronously.
        
        Args:
            bus: Bus number (0 or 1)
            servo_id: Servo ID
            operation: 'r' for read, 'W' for write
            address: Memory address
            value: Value to write (for write operations)
            
        Returns:
            Response memorymap or None on failure
        """
        request = GDBServoService.Request()
        request.servo = servo_id
        request.operation = ord(operation)
        request.address = address
        request.value = value
        
        client = self._get_client(bus)
        future = client.call_async(request)
        
        # Wait for result with timeout (background thread handles spinning)
        timeout = 2.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.01)
        
        if future.done():
            result = future.result()
            if result:
                return result.memorymap
        return None
    
    def _read_offset(self, servo_name: str, max_retries: int = 5) -> int:
        """Read position offset from servo EEPROM.
        
        Retries if 0 is returned since actual offsets are never 0.
        """
        config = SERVO_CONFIG[servo_name]
        for attempt in range(max_retries):
            memmap = self._call_service(config['bus'], config['id'], 'r', 0)
            if memmap and memmap.positioncorrection != 0:
                return memmap.positioncorrection
            time.sleep(0.05)  # Small delay before retry
        # Return last value even if 0 after all retries
        if memmap:
            return memmap.positioncorrection
        return 0
    
    def _write_offset(self, servo_name: str, value: int) -> bool:
        """Write position offset to servo EEPROM.
        
        Sequence: unlock EEPROM, write value, read back, lock EEPROM.
        """
        config = SERVO_CONFIG[servo_name]
        bus = config['bus']
        servo_id = config['id']
        
        # Clamp value to valid range
        value = max(-2047, min(2047, value))
        
        # 1. Unlock EEPROM (write 0 to lock mark)
        self._call_service(bus, servo_id, 'W', ADDR_LOCK_MARK, 0)
        
        # 2. Write position correction value
        self._call_service(bus, servo_id, 'W', ADDR_POSITION_CORRECTION, value)
        
        # 3. Read back to verify
        memmap = self._call_service(bus, servo_id, 'r', 0)
        readback = memmap.positioncorrection if memmap else None
        
        # 4. Lock EEPROM (write 1 to lock mark)
        self._call_service(bus, servo_id, 'W', ADDR_LOCK_MARK, 1)
        
        if readback == value:
            self.offset_values[servo_name] = value
            return True
        return False
    
    def _read_all_offsets(self):
        """Read offset values from all servos."""
        self.status_msg = "Reading servo offsets..."
        for name in SERVO_CONFIG:
            self.offset_values[name] = self._read_offset(name)
        self.status_msg = "Ready - Use arrows/Tab to navigate, type value, Enter to write"
    
    def _publish_zero_position(self):
        """Publish RobotLowCmd with all servos at 0.0 radians."""
        cmd = RobotLowCmd()
        
        # Initialize all 15 servos - use list assignment for fixed-size array
        cmd.servo_cmd = [ServoCmd() for _ in range(15)]
        for i in range(15):
            cmd.servo_cmd[i].enabled = True
            cmd.servo_cmd[i].target_location = 0.0
            cmd.servo_cmd[i].target_speed = 0.5  # Slow speed for safety
            cmd.servo_cmd[i].target_acceleration = 0.0
            cmd.servo_cmd[i].target_torque = 500.0
        
        self.robot_cmd_pub.publish(cmd)
    
    def _draw_ui(self, stdscr):
        """Draw the TUI interface."""
        stdscr.clear()
        height, width = stdscr.getmaxyx()
        
        # Colors
        curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLUE)   # Header
        curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_WHITE)  # Selected
        curses.init_pair(3, curses.COLOR_GREEN, curses.COLOR_BLACK)  # Normal value
        curses.init_pair(4, curses.COLOR_YELLOW, curses.COLOR_BLACK) # Editing
        curses.init_pair(5, curses.COLOR_CYAN, curses.COLOR_BLACK)   # Labels
        
        # Title
        title = "═══ ALFIE SERVO OFFSETS CALIBRATION TOOL ═══"
        stdscr.attron(curses.color_pair(1) | curses.A_BOLD)
        stdscr.addstr(0, (width - len(title)) // 2, title)
        stdscr.attroff(curses.color_pair(1) | curses.A_BOLD)
        
        # Instructions
        instructions = "Tab/Arrows: Navigate | Type value + Enter: Write | r: Refresh | q: Quit"
        stdscr.addstr(1, (width - len(instructions)) // 2, instructions)
        
        # Head section (centered at top, vertical layout)
        head_y = 4
        head_x = (width - 28) // 2
        
        stdscr.attron(curses.color_pair(5) | curses.A_BOLD)
        stdscr.addstr(head_y - 1, head_x, "═══ HEAD ═══")
        stdscr.attroff(curses.color_pair(5) | curses.A_BOLD)
        
        head_servos = ['head_yaw', 'head_pitch', 'head_roll']
        for i, name in enumerate(head_servos):
            self._draw_field(stdscr, head_y + i, head_x, name)
        
        # Arm sections
        arm_y = 9
        left_x = 5
        right_x = width - 32  # 28 chars for field + 4 space margin
        
        # Left arm header
        stdscr.attron(curses.color_pair(5) | curses.A_BOLD)
        stdscr.addstr(arm_y - 1, left_x, "═══ LEFT ARM ═══")
        stdscr.attroff(curses.color_pair(5) | curses.A_BOLD)
        
        # Right arm header
        stdscr.attron(curses.color_pair(5) | curses.A_BOLD)
        stdscr.addstr(arm_y - 1, right_x, "═══ RIGHT ARM ═══")
        stdscr.attroff(curses.color_pair(5) | curses.A_BOLD)
        
        left_servos = [
            'left_shoulder_yaw', 'left_shoulder1_pitch', 'left_shoulder2_pitch',
            'left_elbow_pitch', 'left_wrist_pitch', 'left_wrist_roll', 'left_hand'
        ]
        right_servos = [
            'right_shoulder_yaw', 'right_shoulder1_pitch', 'right_shoulder2_pitch',
            'right_elbow_pitch', 'right_wrist_pitch', 'right_wrist_roll', 'right_hand'
        ]
        
        for i, name in enumerate(left_servos):
            self._draw_field(stdscr, arm_y + i, left_x, name)
            
        for i, name in enumerate(right_servos):
            self._draw_field(stdscr, arm_y + i, right_x, name)
        
        # Status bar
        status_y = height - 2
        stdscr.addstr(status_y, 2, f"Status: {self.status_msg[:width-12]}")
        
        stdscr.refresh()
    
    def _draw_field(self, stdscr, y, x, servo_name: str):
        """Draw a single servo field."""
        config = SERVO_CONFIG[servo_name]
        label = config['label']
        value = self.offset_values[servo_name]
        
        nav_idx = NAV_ORDER.index(servo_name)
        is_selected = nav_idx == self.selected_idx
        
        # Label
        stdscr.addstr(y, x, f"{label}:")
        
        # Value field
        value_x = x + 18
        
        if is_selected:
            if self.editing:
                # Show input buffer with cursor
                display = self.input_buffer + "_"
                stdscr.attron(curses.color_pair(4) | curses.A_BOLD)
                stdscr.addstr(y, value_x, f"[{display:>7}]")
                stdscr.attroff(curses.color_pair(4) | curses.A_BOLD)
            else:
                # Highlighted
                stdscr.attron(curses.color_pair(2) | curses.A_BOLD)
                stdscr.addstr(y, value_x, f"[{value:>6}]")
                stdscr.attroff(curses.color_pair(2) | curses.A_BOLD)
        else:
            # Normal
            stdscr.attron(curses.color_pair(3))
            stdscr.addstr(y, value_x, f" {value:>6} ")
            stdscr.attroff(curses.color_pair(3))
    
    def _handle_input(self, key):
        """Handle keyboard input."""
        current_name = NAV_ORDER[self.selected_idx]
        
        if self.editing:
            # In editing mode
            if key == ord('\n') or key == curses.KEY_ENTER or key == 10:
                # Commit value
                if self.input_buffer:
                    try:
                        value = int(self.input_buffer)
                        self.status_msg = f"Writing {value} to {current_name}..."
                        if self._write_offset(current_name, value):
                            self.status_msg = f"Wrote {value} to {current_name} successfully"
                        else:
                            self.status_msg = f"Failed to write to {current_name}"
                    except ValueError:
                        self.status_msg = "Invalid number"
                self.editing = False
                self.input_buffer = ""
            elif key == 27:  # Escape
                self.editing = False
                self.input_buffer = ""
                self.status_msg = "Edit cancelled"
            elif key == curses.KEY_BACKSPACE or key == 127 or key == 8:
                self.input_buffer = self.input_buffer[:-1]
            elif chr(key) in '0123456789-':
                if len(self.input_buffer) < 6:
                    self.input_buffer += chr(key)
        else:
            # Navigation mode
            if key == ord('q') or key == ord('Q'):
                return False  # Quit
            elif key == ord('\t') or key == curses.KEY_RIGHT:
                self.selected_idx = (self.selected_idx + 1) % len(NAV_ORDER)
            elif key == curses.KEY_BTAB or key == curses.KEY_LEFT:
                self.selected_idx = (self.selected_idx - 1) % len(NAV_ORDER)
            elif key == curses.KEY_DOWN:
                self._navigate_vertical(1)
            elif key == curses.KEY_UP:
                self._navigate_vertical(-1)
            elif key == ord('r') or key == ord('R'):
                # Refresh only the currently selected servo
                current_name = NAV_ORDER[self.selected_idx]
                self.status_msg = f"Refreshing {current_name}..."
                self.offset_values[current_name] = self._read_offset(current_name)
                self.status_msg = f"Refreshed {current_name}: {self.offset_values[current_name]}"
            elif chr(key) in '0123456789-':
                # Start editing
                self.editing = True
                self.input_buffer = chr(key)
        
        return True
    
    def _navigate_vertical(self, direction: int):
        """Navigate vertically within the current column."""
        current = NAV_ORDER[self.selected_idx]
        
        # Determine column and position
        if current.startswith('head_'):
            # Head row - move to arms
            if direction > 0:
                if current == 'head_yaw':
                    self.selected_idx = NAV_ORDER.index('left_shoulder_yaw')
                elif current == 'head_roll':
                    self.selected_idx = NAV_ORDER.index('right_shoulder_yaw')
                else:  # head_pitch - go to left arm
                    self.selected_idx = NAV_ORDER.index('left_shoulder_yaw')
        elif current.startswith('left_'):
            left_servos = [n for n in NAV_ORDER if n.startswith('left_')]
            idx = left_servos.index(current)
            if direction < 0 and idx == 0:
                self.selected_idx = NAV_ORDER.index('head_yaw')
            else:
                new_idx = max(0, min(len(left_servos) - 1, idx + direction))
                self.selected_idx = NAV_ORDER.index(left_servos[new_idx])
        elif current.startswith('right_'):
            right_servos = [n for n in NAV_ORDER if n.startswith('right_')]
            idx = right_servos.index(current)
            if direction < 0 and idx == 0:
                self.selected_idx = NAV_ORDER.index('head_roll')
            else:
                new_idx = max(0, min(len(right_servos) - 1, idx + direction))
                self.selected_idx = NAV_ORDER.index(right_servos[new_idx])
    
    def _curses_main(self, stdscr):
        """Main curses loop."""
        # Setup curses
        curses.curs_set(0)  # Hide cursor
        stdscr.nodelay(False)
        stdscr.timeout(100)  # 100ms timeout for getch
        
        # Wait for services
        self._wait_for_services()
        
        # Timer is already publishing at 100Hz via background spin thread
        self.status_msg = "100Hz command publisher started..."
        
        # Read all offsets
        self._read_all_offsets()
        
        # Main loop
        running = True
        while running:
            self._draw_ui(stdscr)
            
            key = stdscr.getch()
            if key != -1:
                running = self._handle_input(key)
            
            # No spin_once needed - background thread handles ROS callbacks
    
    def run(self):
        """Run the TUI application."""
        curses.wrapper(self._curses_main)
