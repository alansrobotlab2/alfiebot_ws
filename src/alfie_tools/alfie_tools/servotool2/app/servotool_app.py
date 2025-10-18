"""Main Gradio application for ServoTool2."""

import gradio as gr
from typing import Optional, Tuple, Dict, Any
import time

from alfie_tools.servotool2.ros.servo_client import ServoServiceClient
from alfie_tools.servotool2.ros.state_monitor import ServoStateMonitor
from alfie_tools.servotool2.app.field_config import (
    EPROM_FIELDS, SRAM_FIELDS, SERVO_CONFIG, EDITABLE_FIELDS
)


class ServoToolApp:
    """Gradio-based web application for servo configuration."""
    
    def __init__(self, node):
        """Initialize the ServoTool application.
        
        Args:
            node: ROS2 node instance for communication
        """
        self.node = node
        self.servo_client = ServoServiceClient(node)
        self.state_monitor = ServoStateMonitor(node)
        
        # Current selection
        self.current_bus = 0
        self.current_id = 1
        
        # Cache for memory map
        self.memory_map_cache = {}
        
        # Component references (will be set during UI creation)
        self.eprom_components = {}
        self.sram_components = {}
        
        self.app = self._create_interface()
    
    def _create_interface(self) -> gr.Blocks:
        """Create the Gradio interface."""
        with gr.Blocks(
            title="ServoTool2 - ST3215 Configuration",
            theme=gr.themes.Soft()
        ) as app:
            gr.Markdown("# 🤖 ServoTool2 - ST3215 Servo Configuration")
            gr.Markdown("Web-based servo configuration and monitoring tool for Alfie Robot")
            
            with gr.Row():
                # Servo selection dropdown
                servo_dropdown = gr.Dropdown(
                    choices=[name for name, _, _ in SERVO_CONFIG],
                    label="Select Servo",
                    value=SERVO_CONFIG[0][0],
                    interactive=True
                )
                
                # Refresh button
                refresh_btn = gr.Button("🔄 Refresh Data", variant="primary")
            
            # Status message
            status_msg = gr.Textbox(
                label="Status",
                value="Ready",
                interactive=False
            )
            
            with gr.Tabs():
                # ===== EPROM Configuration Tab =====
                with gr.Tab("⚙️ EPROM Settings (Persistent)"):
                    gr.Markdown("""
                    ### EPROM Settings
                    These settings are stored in non-volatile memory and persist across power cycles.
                    **Use the 'Write to EPROM' button to save changes.**
                    """)
                    
                    eprom_components = {}
                    self.eprom_components = eprom_components  # Store reference
                    
                    with gr.Accordion("🎯 Basic Limits", open=True):
                        with gr.Row():
                            for field in [f for f in EPROM_FIELDS if 'angle' in f.widget_name or 'temperature' in f.widget_name]:
                                eprom_components[field.widget_name] = gr.Number(
                                    label=f"{field.label} ({field.unit})",
                                    value=field.default,
                                    minimum=field.range[0],
                                    maximum=field.range[1],
                                    info=field.description
                                )
                    
                    with gr.Accordion("⚡ Voltage & Current", open=False):
                        with gr.Row():
                            for field in [f for f in EPROM_FIELDS if 'voltage' in f.widget_name or 'current' in f.widget_name or 'torque' in f.widget_name]:
                                eprom_components[field.widget_name] = gr.Number(
                                    label=f"{field.label} ({field.unit})",
                                    value=field.default,
                                    minimum=field.range[0],
                                    maximum=field.range[1],
                                    info=field.description
                                )
                    
                    with gr.Accordion("🎛️ PID Control", open=False):
                        with gr.Row():
                            for field in [f for f in EPROM_FIELDS if 'coefficient' in f.widget_name]:
                                eprom_components[field.widget_name] = gr.Number(
                                    label=f"{field.label}",
                                    value=field.default,
                                    minimum=field.range[0],
                                    maximum=field.range[1],
                                    info=field.description
                                )
                    
                    with gr.Accordion("🔧 Advanced Settings", open=False):
                        for field in [f for f in EPROM_FIELDS if f.widget_name not in eprom_components]:
                            eprom_components[field.widget_name] = gr.Number(
                                label=f"{field.label} ({field.unit})" if field.unit else field.label,
                                value=field.default,
                                minimum=field.range[0],
                                maximum=field.range[1],
                                info=field.description
                            )
                    
                    with gr.Row():
                        write_eprom_btn = gr.Button("💾 Write All EPROM Settings", variant="primary", size="lg")
                        lock_toggle = gr.Checkbox(
                            label="🔒 Lock EPROM (prevent writes)",
                            value=False,
                            info="When locked, EPROM writes are disabled"
                        )
                
                # ===== SRAM Control Tab =====
                with gr.Tab("🎮 SRAM Control (Volatile)"):
                    gr.Markdown("""
                    ### SRAM Settings
                    These settings are volatile and reset on power cycle.
                    **Changes are applied immediately when modified.**
                    """)
                    
                    sram_components = {}
                    
                    with gr.Row():
                        # Position control with slider
                        with gr.Column(scale=2):
                            target_slider = gr.Slider(
                                minimum=0,
                                maximum=4096,
                                value=2048,
                                step=1,
                                label="🎯 Target Position (Interactive)",
                                info="Move slider to control servo position (requires torque enabled)"
                            )
                            
                            with gr.Row():
                                torque_enable = gr.Checkbox(
                                    label="⚡ Enable Torque",
                                    value=False,
                                    info="Enable to hold position and respond to commands"
                                )
                                zero_btn = gr.Button("0️⃣ Zero Servo")
                                set_min_btn = gr.Button("📍 Set as Min Angle")
                                set_max_btn = gr.Button("📍 Set as Max Angle")
                    
                    with gr.Row():
                        for field in SRAM_FIELDS:
                            if field.widget_name != 'target_location':  # Already have slider for this
                                sram_components[field.widget_name] = gr.Number(
                                    label=f"{field.label} ({field.unit})" if field.unit else field.label,
                                    value=field.default,
                                    minimum=field.range[0],
                                    maximum=field.range[1],
                                    info=field.description
                                )
                    
                    sram_components['target_location'] = target_slider
                    self.sram_components = sram_components  # Store reference for value extraction
                    
                    write_sram_btn = gr.Button("✍️ Write All SRAM Settings", variant="secondary", size="lg")
                
                # ===== Status Monitor Tab =====
                with gr.Tab("📊 Live Status"):
                    gr.Markdown("""
                    ### Real-time Servo Status
                    Status updates automatically every second.
                    """)
                    
                    status_components = {}
                    
                    with gr.Row():
                        with gr.Column():
                            gr.Markdown("**Position & Motion**")
                            status_components['current_location'] = gr.Number(
                                label="Current Position (steps)",
                                interactive=False
                            )
                            status_components['current_speed'] = gr.Number(
                                label="Current Speed (step/s)",
                                interactive=False
                            )
                            status_components['acceleration'] = gr.Number(
                                label="Acceleration (100 step/s²)",
                                interactive=False
                            )
                            status_components['target_location'] = gr.Number(
                                label="Target Position (steps)",
                                interactive=False
                            )
                            status_components['mobile_sign'] = gr.Textbox(
                                label="Movement Status",
                                interactive=False
                            )
                        
                        with gr.Column():
                            gr.Markdown("**Electrical Status**")
                            status_components['current_voltage'] = gr.Number(
                                label="Voltage (V)",
                                interactive=False
                            )
                            status_components['current_current'] = gr.Number(
                                label="Current (mA)",
                                interactive=False
                            )
                            status_components['current_temperature'] = gr.Number(
                                label="Temperature (°C)",
                                interactive=False
                            )
                            status_components['torque_switch'] = gr.Textbox(
                                label="Torque Status",
                                interactive=False
                            )
                            status_components['servo_status'] = gr.Textbox(
                                label="Servo Status Code",
                                interactive=False
                            )
                    
                    with gr.Row():
                        with gr.Column():
                            gr.Markdown("**Servo Information**")
                            status_components['servo_id'] = gr.Number(
                                label="Servo ID",
                                interactive=False
                            )
                            status_components['baudrate'] = gr.Textbox(
                                label="Baudrate",
                                interactive=False
                            )
                        
                        with gr.Column():
                            gr.Markdown("**Firmware Version**")
                            status_components['firmware_major'] = gr.Number(
                                label="Firmware Major",
                                interactive=False
                            )
                            status_components['firmware_minor'] = gr.Number(
                                label="Firmware Minor",
                                interactive=False
                            )
                
                # ===== Memory Map Tab =====
                with gr.Tab("🗺️ Memory Map"):
                    gr.Markdown("""
                    ### Complete Memory Map
                    Raw view of all servo memory addresses.
                    """)
                    
                    memory_table = gr.Dataframe(
                        headers=["Address", "Value (Dec)", "Value (Hex)", "Description"],
                        interactive=False,
                        wrap=True
                    )
                    
                    load_memory_btn = gr.Button("📥 Load Memory Map", variant="primary")
            
            # ===== Event Handlers =====
            
            # Servo selection change
            servo_dropdown.change(
                fn=self._on_servo_change,
                inputs=[servo_dropdown],
                outputs=[status_msg] + list(eprom_components.values()) + list(sram_components.values()) + [lock_toggle, torque_enable]
            )
            
            # Refresh button
            refresh_btn.click(
                fn=self._refresh_data,
                inputs=[servo_dropdown],
                outputs=[status_msg] + list(eprom_components.values()) + list(sram_components.values()) + [lock_toggle, torque_enable]
            )
            
            # EPROM write button
            write_eprom_btn.click(
                fn=self._write_eprom_fields,
                inputs=[servo_dropdown, lock_toggle] + list(eprom_components.values()),
                outputs=[status_msg]
            )
            
            # SRAM write button
            write_sram_btn.click(
                fn=self._write_sram_fields,
                inputs=[servo_dropdown] + list(sram_components.values()),
                outputs=[status_msg]
            )
            
            # Target position slider (real-time control)
            target_slider.change(
                fn=self._on_target_position_change,
                inputs=[servo_dropdown, target_slider, torque_enable],
                outputs=[status_msg]
            )
            
            # Torque enable toggle
            torque_enable.change(
                fn=self._toggle_torque,
                inputs=[servo_dropdown, torque_enable],
                outputs=[status_msg]
            )
            
            # Lock toggle
            lock_toggle.change(
                fn=self._toggle_lock,
                inputs=[servo_dropdown, lock_toggle],
                outputs=[status_msg]
            )
            
            # Quick action buttons
            zero_btn.click(
                fn=self._zero_servo,
                inputs=[servo_dropdown],
                outputs=[status_msg, target_slider]
            )
            
            set_min_btn.click(
                fn=self._set_min_angle,
                inputs=[servo_dropdown, target_slider],
                outputs=[status_msg] + [eprom_components['min_angle']]
            )
            
            set_max_btn.click(
                fn=self._set_max_angle,
                inputs=[servo_dropdown, target_slider],
                outputs=[status_msg] + [eprom_components['max_angle']]
            )
            
            # Memory map loader
            load_memory_btn.click(
                fn=self._load_memory_map,
                inputs=[servo_dropdown],
                outputs=[status_msg, memory_table]
            )
            
            # Auto-refresh status every second using timer
            def update_status_loop(dropdown, torque_enabled):
                """Periodic status update."""
                state = self._get_servo_status(dropdown)
                outputs = []
                for key in status_components.keys():
                    if key in state:
                        outputs.append(state[key])
                    else:
                        outputs.append(None)
                
                # Update slider position if torque is disabled
                if not torque_enabled and state and 'current_location' in state:
                    current_pos = state['current_location']
                    # Clamp to slider range
                    slider_pos = max(0, min(4096, current_pos))
                    outputs.append(slider_pos)
                else:
                    # Don't update slider (use gr.update to skip)
                    outputs.append(gr.skip())
                
                return outputs
            
            # Create periodic update using dependency with gr.Timer
            timer = gr.Timer(value=1.0)
            timer.tick(
                fn=update_status_loop,
                inputs=[servo_dropdown, torque_enable],
                outputs=list(status_components.values()) + [target_slider]
            )
        
        return app
    
    def _parse_servo_selection(self, dropdown_value: str) -> Tuple[int, int]:
        """Parse servo dropdown selection to bus and ID.
        
        Args:
            dropdown_value: Selected dropdown text
            
        Returns:
            Tuple of (bus, servo_id)
        """
        for name, bus, servo_id in SERVO_CONFIG:
            if name == dropdown_value:
                self.current_bus = bus
                self.current_id = servo_id
                return bus, servo_id
        return 0, 1
    
    def _on_servo_change(self, dropdown_value: str):
        """Handle servo selection change."""
        bus, servo_id = self._parse_servo_selection(dropdown_value)
        
        # Read memory map
        memory_map = self.servo_client.read_memory_map(bus, servo_id)
        
        if memory_map is None:
            status = f"❌ Failed to read from bus {bus}, servo {servo_id}"
            # Return status + empty values for all fields + checkboxes
            return [status] + [None] * len(EDITABLE_FIELDS) + [False, False]
        
        # Cache memory map
        self.memory_map_cache[(bus, servo_id)] = memory_map
        
        # Map addresses to memory map field names
        address_to_field = {
            7: 'returndelay',
            9: 'minanglelimit',
            11: 'maxanglelimit',
            13: 'maxtemplimit',
            14: 'maxinputvoltage',
            15: 'mininputvoltage',
            16: 'maxtorque',
            21: 'pcoefficient',
            22: 'dcoefficient',
            23: 'icoefficient',
            24: 'minstartupforce',
            26: 'clockwiseinsensitivearea',
            27: 'counterclockwiseinsensitiveregion',
            28: 'protectioncurrent',
            30: 'angularresolution',
            31: 'positioncorrection',
            33: 'operationmode',
            34: 'protectivetorque',
            35: 'protectiontime',
            36: 'overloadtorque',
            37: 'speedclosedlooppcoefficient',
            38: 'overcurrentprotectiontime',
            39: 'velocityclosedloopicoefficient',
            41: 'acceleration',
            42: 'targetlocation',
            44: 'runningtime',
            46: 'runningspeed',
            48: 'torquelimit',
        }
        
        # Extract field values and match to component order
        outputs = [f"✅ Loaded servo {servo_id} on bus {bus}"]
        
        # Populate EPROM components by widget name
        for widget_name in self.eprom_components.keys():
            # Find the field config for this widget
            field = next((f for f in EPROM_FIELDS if f.widget_name == widget_name), None)
            if field:
                field_name = address_to_field.get(field.address)
                if field_name:
                    value = getattr(memory_map, field_name, None)
                    outputs.append(value if value is not None else field.default)
                else:
                    outputs.append(field.default)
            else:
                outputs.append(None)
        
        # Populate SRAM components by widget name
        for widget_name in self.sram_components.keys():
            # Find the field config for this widget
            field = next((f for f in SRAM_FIELDS if f.widget_name == widget_name), None)
            if field:
                field_name = address_to_field.get(field.address)
                if field_name:
                    value = getattr(memory_map, field_name, None)
                    outputs.append(value if value is not None else field.default)
                else:
                    outputs.append(field.default)
            else:
                outputs.append(None)
        
        # Add checkbox states for lock and torque
        # Lock mark: 0 = unlocked, 1 = locked
        lock_status = memory_map.lockmark == 1
        # Torque switch: 0 = disabled, 1 = enabled
        torque_status = memory_map.torqueswitch == 1
        
        outputs.append(lock_status)
        outputs.append(torque_status)
        
        return outputs
    
    def _refresh_data(self, dropdown_value: str):
        """Refresh data from servo."""
        return self._on_servo_change(dropdown_value)
    
    def _write_eprom_fields(self, dropdown_value: str, lock_enabled: bool, *field_values):
        """Write all EPROM fields to servo."""
        if lock_enabled:
            return "🔒 EPROM is locked. Unlock to write settings."
        
        bus, servo_id = self._parse_servo_selection(dropdown_value)
        
        success_count = 0
        fail_count = 0
        
        for field, value in zip(EPROM_FIELDS, field_values):
            if value is not None:
                # Validate range
                if not (field.range[0] <= value <= field.range[1]):
                    fail_count += 1
                    continue
                
                # Write to servo
                if self.servo_client.write_value(bus, servo_id, field.address, int(value)):
                    success_count += 1
                else:
                    fail_count += 1
        
        return f"✅ Wrote {success_count} EPROM fields. ❌ Failed: {fail_count}"
    
    def _write_sram_fields(self, dropdown_value: str, *field_values):
        """Write all SRAM fields to servo."""
        bus, servo_id = self._parse_servo_selection(dropdown_value)
        
        success_count = 0
        fail_count = 0
        
        for field, value in zip(SRAM_FIELDS, field_values):
            if value is not None:
                # Validate range
                if not (field.range[0] <= value <= field.range[1]):
                    fail_count += 1
                    continue
                
                # Write to servo
                if self.servo_client.write_value(bus, servo_id, field.address, int(value)):
                    success_count += 1
                else:
                    fail_count += 1
        
        return f"✅ Wrote {success_count} SRAM fields. ❌ Failed: {fail_count}"
    
    def _on_target_position_change(self, dropdown_value: str, target: int, torque_enabled: bool):
        """Handle target position slider change."""
        if not torque_enabled:
            return "⚠️ Enable torque to control position"
        
        bus, servo_id = self._parse_servo_selection(dropdown_value)
        
        # Write target position (address 42)
        if self.servo_client.write_value(bus, servo_id, 42, int(target)):
            return f"✅ Set target position to {target}"
        else:
            return f"❌ Failed to set target position"
    
    def _toggle_torque(self, dropdown_value: str, enabled: bool):
        """Toggle torque enable/disable."""
        bus, servo_id = self._parse_servo_selection(dropdown_value)
        
        # Torque switch is at address 40
        value = 1 if enabled else 0
        
        if self.servo_client.write_value(bus, servo_id, 40, value):
            status = "⚡ Torque ENABLED" if enabled else "🔓 Torque DISABLED"
            return status
        else:
            return "❌ Failed to toggle torque"
    
    def _toggle_lock(self, dropdown_value: str, locked: bool):
        """Toggle EPROM lock."""
        bus, servo_id = self._parse_servo_selection(dropdown_value)
        
        # Lock mark is at address 55
        value = 48 if locked else 0
        
        if self.servo_client.write_value(bus, servo_id, 55, value):
            status = "🔒 EPROM LOCKED" if locked else "🔓 EPROM UNLOCKED"
            return status
        else:
            return "❌ Failed to toggle lock"
    
    def _zero_servo(self, dropdown_value: str):
        """Set servo to center position (2048)."""
        bus, servo_id = self._parse_servo_selection(dropdown_value)
        
        # Center position is 2048 (middle of 0-4096 range)
        center_pos = 2048
        if self.servo_client.write_value(bus, servo_id, 42, center_pos):
            return f"✅ Servo centered at {center_pos}", center_pos
        else:
            return "❌ Failed to zero servo", center_pos
    
    def _set_min_angle(self, dropdown_value: str, current_position: int):
        """Set current position as minimum angle."""
        bus, servo_id = self._parse_servo_selection(dropdown_value)
        
        # Write to min angle (address 9)
        if self.servo_client.write_value(bus, servo_id, 9, int(current_position)):
            return f"✅ Min angle set to {current_position}", current_position
        else:
            return "❌ Failed to set min angle", current_position
    
    def _set_max_angle(self, dropdown_value: str, current_position: int):
        """Set current position as maximum angle."""
        bus, servo_id = self._parse_servo_selection(dropdown_value)
        
        # Write to max angle (address 11)
        if self.servo_client.write_value(bus, servo_id, 11, int(current_position)):
            return f"✅ Max angle set to {current_position}", current_position
        else:
            return "❌ Failed to set max angle", current_position
    
    def _get_servo_status(self, dropdown_value: str) -> Dict[str, Any]:
        """Get current servo status from state monitor."""
        bus, servo_id = self._parse_servo_selection(dropdown_value)
        
        state = self.state_monitor.get_servo_state(bus, servo_id)
        
        if state is None:
            return {}
        
        # Format output
        return {
            'current_location': state['current_location'],
            'current_speed': state['current_speed'],
            'acceleration': state['acceleration'],
            'target_location': state['target_location'],
            'mobile_sign': "🏃 Moving" if state['mobile_sign'] else "🛑 Stopped",
            'current_voltage': round(state['current_voltage'], 2),
            'current_current': round(state['current_current'], 1),
            'current_temperature': state['current_temperature'],
            'torque_switch': "⚡ ON" if state['torque_switch'] else "🔓 OFF",
            'servo_status': f"0x{state['servo_status']:02X}",
            'servo_id': servo_id,
            'baudrate': "115200 bps",  # Assuming default
            'firmware_major': 0,  # Would need to read from memory map
            'firmware_minor': 0,
        }
    
    def _load_memory_map(self, dropdown_value: str):
        """Load complete memory map as table."""
        bus, servo_id = self._parse_servo_selection(dropdown_value)
        
        memory_map = self.servo_client.read_memory_map(bus, servo_id)
        
        if memory_map is None:
            return "❌ Failed to read memory map", []
        
        # Build table data with complete descriptions from ST3215 spec
        # Map memory map field names to address, description, and value
        table_data = []
        
        # Memory map field mapping: (address, field_name, description)
        memory_fields = [
            (0, 'firmwaremajor', "Firmware major version number (EPROM, read only)"),
            (1, 'firmwaresub', "Firmware minor version number (EPROM, read only)"),
            (3, 'servomajor', "Servo major version number (EPROM, read only)"),
            (4, 'servosub', "Servo minor version number (EPROM, read only)"),
            (5, 'servoid', "ID - Unique identification on bus (EPROM, 0-253)"),
            (6, 'baudrate', "Baudrate - 0-7: 1M/500k/250k/128k/115.2k/76.8k/57.6k/38.4k (EPROM)"),
            (7, 'returndelay', "Return delay - Response delay in 2μs units (EPROM, 0-254, max 508μs)"),
            (8, 'responsestatuslevel', "Response status level - 0: No response except READ/PING, 1: All respond (EPROM)"),
            (9, 'minanglelimit', "Minimum angle - Range limit (EPROM, -32766 to +32767 steps, 0 for multi-turn)"),
            (11, 'maxanglelimit', "Maximum angle - Range limit (EPROM, -32766 to +32767 steps, 0 for multi-turn)"),
            (13, 'maxtemplimit', "Maximum temperature - Limit in °C (EPROM, 0-100°C)"),
            (14, 'maxinputvoltage', "Maximum input voltage - In 0.1V units (EPROM, 0-254, e.g. 80 = 8.0V)"),
            (15, 'mininputvoltage', "Minimum input voltage - In 0.1V units (EPROM, 0-254, e.g. 40 = 4.0V)"),
            (16, 'maxtorque', "Maximum torque - Limit (EPROM, 0-1000, 1000 = 100%)"),
            (18, 'phase', "Phase - Special function byte (EPROM, do not modify)"),
            (19, 'unloadingcondition', "Unloading conditions - Protection bits (EPROM, Bit0-5: Voltage/Sensor/Temp/Current/Angle/Overload)"),
            (20, 'ledalarmcondition', "LED alarm conditions - LED flash bits (EPROM, Bit0-5: Voltage/Sensor/Temp/Current/Angle/Overload)"),
            (21, 'pcoefficient', "Position loop P coefficient - Proportional gain (EPROM, 0-254)"),
            (22, 'dcoefficient', "Position loop D coefficient - Derivative gain (EPROM, 0-254)"),
            (23, 'icoefficient', "Position loop I coefficient - Integral gain (EPROM, 0-254)"),
            (24, 'minstartupforce', "Minimum starting force - Startup torque (EPROM, 0-1000, 0.1% units)"),
            (26, 'clockwiseinsensitivearea', "Clockwise insensitive zone - Dead zone in steps (EPROM, 0-32 steps)"),
            (27, 'counterclockwiseinsensitiveregion', "Counter-clockwise insensitive zone - Dead zone in steps (EPROM, 0-32 steps)"),
            (28, 'protectioncurrent', "Protection current - Threshold (EPROM, 0-511, 6.5mA units, max 3321.5mA)"),
            (30, 'angularresolution', "Angle resolution - Amplification factor 1-3 (EPROM, affects step/degree)"),
            (31, 'positioncorrection', "Position correction - Offset (EPROM, -2047 to +2047 steps offset)"),
            (33, 'operationmode', "Operation mode - 0:servo 1:motor 2:PWM 3:step (EPROM)"),
            (34, 'protectivetorque', "Protection torque - Torque after overload (EPROM, 0-100, 1.0% units)"),
            (35, 'protectiontime', "Protection time - Overload duration (EPROM, 0-254, 10ms units, max 2.54s)"),
            (36, 'overloadtorque', "Overload torque - Threshold for protection (EPROM, 0-100, 1.0% units)"),
            (37, 'speedclosedlooppcoefficient', "Speed P coefficient - For constant speed mode (EPROM, 0-100)"),
            (38, 'overcurrentprotectiontime', "Overcurrent protection time - Duration (EPROM, 0-254, 10ms units, max 2.54s)"),
            (39, 'velocityclosedloopicoefficient', "Velocity I coefficient - For constant speed mode (EPROM, 0-254, 1/10 units)"),
            (40, 'torqueswitch', "Torque switch - 0:disable 1:enable 128:zero (SRAM)"),
            (41, 'acceleration', "Acceleration - Rate (SRAM, 0-254, 100 step/s² units)"),
            (42, 'targetlocation', "Target location - Position (SRAM, -30719 to +30719 steps)"),
            (44, 'runningtime', "Running time - Duration (SRAM, 0-1000, 0.1% units, BIT10 is direction)"),
            (46, 'runningspeed', "Running speed - Velocity (SRAM, 0-3400 step/s, 50 step/s = 0.732 RPM)"),
            (48, 'torquelimit', "Torque limit - Current limit (SRAM, 0-1000, 1.0% units)"),
            (55, 'lockmark', "Lock flag - 0:unlocked 1:locked (SRAM, prevents EPROM saves)"),
            (56, 'currentlocation', "Current location - Position feedback (SRAM, read only)"),
            (58, 'currentspeed', "Current speed - Velocity feedback (SRAM, read only, step/s)"),
            (60, 'currentload', "Current load - PWM duty cycle (SRAM, read only, 0.1%)"),
            (62, 'currentvoltage', "Current voltage - Operating voltage (SRAM, read only, 0.1V units)"),
            (63, 'currenttemperature', "Current temperature - Internal temp (SRAM, read only, °C)"),
            (64, 'asyncwriteflag', "Asynchronous write flag - For async writes (SRAM, read only)"),
            (65, 'servostatus', "Servo status - Error bits (SRAM, read only, Bit0-5: Voltage/Sensor/Temp/Current/Angle/Overload)"),
            (66, 'mobilesign', "Move flag - 1:moving 0:stopped (SRAM, read only)"),
            (69, 'currentcurrent', "Current current - Draw measurement (SRAM, read only, 6.5mA units)"),
        ]
        
        # Build table from memory map fields
        for addr, field_name, description in memory_fields:
            value = getattr(memory_map, field_name, None)
            
            if value is None:
                continue
            
            # Handle signed values for hex display
            if isinstance(value, int) and value < 0:
                hex_val = f"0x{value & 0xFFFF:04X}" if abs(value) > 255 else f"0x{value & 0xFF:02X}"
            else:
                hex_val = f"0x{value:04X}" if value > 255 else f"0x{value:02X}"
            
            table_data.append([
                f"0x{addr:02X} ({addr})",
                str(value),
                hex_val,
                description
            ])
        
        return f"✅ Memory map loaded: {len(table_data)} addresses", table_data
    
    def launch(self):
        """Launch the Gradio application."""
        self.node.get_logger().info("Launching ServoTool2 web interface...")
        self.node.get_logger().info("Open your browser to: http://localhost:7860")
        
        self.app.launch(
            server_name="0.0.0.0",
            server_port=7860,
            share=False,
            inbrowser=False
        )
