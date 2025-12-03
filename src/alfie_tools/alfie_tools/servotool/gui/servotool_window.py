"""Main window for ServoTool application."""

from pathlib import Path
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QSize, QTimer

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from alfie_msgs.msg import GDBState

from alfie_tools.servotool.gui.field_config import EDITABLE_FIELDS, SERVO_CONFIG, FIELD_RANGE_MAP, FIELD_ADDRESS_MAP
from alfie_tools.servotool.ros.servo_client import ServoServiceClient


class ServoToolWindow(QtWidgets.QMainWindow):
    """Servo configuration and monitoring GUI."""
    
    def __init__(self, node, *args, **kwargs):
        """Initialize the ServoTool main window.
        
        Args:
            node: ROS2 node instance for communication
            *args: Variable length argument list passed to QMainWindow
            **kwargs: Arbitrary keyword arguments passed to QMainWindow
        """
        super().__init__(*args, **kwargs)
        
        self.loaded = False
        self.node = node
        self.servo_client = ServoServiceClient(node)
        
        # Store original values for each field to revert on focus loss without Enter
        self.field_original_values = {}
        
        # Use field ranges from config
        self.field_ranges = FIELD_RANGE_MAP
        
        # Current servo selection
        self.bus = 0
        self.id = 1
        
        # Store latest driver states
        self.driver0_state = None
        self.driver1_state = None
        
        self._load_ui()
        self._setup_subscriber()
        self._setup_connections()
        self._populate_servo_dropdown()
        self._disable_all_fields()
        self._clear_all_fields()
        self._enable_all_fields()
        
        self.loaded = True
        self.onServoSelected()
    
    def _load_ui(self):
        """Load UI from file and configure window."""
        # UI file is in parent directory (servotool/)
        script_dir = Path(__file__).parent.parent
        ui_file = script_dir / "servotool.ui"
        
        uic.loadUi(str(ui_file), self)
        self.setFixedSize(QSize(770, 780))
        
        # Configure slider range for target location (0 to 4096)
        self.sliderTargetLocation.setMinimum(0)
        self.sliderTargetLocation.setMaximum(4096)
    
    def _setup_subscriber(self):
        """Create subscriptions to driver state topics."""
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
        self.driver0_state_sub = self.node.create_subscription(
            GDBState,
            "/alfie/low/gdb0state",
            self.GDB0StateCallback,
            qos
        )
        
        self.driver1_state_sub = self.node.create_subscription(
            GDBState,
            "/alfie/low/gdb1state",
            self.GDB1StateCallback,
            qos
        )
    
    def _setup_connections(self):
        """Connect UI signals to slots."""
        self.cmbServos.currentIndexChanged.connect(self.onServoSelected)
        self.sliderTargetLocation.valueChanged.connect(self.onSliderValueChanged)
        self.toggleLockMark.clicked.connect(self.onToggleLockClicked)
        self.toggleTorqueSwitch.clicked.connect(self.onToggleTorqueSwitchClicked)
        self.btnZeroServo.clicked.connect(self.onBtnZeroServoClicked)
        self.btnSetMinAngle.clicked.connect(self.onBtnSetMinAngleClicked)
        self.btnSetMaxAngle.clicked.connect(self.onBtnSetMaxAngleClicked)
        
        # Connect all editable fields dynamically using config
        for field in EDITABLE_FIELDS:
            widget = getattr(self, field.widget_name)
            widget.returnPressed.connect(
                lambda f=field: self.onTextFieldReturnPressed(f.widget_name, f.address)
            )
            widget.editingFinished.connect(
                lambda f=field: self.onTextFieldFocusOut(f.widget_name)
            )
    
    def _populate_servo_dropdown(self):
        """Populate the servo selection dropdown with available servos."""
        for display_name, bus, servo_id in SERVO_CONFIG:
            self.cmbServos.addItem(display_name, [bus, servo_id])

    def GDB0StateCallback(self, msg):
        """Callback for GDB0 state updates.

        Args:
            msg (GDBState): Message containing current state of GDB0 servos
        """
        self.driver0_state = msg
        # Update UI if currently viewing a servo on this bus
        if self.bus == 0:
            self._update_servo_status_display()

    def GDB1StateCallback(self, msg):
        """Callback for GDB1 state updates.

        Args:
            msg (GDBState): Message containing current state of GDB1 servos
        """
        self.driver1_state = msg
        # Update UI if currently viewing a servo on this bus
        if self.bus == 1:
            self._update_servo_status_display()
    
    def _update_servo_status_display(self):
        """Update the UI with current servo status from appropriate driver state."""
        # Get the appropriate driver state based on current bus
        driver_state = self.driver0_state if self.bus == 0 else self.driver1_state
        
        # Check if we have data
        if driver_state is None:
            return
        
        # Servo ID is 1-indexed, array is 0-indexed
        servo_index = self.id - 1
        
        if servo_index < len(driver_state.servo_state):
            servo = driver_state.servo_state[servo_index]
            
            # Update status fields
            self.txtStatusTorqueSwitch.setText(str(servo.torque_switch))
            self.txtStatusTargetLocation.setText(str(servo.target_location))
            self.txtStatusAcceleration.setText(str(servo.target_acceleration))
            self.txtStatusCurrentLocation.setText(str(servo.current_location))
            self.txtStatusCurrentSpeed.setText(str(servo.current_speed))
            self.txtStatusCurrentVoltage.setText(str(servo.current_voltage / 10.0))
            self.txtStatusCurrentCurrent.setText(str(servo.current_current * 6.5))
            self.txtStatusCurrentTemperature.setText(str(servo.current_temperature))
            self.txtStatusServoStatus.setText(str(servo.servo_status))
            self.txtStatusMobileSign.setText(str(servo.mobile_sign))
            
            # Update slider position from current location only when torque is disabled
            if not self.toggleTorqueSwitch.isChecked():
                self.sliderTargetLocation.setValue(servo.current_location)
    
    def onServoSelected(self):
        """Handle servo selection change event."""
        if self.loaded:
            currentIndex = self.cmbServos.currentIndex()
            if currentIndex >= 0:
                self.bus, self.id = self.cmbServos.itemData(currentIndex)
                print(f"Selected Servo: Bus {self.bus}, ID {self.id}")
            else:
                print("No servo selected")
            
            # Pull in data for selected servo from bus
            self._disable_all_fields()
            self._clear_all_fields()
            self._populate_fields()
            self._enable_all_fields()
    
    def onSliderValueChanged(self):
        """Handle slider value change event."""
        value = self.sliderTargetLocation.value()
        self.txtTargetLocation.setText(str(value))
        
        # Only send command to servo if loaded and torque is enabled
        if self.loaded and self.toggleTorqueSwitch.isChecked():
            print(f"Slider Value: {value} - sending to servo")
            
            # Write the target position to address 42
            success = self.servo_client.write_value(self.bus, self.id, 42, value)
            
            if success:
                # Update the stored original value for the target location field
                self.field_original_values['txtTargetLocation'] = str(value)
            else:
                print(f"Failed to set target position to {value}")
        else:
            if self.loaded:
                print(f"Slider Value: {value} - torque disabled, not sending to servo")
    
    def onToggleLockClicked(self):
        """Handle toggleLock button click event."""
        if not self.loaded:
            return
        
        # Checked = unlocked = 0, Unchecked = locked = 1
        if self.toggleLockMark.isChecked():
            lockmark_value = 0
            button_text = "EEPROM UNLocked"
        else:
            lockmark_value = 1
            button_text = "EEPROM Locked"
        
        print(f"Toggle Lock clicked: setting lockmark to {lockmark_value}")
        
        # Write the value to servo at address 55
        success = self.servo_client.write_value(self.bus, self.id, 55, lockmark_value)
        
        if success:
            self.toggleLockMark.setText(button_text)
            self.txtLockMark.setText(str(lockmark_value))
            print(f"Successfully set lockmark to {lockmark_value}")
        else:
            print(f"Failed to set lockmark to {lockmark_value}")
            # Revert the button state if write failed
            self.toggleLockMark.setChecked(not self.toggleLockMark.isChecked())
    
    def onToggleTorqueSwitchClicked(self):
        """Handle toggleTorqueSwitch button click event."""
        if not self.loaded:
            return
        
        success = False
        
        if self.toggleTorqueSwitch.isChecked():
            # Enabling torque: set target position to current location
            button_text = "Torque Enabled"
            
            try:
                current_location_text = self.txtStatusCurrentLocation.text()
                if not current_location_text:
                    print("Error: Current location is empty")
                    self.toggleTorqueSwitch.setChecked(False)
                    return
                
                current_location = int(current_location_text)
                
                print(f"Toggle Torque Switch clicked: enabling torque by setting target position to {current_location}")
                
                # Write current location to target position (address 42)
                success = self.servo_client.write_value(self.bus, self.id, 42, current_location)
                
                if success:
                    # Also update the target location field and slider
                    self.txtTargetLocation.setText(str(current_location))
                    self.sliderTargetLocation.setValue(current_location)
                    self.field_original_values['txtTargetLocation'] = str(current_location)
                    print(f"Successfully enabled torque at position {current_location}")
                else:
                    print(f"Failed to set target position to {current_location}")
                    
            except ValueError as e:
                print(f"Error: Invalid current location value - {e}")
                self.toggleTorqueSwitch.setChecked(False)
                return
        else:
            # Disabling torque: write 0 to torqueswitch
            button_text = "Torque Disabled"
            
            print(f"Toggle Torque Switch clicked: disabling torque")
            
            # Write 0 to torqueswitch (address 40)
            success = self.servo_client.write_value(self.bus, self.id, 40, 0)
            
            if success:
                print(f"Successfully disabled torque")
            else:
                print(f"Failed to disable torque")
        
        # Update button text or revert state based on success
        if success:
            self.toggleTorqueSwitch.setText(button_text)
        else:
            # Revert the button state if write failed
            self.toggleTorqueSwitch.setChecked(not self.toggleTorqueSwitch.isChecked())
    
    def onBtnZeroServoClicked(self):
        """Handle Zero Servo button click event."""
        if not self.loaded:
            return
        
        try:
            current_location_text = self.txtStatusCurrentLocation.text()
            current_offset_text = self.txtPositionCorrection.text()
            if not current_location_text:
                print("Error: Current location is empty")
                return
            
            current_location = int(current_location_text)
            current_offset = int(current_offset_text) if current_offset_text else 0
            
            # Calculate the position correction needed to zero the servo
            target_center = 2048
            position_correction = current_offset + (current_location - target_center)
            
            # Validate the correction is within range
            if position_correction < -2047 or position_correction > 2047:
                print(f"Error: Calculated position correction {position_correction} is out of range (-2047 to 2047)")
                return
            
            print(f"Zero Servo clicked: current_location={current_location}, calculated correction={position_correction}")
            
            # Write the position correction value to address 31
            success = self.servo_client.write_value(self.bus, self.id, 31, position_correction)
            
            if success:
                # Update the position correction field to show the new value
                self.txtPositionCorrection.setText(str(position_correction))
                self.field_original_values['txtPositionCorrection'] = str(position_correction)
                print(f"Successfully set position correction to {position_correction}")
            else:
                print(f"Failed to set position correction to {position_correction}")
                
        except ValueError as e:
            print(f"Error: Invalid current location value - {e}")
        except Exception as e:
            print(f"Error in onBtnZeroServoClicked: {e}")
            import traceback
            traceback.print_exc()
    
    def onBtnSetMinAngleClicked(self):
        """Handle Set Min Angle button click event."""
        if not self.loaded:
            return
        
        try:
            current_location_text = self.txtStatusCurrentLocation.text()
            if not current_location_text:
                print("Error: Current location is empty")
                return
            
            current_location = int(current_location_text)
            
            # Verify that current location is less than 2048 (center position)
            if current_location >= 2048:
                print(f"WARNING: Current position ({current_location}) is not less than 2048!")
                print(f"WARNING: Min angle should be set to a position less than center (2048)")
                print(f"WARNING: Not setting min angle - move servo to a lower position first")
                return
            
            print(f"Set Min Angle clicked: setting min angle to {current_location}")
            
            # Write the current location to min angle address (9)
            success = self.servo_client.write_value(self.bus, self.id, 9, current_location)
            
            if success:
                # Update the min angle field to show the new value
                self.txtMinAngle.setText(str(current_location))
                self.field_original_values['txtMinAngle'] = str(current_location)
                print(f"Successfully set min angle to {current_location}")
            else:
                print(f"Failed to set min angle to {current_location}")
                
        except ValueError as e:
            print(f"Error: Invalid current location value - {e}")
        except Exception as e:
            print(f"Error in onBtnSetMinAngleClicked: {e}")
            import traceback
            traceback.print_exc()
    
    def onBtnSetMaxAngleClicked(self):
        """Handle Set Max Angle button click event."""
        if not self.loaded:
            return
        
        try:
            current_location_text = self.txtStatusCurrentLocation.text()
            if not current_location_text:
                print("Error: Current location is empty")
                return
            
            current_location = int(current_location_text)
            
            # Verify that current location is greater than 2048 (center position)
            if current_location <= 2048:
                print(f"WARNING: Current position ({current_location}) is not greater than 2048!")
                print(f"WARNING: Max angle should be set to a position greater than center (2048)")
                print(f"WARNING: Not setting max angle - move servo to a higher position first")
                return
            
            print(f"Set Max Angle clicked: setting max angle to {current_location}")
            
            # Write the current location to max angle address (11)
            success = self.servo_client.write_value(self.bus, self.id, 11, current_location)
            
            if success:
                # Update the max angle field to show the new value
                self.txtMaxAngle.setText(str(current_location))
                self.field_original_values['txtMaxAngle'] = str(current_location)
                print(f"Successfully set max angle to {current_location}")
            else:
                print(f"Failed to set max angle to {current_location}")
                
        except ValueError as e:
            print(f"Error: Invalid current location value - {e}")
        except Exception as e:
            print(f"Error in onBtnSetMaxAngleClicked: {e}")
            import traceback
            traceback.print_exc()
    
    def onTextFieldFocusOut(self, field_name):
        """Handle text field losing focus event.
        
        Args:
            field_name (str): Name of the text field widget
        """
        if not self.loaded:
            return
        
        field_widget = getattr(self, field_name)
        current_value = field_widget.text()
        original_value = self.field_original_values.get(field_name, "")
        
        # If the value changed from original, revert it
        if current_value != original_value:
            print(f"DEBUG: Reverting {field_name} from '{current_value}' to '{original_value}'")
            field_widget.setText(original_value)
            field_widget.setStyleSheet("")
            field_widget.setToolTip("")
    
    def onTextFieldReturnPressed(self, field_name, address):
        """Handle text field Enter key press event.
        
        Args:
            field_name (str): Name of the text field widget
            address (int): Servo memory address to write the value to
        """
        if not self.loaded:
            return
        
        print(f"DEBUG: Enter pressed in {field_name}")
        
        # Process the update
        success = self._process_text_field_update(field_name, address)
        
        # Only update the stored original value if the update was successful
        if success:
            field_widget = getattr(self, field_name)
            self.field_original_values[field_name] = field_widget.text()
            print(f"DEBUG: Updated original value for {field_name} to '{self.field_original_values[field_name]}'")
    
    def _process_text_field_update(self, field_name, address):
        """Process text field update after Enter key is pressed.
        
        Args:
            field_name (str): Name of the text field widget to process
            address (int): Servo memory address to write the validated value to
            
        Returns:
            bool: True if update was successful, False otherwise
        """
        # Get the text field widget by name
        field_widget = getattr(self, field_name)
        text = field_widget.text().strip()
        
        # Skip if empty
        if not text:
            return False
            
        try:
            # Convert text to integer
            value = int(text)
            
            # Get validation range for this field
            min_val, max_val = self.field_ranges.get(field_name, (0, 65535))
            
            # Validate range
            if value < min_val or value > max_val:
                print(f"Warning: {field_name} value {value} out of range ({min_val}-{max_val})")
                field_widget.setStyleSheet("background-color: #ffcccc;")
                field_widget.setToolTip(f"Value must be between {min_val} and {max_val}")
                return False
            else:
                # Clear any error styling and tooltip
                field_widget.setStyleSheet("")
                field_widget.setToolTip("")
                
            # Send the value to the servo
            success = self.servo_client.write_value(self.bus, self.id, address, value)
            if success:
                print(f"Updated {field_name} to {value} at address {address}")
                # Flash green to indicate success
                field_widget.setStyleSheet("background-color: #ccffcc;")
                QTimer.singleShot(1000, lambda: field_widget.setStyleSheet(""))
                return True
            else:
                print(f"Failed to update {field_name}")
                field_widget.setStyleSheet("background-color: #ffcccc;")
                field_widget.setToolTip("Failed to send to servo")
                QTimer.singleShot(2000, lambda: (field_widget.setStyleSheet("") or field_widget.setToolTip("")))
                return False
            
        except ValueError:
            print(f"Invalid value in {field_name}: {text}")
            field_widget.setStyleSheet("background-color: #ffcccc;")
            field_widget.setToolTip("Invalid number format")
            QTimer.singleShot(2000, lambda: (field_widget.setStyleSheet("") or field_widget.setToolTip("")))
            return False
        except Exception as e:
            print(f"Error updating {field_name}: {e}")
            return False
    
    def _set_field_text(self, field_name, value):
        """Set text field value and store it as the original value.
        
        Args:
            field_name (str): Name of the text field widget
            value: Value to set (will be converted to string)
        """
        field_widget = getattr(self, field_name)
        str_value = str(value)
        field_widget.setText(str_value)
        self.field_original_values[field_name] = str_value
    
    def _populate_fields(self):
        """Populate all UI fields with values from servo memory map."""
        print("Populate Fields")
        memorymap = self.servo_client.read_memory_map(self.bus, self.id)
        
        if memorymap is None:
            print("Failed to retrieve memory map")
            return
        
        # Read-only fields - use setText directly
        self.txtFirmwareMajor.setText(str(memorymap.firmwaremajor))
        self.txtFirmwareMinor.setText(str(memorymap.firmwaresub))
        self.txtServoMainVersion.setText(str(memorymap.servomajor))
        self.txtServoSubVersion.setText(str(memorymap.servosub))
        self.txtServoID.setText(str(memorymap.servoid))
        self.txtBaudrate.setText(str(memorymap.baudrate))
        self.txtLockMark.setText(str(memorymap.lockmark))
        
        # Editable fields - use setFieldText to store original values
        self._set_field_text('txtPositionCorrection', memorymap.positioncorrection)
        self._set_field_text('txtMinAngle', memorymap.minanglelimit)
        self._set_field_text('txtMaxAngle', memorymap.maxanglelimit)
        self._set_field_text('txtMaxTemperature', memorymap.maxtemplimit)
        self._set_field_text('txtMaxVoltage', memorymap.maxinputvoltage)
        self._set_field_text('txtMinVoltage', memorymap.mininputvoltage)
        self._set_field_text('txtMaxTorque', memorymap.maxtorque)
        self._set_field_text('txtAngularResolution', memorymap.angularresolution)
        self._set_field_text('txtProtectionCurrent', memorymap.protectioncurrent)
        self._set_field_text('txtClockwiseInsensitiveRegion', memorymap.clockwiseinsensitivearea)
        self._set_field_text('txtCounterClockwiseInsensitiveRegion', memorymap.counterclockwiseinsensitiveregion)
        self._set_field_text('txtMinimumStartForce', memorymap.minstartupforce)
        self._set_field_text('txtPCoefficient', memorymap.pcoefficient)
        self._set_field_text('txtICoefficient', memorymap.icoefficient)
        self._set_field_text('txtDCoefficient', memorymap.dcoefficient)
        self._set_field_text('txtProtectiveTorque', memorymap.protectivetorque)
        self._set_field_text('txtProtectionTime', memorymap.protectiontime)
        self._set_field_text('txtOverloadTorque', memorymap.overloadtorque)
        self._set_field_text('txtSpeedPCoefficient', memorymap.speedclosedlooppcoefficient)
        self._set_field_text('txtOvercurrentProtection', memorymap.overcurrentprotectiontime)
        self._set_field_text('txtVelocityICoefficient', memorymap.velocityclosedloopicoefficient)
        self._set_field_text('txtAcceleration', memorymap.acceleration)
        self._set_field_text('txtTargetLocation', memorymap.targetlocation)
        self._set_field_text('txtRunningTime', memorymap.runningtime)
        self._set_field_text('txtRunningSpeed', memorymap.runningspeed)
        self._set_field_text('txtTorqueLimit', memorymap.torquelimit)
        self._set_field_text('txtReturnDelay', memorymap.returndelay)
        
        self.txtResponseStatusLevel.setText(str(memorymap.responsestatuslevel))
        self.txtOperationMode.setText(str(memorymap.operationmode))
        self.txtLEDAlarm.setText(str(memorymap.ledalarmcondition))
        self.txtUnloadCondition.setText(str(memorymap.unloadingcondition))
        self.txtPhase.setText(str(memorymap.phase))
        
        # Set toggleLock button based on lockmark value
        if memorymap.lockmark == 0:
            self.toggleLockMark.setChecked(True)
            self.toggleLockMark.setText("EEPROM UNLocked")
        else:
            self.toggleLockMark.setChecked(False)
            self.toggleLockMark.setText("EEPROM Locked")
        
        # Set toggleTorqueSwitch button based on torqueswitch value
        if memorymap.torqueswitch == 1:
            self.toggleTorqueSwitch.setChecked(True)
            self.toggleTorqueSwitch.setText("Torque Enabled")
        else:
            self.toggleTorqueSwitch.setChecked(False)
            self.toggleTorqueSwitch.setText("Torque Disabled")
    
    def _disable_all_fields(self):
        """Disable all editable UI fields."""
        self.toggleTorqueSwitch.setDisabled(True)
        self.toggleLockMark.setDisabled(True)
        self.sliderTargetLocation.setDisabled(True)
        self.txtPositionCorrection.setDisabled(True)
        self.txtMinAngle.setDisabled(True)
        self.txtMaxAngle.setDisabled(True)
        self.txtMaxTemperature.setDisabled(True)
        self.txtMaxVoltage.setDisabled(True)
        self.txtMinVoltage.setDisabled(True)
        self.txtMaxTorque.setDisabled(True)
        self.txtAngularResolution.setDisabled(True)
        self.txtProtectionCurrent.setDisabled(True)
        self.txtCounterClockwiseInsensitiveRegion.setDisabled(True)
        self.txtClockwiseInsensitiveRegion.setDisabled(True)
        self.txtMinimumStartForce.setDisabled(True)
        self.txtPCoefficient.setDisabled(True)
        self.txtICoefficient.setDisabled(True)
        self.txtDCoefficient.setDisabled(True)
        self.txtProtectiveTorque.setDisabled(True)
        self.txtProtectionTime.setDisabled(True)
        self.txtOverloadTorque.setDisabled(True)
        self.txtSpeedPCoefficient.setDisabled(True)
        self.txtOvercurrentProtection.setDisabled(True)
        self.txtVelocityICoefficient.setDisabled(True)
        self.txtAcceleration.setDisabled(True)
        self.txtTargetLocation.setDisabled(True)
        self.txtRunningTime.setDisabled(True)
        self.txtRunningSpeed.setDisabled(True)
        self.txtTorqueLimit.setDisabled(True)
        self.txtReturnDelay.setDisabled(True)
        self.txtResponseStatusLevel.setDisabled(True)
        self.txtOperationMode.setDisabled(True)
        self.txtLEDAlarm.setDisabled(True)
        self.txtUnloadCondition.setDisabled(True)
        self.txtPhase.setDisabled(True)
    
    def _clear_all_fields(self):
        """Clear all editable UI text fields."""
        self.txtPositionCorrection.setText("")
        self.txtMinAngle.setText("")
        self.txtMaxAngle.setText("")
        self.txtMaxTemperature.setText("")
        self.txtMaxVoltage.setText("")
        self.txtMinVoltage.setText("")
        self.txtMaxTorque.setText("")
        self.txtAngularResolution.setText("")
        self.txtProtectionCurrent.setText("")
        self.txtCounterClockwiseInsensitiveRegion.setText("")
        self.txtClockwiseInsensitiveRegion.setText("")
        self.txtMinimumStartForce.setText("")
        self.txtPCoefficient.setText("")
        self.txtICoefficient.setText("")
        self.txtDCoefficient.setText("")
        self.txtProtectiveTorque.setText("")
        self.txtProtectionTime.setText("")
        self.txtOverloadTorque.setText("")
        self.txtSpeedPCoefficient.setText("")
        self.txtOvercurrentProtection.setText("")
        self.txtVelocityICoefficient.setText("")
        self.txtAcceleration.setText("")
        self.txtTargetLocation.setText("")
        self.txtRunningTime.setText("")
        self.txtRunningSpeed.setText("")
        self.txtTorqueLimit.setText("")
        self.txtReturnDelay.setText("")
        self.txtResponseStatusLevel.setText("")
        self.txtOperationMode.setText("")
        self.txtLEDAlarm.setText("")
        self.txtUnloadCondition.setText("")
        self.txtPhase.setText("")
    
    def _enable_all_fields(self):
        """Enable all editable UI fields."""
        self.toggleTorqueSwitch.setDisabled(False)
        self.toggleLockMark.setDisabled(False)
        self.sliderTargetLocation.setDisabled(False)
        self.txtPositionCorrection.setDisabled(False)
        self.txtMinAngle.setDisabled(False)
        self.txtMaxAngle.setDisabled(False)
        self.txtMaxTemperature.setDisabled(False)
        self.txtMaxVoltage.setDisabled(False)
        self.txtMinVoltage.setDisabled(False)
        self.txtMaxTorque.setDisabled(False)
        self.txtAngularResolution.setDisabled(False)
        self.txtProtectionCurrent.setDisabled(False)
        self.txtCounterClockwiseInsensitiveRegion.setDisabled(False)
        self.txtClockwiseInsensitiveRegion.setDisabled(False)
        self.txtMinimumStartForce.setDisabled(False)
        self.txtPCoefficient.setDisabled(False)
        self.txtICoefficient.setDisabled(False)
        self.txtDCoefficient.setDisabled(False)
        self.txtProtectiveTorque.setDisabled(False)
        self.txtProtectionTime.setDisabled(False)
        self.txtOverloadTorque.setDisabled(False)
        self.txtSpeedPCoefficient.setDisabled(False)
        self.txtOvercurrentProtection.setDisabled(False)
        self.txtVelocityICoefficient.setDisabled(False)
        self.txtAcceleration.setDisabled(False)
        self.txtTargetLocation.setDisabled(False)
        self.txtRunningTime.setDisabled(False)
        self.txtRunningSpeed.setDisabled(False)
        self.txtTorqueLimit.setDisabled(False)
        self.txtReturnDelay.setDisabled(False)
        self.txtResponseStatusLevel.setDisabled(False)
        self.txtOperationMode.setDisabled(False)
        self.txtLEDAlarm.setDisabled(False)
        self.txtUnloadCondition.setDisabled(False)
        self.txtPhase.setDisabled(False)
