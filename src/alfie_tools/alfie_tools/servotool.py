"""
Servo Configuration Tool for Alfie Robot.

This module provides a PyQt5-based GUI application for configuring and monitoring
servo motors in the Alfie humanoid robot system. It enables real-time parameter
adjustment, live status monitoring, and servo memory map manipulation through
ROS2 service calls.

Features:
    - Real-time servo status monitoring via /robotlowstate topic
    - Interactive parameter editing with automatic validation
    - Debounced text field updates to prevent command flooding
    - Visual feedback for successful/failed operations
    - Support for dual servo bus architecture (driver0 and driver1)
    - Comprehensive servo parameter coverage (PID, limits, protection, etc.)

Usage:
    ros2 run alfie_tools servotool

Author: Alan's Robot Lab
Date: 2025-10-15
"""

import sys
import os
from pathlib import Path
from PyQt5 import QtWidgets
from PyQt5.QtCore import QSize, QTimer
from PyQt5 import uic
import qdarktheme
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from alfie_msgs.msg import RobotLowState
from alfie_msgs.msg import ServoMemoryMap
from alfie_msgs.srv import ServoService


class ServoTool(QtWidgets.QMainWindow):
    """
    Servo configuration and monitoring tool for Alfie robot.
    
    This class provides a PyQt5-based GUI application for configuring and monitoring
    servo motors in the Alfie robot system. It supports real-time parameter adjustment,
    live status monitoring, and servo memory map manipulation via ROS2 services.
    
    Attributes:
        loaded (bool): Flag indicating UI has completed initialization
        field_timers (dict): QTimer objects for debouncing text field updates
        field_ranges (dict): Valid value ranges for each editable field
        bus (int): Current servo bus number (0 or 1)
        id (int): Current servo ID (1-10)
        node (rclpy.node.Node): ROS2 node for communication
        robot_low_state_sub: Subscription to /robotlowstate topic
        driver0service: Service client for driver0 servo bus
        driver1service: Service client for driver1 servo bus
        servicerequest: Reusable service request object
    """

    def __init__(self, *args, **kwargs):
        """
        Initialize the ServoTool main window.
        
        Loads the UI from file, sets up ROS2 connections, configures field validation
        ranges, and connects UI signals to handlers. Also populates the servo dropdown
        with available servos.
        
        Args:
            *args: Variable length argument list passed to QMainWindow
            **kwargs: Arbitrary keyword arguments passed to QMainWindow
        """
        self.loaded = False
        super().__init__(*args, **kwargs)
        
        # Store original values for each field to revert on focus loss without Enter
        self.field_original_values = {}
        
        # Define validation ranges for specific fields
        self.field_ranges = {
            'txtPositionCorrection': (-2047, 2047),             # verified
            'txtMinAngle': (-32766, 32767),                     # verified
            'txtMaxAngle': (-32766, 32767),                     # verified
            'txtMaxTemperature': (0, 100),                      # verified
            'txtMaxVoltage': (0, 254),                          # verified
            'txtMinVoltage': (0, 254),                          # verified
            'txtMaxTorque': (0, 1000),                          # verified
            'txtAngularResolution': (1, 3),                     # verified
            'txtProtectionCurrent': (0, 511),                   # verified
            'txtClockwiseInsensitiveRegion': (0, 32),           # verified
            'txtCounterClockwiseInsensitiveRegion': (0, 32),    # verified
            'txtMinimumStartForce': (0, 1000),                  # verified
            'txtPCoefficient': (0, 254),                        # verified
            'txtICoefficient': (0, 254),                        # verified
            'txtDCoefficient': (0, 254),                        # verified
            'txtProtectiveTorque': (0, 100),                    # verified
            'txtProtectionTime': (0, 254),                      # verified
            'txtOverloadTorque': (0, 100),                      # verified
            'txtSpeedPCoefficient': (0, 100),                   # verified
            'txtOvercurrentProtection': (0, 254),               # verified
            'txtVelocityICoefficient': (0, 254),                # verified
            'txtAcceleration': (0, 100),                        # verified
            'txtTargetLocation': (-30719, 30719),               # verified
            'txtRunningTime': (0, 1000),                        # verified
            'txtRunningSpeed': (0, 3400),                       # verified  
            'txtTorqueLimit': (0, 1000),                        # verified
            'txtReturnDelay': (0, 254)                          # verified
        }
        
        # Get the directory where this script is located
        script_dir = Path(__file__).parent
        ui_file = script_dir / "servotool.ui"
        
        uic.loadUi(str(ui_file), self)
        self.setFixedSize(QSize(770, 700))
        
        # Configure slider range for target location (0 to 4096)
        self.sliderTargetLocation.setMinimum(0)
        self.sliderTargetLocation.setMaximum(4096)
        
        self.connectUI()
        self.populateServoDropdown()
        self.setupROS()
        self.disableAllFields()
        self.clearAllFields()
        self.enableAllFields()
        #self.txtFirmwareMajor.setText("Hello World")
        #qdarktheme.setup_theme()
        self.loaded = True
        self.onServoSelected()

        
    def setupROS(self):
        """
        Initialize ROS2 node and create service clients.
        
        Creates a ROS2 node named 'servotool', subscribes to the /robotlowstate topic
        for real-time servo status updates, and establishes service clients for
        communicating with both servo driver buses (driver0 and driver1). Waits for
        both services to become available before proceeding.
        """
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        rclpy.init()
        self.node = rclpy.create_node("servotool")

        # Create a subscription to the robot low state topic
        self.robot_low_state_sub = self.node.create_subscription(
            RobotLowState, 
            "/robotlowstate", 
            self.robotLowStateCallback, 
            qos
            )
        
        self.driver0service = self.node.create_client(ServoService, "/driver0servoservice")
        while not self.driver0service.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                "Driver0 Service not available, waiting again...")
            print("Driver0 Service not available, waiting again...")
            
        self.driver1service = self.node.create_client(ServoService, "/driver1servoservice")
        while not self.driver1service.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                "Driver1 Service not available, waiting again...")
            print("Driver1 Service not available, waiting again...")

        self.servicerequest = ServoService.Request()


    def robotLowStateCallback(self, msg):
        """
        Callback for real-time robot low state updates.
        
        Updates the UI status fields with current servo readings including position,
        speed, voltage, current, temperature, and other real-time parameters. The servo
        index is calculated based on bus number and servo ID.
        
        Args:
            msg (RobotLowState): Message containing current state of all servos
        
        Note:
            Bus 0: servos 1-10 (indices 0-9)
            Bus 1: servos 11-17 (indices 10-16)
        """
        # 
        if self.bus == 1: # left arm, first in combined array
            servo_index = self.id - 1
        else:
            servo_index = (7 + (self.id - 1))

        #print("Received RobotLowState for Bus ", self.bus, " Servo ", self.id, " Index ", servo_index)
        
        if servo_index < len(msg.servo_state):
            # Note: servo_loop_ms is not available in RobotLowState
            # self.txtStatusServoLoopPeriod.setText(str((msg.servo_loop_ms / 10.0)))
            self.txtStatusTorqueSwitch.setText(str(msg.servo_state[servo_index].torqueswitch))
            self.txtStatusTargetLocation.setText(str(msg.servo_state[servo_index].targetlocation))
            self.txtStatusAcceleration.setText(str(msg.servo_state[servo_index].acceleration))
            self.txtStatusCurrentLocation.setText(str(msg.servo_state[servo_index].currentlocation))
            self.txtStatusCurrentSpeed.setText(str(msg.servo_state[servo_index].currentspeed))
            self.txtStatusCurrentVoltage.setText(str((msg.servo_state[servo_index].currentvoltage / 10.0)))
            self.txtStatusCurrentCurrent.setText(str(msg.servo_state[servo_index].currentcurrent * 6.5))
            self.txtStatusCurrentTemperature.setText(str(msg.servo_state[servo_index].currenttemperature))
            self.txtStatusServoStatus.setText(str(msg.servo_state[servo_index].servostatus))
            self.txtStatusMobileSign.setText(str(msg.servo_state[servo_index].mobilesign))

            # Update slider position from current location only when torque is disabled (unchecked)
            if not self.toggleTorqueSwitch.isChecked():
                self.sliderTargetLocation.setValue(msg.servo_state[servo_index].currentlocation)

            if (self.toggleTorqueSwitch.isChecked() and int(self.txtStatusTargetLocation.text()) != int(self.sliderTargetLocation.value())):
                print("Bus ", self.bus, " New Slider Value: ", self.sliderTargetLocation.value(), ": TargetLocation Value: ", self.txtStatusTargetLocation.text())
                # service update position
                # results in ~ 100hz update rate


    def print_memory_map(self, memory_map):
        """
        Print servo memory map in human-readable format.
        
        Formats and prints key servo information including firmware version,
        servo version, and servo ID to the console for debugging purposes.
        
        Args:
            memory_map (ServoMemoryMap): Complete servo memory map containing all parameters
        """
        print("\n" + "="*60)
        print("SERVO MEMORY MAP")
        print("="*60)
        
        # Firmware information
        print(f"Firmware Version: {memory_map.firmwaremajor}.{memory_map.firmwaresub}")
        print(f"Servo Version: {memory_map.servomajor}.{memory_map.servosub}")
        print(f"Servo ID: {memory_map.servoid}")  


    def retrieveMemoryMap(self, driverboard, servo):
        """
        Retrieve complete memory map from servo.
        
        Sends a read request to the servo at address 0 to retrieve the complete
        memory map containing all configuration and status parameters. Blocks until
        the service responds or times out after 5 seconds.
        
        Args:
            bus (int): Servo bus number (0 or 1)
            
        Returns:
            ServoMemoryMap: Complete servo memory map object, or -1 on timeout
        """
        print("Retrieve Value for Driverboard ", driverboard, " Servo ", servo)
        self.servicerequest.servo = servo
        self.servicerequest.operation = ord('r')
        self.servicerequest.address = 0
        self.servicerequest.value = 0

        # pick the correct client : weird ternary operator
        if driverboard == 0:
            client = self.driver0service
        else:
            client = self.driver1service

        # make the async call
        future = client.call_async(self.servicerequest)
        # block here until the service responds (or times out)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        if future.done():
            resp = future.result()
            print("Memory Map Retrieved:")
            print(resp.memorymap)
            self.print_memory_map(resp.memorymap)
            return resp.memorymap
        else:
            return -1


    def setFieldText(self, field_name, value):
        """
        Set text field value and store it as the original value.
        
        This helper ensures that when we programmatically set field values
        (e.g., when populating from servo), we also store them as the "original"
        values for revert-on-focus-loss functionality.
        
        Args:
            field_name (str): Name of the text field widget
            value: Value to set (will be converted to string)
        """
        field_widget = getattr(self, field_name)
        str_value = str(value)
        field_widget.setText(str_value)
        self.field_original_values[field_name] = str_value

    def populateFields(self):
        """
        Populate all UI fields with values from servo memory map.
        
        Retrieves the complete memory map from the currently selected servo
        and updates all UI text fields with the corresponding values including
        firmware version, servo configuration, PID parameters, limits, and
        operational settings.
        """
        print("Populate Fields")
        memorymap = self.retrieveMemoryMap(self.bus, self.id)
        # Read-only fields - use setText directly
        self.txtFirmwareMajor.setText(str(memorymap.firmwaremajor))
        self.txtFirmwareMinor.setText(str(memorymap.firmwaresub))
        self.txtServoMainVersion.setText(str(memorymap.servomajor))
        self.txtServoSubVersion.setText(str(memorymap.servosub))
        self.txtServoID.setText(str(memorymap.servoid))
        self.txtBaudrate.setText(str(memorymap.baudrate))
        # Editable fields - use setFieldText to store original values
        self.setFieldText('txtPositionCorrection', memorymap.positioncorrection)
        self.setFieldText('txtMinAngle', memorymap.minanglelimit)
        self.setFieldText('txtMaxAngle', memorymap.maxanglelimit)
        self.setFieldText('txtMaxTemperature', memorymap.maxtemplimit)
        self.setFieldText('txtMaxVoltage', memorymap.maxinputvoltage)
        self.setFieldText('txtMinVoltage', memorymap.mininputvoltage)
        self.setFieldText('txtMaxTorque', memorymap.maxtorque)
        self.setFieldText('txtAngularResolution', memorymap.angularresolution)
        self.setFieldText('txtProtectionCurrent', memorymap.protectioncurrent)
        self.setFieldText('txtClockwiseInsensitiveRegion', memorymap.clockwiseinsensitivearea)
        self.setFieldText('txtCounterClockwiseInsensitiveRegion', memorymap.counterclockwiseinsensitiveregion)
        self.setFieldText('txtMinimumStartForce', memorymap.minstartupforce)
        self.setFieldText('txtPCoefficient', memorymap.pcoefficient)
        self.setFieldText('txtICoefficient', memorymap.icoefficient)
        self.setFieldText('txtDCoefficient', memorymap.dcoefficient)
        self.setFieldText('txtProtectiveTorque', memorymap.protectivetorque)
        self.setFieldText('txtProtectionTime', memorymap.protectiontime)
        self.setFieldText('txtOverloadTorque', memorymap.overloadtorque)
        self.setFieldText('txtSpeedPCoefficient', memorymap.speedclosedlooppcoefficient)
        self.setFieldText('txtOvercurrentProtection', memorymap.overcurrentprotectiontime)
        self.setFieldText('txtVelocityICoefficient', memorymap.velocityclosedloopicoefficient)
        self.setFieldText('txtAcceleration', memorymap.acceleration)
        self.setFieldText('txtTargetLocation', memorymap.targetlocation)
        self.setFieldText('txtRunningTime', memorymap.runningtime)
        self.setFieldText('txtRunningSpeed', memorymap.runningspeed)
        self.setFieldText('txtTorqueLimit', memorymap.torquelimit)
        self.setFieldText('txtReturnDelay', memorymap.returndelay)
        self.txtResponseStatusLevel.setText(str(memorymap.responsestatuslevel))
        self.txtOperationMode.setText(str(memorymap.operationmode))
        self.txtLEDAlarm.setText(str(memorymap.ledalarmcondition))
        self.txtUnloadCondition.setText(str(memorymap.unloadingcondition))
        self.txtPhase.setText(str(memorymap.phase))
        
        # Set toggleLock button based on lockmark value
        # lockmark == 0 means EEPROM is locked (button should be checked)
        # lockmark == 1 means EEPROM is unlocked (button should be unchecked)
        if memorymap.lockmark == 0:
            self.toggleLock.setChecked(True)
            self.toggleLock.setText("EEPROM UNLocked")
        else:
            self.toggleLock.setChecked(False)
            self.toggleLock.setText("EEPROM Locked")
        
        # Set toggleTorqueSwitch button based on torqueswitch value
        # torqueswitch == 1 means torque is enabled (button should be checked)
        # torqueswitch == 0 means torque is disabled (button should be unchecked)
        if memorymap.torqueswitch == 1:
            self.toggleTorqueSwitch.setChecked(True)
            self.toggleTorqueSwitch.setText("Torque Enabled")
        else:
            self.toggleTorqueSwitch.setChecked(False)
            self.toggleTorqueSwitch.setText("Torque Disabled")


    

    def populateServoDropdown(self):
        """
        Populate the servo selection dropdown with available servos.
        
        Adds all available servos to the dropdown menu, organized by driver bus
        and servo function (left arm, right arm, head). Each item stores the
        bus number and servo ID as associated data.
        
        Sets default selection to driver0, servo ID 1.
        """
        self.cmbServos.addItem("driver1/servo01 - left shoulder yaw",[1,1])
        self.cmbServos.addItem("driver1/servo02 - left shoulder1 pitch",[1,2])
        self.cmbServos.addItem("driver1/servo03 - left shoulder2 pitch",[1,3])
        self.cmbServos.addItem("driver1/servo04 - left elbow pitch",[1,4])
        self.cmbServos.addItem("driver1/servo05 - left wrist pitch",[1,5])
        self.cmbServos.addItem("driver1/servo06 - left wrist roll",[1,6])
        self.cmbServos.addItem("driver1/servo07 - left hand",[1,7])

        self.cmbServos.addItem("driver0/servo01 - right shoulder yaw",[0,1])
        self.cmbServos.addItem("driver0/servo02 - right shoulder1 pitch",[0,2])
        self.cmbServos.addItem("driver0/servo03 - right shoulder2 pitch",[0,3])
        self.cmbServos.addItem("driver0/servo04 - right elbow pitch",[0,4])
        self.cmbServos.addItem("driver0/servo05 - right wrist pitch",[0,5])
        self.cmbServos.addItem("driver0/servo06 - right wrist roll",[0,6])
        self.cmbServos.addItem("driver0/servo07 - right hand",[0,7])

        self.cmbServos.addItem("driver0/servo08 - head yaw",[0,8])
        self.cmbServos.addItem("driver0/servo09 - head pitch",[0,9])
        self.cmbServos.addItem("driver0/servo10 - head roll",[0,10])

        self.bus = 0
        self.id = 1


    def onServoSelected(self):
        """
        Handle servo selection change event.
        
        Called when user selects a different servo from the dropdown. Updates
        the current bus and ID, then refreshes all UI fields by disabling them,
        clearing old values, fetching new parameters from the selected servo,
        and re-enabling the fields.
        
        Only processes if UI has finished loading (self.loaded == True).
        """
        if(self.loaded == True):
            currentIndex = self.cmbServos.currentIndex()
            if currentIndex >= 0:
                self.bus, self.id = self.cmbServos.itemData(currentIndex)
                print(f"Selected Servo: Bus {self.bus}, ID {self.id}")
            else:
                print("No servo selected")

            # pull in data for selected servo from bus
            self.disableAllFields()
            self.clearAllFields()
            self.populateFields()
            self.enableAllFields()
            # set torque switch

            # set slider


    def onSliderValueChanged(self):
        """
        Handle slider value change event.
        
        Called when user moves the target location slider. Updates the target
        location text field to match the slider value. If torque is enabled
        (toggleTorqueSwitch is checked), sends a service call to update the
        servo's target position (address 42). If torque is disabled, the slider
        value is only reflected in the UI but not sent to the servo.
        
        Only sends commands if UI has finished loading (self.loaded == True).
        """
        value = self.sliderTargetLocation.value()
        self.txtTargetLocation.setText(str(value))
        
        # Only send command to servo if loaded and torque is enabled
        if self.loaded and self.toggleTorqueSwitch.isChecked():
            print(f"Slider Value: {value} - sending to servo")
            
            # Write the target position to address 42
            success = self.writeValueToServo(self.bus, self.id, 42, value)
            
            if success:
                # Update the stored original value for the target location field
                self.field_original_values['txtTargetLocation'] = str(value)
            else:
                print(f"Failed to set target position to {value}")
        else:
            if self.loaded:
                print(f"Slider Value: {value} - torque disabled, not sending to servo")


    def onToggleLockClicked(self):
        """
        Handle toggleLock button click event.
        
        Called when user clicks the EEPROM lock toggle button. Writes the lockmark
        value to the servo based on the button's checked state:
        - Checked (locked): writes 0 to address 55
        - Unchecked (unlocked): writes 1 to address 55
        
        Also updates the button text to reflect the current lock status.
        
        Only processes if UI has finished loading (self.loaded == True).
        """
        if not self.loaded:
            return
        
        # Determine the value based on checked state
        # Checked = locked = 0, Unchecked = unlocked = 1
        if self.toggleLock.isChecked():
            lockmark_value = 0
            button_text = "EEPROM Locked"
        else:
            lockmark_value = 1
            button_text = "EEPROM Unlocked"
        
        print(f"Toggle Lock clicked: setting lockmark to {lockmark_value}")
        
        # Write the value to servo at address 55
        success = self.writeValueToServo(self.bus, self.id, 55, lockmark_value)
        
        if success:
            self.toggleLock.setText(button_text)
            print(f"Successfully set lockmark to {lockmark_value}")
        else:
            print(f"Failed to set lockmark to {lockmark_value}")
            # Revert the button state if write failed
            self.toggleLock.setChecked(not self.toggleLock.isChecked())


    def onToggleTorqueSwitchClicked(self):
        """
        Handle toggleTorqueSwitch button click event.
        
        Called when user clicks the torque enable/disable toggle button. Behavior depends
        on the checked state:
        - Checking (enabling): Sets target position to current location (address 42)
          This enables torque by commanding the servo to hold its current position
        - Unchecking (disabling): Writes 0 to torqueswitch (address 40) to disable torque
        
        Also updates the button text to reflect the current torque status.
        
        Only processes if UI has finished loading (self.loaded == True).
        """
        if not self.loaded:
            return
        
        success = False
        
        if self.toggleTorqueSwitch.isChecked():
            # Enabling torque: set target position to current location
            button_text = "Torque Enabled"
            
            try:
                # Get current location from status field
                current_location_text = self.txtStatusCurrentLocation.text()
                if not current_location_text:
                    print("Error: Current location is empty")
                    self.toggleTorqueSwitch.setChecked(False)
                    return
                
                current_location = int(current_location_text)
                
                print(f"Toggle Torque Switch clicked: enabling torque by setting target position to {current_location}")
                
                # Write current location to target position (address 42)
                success = self.writeValueToServo(self.bus, self.id, 42, current_location)
                
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
            success = self.writeValueToServo(self.bus, self.id, 40, 0)
            
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
        """
        Handle Zero Servo button click event.
        
        Called when user clicks the Zero Servo button. This function:
        1. Reads the current location from txtStatusCurrentLocation
        2. Calculates the difference between current location and 2048 (center position)
        3. Writes that difference to the position correction address (31)
        
        The position correction value adjusts the servo's zero point so that the
        current position becomes the new center/zero position.
        
        Only processes if UI has finished loading (self.loaded == True).
        """
        if not self.loaded:
            return
        
        try:
            # Get the current location from the status field
            current_location_text = self.txtStatusCurrentLocation.text()
            current_offset_text = self.txtPositionCorrection.text()
            if not current_location_text:
                print("Error: Current location is empty")
                return
            
            current_location = int(current_location_text)
            current_offset = int(current_offset_text) if current_offset_text else 0
            
            # Calculate the position correction needed to zero the servo
            # Center position is 2048, so correction = 2048 - current_location
            target_center = 2048
            position_correction = (target_center - (current_location - current_offset))
            
            # Validate the correction is within range (-2047 to 2047)
            if position_correction < -2047 or position_correction > 2047:
                print(f"Error: Calculated position correction {position_correction} is out of range (-2047 to 2047)")
                return
            
            print(f"Zero Servo clicked: current_location={current_location}, calculated correction={position_correction}")
            
            # Write the position correction value to address 31
            success = self.writeValueToServo(self.bus, self.id, 31, position_correction)
            
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


    def onTextFieldFocusOut(self, field_name):
        """
        Handle text field losing focus event.
        
        If the user clicks away without pressing Enter, revert to the original value.
        This is called by editingFinished signal which fires both on Enter and focus loss,
        so we check if the value matches the stored original. If Enter was pressed,
        the original value would have been updated already.
        
        Args:
            field_name (str): Name of the text field widget
        """
        if not self.loaded:
            return
        
        field_widget = getattr(self, field_name)
        current_value = field_widget.text()
        original_value = self.field_original_values.get(field_name, "")
        
        # If the value changed from original, revert it
        # (If Enter was pressed, original would have been updated to match current)
        if current_value != original_value:
            print(f"DEBUG: Reverting {field_name} from '{current_value}' to '{original_value}'")
            field_widget.setText(original_value)
            field_widget.setStyleSheet("")  # Clear any styling
            field_widget.setToolTip("")  # Clear any tooltips

    def onTextFieldReturnPressed(self, field_name, address):
        """
        Handle text field Enter key press event.
        
        Called when the user presses Enter in an editable text field. Validates
        and submits the new value to the servo. Updates the stored original value
        so that subsequent editingFinished (focus loss) won't revert the change.
        
        Args:
            field_name (str): Name of the text field widget (e.g., 'txtPCoefficient')
            address (int): Servo memory address to write the value to
            
        Note:
            Only processes changes after UI has finished loading (self.loaded == True)
        """
        if not self.loaded:
            return  # Don't process changes during initial loading

        print(f"DEBUG: Enter pressed in {field_name}")
        
        # Process the update
        success = self.processTextFieldUpdate(field_name, address)
        
        # Only update the stored original value if the update was successful
        # This way, if validation fails, focus loss will still revert to the good value
        if success:
            field_widget = getattr(self, field_name)
            self.field_original_values[field_name] = field_widget.text()
            print(f"DEBUG: Updated original value for {field_name} to '{self.field_original_values[field_name]}'")
        """
        # Cancel any existing timer for this field
        if field_name in self.field_timers:
            self.field_timers[field_name].stop()
        
        # Create a new timer that will trigger the update after a delay
        timer = QTimer()
        timer.setSingleShot(True)
        timer.timeout.connect(lambda: self.processTextFieldUpdate(field_name, address))
        timer.start(1000)  # 500ms delay
        
        self.field_timers[field_name] = timer
        """

    def processTextFieldUpdate(self, field_name, address):
        """
        Process text field update after Enter key is pressed.
        
        Validates the input value against the field's allowed range, sends the value 
        to the servo if valid, and provides visual feedback (green for success, red 
        for errors) via background color changes.
        
        Args:
            field_name (str): Name of the text field widget to process
            address (int): Servo memory address to write the validated value to
            
        Returns:
            bool: True if update was successful, False otherwise
            
        Visual Feedback:
            - Green background flash (1s): Value successfully sent to servo
            - Red background (2s): Invalid value or transmission failure
            - Tooltip shows error details for invalid values
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
                # Set background color to indicate error
                field_widget.setStyleSheet("background-color: #ffcccc;")
                field_widget.setToolTip(f"Value must be between {min_val} and {max_val}")
                return False
            else:
                # Clear any error styling and tooltip
                field_widget.setStyleSheet("")
                field_widget.setToolTip("")
                
            # Send the value to the servo
            success = self.writeValueToServo(self.bus, self.id, address, value)
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
                QTimer.singleShot(2000, lambda: (field_widget.setStyleSheet(""), field_widget.setToolTip("")))
                return False
            
        except ValueError:
            print(f"Invalid value in {field_name}: {text}")
            field_widget.setStyleSheet("background-color: #ffcccc;")
            field_widget.setToolTip("Invalid number format")
            QTimer.singleShot(2000, lambda: (field_widget.setStyleSheet(""), field_widget.setToolTip("")))
            return False
        except Exception as e:
            print(f"Error updating {field_name}: {e}")
            return False


    def writeValueToServo(self, bus, servo_id, address, value):
        """
        Write a value to a specific memory address on the servo.
        
        Sends a write command to the specified servo via ROS2 service. The service
        call is synchronous with a 1-second timeout to prevent UI freezing.
        
        Args:
            bus (int): Servo bus number (0 or 1)
            servo_id (int): Servo ID (1-10)
            address (int): Memory address to write to
            value (int): Value to write to the address
            
        Returns:
            bool: True if the service call completed successfully, False on timeout or error
            
        Note:
            Success is determined by service call completion, not by servo acknowledgment.
            The response contains a memorymap but no explicit success/failure indicator.
        """
        try:
            print(f"DEBUG: writeValueToServo called - bus={bus}, servo={servo_id}, addr={address}, value={value}")
            self.servicerequest.servo = servo_id
            self.servicerequest.operation = ord('W')  # write operation
            self.servicerequest.address = address
            self.servicerequest.value = value

            # Pick the correct client based on bus
            client = self.driver0service if bus == 0 else self.driver1service
            print(f"DEBUG: Using {'driver0service' if bus == 0 else 'driver1service'}")

            # Make the async call with timeout
            print(f"DEBUG: Calling service asynchronously...")
            future = client.call_async(self.servicerequest)
            print(f"DEBUG: Spinning until complete (timeout=1.0s)...")
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            
            if future.done():
                print(f"DEBUG: Future completed successfully")
                resp = future.result()
                print(f"DEBUG: Response received: {resp}")
                # For write operations, success is indicated by the service completing
                # The response contains a memorymap, but we just check if the call succeeded
                return True
            else:
                print(f"DEBUG: Timeout writing to servo {servo_id} at address {address}")
                return False
                
        except Exception as e:
            print(f"DEBUG: Exception writing to servo: {e}")
            import traceback
            traceback.print_exc()
            return False


    def connectUI(self):
        """
        Connect UI widget signals to their respective handler methods.
        
        Establishes signal-slot connections for all interactive UI elements including
        the servo selection dropdown, target location slider, and all editable text
        fields. Each text field is connected to returnPressed (Enter key) for submission
        and editingFinished (focus loss) for reverting unsaved changes.
        
        Note:
            Status fields (txtStatus*) are read-only and not connected to handlers.
        """
        self.cmbServos.currentIndexChanged.connect(self.onServoSelected)
        self.sliderTargetLocation.valueChanged.connect(self.onSliderValueChanged)
        self.toggleLock.clicked.connect(self.onToggleLockClicked)
        self.toggleTorqueSwitch.clicked.connect(self.onToggleTorqueSwitchClicked)
        self.btnZeroServo.clicked.connect(self.onBtnZeroServoClicked)
        
        # Connect all editable text fields to returnPressed and focus events
        # returnPressed: Submit value when Enter is pressed
        # editingFinished: Revert to original value if Enter wasn't pressed
        # Note: Status fields (txtStatus*) are read-only and should not be connected
        
        # Helper function to connect both signals for each field
        def connect_field(field_name, address):
            field_widget = getattr(self, field_name)
            field_widget.returnPressed.connect(lambda: self.onTextFieldReturnPressed(field_name, address))
            field_widget.editingFinished.connect(lambda: self.onTextFieldFocusOut(field_name))

        connect_field('txtReturnDelay', 7)
        connect_field('txtMinAngle', 9)
        connect_field('txtMaxAngle', 11)
        connect_field('txtMaxTemperature', 13)
        connect_field('txtMaxVoltage', 14)
        connect_field('txtMinVoltage', 15)
        connect_field('txtMaxTorque', 16)
        #connect_field('txtPhase', 18)
        #connect_field('txtUnloadCondition', 19)
        #connect_field('txtLEDAlarm', 20)
        connect_field('txtPCoefficient', 21)
        connect_field('txtDCoefficient', 22)
        connect_field('txtICoefficient', 23)
        connect_field('txtMinimumStartForce', 24)
        connect_field('txtClockwiseInsensitiveRegion', 26)
        connect_field('txtCounterClockwiseInsensitiveRegion', 27)
        connect_field('txtProtectionCurrent', 28)
        connect_field('txtAngularResolution', 30)
        connect_field('txtPositionCorrection', 31)
        connect_field('txtOperationMode', 33)
        connect_field('txtProtectiveTorque', 34)
        connect_field('txtProtectionTime', 35)
        connect_field('txtOverloadTorque', 36)
        connect_field('txtSpeedPCoefficient', 37)
        connect_field('txtOvercurrentProtection', 38)
        connect_field('txtVelocityICoefficient', 39)
        #torqueswitch, 40
        connect_field('txtAcceleration', 41)
        connect_field('txtTargetLocation', 42)
        connect_field('txtRunningTime', 44)
        connect_field('txtRunningSpeed', 46)
        connect_field('txtTorqueLimit', 48)
        #lockflag, 55
        


    def disableAllFields(self):
            """
            Disable all editable UI fields.
            
            Sets all text fields, toggles, and sliders to disabled state to prevent
            user input. Typically called when switching servos or during initialization.
            """
            self.toggleTorqueSwitch.setDisabled(True)
            self.toggleLock.setDisabled(True)
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


    def clearAllFields(self):
        """
        Clear all editable UI text fields.
        
        Sets all text fields to empty strings. Typically called when switching
        servos or resetting the UI state.
        """
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


    def enableAllFields(self):
        """
        Enable all editable UI fields.
        
        Sets all text fields, toggles, and sliders to enabled state to allow
        user input. Typically called after loading servo parameters.
        """
        self.toggleTorqueSwitch.setDisabled(False)
        self.toggleLock.setDisabled(False)
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




def main(args=None):
    """
    Main entry point for the ServoTool application.
    
    Creates the Qt application, applies dark theme styling, instantiates the
    ServoTool window, and starts both the Qt event loop and a background thread
    for ROS2 communication. Handles cleanup on application exit.
    
    Args:
        args: Command line arguments (currently unused)
        
    The function:
        1. Creates QApplication instance
        2. Applies qdarktheme dark mode styling
        3. Creates and shows ServoTool window
        4. Starts ROS2 spin in background daemon thread
        5. Runs Qt event loop
        6. Performs cleanup on exit
    """
    #rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)

    # apply dark theme
    palette    = qdarktheme.load_palette(theme="dark")
    stylesheet = qdarktheme.load_stylesheet(theme="dark")
    app.setPalette(palette)
    app.setStyleSheet(stylesheet)

    # create your window (which also created `self.node` internally)
    window = ServoTool()
    window.show()

    # spin ROS in a background thread
    ros_thread = threading.Thread(
        target=lambda: rclpy.spin(window.node),
        daemon=True
    )
    ros_thread.start()

    # now start the Qt event loop
    exit_code = app.exec()

    # cleanup
    window.node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == "__main__":
    main()