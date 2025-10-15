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


    def __init__(self, *args, **kwargs):
        self.loaded = False
        super().__init__(*args, **kwargs)
        
        # Initialize text field update timers for debouncing
        self.field_timers = {}
        
        # Define validation ranges for specific fields
        self.field_ranges = {
            'txtPositionCorrection': (-4096, 4096),
            'txtMinAngle': (0, 1000),
            'txtMaxAngle': (0, 1000),
            'txtMaxTemperature': (50, 100),
            'txtMaxVoltage': (60, 140),
            'txtMinVoltage': (60, 140),
            'txtMaxTorque': (0, 1000),
            'txtAngularResolution': (1, 30),
            'txtProtectionCurrent': (50, 300),
            'txtClockwiseInsensitiveRegion': (0, 32),
            'txtCounterClockwiseInsensitiveRegion': (0, 32),
            'txtMinimumStartForce': (0, 100),
            'txtPCoefficient': (0, 254),
            'txtICoefficient': (0, 254),
            'txtDCoefficient': (0, 254),
            'txtProtectiveTorque': (50, 1000),
            'txtProtectionTime': (1, 50),
            'txtOverloadTorque': (0, 1000),
            'txtSpeedPCoefficient': (0, 254),
            'txtOvercurrentProtection': (1, 50),
            'txtVelocityICoefficient': (0, 254),
            'txtAcceleration': (0, 254),
            'txtTargetLocation': (0, 1000),
            'txtRunningTime': (0, 30000),
            'txtRunningSpeed': (0, 1500),
            'txtTorqueLimit': (10, 1000),
            'txtReturnDelay': (0, 254)
        }
        
        # Get the directory where this script is located
        script_dir = Path(__file__).parent
        ui_file = script_dir / "servotool.ui"
        
        uic.loadUi(str(ui_file), self)
        self.setFixedSize(QSize(770, 700))
        
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
        # Initialize the ROS node

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
        #print("RobotLowState Callback")
        # The RobotLowState message contains servo_state array with 17 servos
        # We calculate the servo index based on bus and id
        # Bus 0: servos 0-9 (indices 0-9)
        # Bus 1: servos 10-16 (indices 10-16)
        servo_index = (self.bus * 8) + (self.id - 1)
        
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

            #self.sliderTargetLocation.setValue(msg.servo_state[servo_index].currentlocation)

            if (self.toggleTorqueSwitch.isChecked() and int(self.txtStatusTargetLocation.text()) != int(self.sliderTargetLocation.value())):
                print("Bus ", self.bus, " New Slider Value: ", self.sliderTargetLocation.value(), ": TargetLocation Value: ", self.txtStatusTargetLocation.text())
                # service update position
                # results in ~ 100hz update rate


    def print_memory_map(self, memory_map):
        """
        Print the servo memory map in a readable format
        """
        print("\n" + "="*60)
        print("SERVO MEMORY MAP")
        print("="*60)
        
        # Firmware information
        print(f"Firmware Version: {memory_map.firmwaremajor}.{memory_map.firmwaresub}")
        print(f"Servo Version: {memory_map.servomajor}.{memory_map.servosub}")
        print(f"Servo ID: {memory_map.servoid}")  

    def retrieveMemoryMap(self, bus):
        # This function retrieves the value from the servo
        # It sends a request to the servo and waits for a response
        print("Retrieve Value")
        self.servicerequest.servo = self.id
        self.servicerequest.operation = ord('r')
        self.servicerequest.address = 0
        self.servicerequest.value = 0

        # pick the correct client : weird ternary operator
        client = self.driver0service if bus == 0 else self.driver1service

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



    def retrieveValue2(self, bus, id, address):
        # This function retrieves the value from the servo
        # It sends a request to the servo and waits for a response
        print("Retrieve Value")
        self.servicerequest.servo = id
        self.servicerequest.operation = ord('r')
        self.servicerequest.address = address
        self.servicerequest.value = 0

        # pick the correct client
        if bus == 0:
            client = self.driver0service
        else:
            client = self.driver1service

        # make the async call
        future = client.call_async(self.servicerequest)
        # block here until the service responds (or times out)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        if future.done():
            resp = future.result()
            # Note: The service response only contains memorymap, not retval
            # For read operations, we may need to extract the value from the memorymap
            # or handle this differently depending on the servo implementation
            print("Warning: retrieveValue2 needs to be updated to handle response correctly")
            return 0  # Placeholder return value
        else:
            return -1


    def populateFields(self):
        print("Populate Fields")
        memorymap = self.retrieveMemoryMap(self.bus)
        self.txtFirmwareMajor.setText(str(memorymap.firmwaremajor))
        self.txtFirmwareMinor.setText(str(memorymap.firmwaresub))
        self.txtServoMainVersion.setText(str(memorymap.servomajor))
        self.txtServoSubVersion.setText(str(memorymap.servosub))
        self.txtServoID.setText(str(memorymap.servoid))
        self.txtBaudrate.setText(str(memorymap.baudrate))
        self.txtPositionCorrection.setText(str(memorymap.positioncorrection))
        self.txtMinAngle.setText(str(memorymap.minanglelimit))
        self.txtMaxAngle.setText(str(memorymap.maxanglelimit))
        self.txtMaxTemperature.setText(str(memorymap.maxtemplimit))
        self.txtMaxVoltage.setText(str(memorymap.maxinputvoltage))
        self.txtMinVoltage.setText(str(memorymap.mininputvoltage))
        self.txtMaxTorque.setText(str(memorymap.maxtorque))
        self.txtAngularResolution.setText(str(memorymap.angularresolution))
        self.txtProtectionCurrent.setText(str(memorymap.protectioncurrent))
        self.txtClockwiseInsensitiveRegion.setText(str(memorymap.clockwiseinsensitivearea))
        self.txtCounterClockwiseInsensitiveRegion.setText(str(memorymap.counterclockwiseinsensitiveregion))
        self.txtMinimumStartForce.setText(str(memorymap.minstartupforce))
        self.txtPCoefficient.setText(str(memorymap.pcoefficient))
        self.txtICoefficient.setText(str(memorymap.icoefficient))
        self.txtDCoefficient.setText(str(memorymap.dcoefficient))
        self.txtProtectiveTorque.setText(str(memorymap.protectivetorque))
        self.txtProtectionTime.setText(str(memorymap.protectiontime))
        self.txtOverloadTorque.setText(str(memorymap.overloadtorque))
        self.txtSpeedPCoefficient.setText(str(memorymap.speedclosedlooppcoefficient))
        self.txtOvercurrentProtection.setText(str(memorymap.overcurrentprotectiontime))
        self.txtVelocityICoefficient.setText(str(memorymap.velocityclosedloopicoefficient))
        self.txtAcceleration.setText(str(memorymap.acceleration))
        self.txtTargetLocation.setText(str(memorymap.targetlocation))
        self.txtRunningTime.setText(str(memorymap.runningtime))
        self.txtRunningSpeed.setText(str(memorymap.runningspeed))
        self.txtTorqueLimit.setText(str(memorymap.torquelimit))
        self.txtReturnDelay.setText(str(memorymap.returndelay))
        self.txtResponseStatusLevel.setText(str(memorymap.responsestatuslevel))
        self.txtOperationMode.setText(str(memorymap.operationmode))
        self.txtLEDAlarm.setText(str(memorymap.ledalarmcondition))
        self.txtUnloadCondition.setText(str(memorymap.unloadingcondition))
        self.txtPhase.setText(str(memorymap.phase))


    def disableAllFields(self):
        # This function disables all the fields in the UI
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


    def clearAllFields(self):
        # This function clears all the fields in the UI
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
        # This function disables all the fields in the UI
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

    def populateServoDropdown(self):
        # This function populates the servo dropdown with available servos
        # For now, we will just add some dummy data
        self.cmbServos.addItem("driver0/servo01 - left shoulder yaw",[0,1])
        self.cmbServos.addItem("driver0/servo02 - left shoulder1 pitch",[0,2])
        self.cmbServos.addItem("driver0/servo03 - left shoulder2 pitch",[0,3])
        self.cmbServos.addItem("driver0/servo04 - left elbow pitch",[0,4])
        self.cmbServos.addItem("driver0/servo05 - left wrist pitch",[0,5])
        self.cmbServos.addItem("driver0/servo06 - left wrist roll",[0,6])
        self.cmbServos.addItem("driver0/servo07 - left hand",[0,7])

        self.cmbServos.addItem("driver1/servo01 - right shoulder yaw",[1,1])
        self.cmbServos.addItem("driver1/servo02 - right shoulder1 pitch",[1,2])
        self.cmbServos.addItem("driver1/servo03 - right shoulder2 pitch",[1,3])
        self.cmbServos.addItem("driver1/servo04 - right elbow pitch",[1,4])
        self.cmbServos.addItem("driver1/servo05 - right wrist pitch",[1,5])
        self.cmbServos.addItem("driver1/servo06 - right wrist roll",[1,6])
        self.cmbServos.addItem("driver1/servo07 - right hand",[1,7])

        self.cmbServos.addItem("driver1/servo08 - head yaw",[1,8])
        self.cmbServos.addItem("driver1/servo09 - head pitch",[1,9])
        self.cmbServos.addItem("driver1/servo10 - head roll",[1,10])

        self.bus = 0
        self.id = 1


    def onServoSelected(self):
        # This function is called when a servo is selected from the dropdown
        # It updates the bus and id based on the selected servo
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
        # This function is called when the slider value is changed
        # It updates the target location text box with the new value
        value = self.sliderTargetLocation.value()
        self.txtTargetLocation.setText(str(value))
        print(f"Slider Value: {value}")

        # send command to servo

    def onTextFieldChanged(self, field_name, address):
        """
        Handle changes to text fields and send updates to servo with debouncing
        """
        if not self.loaded:
            return  # Don't process changes during initial loading
            
        # Cancel any existing timer for this field
        if field_name in self.field_timers:
            self.field_timers[field_name].stop()
        
        # Create a new timer that will trigger the update after a delay
        timer = QTimer()
        timer.setSingleShot(True)
        timer.timeout.connect(lambda: self.processTextFieldUpdate(field_name, address))
        timer.start(500)  # 500ms delay
        
        self.field_timers[field_name] = timer

    def processTextFieldUpdate(self, field_name, address):
        """
        Process the actual text field update after debouncing delay
        """
        # Get the text field widget by name
        field_widget = getattr(self, field_name)
        text = field_widget.text().strip()
        
        # Skip if empty 
        if not text:
            return
            
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
                return
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
            else:
                print(f"Failed to update {field_name}")
                field_widget.setStyleSheet("background-color: #ffcccc;")
                field_widget.setToolTip("Failed to send to servo")
                QTimer.singleShot(2000, lambda: (field_widget.setStyleSheet(""), field_widget.setToolTip("")))
            
        except ValueError:
            print(f"Invalid value in {field_name}: {text}")
            field_widget.setStyleSheet("background-color: #ffcccc;")
            field_widget.setToolTip("Invalid number format")
            QTimer.singleShot(2000, lambda: (field_widget.setStyleSheet(""), field_widget.setToolTip("")))
        except Exception as e:
            print(f"Error updating {field_name}: {e}")

    def writeValueToServo(self, bus, servo_id, address, value):
        """
        Write a value to a specific address on the servo
        Returns True if successful, False otherwise
        """
        try:
            self.servicerequest.servo = servo_id
            self.servicerequest.operation = ord('w')  # write operation
            self.servicerequest.address = address
            self.servicerequest.value = value

            # Pick the correct client based on bus
            client = self.driver0service if bus == 0 else self.driver1service

            # Make the async call with timeout
            future = client.call_async(self.servicerequest)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            
            if future.done():
                resp = future.result()
                # For write operations, success is indicated by the service completing
                # The response contains a memorymap, but we just check if the call succeeded
                return True
            else:
                print(f"Timeout writing to servo {servo_id} at address {address}")
                return False
                
        except Exception as e:
            print(f"Exception writing to servo: {e}")
            return False


    def connectUI(self):
        self.cmbServos.currentIndexChanged.connect(self.onServoSelected)
        self.sliderTargetLocation.valueChanged.connect(self.onSliderValueChanged)
        
        # Connect all editable text fields to their change handlers
        # Note: Status fields (txtStatus*) are read-only and should not be connected
        self.txtPositionCorrection.textChanged.connect(lambda: self.onTextFieldChanged('txtPositionCorrection', 16))
        self.txtMinAngle.textChanged.connect(lambda: self.onTextFieldChanged('txtMinAngle', 9))
        self.txtMaxAngle.textChanged.connect(lambda: self.onTextFieldChanged('txtMaxAngle', 11))
        self.txtMaxTemperature.textChanged.connect(lambda: self.onTextFieldChanged('txtMaxTemperature', 13))
        self.txtMaxVoltage.textChanged.connect(lambda: self.onTextFieldChanged('txtMaxVoltage', 14))
        self.txtMinVoltage.textChanged.connect(lambda: self.onTextFieldChanged('txtMinVoltage', 15))
        self.txtMaxTorque.textChanged.connect(lambda: self.onTextFieldChanged('txtMaxTorque', 17))
        self.txtAngularResolution.textChanged.connect(lambda: self.onTextFieldChanged('txtAngularResolution', 18))
        self.txtProtectionCurrent.textChanged.connect(lambda: self.onTextFieldChanged('txtProtectionCurrent', 19))
        self.txtClockwiseInsensitiveRegion.textChanged.connect(lambda: self.onTextFieldChanged('txtClockwiseInsensitiveRegion', 20))
        self.txtCounterClockwiseInsensitiveRegion.textChanged.connect(lambda: self.onTextFieldChanged('txtCounterClockwiseInsensitiveRegion', 21))
        self.txtMinimumStartForce.textChanged.connect(lambda: self.onTextFieldChanged('txtMinimumStartForce', 22))
        self.txtPCoefficient.textChanged.connect(lambda: self.onTextFieldChanged('txtPCoefficient', 23))
        self.txtICoefficient.textChanged.connect(lambda: self.onTextFieldChanged('txtICoefficient', 24))
        self.txtDCoefficient.textChanged.connect(lambda: self.onTextFieldChanged('txtDCoefficient', 25))
        self.txtProtectiveTorque.textChanged.connect(lambda: self.onTextFieldChanged('txtProtectiveTorque', 26))
        self.txtProtectionTime.textChanged.connect(lambda: self.onTextFieldChanged('txtProtectionTime', 27))
        self.txtOverloadTorque.textChanged.connect(lambda: self.onTextFieldChanged('txtOverloadTorque', 28))
        self.txtSpeedPCoefficient.textChanged.connect(lambda: self.onTextFieldChanged('txtSpeedPCoefficient', 29))
        self.txtOvercurrentProtection.textChanged.connect(lambda: self.onTextFieldChanged('txtOvercurrentProtection', 30))
        self.txtVelocityICoefficient.textChanged.connect(lambda: self.onTextFieldChanged('txtVelocityICoefficient', 31))
        self.txtAcceleration.textChanged.connect(lambda: self.onTextFieldChanged('txtAcceleration', 32))
        self.txtTargetLocation.textChanged.connect(lambda: self.onTextFieldChanged('txtTargetLocation', 33))
        self.txtRunningTime.textChanged.connect(lambda: self.onTextFieldChanged('txtRunningTime', 34))
        self.txtRunningSpeed.textChanged.connect(lambda: self.onTextFieldChanged('txtRunningSpeed', 35))
        self.txtTorqueLimit.textChanged.connect(lambda: self.onTextFieldChanged('txtTorqueLimit', 36))
        self.txtReturnDelay.textChanged.connect(lambda: self.onTextFieldChanged('txtReturnDelay', 37))





def main(args=None):
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