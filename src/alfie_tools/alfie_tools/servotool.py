import sys
from PyQt5 import QtWidgets
from PyQt5.QtCore import QSize
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
        uic.loadUi("servotool.ui", self)
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


    def retrieveMemoryMap(self, bus):
        # This function retrieves the value from the servo
        # It sends a request to the servo and waits for a response
        print("Retrieve Value")
        self.servicerequest.servo = 0
        self.servicerequest.operation = ord('R')
        self.servicerequest.address = 0
        self.servicerequest.value = 0

        # pick the correct client : weird ternary operator
        client = self.driver0service if bus == 0 else self.driver1service

        # make the async call
        future = client.call_async(self.servicerequest)
        # block here until the service responds (or times out)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        if future.done():
            resp = future.result()
            print("Memory Map Retrieved:")
            print(resp.memorymap)
            return resp.memorymap
        else:
            return -1



    def retrieveValue2(self, bus, id, address):
        # This function retrieves the value from the servo
        # It sends a request to the servo and waits for a response
        print("Retrieve Value")
        self.servicerequest.servo = id
        self.servicerequest.operation = ord('R')
        self.servicerequest.address = address
        self.servicerequest.value = 0

        # pick the correct client
        client = self.driver0service if bus == 0 else self.driver1service

        # make the async call
        future = client.call_async(self.servicerequest)
        # block here until the service responds (or times out)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        if future.done():
            resp = future.result()
            return resp.retval
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
        # time.sleep(0.1)
        # self.txtFirmwareMinor.setText(str(self.retrieveValue(self.bus, self.id, 0x01)))
        # time.sleep(0.1)
        # self.txtServoMainVersion.setText(str(self.retrieveValue(self.bus, self.id, 0x03)))
        # time.sleep(0.1)
        # self.txtServoSubVersion.setText(str(self.retrieveValue(self.bus, self.id, 0x04)))
        # time.sleep(0.1)
        # self.txtServoID.setText(str(self.retrieveValue(self.bus, self.id, 0x05)))
        # time.sleep(0.1)
        # self.txtBaudrate.setText(str(self.retrieveValue(self.bus, self.id, 0x06)))
        # time.sleep(0.1)
        # self.txtPositionCorrection.setText(str(self.retrieveValue(self.bus, self.id, 0x1F)))
        # time.sleep(0.1)
        # self.txtMinAngle.setText(str(self.retrieveValue(self.bus, self.id, 0x09)))
        # time.sleep(0.1)
        # self.txtMaxAngle.setText(str(self.retrieveValue(self.bus, self.id, 0x0B)))
        # time.sleep(0.1)
        # self.txtMaxTemperature.setText(str(self.retrieveValue(self.bus, self.id, 0x0D)))
        # time.sleep(0.1)
        # self.txtMaxVoltage.setText(str(self.retrieveValue(self.bus, self.id, 0x0E)))
        # time.sleep(0.1)
        # self.txtMinVoltage.setText(str(self.retrieveValue(self.bus, self.id, 0x0F)))
        # time.sleep(0.1)
        # self.txtMaxTorque.setText(str(self.retrieveValue(self.bus, self.id, 0x10)))
        # time.sleep(0.1)
        # self.txtAngularResolution.setText(str(self.retrieveValue(self.bus, self.id, 0x1E)))
        # time.sleep(0.1)
        # self.txtProtectionCurrent.setText(str(self.retrieveValue(self.bus, self.id, 0x1C)))
        # time.sleep(0.1)
        # self.txtClockwiseInsensitiveRegion.setText(str(self.retrieveValue(self.bus, self.id, 0x1A)))
        # time.sleep(0.1)
        # self.txtCounterClockwiseInsensitiveRegion.setText(str(self.retrieveValue(self.bus, self.id, 0x1B)))
        # time.sleep(0.1)
        # self.txtMinimumStartForce.setText(str(self.retrieveValue(self.bus, self.id, 0x18)))
        # time.sleep(0.1)
        # self.txtPCoefficient.setText(str(self.retrieveValue(self.bus, self.id, 0x15)))
        # time.sleep(0.1)
        # self.txtICoefficient.setText(str(self.retrieveValue(self.bus, self.id, 0x17)))
        # time.sleep(0.1)
        # self.txtDCoefficient.setText(str(self.retrieveValue(self.bus, self.id, 0x16)))
        # time.sleep(0.1)
        # self.txtProtectiveTorque.setText(str(self.retrieveValue(self.bus, self.id, 0x22)))
        # time.sleep(0.1)
        # self.txtProtectionTime.setText(str(self.retrieveValue(self.bus, self.id, 0x23)))
        # time.sleep(0.1)
        # self.txtOverloadTorque.setText(str(self.retrieveValue(self.bus, self.id, 0x24)))
        # time.sleep(0.1)
        # self.txtSpeedPCoefficient.setText(str(self.retrieveValue(self.bus, self.id, 0x25)))
        # time.sleep(0.1)
        # self.txtOvercurrentProtection.setText(str(self.retrieveValue(self.bus, self.id, 0x26)))
        # time.sleep(0.1)
        # self.txtVelocityICoefficient.setText(str(self.retrieveValue(self.bus, self.id, 0x27)))
        # time.sleep(0.1)
        # self.txtAcceleration.setText(str(self.retrieveValue(self.bus, self.id, 0x29)))
        # time.sleep(0.1)
        # self.txtTargetLocation.setText(str(self.retrieveValue(self.bus, self.id, 0x2A)))
        # time.sleep(0.1)
        # self.txtRunningTime.setText(str(self.retrieveValue(self.bus, self.id, 0x2C)))
        # time.sleep(0.1)
        # self.txtRunningSpeed.setText(str(self.retrieveValue(self.bus, self.id, 0x2E)))
        # time.sleep(0.1)
        # self.txtTorqueLimit.setText(str(self.retrieveValue(self.bus, self.id, 0x30)))
        # time.sleep(0.1)
        # self.txtReturnDelay.setText(str(self.retrieveValue(self.bus, self.id, 0x07)))
        # time.sleep(0.1)
        # self.txtResponseStatusLevel.setText(str(self.retrieveValue(self.bus, self.id, 0x08)))
        # time.sleep(0.1)
        # self.txtOperationMode.setText(str(self.retrieveValue(self.bus, self.id, 0x21)))
        # time.sleep(0.1)
        # self.txtLEDAlarm.setText(str(self.retrieveValue(self.bus, self.id, 0x14)))
        # time.sleep(0.1)
        # self.txtUnloadCondition.setText(str(self.retrieveValue(self.bus, self.id, 0x13)))
        # time.sleep(0.1)
        # self.txtPhase.setText(str(self.retrieveValue(self.bus, self.id, 0x12)))
        # time.sleep(0.1)



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
        self.cmbServos.addItem("driver0/servo01 - left shoulder yaw",[0,0])
        self.cmbServos.addItem("driver0/servo02 - left shoulder1 pitch",[0,1])
        self.cmbServos.addItem("driver0/servo03 - left shoulder2 pitch",[0,2])
        self.cmbServos.addItem("driver0/servo04 - left elbow pitch",[0,3])
        self.cmbServos.addItem("driver0/servo05 - left wrist pitch",[0,4])
        self.cmbServos.addItem("driver0/servo06 - left wrist roll",[0,5])
        self.cmbServos.addItem("driver0/servo07 - left hand",[0,6])

        self.cmbServos.addItem("driver1/servo01 - right shoulder yaw",[1,0])
        self.cmbServos.addItem("driver1/servo02 - right shoulder1 pitch",[1,1])
        self.cmbServos.addItem("driver1/servo03 - right shoulder2 pitch",[1,2])
        self.cmbServos.addItem("driver1/servo04 - right elbow pitch",[1,3])
        self.cmbServos.addItem("driver1/servo05 - right wrist pitch",[1,4])
        self.cmbServos.addItem("driver1/servo06 - right wrist roll",[1,5])
        self.cmbServos.addItem("driver1/servo07 - right hand",[1,6])

        self.cmbServos.addItem("driver1/servo08 - head yaw",[1,7])
        self.cmbServos.addItem("driver1/servo09 - head pitch",[1,8])
        self.cmbServos.addItem("driver1/servo10 - head roll",[1,9])

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


    def connectUI(self):
        self.cmbServos.currentIndexChanged.connect(self.onServoSelected)
        self.sliderTargetLocation.valueChanged.connect(self.onSliderValueChanged)





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