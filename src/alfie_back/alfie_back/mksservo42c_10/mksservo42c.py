import serial
import time

class MKSServo42C:
    """
    Class to control the MKS Servo 42C.

    Tailored for v1.0 firmware.
    """

    _READ_ENCODER_VALUE                              = 0x30
    _READ_NUMBER_OF_PULSES_RECEIVED                  = 0x33
    _READ_MOTOR_SHAFT_ANGLE                          = 0x36
    _READ_MOTOR_SHAFT_ERROR_ANGLE                    = 0x39
    _READ_EN_PIN_STATUS                              = 0x3A
    #_READ_MOTOR_SHAFT_LOCKED_ROTOR_PROTECTION_STATE  = 0x3D
    _READ_MOTOR_SHAFT_STATUS                         = 0x3E
    #_WRITE_RESTORE_DEFAULT_PARAMETERS                = 0x3F

    #_WRITE_CALIBRATE_ENCODER_VALUE                   = 0x80
    #_WRITE_SET_MOTOR_TYPE                            = 0x81 
    #_WRITE_SET_WORK_MODE                             = 0x82
    #_WRITE_SET_CURRENT                               = 0x83
    _WRITE_SET_SUBDIVISON                            = 0x84
    _WRITE_SET_ACTIVE_EN_PIN                         = 0x85
    #_WRITE_SET_DIRECTION_OF_MOTOR_ROTATION           = 0x86
    #_WRITE_SET_AUTOMATIC_TURN_OFF_SCREEN             = 0x87
    #_WRITE_SET_MOTOR_SHAFT_LOCKED_ROTOR_PROTECTION   = 0x88
    #_WRITE_SET_SUBDIVISION_INTERPOLATION             = 0x89
    #_WRITE_SET_BAUD_RATE                             = 0x8A
    #_WRITE_SET_SLAVE_ADDRESS                         = 0x8B

    #_WRITE_SET_ACCELERATION_PARAMETER                = 0xA4
    #_WRITE_SET_MAXIMUM_TORQUE                        = 0xA5    

    _WRITE_SET_EN_PIN_STATUS                         = 0xF3
    _WRITE_RUN_CONSTANT_SPEED                        = 0xF6
    _WRITE_STOP_MOTOR                                = 0xF7
    _WRITE_RUN_MOTOR_TO_POSITION                     = 0xFD
    _WRITE_SAVE_CLEAR_F6_STATUS                      = 0xFF

    

    def __init__(
            self, 
            port: str = '/dev/ttyUSB1', 
            baudrate: int = 9600, 
            timeout: float = 1.0, 
            address: int = 0xE0
        ):
        """
        Initialize the MKSServo42C class.

        :param port: The serial port to connect to the servo. default '/dev/ttyUSB0'
        :param baudrate: The baud rate for the serial connection. default 9600
        :param timeout: The timeout for the serial connection. default 1 second
        :param address: The address of the servo. default 0xE0
        """
        self._port = port
        self._address = address
        self._baudrate = baudrate
        self._timeout = timeout
        self._position = 0
        self._acceleration = 0
        self._speed = 0

        self._position = -1

        self._ser = serial.Serial(
            port=self._port,  # Replace with your serial port
            baudrate=self._baudrate,        # Replace with your baud rate
            timeout=self._timeout,
        )

        self.connect()

    def connect(self):
        """
        Connect to the servo.
        """
        # Code to connect to the servo
        try:
            # Open the serial port if not already open
            if not self._ser.is_open:
                self._ser.open()
                print("Serial port opened successfully.")

        except serial.SerialException as e:
            print(f"Serial error: {e}")
        pass

    def disconnect(self):
        """
        Disconnect from the servo.
        """
        # Close the serial port
        if self._ser.is_open:
            self._ser.close()


        pass

    def calculate_checksum(self, data):
        packed = b''.join(data)
        return sum(packed) & 0xFF
    

    def read_motor_shaft_status(self) -> int:
        """
        Get the status of the motor shaft.

        :return: The status of the motor shaft.
        """
        commandpacket = [
            self._address.to_bytes(1, 'big'), 
            self._READ_MOTOR_SHAFT_STATUS.to_bytes(1, 'big')]

        #checksum = self.calculate_checksum(commandpacket)
        #commandpacket.append(checksum.to_bytes(1, 'big'))

        # 3) Join into one bytes object and send
        packet_bytes = b''.join(commandpacket)
        #print(f"{len(packet_bytes)} bytes to send: {packet_bytes.hex()}")
        self._ser.write(packet_bytes)

        # Read raw bytes from the serial port
        response = self._ser.read(2)
        hex_response = ' '.join(f'{byte:02X} ' for byte in response)
        #print(f"Received (hex): {hex_response}")

        return response[1]

    def read_motor_shaft_angle(self) -> int:
        """
        Get the angle of the motor shaft.

        :return: The angle of the motor shaft.
        """
        commandpacket = [
            self._address.to_bytes(1, 'big'), 
            self._READ_MOTOR_SHAFT_ANGLE.to_bytes(1, 'big')]

        #checksum = self.calculate_checksum(commandpacket)
        #commandpacket.append(checksum.to_bytes(1, 'big'))

        # 3) Join into one bytes object and send
        packet_bytes = b''.join(commandpacket)
        #print(f"{len(packet_bytes)} bytes to send: {packet_bytes.hex()}")
        self._ser.write(packet_bytes)

        # Read raw bytes from the serial port
        response = self._ser.read(5)
        #hex_response = ' '.join(f'{byte:02X} ' for byte in response)
        #print(f"Received (hex): {hex_response}")

        return int.from_bytes(response[1:5], byteorder='big', signed=False)

    def read_motor_shaft_error_angle(self) -> int:
        """
        Get the error angle of the motor shaft.

        :return: The error angle of the motor shaft.
        """
        commandpacket = [
            self._address.to_bytes(1, 'big'), 
            self._READ_MOTOR_SHAFT_ERROR_ANGLE.to_bytes(1, 'big')]

        #checksum = self.calculate_checksum(commandpacket)
        #commandpacket.append(checksum.to_bytes(1, 'big'))

        # 3) Join into one bytes object and send
        packet_bytes = b''.join(commandpacket)
        #print(f"{len(packet_bytes)} bytes to send: {packet_bytes.hex()}")
        self._ser.write(packet_bytes)

        # Read raw bytes from the serial port
        response = self._ser.read(3)
        hex_response = ' '.join(f'{byte:02X} ' for byte in response)
        #print(f"Received (hex): {hex_response}")

        return int.from_bytes(response[1:3], byteorder='big', signed=True)

    def read_pulses_received(self) -> int: 
        """
        Get the number of pulses received by the servo.

        :return: The number of pulses received.
        """
        commandpacket = [
            self._address.to_bytes(1, 'big'), 
            self._READ_NUMBER_OF_PULSES_RECEIVED.to_bytes(1, 'big')]

        #checksum = self.calculate_checksum(commandpacket)
        #commandpacket.append(checksum.to_bytes(1, 'big'))

        # 3) Join into one bytes object and send
        packet_bytes = b''.join(commandpacket)
        #print(f"{len(packet_bytes)} bytes to send: {packet_bytes.hex()}")
        self._ser.write(packet_bytes)

        #self._ser.flush()                        # ensure bytes really left your PC
        #time.sleep(0.05)                   # 50 ms is usually enough; adjust as needed

        # Read raw bytes from the serial port
        response = self._ser.read(5)
        hex_response = ' '.join(f'{byte:02X} ' for byte in response)
        #print(f"Received (hex): {hex_response}")

        response = response[1:5]
        retVal = int.from_bytes(response, byteorder='big', signed=True)
        return retVal


    def _set_direction(self, direction: str):
        """
        DOES NOT WORK ON 1.0 FIRMWARE

        Set the direction of the servo.

        :param direction: The direction to set the servo to. allowable values are 'CW' or 'CCW'.
        """
        commandpacket = [
            self._address.to_bytes(1, 'big'),
            self._WRITE_SET_DIRECTION_OF_MOTOR_ROTATION.to_bytes(1, 'big'),
            ]
        if direction == 'CW':
            commandpacket.append(0x01.to_bytes(1, 'big'))
        elif direction == 'CCW':
            commandpacket.append(0x00.to_bytes(1, 'big'))
        else:
            raise ValueError("Direction must be 'CW' or 'CCW'")
        
        # 3) Join into one bytes object and send
        packet_bytes = b''.join(commandpacket)
        #print(f"{len(packet_bytes)} bytes to send: {packet_bytes.hex()}")
        self._ser.write(packet_bytes)

        self._ser.flush()                        # ensure bytes really left your PC
        time.sleep(0.05)                   # 50 ms is usually enough; adjust as needed

        # Read raw bytes from the serial port
        response = self._ser.read(2)
        hex_response = ' '.join(f'{byte:02X} ' for byte in response)
        #print(f"Received (hex): {hex_response}")


    def read_encoder_value(self) -> int:
        """
        Set the position of the servo.

        :param position: The position to set the servo to.
        """
        commandpacket = [
            self._address.to_bytes(1, 'big'), 
            self._READ_ENCODER_VALUE.to_bytes(1, 'big')]

        #checksum = self.calculate_checksum(commandpacket)
        #commandpacket.append(checksum.to_bytes(1, 'big'))

        # 3) Join into one bytes object and send
        packet_bytes = b''.join(commandpacket)
        #print(f"{len(packet_bytes)} bytes to send: {packet_bytes.hex()}")
        self._ser.write(packet_bytes)

        # Read raw bytes from the serial port
        response = self._ser.read(3)
        hex_response = ' '.join(f'{byte:02X}' for byte in response)
        #print(f"Received (hex): {hex_response}")

    def set_subdivision(self, subdivision: int):
        """
        Set the subdivision of the servo.

        :param subdivision: The subdivision to set the servo to.
        """

        commandpacket = [
            self._address.to_bytes(1, 'big'),
            self._WRITE_SET_SUBDIVISON.to_bytes(1, 'big'),
            subdivision.to_bytes(1, 'big'),
            ]
        
        #checksum = self.calculate_checksum(commandpacket)
        #commandpacket.append(checksum.to_bytes(1, 'big'))

        # 3) Join into one bytes object and send
        packet_bytes = b''.join(commandpacket)
        #print(f"{len(packet_bytes)} bytes to send: {packet_bytes.hex()}")
        self._ser.write(bytes(packet_bytes))

        #self._ser.flush()                        # ensure bytes really left your PC
        #time.sleep(0.05)                   # 50 ms is usually enough; adjust as needed

        # Read raw bytes from the serial port
        response = self._ser.read(2)
        hex_response = ' '.join(f'{byte:02X} ' for byte in response)
        #print(f"Received (hex): {hex_response}")

    def move_to(self, dir: str, speed: int, position: int):
        """
        Move shoulders to desired position 

        :param dir: either 'CW' or 'CCW' (CW is up)
        :param speed: The speed to set the servo to.
        :param position: The relative position to set the servo to.
        """

        
        if dir == 'CCW':
            speed = speed
        elif dir == 'CW':
            speed = speed | 0x80
        else:       
            raise ValueError("Direction must be 'CW' or 'CCW'" )

        speed = speed.to_bytes(1, 'big')

        commandpacket = [
            self._address.to_bytes(1, 'big'),
            self._WRITE_RUN_MOTOR_TO_POSITION.to_bytes(1, 'big'),
            speed,
            position.to_bytes(2, 'big'),
            ]
        
        #checksum = self.calculate_checksum(commandpacket)
        #commandpacket.append(checksum.to_bytes(1, 'big'))

        # 3) Join into one bytes object and send
        packet_bytes = b''.join(commandpacket)
        #print(f"{len(packet_bytes)} bytes to send: {packet_bytes.hex()}")
        self._ser.write(packet_bytes)

        #self._ser.flush()                        # ensure bytes really left your PC
        #time.sleep(0.05)                   # 50 ms is usually enough; adjust as needed

        # Read raw bytes from the serial port
        response = self._ser.read(2)
        hex_response = ' '.join(f'{byte:02X}' for byte in response)
        #print(f"Received (hex): {hex_response}")

    def move(self, direction: str, speed: int):
        """
        Move shoulders to desired position 

        :param direction:  'CW' or 'CCW'
        :param speed: The speed to set the servo to.
        """

        if direction == 'CCW':
            speed = speed
        elif direction == 'CW':
            speed = speed | 0x80
        else:
            raise ValueError("Direction must be 'CW' or 'CCW'")
        
        #speed = speed.to_bytes(1, 'big')

        commandpacket = [
            self._address.to_bytes(1, 'big'),
            self._WRITE_RUN_CONSTANT_SPEED.to_bytes(1, 'big'),
            speed.to_bytes(1, 'big'),
            ]
        
        #checksum = self.calculate_checksum(commandpacket)
        #commandpacket.append(checksum.to_bytes(1, 'big'))

        # 3) Join into one bytes object and send
        packet_bytes = b''.join(commandpacket)
        #print(f"{len(packet_bytes)} bytes to send: {packet_bytes.hex()}")
        self._ser.write(bytes(packet_bytes))

        #self._ser.flush()                # ensure bytes really left your PC
        #time.sleep(0.05)                   # 50 ms is usually enough; adjust as needed

        # Read raw bytes from the serial port
        response = self._ser.read(2)
        hex_response = ' '.join(f'{byte:02X}' for byte in response)
        #print(f"Received (hex): {hex_response}")


    def stop(self):
        """
        Stop the servo.
        """

        commandpacket = [
            self._address.to_bytes(1, 'big'),
            self._WRITE_STOP_MOTOR.to_bytes(1, 'big'),
            ]
        
        #checksum = self.calculate_checksum(commandpacket)
        #commandpacket.append(checksum.to_bytes(1, 'big'))

        # 3) Join into one bytes object and send
        packet_bytes = b''.join(commandpacket)
        #print(f"{len(packet_bytes)} bytes to send: {packet_bytes.hex()}")
        self._ser.write(packet_bytes)

        self._ser.flush()                # ensure bytes really left your PC
        time.sleep(0.05)                   # 50 ms is usually enough; adjust as needed

        # Read raw bytes from the serial port
        response = self._ser.read(2)
        hex_response = ' '.join(f'{byte:02X}' for byte in response)
        #print(f"Received (hex): {hex_response}")


    def set_en_pin_status(self, status: int):
        """
        Set the EN pin status of the servo.

        :param status: The status to set the servo to.
        """

        commandpacket = [
            self._address.to_bytes(1, 'big'),
            self._WRITE_SET_EN_PIN_STATUS.to_bytes(1, 'big'),
            status.to_bytes(1, 'big'),
            ]
        
        #checksum = self.calculate_checksum(commandpacket)
        #commandpacket.append(checksum.to_bytes(1, 'big'))

        # 3) Join into one bytes object and send
        packet_bytes = b''.join(commandpacket)
        #print(f"{len(packet_bytes)} bytes to send: {packet_bytes.hex()}")
        self._ser.write(packet_bytes)

        self._ser.flush()                        # ensure bytes really left your PC
        time.sleep(0.05)                   # 50 ms is usually enough; adjust as needed

        # Read raw bytes from the serial port
        response = self._ser.read(3)
        hex_response = ' '.join(f'{byte:02X}' for byte in response)
        #print(f"Received (hex): {hex_response}")

    def set_active_of_en_pin(self, active: int) :
        """
        Set the active of EN pin status of the servo.

        :param active: The status to set the servo to.
        """

        commandpacket = [
            self._address.to_bytes(1, 'big'),
            self._WRITE_SET_ACTIVE_EN_PIN.to_bytes(1, 'big'),
            active.to_bytes(1, 'big'),
            ]
        
        checksum = self.calculate_checksum(commandpacket)
        commandpacket.append(checksum.to_bytes(1, 'big'))

        # 3) Join into one bytes object and send
        packet_bytes = b''.join(commandpacket)
        #print(f"{len(packet_bytes)} bytes to send: {packet_bytes.hex()}")
        self._ser.write(packet_bytes)

        # Read raw bytes from the serial port
        response = self._ser.read(3)
        hex_response = ' '.join(f'{byte:02X}' for byte in response)
        #print(f"Received (hex): {hex_response}")