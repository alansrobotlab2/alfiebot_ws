from mksservo42c import MKSServo42C
import gpiod
from gpiod.line import Direction, Value

import time

SUBDIVISION = 8
PULLEYTEETH = 20
GT2_PITCH = 2.0 #mm
STEPPER_DEGREES = 1.8 #degrees
STEPS_PER_MM = int(((360.0/STEPPER_DEGREES) * SUBDIVISION)/ (GT2_PITCH * PULLEYTEETH))
MAX_HEIGHT = 400 #mm

print(f"STEPS_PER_MM:  {STEPS_PER_MM}")

MIN_SPEED = 0x50
MAX_DOWN_SPEED = 0x78
MAX_UP_SPEED = 0x7D

ACCEL_DECEL_DISTANCE = 0x14 # mm to get to full speed
ACCEL_DECEL_DELTA = 0x06

pulses_offset = 0

shoulders = MKSServo42C(
    port='/dev/ttyACM0', 
    baudrate=115200, 
    timeout=1.0, 
    address=0xE0
    )

#orientation of shoulder setup on Alfie, CW is up
#shoulders.set_direction("CCW")
shoulders.set_subdivision(SUBDIVISION)

def map_scale(value, input_min, input_max, output_min, output_max):
  """
  Maps a value from one scale to another.

  Args:
    value: The input value.
    input_min: The minimum value of the input scale.
    input_max: The maximum value of the input scale.
    output_min: The minimum value of the output scale.
    output_max: The maximum value of the output scale.

  Returns:
    The mapped value in the output scale.
  """
  return (value - input_min) * (output_max - output_min) / (input_max - input_min) + output_min



def get_limit_sensor() -> True:
    """
    Get the value of the sensor.
    :return: True if the sensor is triggered, False otherwise.
    """
    with gpiod.request_lines(
        "/dev/gpiochip1",
        consumer="get-line-value",
        config={8: gpiod.LineSettings(direction=Direction.INPUT)},
    ) as request:
        value = request.get_value(8)
        #print("{}={}".format(8, value))

    if(value == Value.ACTIVE):
        return True
    else:
        return False

def get_height() -> float:
    """
    Get the height of the shoulder in mm.
    :return: The height of the shoulder in mm.
    """
    # get pulses
    pulses = shoulders.get_pulses_received()
    # subtract offset
    pulses -= pulses_offset
    # convert to mm
    height = pulses / STEPS_PER_MM
    print(f"Height: {height} mm")
    return height

def async_moveto_height(height: int, speed: int) -> None:
    """
    Move the shoulder to the specified height.
    :param height: The height to move to in mm.
    """

    if ( height < 0 or height > MAX_HEIGHT):
        print("Height out of range")
        return
    
    # get current height
    current_height = get_height()
    # calculate height difference
    height_diff = height - current_height 
    # check if height difference is positive or negative
    if (height_diff > 0):
        # move up
        direction = "CW"
        # set speed to the lesser of speed and MAX_UP_SPEED
        speed = min(speed, MAX_UP_SPEED)
    else:
        # move down
        direction = "CCW"
        speed = min(speed,MAX_DOWN_SPEED)

    # convert height difference to pulses
    pulses = int(abs(height_diff) * STEPS_PER_MM)

    # move to position
    shoulders.move_to(direction, speed, pulses)


def moveto_height(target_height: int, speed: int) -> None:
    """
    Move the shoulder to the specified height.
    :param height: The height to move to in mm.
    """

    if ( target_height < 0 or target_height > MAX_HEIGHT):
        print("Height out of range")
        return
    
    # get current height
    current_height = get_height()
    # calculate height difference
    height_diff = target_height - current_height 
    # check if height difference is positive or negative
    if (height_diff > 0):
        # move up
        direction = "CW"
        # set speed to the lesser of speed and MAX_UP_SPEED
        speed = min(speed, MAX_UP_SPEED)
    else:
        # move down
        direction = "CCW"
        speed = min(speed,MAX_DOWN_SPEED)

    # move to position
    #shoulders.move(direction, speed)

    slow_speed = speed - ACCEL_DECEL_DELTA

    current_height = get_height()
    if(direction == "CW"):
        # move until height reaches target height
        #period = 0
        while (current_height <= target_height):
            diff = abs(current_height - target_height)
            subspeed = int(min(speed, (speed * (diff / ACCEL_DECEL_DISTANCE) )))
            subspeed = int(min(speed, map_scale(diff, 0,ACCEL_DECEL_DISTANCE, slow_speed,MAX_UP_SPEED)))
            shoulders.move(direction, subspeed)
            #shoulders.move(direction, speed)
            print(f"Acceleration:  {map_scale(diff, 0,ACCEL_DECEL_DISTANCE, slow_speed,MAX_UP_SPEED)}")
            time.sleep(0.01)
            current_height = get_height()
            #period += 1
        shoulders.stop()
    else:
        # move until hight reaches target height
        #period = 0
        while (current_height >= target_height):
            diff = abs(current_height - target_height)
            subspeed = int(min(speed, (speed * (diff / ACCEL_DECEL_DISTANCE) )))
            subspeed = int(min(speed, map_scale(diff, 0,ACCEL_DECEL_DISTANCE, slow_speed,MAX_DOWN_SPEED)))
            shoulders.move(direction, subspeed)
            #shoulders.move(direction, speed)
            print(f"Acceleration:  {map_scale(diff, 0,ACCEL_DECEL_DISTANCE, slow_speed,MAX_DOWN_SPEED)}")
            time.sleep(0.01)
            current_height = get_height()
            #period += 1
        shoulders.stop()
  


# time.sleep(5.0)


# check if sensor is triggered

limit = get_limit_sensor()
if(limit == True):
    print("Sensor is triggered")
else:
    print("Sensor is not triggered")


"""
Now we check to see if the limit switch is triggered.

If it is, then that means that the shoulder are at their minimum.

If it is not, then either someone held up the shoulders during power up,
or maybe the arms are down and the shoulders are resting at some point above minimum.
In the final code we need to check the status of the arms first.
For now we'll assume that they're tucked in.
"""

#print(shoulders.get_pulses_received())

if(limit == True):
    # slowly move shoulders up until limit switch is triggered, then stop
    shoulders.move("CW", MAX_UP_SPEED)

    while(get_limit_sensor() == True):
        time.sleep(0.01)
        print(f"Read motor shaft status: {shoulders.read_motor_shaft_status()}")

    shoulders.stop()
else:
    # move shoulders down until limit switch is triggered, then stop
    shoulders.move("CCW", MAX_DOWN_SPEED)

    while(get_limit_sensor() == False):
        time.sleep(0.01)
        print(f"Read motor shaft status: {shoulders.read_motor_shaft_status()}")
    shoulders.stop()


#print(shoulders.get_pulses_received())

pulses_offset = shoulders.get_pulses_received()
#print("Pulses offset: ", pulses_offset)

#async_moveto_height(390, MAX_UP_SPEED)

moveto_height(390, MAX_UP_SPEED)

time.sleep(0.5)

moveto_height(400, MAX_UP_SPEED)

get_height()

moveto_height(390, MAX_DOWN_SPEED)

get_height()

time.sleep(1)

print(f"Read motor shaft status: {shoulders.read_motor_shaft_status()}")

moveto_height(200, MAX_DOWN_SPEED)

moveto_height(390, MAX_UP_SPEED)

