#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()



# Write your program here.
ev3.speaker.beep()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
arm_motor = Motor(Port.B)
robot = DriveBase(left_motor,right_motor,55.5,104)
left_sensor = ColorSensor(Port.S1)
ultra_sensor = UltrasonicSensor(Port.S3)
object_detector = ColorSensor(Port.S2)
right_sensor = ColorSensor(Port.S4)

threshold = 50
kp = 5

ev3.speaker.beep()
def left_line_following(speed, kp):
    threshold = 50
    left_reflection = left_sensor.reflection()
    error = left_reflection - threshold
    turn_rate = kp * error
    robot.drive(speed,turn_rate)


def right_line_following(speed, kp):
    threshold = 50
    right_reflection = right_sensor.reflection()
    error = right_reflection - threshold
    turn_rate = kp * error
    robot.drive(speed,turn_rate)


def n_move(n, direction = "right"):
    for _ in range(n):
        if direction == 'right':
            while right_sensor.reflection() > 55:
                left_line_following(100, 1.2)
            while right_sensor.reflection() <= 55:
                right_line_following(100, 1.2)
        elif direction == 'left':
            while left_sensor.reflection() > 50:
                right_line_following(100, 1.2)
            while left_sensor.reflection() <= 50:
                left_line_following(100, 1.2)
    
    robot.stop()


def grab_object():

    arm_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=80)
def release_object():
    arm_motor.run_until_stalled(-200,then=Stop.COAST,duty_limit=50)

release_object()
robot.straight(100)
n_move(1,direction="left")
while ultra_sensor.distance()>50:
    left_line_following(100,1.2)
robot.stop()
robot.straight(40)

grab_object()
wait(500)
object_color = object_detector.color()
print("Detected object color",object_color)


if object_color == Color.RED:
    robot.turn(190)
    n_move(1,direction='left')
    robot.straight(40)
    robot.turn(-95)
    n_move(1,direction='right')
    robot.straight(40)
    robot.turn(95)
    n_move(2,direction='right')
    robot.straight(40)
    release_object()

elif object_color == Color.GREEN:
    robot.turn(190)
    n_move(1,direction='right')
    robot.straight(50)
    robot.turn(-95)
    n_move(2,direction='left')
    robot.straight(40)
    robot.turn(95)
    n_move(2,direction='left')
    robot.straight(40)
    release_object()





