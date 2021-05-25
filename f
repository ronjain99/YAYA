import machine
import time
import pyb
from pyb import Servo
from encodedmotor import EncodedMotor
import ultrasonic

# Debug flags
DEBUG = 1
black = 1000
WHITE = (179, 204, 185, 226, 242)

# Boiler plate: PID
duty = 64  # Default duty cycle of the motors
kP = 10.000000  # Proportional Constant
kI = 0  # Integral Constant
kD = 0  # Derivative Constant
pError = 0  # Error on last iteration

# Boiler Plate: Motors
motor_left = EncodedMotor('left', 'D8', 'D9', 'D5', 'D3')
motor_right = EncodedMotor('right', 'D6', 'D7', 'D4', 'D2')

# Boil Plate: Ultrasonic Sensor
TRIG = "D13"
ECHO = "D12"
ultrasonic_sensor = ultrasonic.HCSR04(TRIG, ECHO)

# Boiler Plate: Servo Motor
servo = Servo(1)

# Boiler Plate: IR Sensors (left -> right)
ir_01 = pyb.ADC(machine.Pin('A1'))
ir_02 = pyb.ADC(machine.Pin('A2'))
ir_03 = pyb.ADC(machine.Pin('A3'))
ir_04 = pyb.ADC(machine.Pin('A4'))
ir_05 = pyb.ADC(machine.Pin('A5'))

btn = machine.Pin('PC13', machine.Pin.IN)


# Supporting Functions

def read_sensor(sensor, min, max):
    return (sensor.read() - min) / (max - min)

def error():
    v_01 = ir_01.read()
    v_02 = ir_02.read()
    v_03 = ir_04.read()
    v_04 = ir_05.read()
    v_05 = ir_05.read()

    print(v_01, v_02, v_03, v_04, v_05)

    error = ((v_01 * -3) + (v_02 * -2) + (v_03 * 1) + (v_04 * 2) + (v_05 * 3)) / (v_01 + v_02 + v_03 + v_04 + v_05)

    return error


# Primary Functions


# Code that enables the robot to exit the garage and position itself
# on the line to begin the test
def exit_garage():
    exit_condition = False
    while exit_condition == False:
       sensor_voltages= [
           ir_01.read(),
           ir_02.read(),
           ir_04.read(),
           ir_05.read(),
           ir_05.read()
       ]
       exit_condition = True in (i >= black for i in sensor_voltages)
       motor_left.move(duty)
       motor_right.move(duty + 2)
       if exit_condition == True:
           time.sleep(0.2)

    while sensor_voltages[1] < black:
        sensor_voltages = [
            ir_01.read(),
            ir_02.read(),
            ir_04.read(),
            ir_05.read(),
            ir_05.read()
        ]
        motor_left.move(0)
        motor_right.move(duty)
    motor_left.move(0)
    motor_right.move(0)



# Primary Line Following Code
# This is an implementation of PID
def follow_line_pid():
    while True:
        P = error() # Current Proportional Term
        I = I + pError # Current Integral term
        D = P - pError # Current Derivative term

        turn_rate = (kP * P) + (kI * I) + (kD * D)

        motor_left.move(duty - turn_rate)
        motor_right.move(duty + turn_rate)

        pError = P


# def hallway():
#     exit_condition = False
#
#     while exit_condition == False:
#         # Following lines test to see if it is back on the line
#        sensor_voltages= [
#            ir_01.read(),
#            ir_02.read(),
#            ir_04.read(),
#            ir_04.read(),
#            ir_05.read()
#        ]
#        exit_condition = True in (i >= black for i in sensor_voltages)

def stop():
    motor_left.move(0)
    motor_right.move(0)
def move_in_a_straight_line():
    motor_left.move(68)
    motor_right.move(66)
def slight_right():
    motor_left.move(70)
    motor_right.move(64)
def slight_left():
    motor_left.move(66)
    motor_right.move(68)
# Steps followed to solve problem
#follow_line_pid(BLACK, WHITE)
#exit_garage()

def hallway(): # if line-break detected, stop use ultrasonic to detect if we are in a hallway, proceed
    for desired_angle in range(-90, 90, +5):  # Anticlockwise
        servo.angle(desired_angle)
        time.sleep(0.01)
    while True:
        w1 = ir_01.read()
        w2 = ir_02.read()
        w3 = ir_03.read()
        w4 = ir_04.read()
        w5 = ir_05.read()
        sum = w1+w2+w3+w4+w5
        if sum < 1500:
            dist = ultrasonic_sensor.distance_mm()
            if dist > 100:
                slight_left()
            if 0 < dist < 100:
                slight_right()
            if dist == 100:
                move_in_a_straight_line()
        else:
            break
def pid():
    while True:
        w1 = ir_01.read()
        w2 = ir_02.read()
        w3 = ir_03.read()
        w4 = ir_04.read()
        w5 = ir_05.read()
        print(w1,w2,w3,w4,w5)
        if w3 > 2000:
            move_in_a_straight_line()
            print('moving in a straight line')
        if w5 > 1800:
            stop()
            print('w4black')
            while True:
                slight_right()
                print('slighting right')
                if w3 > 1500:
                    break
            stop()
            print('w3black')
            move_in_a_straight_line()
        if w2 > 1800:
            stop()
            print('w2black')
            while True:
                slight_left()
                print('slighting left')
                if w2 > 1500:
                    break
            stop()
            print('w3black')
            move_in_a_straight_line()


pid()
