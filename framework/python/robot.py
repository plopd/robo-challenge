#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import ev3dev.ev3 as ev3

# default sleep timeout in sec
DEFAULT_SLEEP_TIMEOUT_IN_SEC = 0.1

# default duty_cycle (0 - 100)
DEFAULT_DUTY_CYCLE = 60

# default threshold distance
DEFAULT_THRESHOLD_DISTANCE = 90

##
# Setup
##

print("Setting up...")

# motors
motor_right = ev3.LargeMotor('outA')
print("motorRight connected: %s" % str(motor_right.connected))

motor_left = ev3.LargeMotor('outB')
print("motorRight connected: %s" % str(motor_right.connected))

motors = [motor_left, motor_right]
motor_right.reset()
motor_left.reset()

# sensors
color_sensor = ev3.ColorSensor()
print("color sensor connected: %s" % str(color_sensor.connected))
color_sensor.mode = 'COL-REFLECT'

ultrasonic_sensor = ev3.UltrasonicSensor()
print("ultrasonic sensor connected: %s" % str(ultrasonic_sensor.connected))
ultrasonic_sensor.mode = 'US-DIST-CM'


##
#  Robot functionality
##
def backward():

    for m in motors:
        duty_cycle = m.duty_cycle_sp
        if duty_cycle > 0:
            m.duty_cycle_sp = duty_cycle * -1

    for m in motors:
        m.run_direct()


def forward():

    for m in motors:
        duty_cycle = m.duty_cycle_sp
        if duty_cycle < 0:
            m.duty_cycle_sp = duty_cycle * -1

    for m in motors:
        m.run_direct()


def set_speed(duty_cycle):
    for m in motors:
        m.duty_cycle_sp = duty_cycle


def brake():
    for m in motors:
        m.stop()


def turn():
    motor_left.stop()
    pos = motor_right.position

    # new absolute position
    abs_pos = pos + 500

    motor_right.position_sp = abs_pos
    motor_right.run_to_abs_pos()

    while abs(motor_right.position - abs_pos) > 10:
        # turn

        # stop when object detected
        if ultrasonic_sensor.value() < DEFAULT_THRESHOLD_DISTANCE:
            break

    set_speed(DEFAULT_DUTY_CYCLE)
    forward()


def teardown():
    print('Tearing down...')
    for m in motors:
        m.stop()
        m.reset()


def run_loop():
    # game loop (endless loop)
    while True:
        time.sleep(DEFAULT_SLEEP_TIMEOUT_IN_SEC)
        print('color value: %s' % str(color_sensor.value()))
        print('ultrasonic value: %s' % str(ultrasonic_sensor.value()))
        print('motor positions (r, l): %s, %s' % (str(motor_right.position), str(motor_left.position)))

        # found obstacle
        if ultrasonic_sensor.value() < DEFAULT_THRESHOLD_DISTANCE:

            brake()

            # drive backwards
            backward()

            new_pos = motor_right.position - 200
            while motor_right.position - new_pos > 10:
                # wait until robot has reached the new position
                pass

            # turn
            turn()

        else:
            forward()


def main():
    print('Run robot, run!')

    set_speed(DEFAULT_DUTY_CYCLE)
    forward()

    try:
        run_loop()

    # doing a cleanup action just before program ends
    # handle ctr+c and system exit
    except (KeyboardInterrupt, SystemExit):
        teardown()
        raise

    # handle exceptions
    except Exception as e:
        print('ohhhh error!')
        print(e)
        teardown()
##
# start the program
##
main()
