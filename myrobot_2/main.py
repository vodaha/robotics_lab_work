#!/usr/bin/env python3

# Must-have libraries
from ev3dev2.motor import MoveSteering, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import ColorSensor, GyroSensor
from ev3dev2.sensor import INPUT_2, INPUT_3, INPUT_4
import time
import csv
import os
import math

# Declare a file instance and delete the file for new data tracking
csv_file = "robot_path.csv"
if os.path.exists(csv_file):
    os.remove(csv_file)

# Modules assignment
move_steering = MoveSteering(OUTPUT_B, OUTPUT_C)
gyro_sensor = GyroSensor(INPUT_2) # Specify input for movement and oscillation gyro
gyro_sensor_tracker = GyroSensor(INPUT_4) # Specify input for position tracking gyro
color_sensor = ColorSensor(INPUT_3) # Specify color sensor input for consistensy

# Color threshholds
GREEN_MAX = 20
GREY_MIN = 30  
GREY_MAX = 45
WHITE_MIN = 55

# Movement constants
FORWARD_SPEED = 35
TURN_GAIN = 1.0
OSCILLATION_SPEED = 20

# Global modifiables
is_line_followed = True
grey_lines_crossed = 0
is_program_complete = False

# Coordinates tracking mechanics
start = time.time()
last_log = 0
LOG_INT = 0.1 #logging interval

# Robot's physique
WHEEL_D = 5.6 #standart diameter in cm for ev3 robot wheel
WHEEL_C = WHEEL_D * math.pi
AXLE = 12.0  #stantart distance between wheels in cm for ev3 robot

# Position tracking variables
x = 0.0
y = 0.0
heading = 0.0  # in radians
lle = 0 # last left encoder
lre = 0 # last right encoder

# A buffer that serves as a sensor values tracker
recent_values = []
BUFFER_SIZE = 15

# A method to create a new file
def new_csv_file():

    with open(csv_file, 'w', newline='') as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow(['timestamp', 'x', 'y', 'state', 'color_value'])
    log(0, 0, "program_start", get_color())

def log(x, y, state, color_value):
    now = time.time() - start
    # Next four lines log real time of the coordinates update
    # The purpose of this is to simplify debugging when identifying flaws of the csv construction
    hrs = int(now // 3600)         #hours
    mins = int((now % 3600) // 60) #minutes
    sec = now % 60                 #seconds
    log_time = "{:02d}:{:02d}:{:06.3f}".format(hrs, mins, sec)

    with open(csv_file, 'a', newline='') as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow([log_time, "{:.2f}".format(x), "{:.2f}".format(y), state, int(color_value)])

# Coordinates arithmetics via odometry
def update():
    global x, y, heading, lle, lre

    # Getting physical position of motors
    left_motor = move_steering.left_motor.position
    right_motor = move_steering.right_motor.position

    # Calculate the distance each wheel travelled 
    l_wheel_distance = (left_motor - lle) * WHEEL_C / 360
    r_wheel_distance = (right_motor - lre) * WHEEL_C / 360

    # Update encoder positions
    lle = left_motor
    lre = right_motor

    # Calculate forward movement and rotation
    distance = (l_wheel_distance + r_wheel_distance) / 2
    rotation = (r_wheel_distance - l_wheel_distance) / AXLE

    # Update heading using gyro (more reliable than wheel encoders for rotation)
    heading = gyro_sensor_tracker.angle * math.pi / 180

    # Update position
    x += distance * math.sin(heading)
    y += distance * math.cos(heading)

    return x, y

def store():
    global last_log

    now = time.time()
    if now - last_log < LOG_INT:
        return None

    last_log = now
    return update()

# Note: Starting from line 124 most of the comments were left out due to irrelecance 
# for the second lab. To get more understanding on line following functionality you
# may reffer to the first lab under directory my_robot in the same github repository
def get_color():
    value = color_sensor.reflected_light_intensity
    recent_values.append(value)
    if len(recent_values) > BUFFER_SIZE:
        recent_values.pop(0)
    return value

def is_green():
    if len(recent_values) < BUFFER_SIZE:
        return get_color() <= GREEN_MAX
    return all(v <= GREEN_MAX for v in recent_values[-7:])

def is_grey():
    value = get_color()  
    if GREY_MIN <= value <= GREY_MAX:
        grey_count = sum(1 for v in recent_values[-5:] if GREY_MIN <= v <= GREY_MAX)
        return grey_count >= 3
    return False

def is_white():
    if len(recent_values) < BUFFER_SIZE:
        return get_color() >= WHITE_MIN
    return all(v >= WHITE_MIN for v in recent_values[-12:])

# It is important to note that the robot only stores the coordinates
# while following the green line for better map picture
def follow_green_line():
    global is_line_followed, grey_lines_crossed, is_program_complete

    sensor_value = get_color()
    error = sensor_value - (GREEN_MAX / 2)

    # Indication of robot's position on the green line
    coords = store()
    if coords:
        x, y = coords
        # Update header info
        log(x, y, "moving", sensor_value) 
    
    if is_white():
        is_line_followed = False
        return

    # Indication of grey line detection
    if is_grey():
        coords = store()
        if coords:
            x, y = coords
            # Update header info
            log(x, y, "grey_detected", sensor_value)

        grey_lines_crossed += 1

        if grey_lines_crossed == 1:
            gyro_sensor.reset()
            move_steering.off()
            time.sleep(0.3)

            move_steering.on(steering=100, speed=20)
            while gyro_sensor.angle < 345:
                time.sleep(0.01)

            move_steering.on(steering=0, speed=FORWARD_SPEED)
            time.sleep(0.3)
            move_steering.off()

        elif grey_lines_crossed >= 2:
            coords = store()
            if coords:
                x, y = coords

                # Update header info to specify the end of the program
                log(x, y, "program_end", sensor_value)
            is_program_complete = True
            move_steering.off()
        return

    steering = error * TURN_GAIN
    steering = max(-80, min(80, steering))
    move_steering.on(steering=steering, speed=FORWARD_SPEED)

def oscillated_search():
    global is_line_followed
    angles = [30, 60, 90, 120, 150, 180]

    for angle in angles:  
        gyro_sensor.reset()   
        move_steering.on(steering=-100, speed=OSCILLATION_SPEED)
        while gyro_sensor.angle > -angle:
            if is_green() or is_grey():
                is_line_followed = True
                return
            time.sleep(0.01)
        move_steering.off()

        gyro_sensor.reset()
        move_steering.on(steering=100, speed=OSCILLATION_SPEED)
        while gyro_sensor.angle < angle * 2:
            if is_green() or is_grey():
                is_line_followed = True
                return
            time.sleep(0.01)
        move_steering.off()

        gyro_sensor.reset()
        move_steering.on(steering=-100, speed=OSCILLATION_SPEED)
        while gyro_sensor.angle > -angle:
            time.sleep(0.01)
        move_steering.off()

    move_steering.off()

# create a new CSV and start the program
new_csv_file()

# Main part of the program
while not is_program_complete: 
    if is_line_followed:
        follow_green_line()
    else:
        oscillated_search()
    time.sleep(0.01)

move_steering.off()