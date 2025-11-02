#!/usr/bin/env python3

from ev3dev2.motor import MoveSteering, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import ColorSensor, GyroSensor, UltrasonicSensor
from ev3dev2.sound import Sound
import time

# Modules assignment
move_steering = MoveSteering(OUTPUT_B, OUTPUT_C)
color_sensor = ColorSensor()
gyro_sensor = GyroSensor()
ultrasonic_sensor = UltrasonicSensor()
sound = Sound()

# Color thresholds
BLACK_MAX = 15
WHITE_MIN = 50

# Movement constants
FORWARD_SPEED = 40
TURN_GAIN = 0.1
OSCILLATION_SPEED = 25

# Global modifiables
is_line_followed = True
is_program_complete = False
obstacle_count = 0
is_obstacle_detected = False

# A buffer that serves as a sensor values tracker.Pretty much the most refundant consept of the
# program in its current state. It is, however, reasonably utilized in the first lab under my_robot
# directory in the same github repository, from which it was taken and decided to be kept in case of
# new changes
recent_values = []
BUFFER_SIZE = 15

# A method to retrieve ultrasonic distance in cm
def uv_distance():
    return ultrasonic_sensor.distance_centimeters

# A method to go around obstacle from the right
def box_skip():
    # Turn 90 degrees to the right
    gyro_sensor.reset()
    move_steering.on(steering=100, speed=OSCILLATION_SPEED)
    while gyro_sensor.angle < 85:
        time.sleep(0.01)
    move_steering.off()
    time.sleep(0.5)
    
    # Go forward for a second
    move_steering.on(steering=0, speed=FORWARD_SPEED)
    time.sleep(1)
    move_steering.off()
    time.sleep(0.5)
    
    # Turn 90 degrees to the left
    gyro_sensor.reset()
    move_steering.on(steering=-100, speed=OSCILLATION_SPEED)
    while gyro_sensor.angle > -85:
        time.sleep(0.01)
    move_steering.off()
    time.sleep(0.5)

    # Smoothly go around the box and continue following the line
    # as soon as it is found
    move_steering.on(steering=-10, speed=FORWARD_SPEED)
    while not is_black():
        time.sleep(0.01)
    move_steering.off()
    return
    
# A method to handle obstacles
def check_obstacles():
    global is_obstacle_detected, obstacle_count
    
    distance = uv_distance()
    
    # An obstacle on 30 cm distance from the uv sensor is the wall
    # The robot does a 90 degree clockwise turn after detection
    # This is done to handle dead end near the end in order not 
    # to waste time on reaching it and coming back
    if distance <= 30 and obstacle_count == 1: #only detect objects on distance<30cm
                                               #if detected obstacle is not the first 
        move_steering.off()
        gyro_sensor.reset()
        move_steering.on(steering=100, speed=25)
        while gyro_sensor.angle < 90:
            time.sleep(0.01)

    # If an obstacle is the first detected obstacle and the distance is no more than 4cm
    # it is a removable box 
    elif distance <= 4:
        move_steering.off()
        sound.beep()
        obstacle_count += 1 #increment the number of obstacles
        box_skip() #skip the box by going around it

    return

# Get the color value and update the buffer
def get_color():
    value = color_sensor.reflected_light_intensity
    recent_values.append(value) #add the most recent value to the array
    if len(recent_values) > BUFFER_SIZE:
        recent_values.pop(0) #pop the old value if the number of last readings exceeds 15
    return value

# Check a sensor reading for black
def is_black():
    return get_color() <= BLACK_MAX

# Check a sensor reading for white
def is_white():
    return get_color() >= WHITE_MIN

# A method to make robot follow a black line
def follow_black_line():
    # Accessing globals
    global is_line_followed, is_program_complete

    # Check for the uv sensor
    distance = uv_distance()
    if distance <= 30:
        check_obstacles()

    # Calculate error
    sensor_value = get_color()
    error = sensor_value - ((11 + 90)/2)

    # If a sensor reading shows white stop following
    if is_white():
        is_line_followed = False
        return

    # Adjust steering motion accroding to error for smooth following
    steering = -error * TURN_GAIN
    steering = max(-80, min(80, steering))
    move_steering.on(steering=steering, speed=FORWARD_SPEED)

# A method to make a robot search for the black line
def oscillated_search():
    # Accessing globals
    global is_line_followed, is_program_complete

    # Check for obstacles
    check_obstacles()

    # A range of angles to use while search 
    angles = [10, 30, 90, 120]
    
    # Repeat oscillated search move for each angle in the range until black is found
    for angle in angles:  
        # Reset gyro value to zero before starting the counter-clockwise oscillation cycle with a new angle
        gyro_sensor.reset()   
        move_steering.on(steering=-100, speed=OSCILLATION_SPEED)

        while gyro_sensor.angle > -angle:
            # While rotating constantly check a color sensor value for a black line and
            # uv sensor for obstacles
            check_obstacles()
            if is_black():
                is_line_followed = True # Switch back to line following mode if a color match was found
                return
            time.sleep(0.01)
        move_steering.off()
        
        # Reset gyro value to zero before proceeding with the clockwise oscillation cycle
        gyro_sensor.reset()
        move_steering.on(steering=100, speed=OSCILLATION_SPEED)
        
        while gyro_sensor.angle < angle * 2: #the angle is doubled for completion of the whole rotation 
            check_obstacles()
            if is_black():
                is_line_followed = True
                return
            time.sleep(0.01)
        move_steering.off()
        
        # Reset gyro as usual and get back to starting position if no match was found
        gyro_sensor.reset()
        move_steering.on(steering=-100, speed=OSCILLATION_SPEED)
        while gyro_sensor.angle > -angle:
            check_obstacles()
            time.sleep(0.01)
        move_steering.off()
    
    # If no match found at all for the specified mark the program as complete
    is_program_complete = True

# Main part of the program
while not is_program_complete: 
    if is_line_followed:    # Keep following the black line until lost
        follow_black_line()
    else:                   # Otherwise search for black line
        oscillated_search()
    time.sleep(0.01)

# Stop movement when the program is complete
move_steering.off()