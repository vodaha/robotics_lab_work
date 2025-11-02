#!/usr/bin/env python3

# Must-have libraries
from ev3dev2.motor import MoveSteering, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import ColorSensor, GyroSensor
import time

# Modules assignment
move_steering = MoveSteering(OUTPUT_B, OUTPUT_C)
color_sensor = ColorSensor()
gyro_sensor = GyroSensor()

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

# A buffer that serves as a sensor values tracker
recent_values = []
BUFFER_SIZE = 15

# A method to retrieve a color sensor value
def get_color():
    value = color_sensor.reflected_light_intensity

    recent_values.append(value)
    if len(recent_values) > BUFFER_SIZE:
        recent_values.pop(0)
    return value

# A method to check if a robot must act upon the color green
def is_green():
    # Only consider it green if sensor recieves green consistently
    if len(recent_values) < BUFFER_SIZE:
        return get_color() <= GREEN_MAX
    return all(v <= GREEN_MAX for v in recent_values[-7:])  # Considers green according to last 7 values

# A method to check if a robot must act upon the color grey
def is_grey():
    # Only consider it grey if sensor recieves grey consistently
    value = get_color()  
    
    if GREY_MIN <= value <= GREY_MAX:
        grey_count = sum(1 for v in recent_values[-5:] if GREY_MIN <= v <= GREY_MAX)
        return grey_count >= 3 # Considers grey if at least 3 out of last five sensor reads were grey
    return False

# A method to check if a robot must act upon the color white
def is_white():
    # Only consider it white if sensor recieves white consistently
    if len(recent_values) < BUFFER_SIZE:
        return get_color() >= WHITE_MIN
    return all(v >= WHITE_MIN for v in recent_values[-12:])  # Considers white according to last 12 values

# A method to make a robot follow a green line
def follow_green_line():
    # Accessing global constants
    global is_line_followed, grey_lines_crossed, is_program_complete
    
    sensor_value = get_color()
    error = sensor_value - (GREEN_MAX / 2)
    
    # Stop following the green line when lost
    if is_white():
        is_line_followed = False
        return
    
    # Found the grey cross
    if is_grey():
        # Increment the number of grey lines crossed when faced with one 
        grey_lines_crossed += 1
        
        # If the grey line is first do the 360
        if grey_lines_crossed == 1:
            gyro_sensor.reset() # Reset the gyro sensor 
            move_steering.off() # and stop moving
            time.sleep(0.3)

            move_steering.on(steering=100, speed=20) # Start spinning
            while gyro_sensor.angle < 345:           # until the angle is
                time.sleep(0.01)                     # 360 (355 is chosen for better accuracy)

            move_steering.on(steering=0, speed=FORWARD_SPEED) # Step forward
            time.sleep(0.3)                                   # for 300ms after 360
            move_steering.off()                               # to prevent false detection

        # If the grey line is second finalize the program
        elif grey_lines_crossed >= 2:
            is_program_complete = True
            move_steering.off()
        return
    
    steering = error * TURN_GAIN
    steering = max(-80, min(80, steering))
    move_steering.on(steering=steering, speed=FORWARD_SPEED)

# A method to make a robot search for the green line (or grey crosses)
def oscillated_search():
    # Accessing a global constant
    global is_line_followed
    
    # A range of angles to use while search 
    angles = [30, 60, 90, 120, 150, 180]
    
    # Repeat oscillated search move for each angle in the range until green or grey is found
    for angle in angles:  
        # Reset gyro value to zero before starting the counter-clockwise oscillation cycle with a new angle
        gyro_sensor.reset()   
        move_steering.on(steering=-100, speed=OSCILLATION_SPEED)

        while gyro_sensor.angle > -angle:
            # While rotating constantly check a color sensor value for a green line or a grey cross
            if is_green() or is_grey():
                is_line_followed = True # Switch back to line following mode if a color match was found
                return
            time.sleep(0.01)
        move_steering.off()
        
        # Reset gyro value to zero before proceeding with the clockwise oscillation cycle
        gyro_sensor.reset()
        move_steering.on(steering=100, speed=OSCILLATION_SPEED)
        
        while gyro_sensor.angle < angle * 2:   #the angle is doubled for completion of the whole rotation 
            if is_green() or is_grey():
                is_line_followed = True
                return
            time.sleep(0.01)
        move_steering.off()
        
        # Reset gyro as usual and get back to starting position if no match was found
        gyro_sensor.reset()
        move_steering.on(steering=-100, speed=OSCILLATION_SPEED)
        while gyro_sensor.angle > -angle:
            time.sleep(0.01)
        move_steering.off()
        
    # If no match found at all stop the motors
    move_steering.off()

# Main part of the program
while not is_program_complete: 
    if is_line_followed:    # Keep following the green line until lost
        follow_green_line()
    else:                   # Otherwise search for non-white match (green or grey)
        oscillated_search()
    time.sleep(0.01)

# Stop movement when the program is complete
move_steering.off()