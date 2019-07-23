#---------------------------------------------------
# This code is part of this project: https://github.com/hany606/COEX-Internship19/tree/master/projects/Human_pose_estimation_drone_control 
# Author: Hany Hamed
# During COEX Internship July 2019
# About: This file contain some of the helping methods to use it with the drone
#---------------------------------------------------

# Importing necessary libraries
import sys
from rospy import sleep
import math


# This method is doing count down like a timer (sleep with fancy way)
# Parameters: times -> the number of times
#             step -> the step of sleep
# total time = times*step
def count_down(times, step = 1):
    for i in range(times+1):
        print("{:}...".format(times-i))
        sleep(step)


# This method is doing a safety check on that all the sensors values are correct
#       as there is a glitch in altitude values that makes the drone fly irrationally
# Parameters: telemetry  -> telemetry clever ros service which has all the sensors values
#             arming         -> arming clever ros service
def safety_check(telemetry,arming):
    velocity_z_axis = telemetry().vz
    if velocity_z_axis > 0.5:
	    tmp = raw_input("The velocity is too high, proceed? -y -n ")
            if(tmp == 'n'):
                    print("Getting Out")
                    arming(False)
                    sys.exit()

# This method is used to calculate the distances between two points in the space
def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

# This method is return if the drone reached specific point or not
# Parameters: telemetry  -> telemetry clever ros service which has all the sensors values
#             x          -> x-coordinate
#             y          -> y-coordinate
#             z          -> z-coordinate
#             frame_id   -> frame id of the position
def arrived(telemetry,x, y, z, frame_id='aruco_map'):
    tolerance = 0.2
    current_pos = telemetry(frame_id = frame_id)
    distance_difference = distance(x,y,z,current_pos.x,current_pos.y,current_pos.z)
    # print("Difference:{:}".format(distance_difference))
    if(distance_difference < tolerance):
        print("################ Arrived ################")
        return 1
    return 0

# This method is held in while loop until the drone arrive to the speific point
# Parameters: telemetry   -> telemetry clever ros service which has all the sensors values
#             x           -> x-coordinate
#             y           -> y-coordinate
#             z           -> z-coordinate
#             frame_id    -> frame id of the position
#             takeoff_mode-> if it is used in take off the drone or not flag
def wait_till_arrive(telemetry, x, y, z, frame_id='aruco_map', takeoff_mode=0):
    while(True):
        current_pos = telemetry(frame_id=frame_id)
        if(takeoff_mode == 1):
	        x,y = current_pos.x,current_pos.y
        # print("Target position:  x={:}, y={:}, z={:}\nCurrent position: x={:}, y={:}, z={:}".format(x,y,z,current_pos.x, current_pos.y, current_pos.z))
        if(arrived(telemetry, x, y, z, frame_id)):
            break
        sleep(0.2)
