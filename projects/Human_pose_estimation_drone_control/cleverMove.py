#---------------------------------------------------
# This code is part of this project: https://github.com/hany606/COEX-Internship19/tree/master/projects/Human_pose_estimation_drone_control 
# Author: Hany Hamed
# During COEX Internship July 2019
# About: This file contain some of the essential methods to make the drone takeoff, fly and land
#---------------------------------------------------

# Importing necessary libraries
import rospy    #ros library for python
# Libraries to use the required services in clever package in ros
from clever import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool
# Important predefined libraries
import settings # to import some parameters
import util     # to import some methods to ease working

# This method is performing landing for the drone
# Parameters: land  -> land clever ros service
#             arming-> arming clever ros service
def drone_land(land, arming):
    print("Landing")

    res = land()    # Start landing of the drone
    #util.wait_till_arrive(telemetry, 0, 0, 0, 'body')

    # If the a landing is success
    if(res.success):
        print("Drone has been landed")

    util.count_down(5)	#to ensure that it is landed already
    #there is a problem in landing that it reads that it lands already and it is not
    arming(False)   # Stop the motors of the drone

# This method is performing taking off for the drone
# Parameters: telemetry  -> telemetry clever ros service which has all the sensors values
#             navigate       -> navigate clever ros service which is responsible to go from position to another one
def take_off(telemetry,navigate):
    start_pos = telemetry() # Get telemetry from the starting position with map frame
    # print(start_pos)
    print("Height: ~{:}m".format(settings.VIEW_HIGHT))
    # Go up until reaching that height
    navigate(z=settings.VIEW_HIGHT, speed=settings.SPEED, frame_id='body', auto_arm=True)
    util.wait_till_arrive(telemetry, 0, 0, start_pos.z+settings.VIEW_HIGHT, 'map',1)
    #util.count_down(4)

# This method is performing move the drone for specific (x,y) in the ArUco map
# Parameters: target_x       -> x-coordinate
#             target_y       -> y-coordinate
#             telemetry  -> telemetry clever ros service which has all the sensors values
#             navigate       -> navigate clever ros service which is responsible to go from position to another one
def move(target_x, target_y, telemetry,navigate, frame_id='aruco_map'):
    current_pos = telemetry(frame_id=frame_id)
    navigate(x=target_x, y=target_y, z=current_pos.z, yaw=float('nan'), speed=settings.SPEED, frame_id=frame_id)
    util.wait_till_arrive(telemetry, target_x, target_y, current_pos.z)
    #util.count_down(3)
