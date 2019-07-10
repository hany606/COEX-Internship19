import sys
from rospy import sleep
import math

def count_down(x, t = 1):
    for i in range(x+1):
        print("{:}...".format(x-i))
        sleep(t)

def safety_check(telemetry,arming):
    velocity_z_axis = telemetry().vz
    if velocity_z_axis > 0.5:
	    tmp = raw_input("The velocity is too high, proceed? -y -n ")
            if(tmp == 'n'):
                    print("Getting Out")
                    arming(False)
                    sys.exit()

def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)


def arrived(telemetry,x, y, z, frame_id='aruco_map'):
    tolerance = 0.2
    current_pos = telemetry(frame_id = frame_id)
    distance_difference = get_distance(x,y,z,current_pos.x,current_pos.y,current_pos.z)
    # print("Difference:{:}".format(distance_difference))
    if(distance_difference < tolerance):
        print("################ Arrived ################")
        return 1
    return 0

def wait_till_arrive(telemetry, x, y, z, frame_id='aruco_map', takeoff_mode=0):
    while(True):
        current_pos = telemetry(frame_id=frame_id)
        if(takeoff_mode == 1):
	        x,y = current_pos.x,current_pos.y
        # print("Target position:  x={:}, y={:}, z={:}\nCurrent position: x={:}, y={:}, z={:}".format(x,y,z,current_pos.x, current_pos.y, current_pos.z))
        if(arrived(telemetry, x, y, z, frame_id)):
            break
        sleep(0.2)
