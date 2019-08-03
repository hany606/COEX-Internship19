###############################################################################################
# This code was to control a drone using EEG Brain signals that 
#   is emulated by a program that emulates each person's emotion with key press
#
# Author: Hany Hamed 
# (This project was a team project and my responsibility to write the code for control the drone)
###############################################################################################
# importing of the necessary libraries
import math
import rospy
from clever import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool
from std_msgs.msg import String
import shared

# Targets coordinates
target_x = 8
target_y = 6
target_yaw = 0

# Parameters of the ArUco map
ArUcoMAPMIN_x = 7
ArUcoMAPMIN_y = 5
ArUcoMAPMAX_x = 9
ArUcoMAPMAX_y = 9
VIEW_HEIGHT = 1.5
SPEED = 0.5

# Necessary global flags
out_flag = 0    # To get out from the main loop by a key press
key_flag = 0  # To indicate which key has been pressed
transition_flag = 0 # To wait until the jey of the transition is pressed as the emotion is continuous signal and to make it more controllable

rospy.init_node("move_node") # name of your ROS node

# Creating proxies to all services
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

# Safety check for the error of throttling speed
shared.safety_check(get_telemetry, arming)

def key_callback(data):
    s = data.data
    print(s)
    tmp = 0
    global key_flag
    global target_yaw
    global transition_flag
    if(transition_flag == 0):
        if(s == 'w'):
            print("Forward")
            tmp = 1
            key_flag = 1
            transition_flag = 1
        elif(s == 's'):
            print("Right_rotate - yaw rotate")
            key_flag = 2
            transition_flag = 1
            target_yaw = ((target_yaw+1)%4)
    if(s == 'o'):
        print("Transition")
        transition_flag = 0
    elif(s == 'q'):
        print("Break")
        global out_flag
        out_flag = 1
    global target_y
    global target_x
    # This conditions is made to make the forward corresponds to the orientation of the drone after rotations
    # min max function is made to limit the playground of the ArUco Map
    if(target_yaw == 1):
        target_y = min(max(target_y+tmp,ArUcoMAPMIN_y),ArUcoMAPMAX_y)
    elif(target_yaw == 0):
        target_x = min(max(target_x+tmp,ArUcoMAPMIN_x),ArUcoMAPMAX_x)
    elif(target_yaw == 3):
        target_y = min(max(target_y-tmp,ArUcoMAPMIN_y),ArUcoMAPMAX_y)
    elif(target_yaw == 2):
        target_x = min(max(target_x-tmp,ArUcoMAPMIN_x),ArUcoMAPMAX_x)
        

def drone_land():
    print("Landing")
    res = land()
    if(res.success):
        print("Drone has been landed")

    shared.count_down(5)	#to ensure that it is landed already
    #there is a problem in landing that it reads that it lands already and it is not
    arming(False)

def take_off():
    # Taking off with height: ~1.5m
    start_pos = get_telemetry()
    print(start_pos)
    print("Height: ~{:}m".format(VIEW_HEIGHT))
    navigate(z=VIEW_HEIGHT, speed=SPEED, frame_id='body', auto_arm=True)
    #shared.wait_till_arrive(get_telemetry, 0, 0, start_pos.z+VIEW_HEIGHT, 'map',1)
    shared.count_down(4)
    
def move(target_x, target_y):
    #current_pos = get_telemetry(frame_id='aruco_map')
    navigate(x=target_x, y=target_y, z=VIEW_HEIGHT, yaw=float('nan'), speed=SPEED, frame_id='aruco_map')
    #shared.wait_till_arrive(get_telemetry, target_x, target_y, current_pos.z)
    shared.count_down(4)

def rotate_right_yaw():
    print("Rotate to: {:} Degree -> {:} Rad".format(-1*target_yaw*90, -1*target_yaw*((math.pi)/2.0)))
    navigate(x=target_x,y=target_y,z=VIEW_HEIGHT, yaw=-1*target_yaw*((math.pi)/2.0), speed=SPEED, frame_id='aruco_map')
    shared.count_down(4)

def main():
    take_off()
    print("move to {:},{:}".format(target_x, target_y))
    move(target_x,target_y)
    print("Ready to be controlled from keyboard")
    rospy.Subscriber("key_value", String, key_callback)
    print("Get ready in:")
    shared.count_down(5)

    while(out_flag == 0):
        global target_x
        global target_y
        global key_flag
        if(key_flag > 0 and transition_flag == 1):
            if(key_flag == 1):
                print("move to {:},{:}".format(target_x,target_y))
                move(target_x,target_y)
            if(key_flag == 2):
                print("Yaw rotation")
                rotate_right_yaw()
            print(target_x, target_y, out_flag, key_flag)
            global key_flag
            key_flag = 0
            
    drone_land()

if __name__ == '__main__':
    print("Starting the task after:")
    shared.count_down(3)
    print("--------------Start--------------")
    main()
    print("--------------Done--------------")
