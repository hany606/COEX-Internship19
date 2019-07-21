#This code is just for testing to get the data of the sensors from the topics, services and move the drone a little bit.

import rospy
from clever import srv
from std_srvs.srv import Trigger
#from time import sleep		#don't use time.sleep with ros it is better to use rospy.sleep as the first one is blocking function which block the ros nodes too
from mavros_msgs.srv import CommandBool
import shared
from shared import count_down
import settings
from std_msgs.msg import String
target_x = 3
target_y = 3
out_flag = 0


def key_callback(data):
    #print(type(data.data))
    s = data.data
    tmp_x = 0
    tmp_y = 0
    if(s == 'w'):
        print("Forward")
        tmp_x = 1
    elif(s == 's'):
        print("Backward")
        tmp_x = -1
    elif(s == 'd'):
        print("Right")
        tmp_y = 1
    elif(s == 'a'):
        print("Left")
        tmp_y = -1
    elif(s == 'q'):
        print("Break")
        global out_flag
        out_flag = 1
    global target_x
    global target_y
    target_x = min(max(target_x+tmp_x,settings.ArUcoMAPMIN_x),settings.ArUcoMAPMAX_x)
    target_y = min(max(target_y+tmp_y,settings.ArUcoMAPMIN_y),settings.ArUcoMAPMAX_y)
rospy.init_node('teleop_move_node') # name of your ROS node

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

shared.safety_check(get_telemetry, arming)

print("Starting the task after:")
count_down(3)
print("Height: ~2.5m")
navigate(x=0, y=0, z=settings.VIEW_HIGHT, speed=settings.SPEED, frame_id='body', auto_arm=True)
#shared.wait_till_arrive(get_telemetry, 0, 0, settings.VIEW_HIGHT, 'map')
count_down(3)
print("Height: 2m move to ArUco (5,3)")
navigate(x=target_x, y=target_y, z=settings.VIEW_HIGHT, speed=settings.SPEED, frame_id='aruco_map')
shared.wait_till_arrive(get_telemetry, target_x, target_y, settings.VIEW_HIGHT)

print("Ready to be controlled from keyboard")
rospy.Subscriber("keyboard_value", String, key_callback)
print("Get ready in:")
count_down(5)

while(out_flag == 0):
    navigate(x=target_x, y=target_y, z=settings.VIEW_HIGHT, speed=settings.SPEED, frame_id='aruco_map')
    shared.wait_till_arrive(get_telemetry, target_x, target_y, settings.VIEW_HIGHT)
    print(target_x, target_y, out_flag) 
    
res = land()
#shared.wait_till_arrive(get_telemetry, 0, 0, 0, 'body')


if(res.success):
    print("Drone has been landed")

count_down(5)	#to ensure that it is landed already
#there is a problem in landing that it reads that it lands already and it is not
arming(False)
print("--------------Done--------------")
