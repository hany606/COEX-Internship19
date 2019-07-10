#This code is just for testing to get the data of the sensors from the topics, services and move the drone a little bit.

import rospy
from clever import srv
from std_srvs.srv import Trigger
#from time import sleep		#don't use time.sleep with ros it is better to use rospy.sleep as the first one is blocking function which block the ros nodes too
from mavros_msgs.srv import CommandBool
from std_msgs.msg import String
from shared import count_down
import shared
import settings


target_x = 3
target_y = 3
out_flag = 0

rospy.init_node("simple_teleop_node") # name of your ROS node

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

def drone_land():
    print("Landing")

    res = land()
    #shared.wait_till_arrive(get_telemetry, 0, 0, 0, 'body')

    if(res.success):
        print("Drone has been landed")

    count_down(5)	#to ensure that it is landed already
    #there is a problem in landing that it reads that it lands already and it is not
    arming(False)

def take_off():
    # Taking off with height: ~1.5m
    start_pos = get_telemetry()
    print(start_pos)
    print("Height: ~1.5m")
    navigate(z=settings.VIEW_HIGHT, speed=settings.SPEED, frame_id='body', auto_arm=True)
    shared.wait_till_arrive(get_telemetry, 0, 0, start_pos.z+settings.VIEW_HIGHT, 'map',1)
    # count_down(3)
    # current_pos = get_telemetry("aruco_map")
    # navigate(x=current_pos.x, y=current_pos.y, z=current_pos.z, speed=settings.SPEED, frame_id='aruco_map')
    # shared.wait_till_arrive(get_telemetry,  current_pos.x, current_pos.y, current_pos.z)

def task1(target_x, target_y):
    # target_x = 3
    # target_y = 3
    current_pos = get_telemetry(frame_id='aruco_map')
    navigate(x=target_x, y=target_y, z=current_pos.z, yaw=float('nan'), speed=settings.SPEED, frame_id='aruco_map')
    shared.wait_till_arrive(get_telemetry, target_x, target_y, current_pos.z)

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

def main():
    
    take_off()
    task1(3,2)
    print("Ready to be controlled from keyboard")
    rospy.Subscriber("key_value", String, key_callback)
    print("Get ready in:")
    count_down(5)

    while(out_flag == 0):
        global target_x
        global target_y
        task1(target_x,target_y)
        print(target_x, target_y, out_flag) 

    drone_land()


if __name__ == '__main__':    
    print("Starting the task after:")
    count_down(3)
    print("--------------Start--------------")
    main()
    print("--------------Done--------------")
