#This code is just for testing to get the data of the sensors from the topics, services and move the drone a little bit.

import rospy
from clever import srv
from std_srvs.srv import Trigger
#from time import sleep		#don't use time.sleep with ros it is better to use rospy.sleep as the first one is blocking function which block the ros nodes too
from mavros_msgs.srv import CommandBool
import shared
from shared import count_down
import settings

rospy.init_node('test_map_node') # name of your ROS node

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
navigate(x=3, y=3, z=settings.VIEW_HIGHT, speed=settings.SPEED, frame_id='aruco_map')
shared.wait_till_arrive(get_telemetry, 3, 3, settings.VIEW_HIGHT)
# count_down(3)

# TODO: Test navigate in not passing some parameters
#navigate(x=4, z=2, speed=0.5, frame_id='aruco_map')
# TODO: Test navigate with passing the paremeter as it is
#navigate(x=x, y=3, z=2, speed=0.5, frame_id='aruco_map')

# TODO: To find out how to know the position of the x and y in aruco markers from navigate not from stroing inside the program


res = land()
shared.wait_till_arrive(get_telemetry, 0, 0, 0, 'body')


if(res.success):
    print("Drone has been landed")

count_down(5)	#to ensure that it is landed already
#there is a problem in landing that it reads that it lands already and it is not
arming(False)
print("--------------Done--------------")
