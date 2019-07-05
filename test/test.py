#This code is just for testing to get the data of the sensors from the topics and services.

import rospy
from clever import srv
from std_srvs.srv import Trigger
#from time import sleep


rospy.init_node('test_node') # name of your ROS node

# Creating proxies to all services
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


print("Starting the task after:")

for i in range(6):
	print(5-i,"...")
	rospy.sleep(1)
navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
print("Height: ~1.5m")
for i in range(4):
	print(3-i,"...")
	rospy.sleep(1)
navigate(x=5, y=3, z=1.5, speed=0.5, frame_id='aruco_map')
print("Height: 1.5m")
for i in range(4):
	print(3-i,"...")
	rospy.sleep(1)

res = land()

if(res.success):
    print("Drone has been landed")

print("--------------Done--------------")



# for i in range(10):
	# print("x: {:} , y: {:} , z: {:}".format(get_telemetry().x, get_telemetry().y, get_telemetry().z))
	#sleep(1)

#for i in range(10):
# while(True):
# 	print(get_telemetry())
# 	print("-------------------")
# 	sleep(1)


