#This code is just for testing to get the data of the sensors from the topics, services and move the drone a little bit.

import rospy
from clever import srv
from std_srvs.srv import Trigger
#from time import sleep		#don't use time.sleep with ros it is better to use rospy.sleep as the first one is blocking function which block the ros nodes too


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




# for i in range(10):
	# print("x: {:} , y: {:} , z: {:}".format(get_telemetry().x, get_telemetry().y, get_telemetry().z))
	#sleep(1)

# Getting the data from the service
while(True):
	print(get_telemetry())
	print("-------------------")
	rospy.sleep(0.5)


