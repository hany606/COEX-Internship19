#This code is to fly on a specific path on the ArUco map
# TODO: Find a way if the x and y of the current aruco markers is being published or stored

import rospy
from clever import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool
import shared
from shared import count_down


length_x = 3
length_y = 3


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
count_down(5)
print("Height: ~{:}m".format(VIEW_HIGHT))
navigate(x=0, y=0, z=VIEW_HIGHT, speed=0.5, frame_id='body', auto_arm=True)
count_down(3)

print("Height: {:}m move to ArUco (5,3)".format(VIEW_HIGHT))
navigate(x=5, y=3, z=VIEW_HIGHT, speed=0.5, frame_id='aruco_map')
count_down(3)


# ----------------------- This part will move in 'L' shape -----------------------
start_x  = 5
start_y  = 3
target_x = max(start_x + length_x, shared.ArUcoMAPMAX_x)
target_y = max(start_y + length_y, shared.ArUcoMAPMAX_y)
for i in range(start_x, target_x+1):
    print("Height: {:}m move to ArUco ({:},3)".format(shared.VIEW_HIGHT,i))
    navigate(x=i, y=3, z=shared.VIEW_HIGHT, speed=0.5, frame_id='aruco_map')
    count_down(3)

for i in range(start_y, target_y+1):
    print("Height: {:}m move to ArUco ({:},{:})".format(shared.VIEW_HIGHT,target_x,i))
    navigate(x=x, y=i, z=shared.VIEW_HIGHT, speed=0.5, frame_id='aruco_map')
    count_down(3)

# -------------------------------------------------------------------------------


res = land()

if(res.success):
    print("Drone has been landed")

count_down(5)
arming(False)
print("--------------Done--------------")
