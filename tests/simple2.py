import rospy
from clever import srv
from std_srvs.srv import Trigger
#from time import sleep		#don't use time.sleep with ros it is better to use rospy.sleep as the first one is blocking function which block the ros nodes too
from mavros_msgs.srv import CommandBool
from shared import count_down
import shared
import settings

rospy.init_node("simple_node") # name of your ROS node

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


z = 2
tolerance = 0.2
start = get_telemetry()

print navigate(z=z, speed=0.5, frame_id='body', auto_arm=True)

while True:
    if get_telemetry().z - start.z + z < tolerance:
        break
    rospy.sleep(0.2)

navigate(x=2, y=2, z=2, speed=1, frame_id='aruco_map')

drone_land()
