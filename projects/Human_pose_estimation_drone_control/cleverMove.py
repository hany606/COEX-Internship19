import rospy
from clever import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool
from std_msgs.msg import String
import settings
import shared

def drone_land(land, arming):
    print("Landing")

    res = land()
    #shared.wait_till_arrive(get_telemetry, 0, 0, 0, 'body')

    if(res.success):
        print("Drone has been landed")

    shared.count_down(5)	#to ensure that it is landed already
    #there is a problem in landing that it reads that it lands already and it is not
    arming(False)

def take_off(get_telemetry,navigate):
    # Taking off with height: ~1.5m
    start_pos = get_telemetry()
    print(start_pos)
    print("Height: ~{:}m".format(settings.VIEW_HIGHT))
    navigate(z=settings.VIEW_HIGHT, speed=settings.SPEED, frame_id='body', auto_arm=True)
    shared.wait_till_arrive(get_telemetry, 0, 0, start_pos.z+settings.VIEW_HIGHT, 'map',1)
    #shared.count_down(2)
def move(target_x, target_y, get_telemetry,navigate):
    current_pos = get_telemetry(frame_id='aruco_map')
    navigate(x=target_x, y=target_y, z=current_pos.z, yaw=float('nan'), speed=settings.SPEED, frame_id='aruco_map')
    shared.wait_till_arrive(get_telemetry, target_x, target_y, current_pos.z)
    #shared.count_down(2)
