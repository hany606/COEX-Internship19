#This code is just for testing to get the data of the sensors from the topics, services and move the drone a little bit.

import rospy
from clever import srv
from std_srvs.srv import Trigger
#from time import sleep		#don't use time.sleep with ros it is better to use rospy.sleep as the first one is blocking function which block the ros nodes too
from mavros_msgs.srv import CommandBool
from std_msgs.msg import String
import imp

shared = imp.load_source('settings', '../../libs/settings.py')
settings = imp.load_source('settings', '../../libs/settings.py')



target_x = 3
target_y = 3
out_flag = 0
return_track_flag = 0
moves_record = []
key_change = 0

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

def memorize_move(target_x, target_y):
    # Take the advantage of lazy evaluation of python
    if(len(moves_record) == 0 or moves_record[-1] != (target_x,target_y)):
        global moves_record
        moves_record.append((target_x, target_y))

def key_callback(data):
    #print(type(data.data))
    s = data.data
    tmp_x = 0
    tmp_y = 0
    global key_change
    key_change = 1
    if(s == 'w'):
        print("Forward")
        tmp_y = 1
    elif(s == 's'):
        print("Backward")
        tmp_y = -1
    elif(s == 'd'):
        print("Right")
        tmp_x = 1
    elif(s == 'a'):
        print("Left")
        tmp_x = -1
    elif(s == 'm'):
        print("Return the track")
        global return_track_flag
        return_track_flag = 1
    elif(s == 'q'):
        print("Break")
        global out_flag
        out_flag = 1
    global target_x
    global target_y
    global moves_record
    target_x = min(max(target_x+tmp_x,settings.ArUcoMAPMIN_x),settings.ArUcoMAPMAX_x)
    target_y = min(max(target_y+tmp_y,settings.ArUcoMAPMIN_y),settings.ArUcoMAPMAX_y)
    memorize_move(target_x, target_y)
def drone_land():
    print("Landing")

    res = land()
    #shared.wait_till_arrive(get_telemetry, 0, 0, 0, 'body')

    if(res.success):
        print("Drone has been landed")

    shared.count_down(5)	#to ensure that it is landed already
    #there is a problem in landing that it reads that it lands already and it is not
    arming(False)

def take_off():
    # Taking off with height: ~1.5m
    start_pos = get_telemetry()
    print(start_pos)
    print("Height: ~{:}m".format(settings.VIEW_HIGHT))
    navigate(z=settings.VIEW_HIGHT, speed=settings.SPEED, frame_id='body', auto_arm=True)
    shared.wait_till_arrive(get_telemetry, 0, 0, start_pos.z+settings.VIEW_HIGHT, 'map',1)

def move(target_x, target_y):
    current_pos = get_telemetry(frame_id='aruco_map')
    navigate(x=target_x, y=target_y, z=current_pos.z, yaw=float('nan'), speed=settings.SPEED, frame_id='aruco_map')
    shared.wait_till_arrive(get_telemetry, target_x, target_y, current_pos.z)

def main():
    take_off()
    move(target_x,target_y)
    print("Ready to be controlled from keyboard")
    rospy.Subscriber("key_value", String, key_callback)
    print("Get ready in:")
    shared.count_down(5)

    while(out_flag == 0):
        global target_x
        global target_y
        global key_change
        if(key_change):
            if(return_track_flag):
                global moves_record
                for i in range(2, len(moves_record)+1):
                    print("Return Return Return to:")
                    move(moves_record[-i][0], moves_record[-i][1])
                    print(moves_record[-i][0], moves_record[-i][1])
                global target_x
                global target_y
                target_x,target_y = moves_record[0]
            else:
                move(target_x,target_y)
                print(target_x, target_y, out_flag)
            global key_change
            key_change = 0
            


    drone_land()


if __name__ == '__main__':    
    print("Starting the task after:")
    shared.count_down(3)
    print("--------------Start--------------")
    main()
    print("--------------Done--------------")
