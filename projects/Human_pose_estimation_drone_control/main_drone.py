#---------------------------------------------------
# This code is part of this project: https://github.com/hany606/COEX-Internship19/tree/master/projects/Human_pose_estimation_drone_control 
# Author: Hany Hamed
# During COEX Internship July 2019
# About: This file contains the main code of the drone
#---------------------------------------------------
# Importing necessary libraries for websocket server
import os.path
import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web

import json
import math

import signal
import sys
# import them here too in order to prevent syntacticly errors as they already imported in other modules
import rospy
from clever import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool
from std_msgs.msg import String

import util
import settings
import cleverMove

# Global parameters of the target coordinates of the drone
target_x = 2
target_y = 5
target_z = 0
out_flag = 0    # Global flag to get out from the main loop


#Tornado Folder Paths
tornado_settings = dict(
	template_path = os.path.join(os.path.dirname(__file__), "templates"),
	static_path = os.path.join(os.path.dirname(__file__), "static")
	)

#Tonado server port
PORT = 8093

# Global flags related to the drone and the poses
initial_pose_flag = False       # Indicate if the user is doing the initial pose or not
action_transition_flag = False  # Indicate if the user done the initial pose after the required pose or not in order to enable the drone to do another move
doing_task_flag = False         # Indicate if the drone is busy doing a move/task or available to do a move


# Different Poses classification parameters
poses_config_thresh = 5
poses_config =[
     {"limits":[{"min": 130-poses_config_thresh, "max": 150+poses_config_thresh}], "name": "Forward",  "joints": ["left_elbow"],    "num_joints": 1}
    ,{"limits":[{"min": 40-poses_config_thresh,  "max": 60+poses_config_thresh}],  "name": "Backward", "joints": ["left_elbow"],    "num_joints": 1}
    ,{"limits":[{"min": 130-poses_config_thresh, "max": 150+poses_config_thresh}], "name": "Right",    "joints": ["right_elbow"],   "num_joints": 1}
    ,{"limits":[{"min": 30-poses_config_thresh,  "max": 50+poses_config_thresh}],  "name": "Left",     "joints": ["right_elbow"],   "num_joints": 1}
    ,{"limits":[{"min": 130-poses_config_thresh, "max": 150+poses_config_thresh}], "name": "Up",       "joints": ["right_shoulder"], "num_joints": 1}
    ,{"limits":[{"min": 50-poses_config_thresh,  "max": 70+poses_config_thresh}],  "name": "Down",     "joints": ["right_shoulder"], "num_joints": 1}
    # To add a new pose:
    # Do like below example
    #,{"limits":[{"min": minimum_angle1,  "max": maximum_anlge1}, {"min": minimum_angle2,  "max": maximum_anlge2}, ...],  "name": pose_name,     "joints": [joint1_name, joint2_name, ...], "num_joints": 2/...}
]



rospy.init_node("HPE_drone_node") # name of your ROS node

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
util.safety_check(get_telemetry, arming)

# Function for handling Ctrl+C and land the drone
def signalHandler(signal, frame):
    print("Ctrl+C is pressed")
    print("Drone is being landed")
    cleverMove.drone_land(land, arming)
    sys.exit(0)

def get_distance(p1, p2):
    return(math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2))

# From Radian to Degree
def to_degree(theta):
    return(180.0*theta/math.pi)

# Get angle between n3 Points
def get_angle(p1, p2, p3):
    # using Cosine law
    len_p1p2 = get_distance(p1, p2) 
    len_p1p3 = get_distance(p1, p3)
    len_p2p3 = get_distance(p2, p3)
    tmp_comp = len_p2p3**2 -  len_p1p2**2 - len_p1p3**2
    tmp_comp /=(-2*len_p1p2*len_p1p3)
    return(math.acos(tmp_comp))

# Calculate the needed angles from the points of the human skeleton
def calc_angles(skeleton):
    # keypoints of interest are rightShoulder(6), rightElbow(8), rightWrist(10), rightHip(12) and leftShoulder(5), rightElbow(7), rightWrist(9), leftHip(11)
    right_shoulder_point = list(skeleton[6]["position"].values())
    right_elbow_point    = list(skeleton[8]["position"].values())
    right_wrist_point    = list(skeleton[10]["position"].values())
    right_hip_point      = list(skeleton[12]["position"].values())
    
    left_shoulder_point = list(skeleton[5]["position"].values())
    left_elbow_point    = list(skeleton[7]["position"].values())
    left_wrist_point    = list(skeleton[9]["position"].values())
    left_hip_point      = list(skeleton[11]["position"].values())
    
    right_shoulder_angle = to_degree(get_angle(right_shoulder_point, right_elbow_point, right_hip_point))
    right_elbow_angle = to_degree(get_angle(right_elbow_point, right_shoulder_point, right_wrist_point))

    left_shoulder_angle = to_degree(get_angle(left_shoulder_point, left_elbow_point, left_hip_point))
    left_elbow_angle = to_degree(get_angle(left_elbow_point, left_shoulder_point, left_wrist_point))
    angles = {
        "right_shoulder": right_shoulder_angle,
        "right_elbow": right_elbow_angle,
        "left_shoulder": left_shoulder_angle,
        "left_elbow":left_elbow_angle
    }
    return angles

# To check the exsitance of the keypoints
def check_keypoints_existance(skeleton):
    right_shoulder_score = skeleton[6]["score"]
    right_elbow_score    = skeleton[8]["score"]
    right_wrist_score    = skeleton[10]["score"]
    right_hip_score      = skeleton[12]["score"]
    
    left_shoulder_score = skeleton[5]["score"]
    left_elbow_score    = skeleton[7]["score"]
    left_wrist_score    = skeleton[9]["score"]
    left_hip_score      = skeleton[11]["score"]

    flags_counter = 0
    score_thresh = 0.5 # Threshold for determine if the keypoint exist or not
    num_needed_keypoints = 6
    if(right_shoulder_score > score_thresh):
        flags_counter += 1
    else:
        print("Right Shoulder is not here, score{:}".format(right_shoulder_score))
    
    if(right_elbow_score > score_thresh):
        flags_counter += 1
    else:
        print("Right elbow is not here, score{:}".format(right_elbow_score))

    if(right_wrist_score > score_thresh):
        flags_counter += 1
    else:
        print("Right Wrist is not here, score{:}".format(right_wrist_score))
    
    if(left_shoulder_score > score_thresh):
        flags_counter += 1
    else:
        print("Left Shoulder is not here, score{:}".format(left_shoulder_score))
    
    if(left_elbow_score > score_thresh):
        flags_counter += 1
    else:
        print("Left elbow is not here, score{:}".format(left_elbow_score))

    if(left_wrist_score > score_thresh):
        flags_counter += 1
    else:
        print("Left Wrist is not here, score{:}".format(left_wrist_score))

    if(flags_counter == num_needed_keypoints):
        return 1
    else:
        print("Try to move back and empty background for more accuraccy")
    return 0

# To check the intial pose is made or not using the angles
def check_initial_pose(angles):
    threshold_err = 10
    flags_counter = 0
    if(angles["right_shoulder"] < 90+threshold_err and angles["right_shoulder"] > 90-threshold_err):
        flags_counter += 1
    else:
        print("Right Shoulder angle is not correct, current: {:}".format(angles["right_shoulder"]))

    if(angles["right_elbow"] < 90+threshold_err and angles["right_elbow"] > 90-threshold_err):
        flags_counter += 1
    else:
        print("Right Elbow angle is not correct, current: {:}".format(angles["right_elbow"]))

    if(angles["left_shoulder"] < 90+threshold_err and angles["left_shoulder"] > 90-threshold_err):
        flags_counter += 1
    else:
        print("Left Shoulder angle is not correct, current: {:}".format(angles["left_shoulder"]))

    if(angles["left_elbow"] < 90+threshold_err and angles["left_elbow"] > 90-threshold_err):
        flags_counter += 1
    else:
        print("Left Elbow angle is not correct, current: {:}".format(angles["left_elbow"]))

    if(flags_counter == 4):
        return 1
    return 0

def recognize_pose(angles):
    #{"limits":[{"min": 100-poses_config_thresh, "max": 120+poses_config_thresh}], "name": "Forward",  "joints": ["left_elbow"],    "num_joints": 1},
    for i,pose in enumerate(poses_config):
        pose_counter_flag = 0
        for x in range(len(pose["limits"])):
            joint = pose["joints"][x]
            min_limit = pose["limits"][x]["min"]
            max_limit = pose["limits"][x]["max"]
            print("Joint: {:}, current: {:}".format(joint, angles[joint]))
            if(angles[joint] >= min_limit and angles[joint] <= max_limit):
                pose_counter_flag += 1
                print("Satisfied")
        if(pose_counter_flag == pose["num_joints"]):
            return i+1
    return 0



def main(data):
    '''
    Example of the data:
      Incoming message: [{"score":0.24410172250560103,
        "keypoints":[
            {"score":0.9910107851028442,"part":"nose","position":{"x":252.86354913971303,"y":479.49954971729085}},
            {"score":0.9988914132118225,"part":"leftEye","position":{"x":217.64691708338398,"y":443.8451744703004}},
            {"score":0.9973353147506714,"part":"rightEye","position":{"x":295.1978541274015,"y":441.88124363524446}},
            {"score":0.526421844959259,"part":"leftEar","position":{"x":170.09496120348973,"y":468.07006360955745}},
            {"score":0.4756258726119995,"part":"rightEar","position":{"x":346.43769308008575,"y":451.6009378989847}},
            {"score":0.007388501428067684,"part":"leftShoulder","position":{"x":180.34555445570885,"y":418.50647685128894}},
            {"score":0.0072729201056063175,"part":"rightShoulder","position":{"x":154.96318226380106,"y":418.94439816011067}},
            {"score":0.017484841868281364,"part":"leftElbow","position":{"x":224.838224062196,"y":460.099917163181}},
            {"score":0.008609074167907238,"part":"rightElbow","position":{"x":340.5925497210907,"y":515.8285653080922}},
            {"score":0.007281377445906401,"part":"leftWrist","position":{"x":221.43342418151138,"y":484.6566760586394}},
            {"score":0.023914989084005356,"part":"rightWrist","position":{"x":296.11626698534775,"y":496.79646176586823}},
            {"score":0.011906704865396023,"part":"leftHip","position":{"x":220.51515381809338,"y":481.1839062879985}},
            {"score":0.016597222536802292,"part":"rightHip","position":{"x":305.0108288716713,"y":503.99571281462795}},
            {"score":0.014175777323544025,"part":"leftKnee","position":{"x":211.4903697448018,"y":483.11360281265206}},
            {"score":0.016598301008343697,"part":"rightKnee","position":{"x":220.3225012159533,"y":484.7732722063473}},
            {"score":0.013498651795089245,"part":"leftAnkle","position":{"x":205.4901745272981,"y":477.2344210732308}},
            {"score":0.015715690329670906,"part":"rightAnkle","position":{"x":238.2701501512342,"y":470.4443430622265}}
        ]}]
    '''
    data_json = json.loads(data.encode("utf-8"))
    if(len(data_json) == 0):
        print("No person in the image")
        return 0
    # print(data_json[0]["keypoints"])
    # print(data_json)
    if(doing_task_flag):
        print("[Status]: Doing the task TTTTTTTT")
        global target_x
        global target_y
        global target_z
        global get_telemetry
        if(target_z == 1):
            cleverMove.take_off(get_telemetry,navigate)
            print("[Status]: Drone take off")
        elif(target_z == 0):
            global land
            global arming
            cleverMove.drone_land(land, arming)
            print("[Status]: Drone landed")
        else:
            cleverMove.move(target_x,target_y,get_telemetry,navigate)
        global target_z
        target_z = -1
        doing_task_flag = False
        print("[Status]: Task is done")
        return 1
    keypoints = data_json[0]["keypoints"]
    # Check if all the keypoints are visible by the score
    keypoints_existance_flag = check_keypoints_existance(keypoints)
    global target_x
    global target_y
    global target_z
    print("[Status]: Current Drone Position ({:},{:},{:})".format(target_x,target_y,target_z))

    if(keypoints_existance_flag):  
        # Check if initial pose was done or not yet
        angles = calc_angles(keypoints)
        if(not(initial_pose_flag)):
            # Order the user to make the initial pose L shape for the arms
            # Check the pose
            print("[Status]: Detecting initial  ################")
            if(check_initial_pose(angles)):
                global initial_pose_flag
                global action_transition_flag
                initial_pose_flag = True
                action_transition_flag = False
                print("############# Successfully done the initial pose #############")
                print("initial pose has been done Hooray!! :)")

        else:
            angles = calc_angles(keypoints)
            pose = recognize_pose(angles)
            print("[Status]: Detecting pose $$$$$$$$$$$$$")
            if(not(action_transition_flag)):
                # Now it will be with discrete actions and values but after that it will be related to a vector of motion related to the angles
                global target_x
                global target_y
                global target_z
                if(pose == 1):
                    print("---------->Forward")
                    target_y += 1
                elif(pose == 2):
                    print("---------->Backward")
                    target_y -= 1
                elif(pose == 3):
                    print("---------->Right")
                    target_x += 1
                elif(pose == 4):
                    print("---------->Left")
                    target_x -= 1
                elif(pose == 5):
                    print("---------->Up")
                    target_z = 1
                elif(pose == 6):
                    print("---------->Down")
                    target_z = 0
                else:
                    print("Undefined Behaviour")
                
                if(pose > 0):
                    print("[Status]: Action transition stage triggered")
                    global action_transition_flag
                    action_transition_flag = True
            else:
                print("[Status]: Transition stage")
                print("Go to the initial pose again")
                global initial_pose_flag
                global doing_task_flag
                initial_pose_flag = False
                doing_task_flag = True
    else:
        print("[Status]: Detecting keypoints ******************")

class WSHandler(tornado.websocket.WebSocketHandler):
  def open(self):
    print ('[WS] Connection was opened.')
 
  def on_message(self, message):
    # print (('[WS] Incoming message:'), message)
    main(message)
  def on_close(self):
    print ('[WS] Connection was closed.')
  def check_origin(self,origin):
    return True

application = tornado.web.Application([(r'/ws', WSHandler)], **tornado_settings)

signal.signal(signal.SIGINT, signalHandler) # Activate the listen to the Ctrl+C


if __name__ == "__main__":
    try:
        http_server = tornado.httpserver.HTTPServer(application)
        http_server.listen(PORT)
        main_loop = tornado.ioloop.IOLoop.instance()

        print ("---- Drone going to origin ----")
        # global get_telemetry
        # global navigate
        # cleverMove.take_off(get_telemetry,navigate)
        # global target_z
        # target_z = settings.VIEW_HIGHT
        # cleverMove.move(target_x,target_y,get_telemetry,navigate)
        print("Now you can start")
        print("Please do the initial position to make your arms like L shape as it is described in the repo and wait")
        print ("Tornado Server started")
        main_loop.start()

    except:
        print ("Exception triggered - Tornado Server stopped.")
        global land
        global arming
        cleverMove.drone_land(land, arming)
        global target_x
        global target_y
        global target_z
        target_z = 0
        print(target_x,target_y,target_z)

