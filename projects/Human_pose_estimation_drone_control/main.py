#! /usr/bin/python
import os.path
import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web
import json
import math

#Tornado Folder Paths
settings = dict(
	template_path = os.path.join(os.path.dirname(__file__), "templates"),
	static_path = os.path.join(os.path.dirname(__file__), "static")
	)

#Tonado server port
PORT = 8091
initial_pose = False 

def get_distance(p1, p2):
    return(math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2))
def to_degree(theta):
    return(180.0*theta/math.pi)
def get_angle(p1, p2, p3):
    # using Cosine law
    len_p1p2 = get_distance(p1, p2) 
    len_p1p3 = get_distance(p1, p3)
    len_p2p3 = get_distance(p2, p3)
    tmp_comp = len_p2p3**2 -  len_p1p2**2 - len_p1p3**2
    tmp_comp /=(-2*len_p1p2*len_p1p3)
    return(math.acos(tmp_comp))

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
    if(right_shoulder_score > 90):
        flags_counter += 1
    else:
        print("Right Shoulder is not here, try to move back and empty background for more accuraccy")
    
    if(right_elbow_score > 90):
        flags_counter += 1
    else:
        print("Right elbow is not here, try to move back and empty background for more accuraccy")

    if(right_wrist_score > 90):
        flags_counter += 1
    else:
        print("Right Wrist is not here, try to move back and empty background for more accuraccy")
    
    if(left_shoulder_score > 90):
        flags_counter += 1
    else:
        print("Left Shoulder is not here, try to move back and empty background for more accuraccy")
    
    if(left_elbow_score > 90):
        flags_counter += 1
    else:
        print("Left elbow is not here, try to move back and empty background for more accuraccy")

    if(left_wrist_score > 90):
        flags_counter += 1
    else:
        print("Left Wrist is not here, try to move back and empty background for more accuraccy")

    if(flags_counter == 4):
        return 1
    return 0
    
def check_initial_pose(angles):
    threshold = 5
    flags_counter = 0
    if(angles["right_shoulder"] < 90+threshold and angles["right_shoulder"] > 90-threshold):
        flags_counter += 1
    else:
        print("Right Shoulder angle is not correct, try it to make it 90, it is now{:}".format(angles["right_shoulder"]))

    if(angles["right_elbow"] < 90+threshold and angles["right_elbow"] > 90-threshold):
        flags_counter += 1
    else:
        print("Right Elbow angle is not correct, try it to make it 90, it is now{:}".format(angles["right_elbow"]))

    if(angles["left_shoulder"] < 90+threshold and angles["left_shoulder"] > 90-threshold):
        flags_counter += 1
    else:
        print("Left Shoulder angle is not correct, try it to make it 90, it is now{:}".format(angles["left_shoulder"]))

    if(angles["left_elbow"] < 90+threshold and angles["left_elbow"] > 90-threshold):
        flags_counter += 1
    else:
        print("Left Elbow angle is not correct, try it to make it 90, it is now{:}".format(angles["left_elbow"]))

    if(flags_counter == 4):
        return 1
    return 0
def main(data):
    '''
    Example:
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
    order = 0
    data_json = json.loads(data)
    # keypoints of interest are rightShoulder(6), rightElbow(8), rightWrist(10), rightHip(12) and leftShoulder(5), rightElbow(7), rightWrist(9), leftHip(11)
    # Check if all the keypoints are visible by the score
    check_keypoints_existance(data_json)
    # Check if initial pose was done or not yet
    angles = calc_angles(data_json)
    if(not(initial_pose)):
        # Order the user to make the initial pose L shape for the arms
        print("Please do the initial position to make your arms like L shape as it is described in the repo")
        # Check the pose
        if(check_initial_pose(angles)):
            global initial_pose
            initial_pose = True
    else:
        # check the score of them to change position
class WSHandler(tornado.websocket.WebSocketHandler):
  def open(self):
    print ('[WS] Connection was opened.')
 
  def on_message(self, message):
    #print (('[WS] Incoming message:'), message)
    main(message)
  def on_close(self):
    print ('[WS] Connection was closed.')
  def check_origin(self,origin):
    return True

application = tornado.web.Application([(r'/ws', WSHandler)], **settings)


if __name__ == "__main__":
    try:
        http_server = tornado.httpserver.HTTPServer(application)
        http_server.listen(PORT)
        main_loop = tornado.ioloop.IOLoop.instance()

        print ("Tornado Server started")
        main_loop.start()

    except:
        print ("Exception triggered - Tornado Server stopped.")

