# Human Pose Estimation drone control Project

This project is about to control clever4 drone from [Copter Express company](https://copterexpress.com/) by different human poses.
In this project, there have been many trials to estimate the human pose by using:

- The Openpose CMU library on google colab in order to ease the problems and the difficulties of setup and installation, computation performance and Hardware differences from one user to another, however, this was not successfully due to problems in getting the video stream from the javascript to python openpose processing module in Colab. However, after successful work in the local computer by making it work using aiortc as it is the python API for webRTC and deploy it as a python web app, I had problems in making it work in google colab as I cannot open this web app by the local link or running two cells in parallel in google colab.

- We thought in deploying it in Google Cloud Platform VMs as it has 300$ free tier but we haven't try it yet.
- Also, one of the options that can work but I haven't tried is to make a real-time stream on a server and process on that server feed from google colab but mostly there will be some lag in the video streaming.
- Opencv is one option to pose estimate by dnn module in it but it was too slow [look here](https://www.learnopencv.com/tag/openpose/) as one frame was processing in averge 2 seconds on my machine. 
- All the failed trials in a folder in the project, you can fork this repo and try to implement them and write us your results. 
- Then after a recommendation from someone from Innopolis AI community, we used the posenet from tensorflow.js as it is easier, faster and easy build. You can find the work of posenet for this project [here](https://github.com/hany606/tfjs-posenet).

## Setup & installation:
### In your main laptop:
#### (It has been tested until now on Ubuntu 18.04)
- Clone the repo of posenet in your computer or download it if you are using Windows without github
```sh
git clone https://github.com/hany606/tfjs-posenet.git
```
- Do the steps of running and setup as it is described in the README [here](https://github.com/hany606/tfjs-posenet/tree/master/posenet)

### In the raspberry pi of the drone (Main controller)
- Access the raspberry pi
- [Switch to Client mode](https://clever.copterexpress.com/en/network.html) and ensure that the network has internet connection. 

Notice: I have already made a bash script based on that tutorial, it is in COEX-Internship19/helpers/ called .to_client.bash
To run it:
```sh
chmod +x .to_client.bash
./.to_client <NAME_OF_NETWORK> <PASSWORD>
```
- Install the tornado library to make a websocket server
```sh
sudo pip install tornado
```
- Clone the main repo on the raspberry pi of the drone
```sh
git clone https://github.com/hany606/COEX-Internship19.git
```
- Go to the project directory
```sh
cd COEX-Internship19/projects/Human_pose_estimation_drone_control/
```
- Run the server to test that everything is correct and run the posenet, you should see a lot of data is printed in the terminal (if you are running the human pose estimation code on your main computer and you refreshed the page in the browser after running the below comman)
```sh
python websocket_server_test.py
```
- Close the server using Ctrl+C / Ctrl+Z
- To run the main file 
```sh
python main_drone.py
```

## How to use it?
- Run the server first from the raspberry pi
- Run Human pose estimation module on your laptop with websocket by
```sh
yarn websocket
```
or refresh the page if you already run it.
- You should see the instructions on the screen of the terminal of the raspberry pi right now.
- First you should do initial pose as it is described in the images below.
- You can perform any pose and try to keep it until your drone finish doing this move that is coressponding to the pose.
- After you do the pose return back to the initial pose in order to give the drone the command to do listen to another pose.

## Poses:
![Poses](https://github.com/hany606/COEX-Internship19/blob/master/projects/Human_pose_estimation_drone_control/Poses.jpg)

## Notes:
- Websockets are used to communicate between the page on the browser and the drone.
- As the model of posenet is already pretrained and using tensorflow.js. So, it is quite fast and can run on different computers without any problems thanks to yarn, parcel and tensorflow.js and we have configured the code of posenet to the minimal configuration to not require a lot of computation power.
- This project has been built in 1 week of working, it took a lot of time trying to make openpose and google colab working but unfortunatly I had many errors and one I decided to move to posenet everything was pretty easy.
- If you have any comments about the codes to try to improve it, I will be happy if you can contact me through telegram: @hany606 or email: h.hamed.elanwar@gmail.com or do pull requests.

## Future improvements
- Avoid using the global variables [].
- Test it on different hardwars and OS [].
- Adding more poses [].
- Add takeoff and land by the poses [].

## Future application
- [Drone wars](https://web.facebook.com/COEXDrones/photos/pcb.1129309377266616/1129308437266710/?type=3&theater): Control the drone during the drones battle using human poses. It requries high speed interaction and more precise control.
- Control drone that draw graffiti using human poses and draw in realtime.
- Playing with balls like pingpong game with the drones. It may requires 3D Human Pose estimation Algorithms.
- Control two drones by your arms and do some task together.
These ideas were produced by me and my internship supervisor Timofey.

