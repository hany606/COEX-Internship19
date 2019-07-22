# Human Pose Estimation drone control Project

This project is about to control clever4 drone from Copter Express company by Human Pose Estimation.
In this project, there have been many trials to estimate the human pose by using:

- openpose CMU library on google colab in order to ease the problems and the difficulties of setup, computation performance and Hardware differences from one user to another, however, this was not successfully due to problems in getting the video stream from the javascript to python openpose processing module.
- After successful work in the local computer by making it work using aiortc as python API for webRTC and deploy it as a python web app, I had problems in making it work in google colab as I cannot open this web app by the link or running two cells in parallel in google colab.
- We thought in deploying it in Google Cloud Platform VM as it has 300$ free tier but we haven't try it.
- Also, one of the options that can work but we didn't try is to make real-time stream on a server and process on that server feed from google colab but mostly there will be some lag in the video streaming.
- Opencv is one option to pose estimate by dnn module in it but it was too slow [look here](https://www.learnopencv.com/tag/openpose/) as one frame was processing in averge 2 seconds on my machine. 
- All the failed trials in a folder in the project, you can fork this repo and try to implement them and write us your results. 
- Then after a recommendation from a person from Innopolis AI community, we used the posenet from tensorflow.js as it is easier, faster and easy build. You can find the work of posenet for this project [here](https://github.com/hany606/tfjs-posenet).

## Setup (How to use this project):
### In your main laptop
#### (It has been tested until now on Ubuntu 18.04)
- Clone the repo of posenet in your computer
```sh
git clone https://github.com/hany606/tfjs-posenet.git
```
- Do the steps of running and setup as it is described in the README [here](https://github.com/hany606/tfjs-posenet/tree/master/posenet)

### In the raspberry pi of the drone (Main controller)
- Access the raspberry pi
- [Switch to Client mode](https://clever.copterexpress.com/en/network.html) and ensure that the network has internet connection.
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
- Run the server to test that everything is correct and run the posenet, you should see a lot of data is printed in the console/terminal
```sh
python websocket_server_test.py
```
- Close the server using Ctrl+C / Ctrl+Z
- To run the main file 
```sh
python main_drone.py
```
