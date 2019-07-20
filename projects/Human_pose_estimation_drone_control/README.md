# Human Pose Estimation drone control Project

This project is about to control clever4 drone from Copter Express company by Human Pose Estimation. In this project, there has been many trials to estimate human pose by openpose CMU library on google colab in order to ease the problems and the difficulties of setup, computation performance and Hardware differences from the users but this was not successfully due to problems in getting the video stream from the javascript to python openpose processing module, then after successful workin in the local computer as making it work using aiortc as python API for webRTC and deploy it as a python web app but I had problems in making it work in google colab as I cannot open this web app by the link or running two cells in parallel in google colab. So, one of the solutions is to deploy it in Google Cloud Platform VM as it has 300$ free tier but we haven't try it. Also, one of the options that can work but we didn't try is to make real-time stream on a server and process on that server feed from google colab but mostly there can be some lag in the video streaming, you can use opencv to pose estimate but it was too slow [look here](https://www.learnopencv.com/tag/openpose/) as one frame was processing in averge 2 seconds. All the failed trials in a folder in the project. Then after a recommendation by a person from Innopolis AI community, we used the posenet from tensorflow.js as it is easier, faster and easy build. You can find the work of posenet for this project [here](https://github.com/hany606/tfjs-posenet).

## Setup (How to use this project):
### In the raspberry pi of the drone (Main controller)
- Access the raspberry pi
- [Switch to Client mode](https://clever.copterexpress.com/en/network.html) and ensure that the network has internet connection
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
cd projects/Human_pose_estimation_drone_control/
```
- Run the server to test that everything is correct
```sh
python websocket_server.py
```
- Close the server using Ctrl-C / Ctrl-Z
- [Switch to Access point mode](https://clever.copterexpress.com/en/network.html)

### In your main laptop
#### (It has been tested until now on Ubuntu 18.04)
- Clone the repo of posenet in your computer
```sh
git clone https://github.com/hany606/tfjs-posenet.git
```
- Do the steps of run and setup as it is [here](https://github.com/hany606/tfjs-posenet/tree/master/posenet)
