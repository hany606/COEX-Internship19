import argparse
import asyncio
import json
import logging
import os
import platform
import ssl
import portpicker

from aiohttp import web
import cv2
from av import VideoFrame

from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaPlayer



from IPython.display import display, Javascript
from google.colab.output import eval_js
from google.colab.patches import cv2_imshow

# ROOT = os.path.dirname(__file__)
ROOT = "/content/COEX-Internship19/projects/Human_pose_estimation/webcam"
#openpose/build/examples/tutorial_api_python/

def process(frame):
    return frame
class VideoTransformTrack(VideoStreamTrack):
    def __init__(self, track):
        super().__init__()  # don't forget this!
        self.track = track

    async def recv(self):
        frame = await self.track.recv()
        print("Recev")
        # perform edge detection
        img = frame.to_ndarray(format="bgr24")
        img = process(img)
        cv2_imshow(img)
        # rebuild a VideoFrame, preserving timing information
        new_frame = VideoFrame.from_ndarray(img, format="bgr24")
        new_frame.pts = frame.pts
        new_frame.time_base = frame.time_base
        return new_frame

async def index(request):
    content = open(os.path.join(ROOT, "index.html"), "r").read()
    return web.Response(content_type="text/html", text=content)


async def javascript(request):
    content = open(os.path.join(ROOT, "client.js"), "r").read()
    display(content)
    data = eval_js('start()')
    return web.Response(content_type="application/javascript", text=content)

    
async def offer(request):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        print("ICE connection state is %s" % pc.iceConnectionState)
        if pc.iceConnectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    # open webcam
    options = {"framerate": "30", "video_size": "640x480"}
    if platform.system() == "Darwin":
        player = MediaPlayer("default:none", format="avfoundation", options=options)
    else:
        player = MediaPlayer("/dev/video0", format="v4l2", options=options)
    
    await pc.setRemoteDescription(offer)
    for t in pc.getTransceivers():
        if t.kind == "audio" and player.audio:
            pc.addTrack(player.audio)
        elif t.kind == "video" and player.video:
            local_video = VideoTransformTrack(player.video)
            pc.addTrack(local_video)

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )


pcs = set()


async def on_shutdown(app):
    # close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()


if __name__ == "__main__":
    ssl_context = None

    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)
    app.router.add_get("/client.js", javascript)
    app.router.add_post("/offer", offer)
    port = portpicker.pick_unused_port()
#     port = 20248
    print("Runnign on port: {:}".format(port))
    web.run_app(app, host= '0.0.0.0',port= port, ssl_context=ssl_context)
