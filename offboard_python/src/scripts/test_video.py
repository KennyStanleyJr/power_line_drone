#!/usr/bin/env python

#imports go here
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2

camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 32
camera.rotation = 270
rawCapture = PiRGBArray(camera, size=(320, 240))

time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF
	rawCapture.truncate(0)
	if key == ord("q"):
		break