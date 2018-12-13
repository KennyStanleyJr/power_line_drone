#!/usr/bin/env python

#imports go here
import rospy, time
from offboard_python.msg import command_info
from ProcessFrame import ProcessFrame
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2

#create a publisher object to publish data to the command info topic.
command_info_pub = rospy.Publisher('command_info_topic', command_info, queue_size=10)

camera = PiCamera()
#camera.resolution = (640, 480)
camera.resolution = (320, 240)
camera.framerate = 32
camera.rotation = 270
#rawCapture = PiRGBArray(camera, size=(640, 480))
rawCapture = PiRGBArray(camera, size=(320, 240))

frame_processing = ProcessFrame()

#this is the program loop, it runs until the node is shut down.
def CVA():
	while not rospy.is_shutdown():
		rospy.init_node('CVA_node', anonymous=True)
		for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                                        image = frame.array

                                        #run at 20Hz
                                        rate = rospy.Rate(20.0)
                                
                                        #Get it from pycamera
                                        distance, angle = frame_processing.run(image, show=True)
                                        #cv2.imshow("Frame", image)
                                        rawCapture.truncate(0)
                                
                                        #create a Comand_info message object to store the captured data in.
                                        data = command_info()
                                        data.angle = angle#store the angle found here.
                                        data.displacement = distance#store the displacement found here.
                                        if data.angle != None and data.displacement != None:
                                                command_info_pub.publish(data) #publish the message, now populated with data, to the topic.
                                        key = cv2.waitKey(1) & 0xFF
                                        #only run at 20Hz. If we finish sooner, sleep until next iteration.
                                        rate.sleep()
                                        if key == ord("q"):
                                                break

if __name__ == '__main__':
    try:
        CVA()
    except rospy.ROSInterruptException:
	pass
