#!/usr/bin/env python

#imports go here
import rospy
from offboard_python.msg import command_info
from ProcessFrame import ProcessFrame
import cv2

#create a publisher object to publish data to the command info topic.
command_info_pub = rospy.Publisher('command_info_topic', command_info, queue_size=10)
cap = cv2.VideoCapture('/home/capstone/capstone_ws/src/offboard_python/src/scripts/video1.mp4')
frame_processing = ProcessFrame()

#this is the program loop, it runs until the node is shut down.
def CVA():
	while not rospy.is_shutdown():
		rospy.init_node('CVA_node', anonymous=True)
		_, frame = cap.read()
		
		cv2.imshow('res', frame)


		#run at 20Hz
		rate = rospy.Rate(20.0)
		
		#Get it from pycamera
		distance, angle = frame_processing.run(frame, show=True)
		
		#create a Comand_info message object to store the captured data in.
		data = command_info()
		data.angle = angle#store the angle found here.
		data.displacement = distance#store the displacement found here.
		if data.angle != None and data.displacement != None:
			command_info_pub.publish(data) #publish the message, now populated with data, to the topic.
		#only run at 20Hz. If we finish sooner, sleep until next iteration.
		rate.sleep()

if __name__ == '__main__':
    try:
        CVA()
    except rospy.ROSInterruptException:
	pass
