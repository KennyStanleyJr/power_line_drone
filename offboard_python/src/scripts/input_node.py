#!/usr/bin/env python

import rospy
import os
from std_msgs.msg import String

os.system('clear')

def input_pub():
    pub = rospy.Publisher('UserInput', String, queue_size=2)
    rospy.init_node('UserInputNode', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    input_str = ""
    while not rospy.is_shutdown():
	if input_str == "":
            print("Please enter a mode \n")
	elif input_str == "hover" or input_str == "land" or input_str == "avoid" or input_str == "track" or input_str == "takeoff" or input_str == "disarm":
	    print("Current mode: %s \n " % (input_str))
	else:
	    print("'%s' is not a valid mode. Please try again \n" % (input_str))
	print("Takeoff Mode ----------------> takeoff ")
	print("Land Mode -------------------> land ")
	print("Disarm ----------------------> disarm ")
	print("Hover Mode ------------------> hover ")
	print("Obstacle Avoidance Mode -----> avoid ")
	print("Line Following Mode ---------> track \n ")
        input_str = raw_input("Mode: ")
	
	os.system('clear')

        pub.publish(input_str)
	
        rate.sleep()

if __name__ == '__main__':
    try:
        input_pub()
    except rospy.ROSInterruptException:
        pass
