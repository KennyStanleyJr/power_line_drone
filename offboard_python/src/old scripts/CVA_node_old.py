#!/usr/bin/env python

import rospy

from offboard_python.msg import command_info

command_info_pub = rospy.Publisher('command_info_topic', command_info, queue_size=10)

def CVA():
	count = 1
	direction = 1
	while not rospy.is_shutdown():
		rospy.init_node('CVA_node', anonymous=True)
		rate = rospy.Rate(20.0)
		
		
		command_info_msg = command_info()
		command_info_msg.angle = count * 0.00
		command_info_msg.displacement = count * 0.00
		command_info_pub.publish(command_info_msg)
		count = count + direction
		if (count >= 3):
			direction = -1
		if (count <= -3):
			direction = 1
		rate.sleep()

if __name__ == '__main__':
    try:
        CVA()
    except rospy.ROSInterruptException:
	pass
