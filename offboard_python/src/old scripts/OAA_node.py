#!/usr/bin/env python

import rospy

from offboard_python.msg import avoidance_command


avoidance_command_pub = rospy.Publisher('obstacle_avoidance_cmd_info', avoidance_command, queue_size=10)


def OAA():
	count = 1
	direction = 1
	while not rospy.is_shutdown():
		rospy.init_node('OAA_node', anonymous=True)
		rate = rospy.Rate(20.0)
		
		
		avoidance_command_msg = avoidance_command()
		avoidance_command_msg.angle = count * 0.00
		avoidance_command_msg.speed = 0.05 * count
		avoidance_command_pub.publish(avoidance_command_msg)

		print(avoidance_command_msg.speed)
			
		count = count + direction
		if (count >= 3):
			direction = -1
		if (count <= -3):
			direction = 1
		rate.sleep()

if __name__ == '__main__':
    try:
        OAA()
    except rospy.ROSInterruptException:
	pass
