#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def state_machine():
    pub = rospy.Publisher('state_machine', String, queue_size=10)
    rospy.init_node('state_machine', anonymous=True)
    rate = rospy.Rate(20) # 20hz
    while not rospy.is_shutdown():
	# State Machine Code goes here
        state_str = "hello world %s" % rospy.get_time()

        rospy.loginfo(state_str)
        pub.publish(state_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        state_machine()
    except rospy.ROSInterruptException:
        pass
