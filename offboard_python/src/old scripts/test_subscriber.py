#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64, String


#callback method for state_machine subscriber
def state_machine_cb(state):
    global state_machine
    state_machine = state.data


state_machine_sub = rospy.Subscriber('state_machine', String, state_machine_cb)


def state_machine():

    rospy.init_node('subscriber', anonymous=True)
    rate = rospy.Rate(20) # 20hz

    while not rospy.is_shutdown():
        print(state_machine)

   
    

if __name__ == '__main__':
    try:
        state_machine()
    except rospy.ROSInterruptException:
        pass
