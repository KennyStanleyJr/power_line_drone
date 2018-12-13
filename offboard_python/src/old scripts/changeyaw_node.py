#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from offboard_python.msg import command_info

# callback method for state subscriber
current_state = State() 
offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state

#callback method for command info subscriber
def command_info_cb(data)
	global angle, displacement
	#accept angle in radians
	angle = data.angle
	#displacement in meters
	displacement = data.displacement

mavros.set_namespace()
#local position publisher to takeoff
local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
#need to publish velocity relative to body frame
vel_pub = rospy.Publisher(mavros.get_topic('setpoint_velocity', 'cmd_vel_unstamped'), Twist, queue_size=10)
#need to publish attitude
attitude_pub = rospy.Publisher(mavros.get_topic('setpoint_attitude', 'cmd_vel'), TwistStamped, queue_size=10)

#create a subscriber to the "command_info_topic" topic, which uses messages of type "command_info" - a custom message type.
#the custom message type contains 2 elements - angle and displacement, both float32. angle in radians, displacement in meters.
command_info_sub = rospy.Subscriber("command_info_topic", command_info, command_info_cb)
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)


pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2

def position_control():
    rospy.init_node('offb_python_node', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    last_request = rospy.get_rostime()
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now 
        else:
            if not current_state.armed and (now - last_request > rospy.Duration(5.)):
               arming_client(True)
               last_request = now 

        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        # Update timestamp and publish pose 
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rospy.sleep(3)

	#now we are in the air! change yaw and velocity based on the command info
	#use setpoint_attitude to change yaw.
	while True:
		 
		attitude_msg = TwistStamped()
		attitude_msg.twist.angular.x = 0 #roll
		attitude_msg.twist.angular.y = 0 #pitch
		attitude_msg.twist.angular.z = 30 #yaw
		attitude_msg.header.stamp = rospy.Time.now()
	
		attitude.pub.publish(attitude_msg)
		rate.sleep()

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
	pass
