#!/usr/bin/env python

import rospy
import mavros
from math import sin, cos, fabs
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Float64
from tf.transformations import *
from sensor_msgs.msg import LaserScan
from offboard_python.msg import command_info

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state

#callback method for rplidar subscriber
def rplidar_cb(data):
    """ Unnecessary information 
    global angle_min
    angle_min = data.angle_min
    global angle_max
    angle_max = data.angle_max
    global angle_increment
    angle_increment = data.angle_increment
    global time_increment
    time_increment = data.time_increment
    global scan_time
    scan_time = data.scan_time
    """
    global range_min
    range_min = data.range_min
    global range_max
    range_max = data.range_max
    global ranges
    ranges = data.ranges
    global intensities
    intensities = data.intensities

#callback method for command info subscriber
def command_info_cb(data):
    global angle
    angle = data.angle
    global displacement
    displacement = data.displacement

#callback method for position subscriber
def position_cb(get_pose):
    global altitude
    altitude = get_pose.pose.position.z


mavros.set_namespace()
local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
body_vel_pub = rospy.Publisher(mavros.get_topic('setpoint_velocity', 'cmd_vel_unstamped'), Twist, queue_size=10)

state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
laser_sub = rospy.Subscriber('scan', LaserScan, rplidar_cb)
CVA_sub = rospy.Subscriber('command_info_topic', command_info, command_info_cb)
local_pos_sub = rospy.Subscriber(mavros.get_topic('setpoint_position', 'local'), PoseStamped, position_cb)

arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 

targetHeight = 2
pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = targetHeight


def position_control():
    rospy.init_node('offb_python_node', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    

    count = 0
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    last_request = rospy.get_rostime()
    while not rospy.is_shutdown():
	count = count + 0.05
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
            offboard_started_time = rospy.get_rostime()
        prev_state = current_state

        # Update timestamp and publish pose
	now = rospy.get_rostime()
	if (current_state.mode == "OFFBOARD" and now - offboard_started_time <= rospy.Duration(10.)):
            pose.header.stamp = rospy.Time.now()
            local_pos_pub.publish(pose)
            rate.sleep()
	else:
	    #create and send velocity setpoints
	    twist = Twist()
	    """
	    VARIABLES
	    twist.linear.x = xvel
	    twist.linear.y = yvel
	    twist.linear.z = zvel
	    twist.angular.x = pitch
	    twist.angular.y = roll
	    twist.angular.z = yaw
	    """

	    if ((min(ranges[0:22]) <= 0.3)|(min(ranges[338:360]) <= 0.3)):
		print('DANGER')
		clear_path_angle = 45
		i=90
		while i >= 0:
		    if (~(ranges[i] > 0.3)):
			right_bound = i

		    i -= 1
		i=90
		while i <= 180:
		    if (~(ranges[i] > 0.3)):
			left_bound = i

		    i += 1
		if (left_bound - right_bound >= 45):
			path_angle = right_bound + (clear_path_angle/2)
		
		xvel = 0
	        yvel = 0
	        zvel = 0

		pitch = 0
		roll = 0	

	    else:
		#print('All good!')
		xvel = (3.0 - 0.4*(math.fabs(displacement))) * math.cos(angle)
	        yvel = 3.0 * math.sin(angle)
	        if (altitude < targetHeight):
		    zvel = 0.2
	    	else:
		    zvel = 0
		
		try:
		    yaw = -(0.5*displacement)*(xvel*math.cos(angle)/angle - yvel*math.sin(angle)/angle) - 0.5*(angle)
	    	except:
		    yaw = 0
		
		pitch = 0
		roll = 0
	       
	        
	    twist.linear.x = xvel
	    twist.linear.y = yvel
	    twist.linear.z = zvel
	    twist.angular.x = pitch
	    twist.angular.y = roll
	    twist.angular.z = yaw  

	    body_vel_pub.publish(twist)

	    # print(ranges.index(min(ranges)))


if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
	pass
