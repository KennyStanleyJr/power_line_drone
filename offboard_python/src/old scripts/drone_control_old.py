#!/usr/bin/env python

import rospy, mavros, time, os
from math import sin, cos, fabs
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Float64, String
from tf.transformations import *
from sensor_msgs.msg import LaserScan

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state

#callback method for position subscriber
def position_cb(get_pose):
    global altitude
    altitude = get_pose.pose.position.z
    global x_position
    x_position = get_pose.pose.position.x
    global y_position
    y_position = get_pose.pose.position.y

#callback method for user input subscriber
def input_cb(user_input):
    global input_str
    input_str = user_input.data
'''
#callback method for rplidar subscriber
def rplidar_cb(data):
    global range_min
    range_min = data.range_min
    global range_max
    range_max = data.range_max
    global ranges
    ranges = data.ranges
'''
'''
#callback method for cva_vals subscriber
def command_info_cb(data):
    global angle
    angle = data.angle
    global displacement
    displacement = data.displacement
'''

mavros.set_namespace()

########## Define Publishers
local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
body_vel_pub = rospy.Publisher(mavros.get_topic('setpoint_velocity', 'cmd_vel_unstamped'), Twist, queue_size=10)

########## Define Subscribers
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
local_pos_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), PoseStamped, position_cb)
input_sub = rospy.Subscriber('UserInput', String, input_cb)
#laser_sub = rospy.Subscriber('scan', LaserScan, rplidar_cb)
#CVA_sub = rospy.Subscriber('command_info_topic', command_info, command_info_cb)

########## Define Services
arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 

targetHeight = 1  # Fly 1 meter high
pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = targetHeight

Kp = 1  # Proportional constant
danger_zone = 0.3  # Dangerous Obstacle Distance
command_str = "hover" # Initialize input command

os.system('clear')  # Clear screen

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
	print('Connecting...')
	os.system('clear')
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
            offboard_started_time = rospy.get_rostime()
        prev_state = current_state

	now = rospy.get_rostime()
	if (current_state.mode == "OFFBOARD" and now - offboard_started_time <= rospy.Duration(10.)):
            pose.header.stamp = rospy.Time.now()
            local_pos_pub.publish(pose)
	else:
	    os.system('clear')

	    #######################################################################################################################
	    ############# HOVER MODE ##############################################################################################
	    #######################################################################################################################
	    if input_str == "hover":
		print("Hovering")

		#create and send velocity setpoints
		twist = Twist()
		
		if altitude < targetHeight:
		    zvel = 0.2
		else:
		    zvel = 0

		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = zvel
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0
		body_vel_pub.publish(twist)
		

	    #######################################################################################################################
	    ############# LAND MODE ###############################################################################################
	    #######################################################################################################################
	    elif input_str == "land":
		print("Landing")

		set_mode_client(base_mode=0, custom_mode="AUTO.LAND")

	    '''
	    #######################################################################################################################
	    ############# OBSTACLE AVOIDANCE MODE #################################################################################
	    #######################################################################################################################
	    elif input_str == "avoid":
                closest_obstacle = min(ranges)
                obstacle_angle = ranges.index(closest_obstacle)
                obstacle_distance = ranges[obstacle_angle]

		# Remove drone legs as obstacles
		if obstacle_angle > 45 and obstacle_angle < 55:
		    closest_obstacle = 3
		elif obstacle_angle > 108 and obstacle_angle < 118:
		    closest_obstacle = 3
		elif obstacle_angle > 238 and obstacle_angle < 248:
		    closest_obstacle = 3
		elif obstacle_angle > 302 and obstacle_angle < 312:
		    closest_obstacle = 3

		# Maintain Altitude
                if altitude < targetHeight:
                    zvel = 0.2
                else:
                    zvel = 0

		# If obstacle is detected, Move in opposite direction 
                if closest_obstacle < danger_zone:
		    print("Avoiding Obstacle: %s \r" % (obstacle_angle) )

                    x_direction = -math.cos(obstacle_angle*(math.pi/180))
                    y_direction = -math.sin(obstacle_angle*(math.pi/180))

                    xvel = x_direction * (danger_zone - obstacle_distance) * 10
                    yvel = y_direction * (danger_zone - obstacle_distance) * 10


		# Else, hover in place
                else:
		    print("Hovering")
			
		    # Move in opposite direction of obstacle
                    xvel = -x_position * Kp
                    yvel = -y_position * Kp

                    twist.linear.x = xvel
                    twist.linear.y = yvel
                    twist.linear.z = zvel
                    twist.angular.x = 0
                    twist.angular.y = 0
                    twist.angular.z = 0

                body_vel_pub.publish(twist)
	    '''

	    #######################################################################################################################
	    ############# LINE FOLLOW MODE ########################################################################################
	    #######################################################################################################################
	    #elif input_str == "follow":
		#print("Following Line")

	    ######## Clear the screen ########
	    #os.system('clear')
 

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
	pass
