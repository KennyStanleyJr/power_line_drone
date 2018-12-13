#!/usr/bin/env python

import rospy, mavros, time, os
import math
from geometry_msgs.msg import PoseStamped, Twist
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import State, Altitude
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Float64, String
from tf.transformations import *
from sensor_msgs.msg import LaserScan, Imu
from offboard_python.msg import command_info

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
    global x_pos
    x_pos = get_pose.pose.position.x
    global y_pos
    y_pos = get_pose.pose.position.y

#callback method for altitude subscriber
def alt_cb(data):
    global rel_alt
    rel_alt = data.relative

#callback method for imu data (accelerometer)
def imu_cb(data):
    global x_accel
    x_accel = data.linear_acceleration.x
    global y_accel
    y_accel = data.linear_acceleration.y
    global z_accel
    z_accel = data.linear_acceleration.z

#callback method for user input subscriber
def input_cb(user_input):
    global input_str
    input_str = user_input.data

#callback method for rplidar subscriber
def rplidar_cb(data):
    global range_min
    range_min = data.range_min
    global range_max
    range_max = data.range_max
    global ranges
    ranges = data.ranges

#callback method for cva_vals subscriber
def command_info_cb(data):
    global angle
    angle = data.angle
    global displacement
    displacement = data.displacement / 10


mavros.set_namespace()

########## Define Publishers
local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
body_vel_pub = rospy.Publisher(mavros.get_topic('setpoint_velocity', 'cmd_vel_unstamped'), Twist, queue_size=10)
set_geo_pub = rospy.Publisher(mavros.get_topic('global_position', 'set_gp_origin'), GeoPointStamped, queue_size=10)

########## Define Subscribers
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
local_pos_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), PoseStamped, position_cb)
alt_sub = rospy.Subscriber(mavros.get_topic('altitude'), Altitude, alt_cb)
imu_sub = rospy.Subscriber(mavros.get_topic('imu', 'data'), Imu, imu_cb)
input_sub = rospy.Subscriber('UserInput', String, input_cb)
laser_sub = rospy.Subscriber('scan', LaserScan, rplidar_cb)
CVA_sub = rospy.Subscriber('command_info_topic', command_info, command_info_cb)

########## Define Services
arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 

targetHeight = 0.8  # Fly 1 meter high

geo_pos = GeoPointStamped()
geo_pos.position.latitude = 0
geo_pos.position.longitude = 0
geo_pos.position.altitude = 0

initial_pose = PoseStamped()
initial_pose.pose.position.x = 0
initial_pose.pose.position.y = 0
initial_pose.pose.position.z = 0

danger_zone = 1.0  # Dangerous Obstacle Distance
input_str = "ready" # Initialize input command

os.system('clear')  # Clear screen

print('Connecting...')

def position_control():
    rospy.init_node('offb_python_node', anonymous=True)
    prev_state = current_state
    freq = 20
    rate = rospy.Rate(freq) # MUST be more then 2Hz
    
    recent_angles = [0]
    recent_displacements = [0]
    global angle
    angle = 0
    global displacement
    displacement = 0
    global xvel
    xvel = 0
    global yvel
    yvel = 0
    global zvel
    zvel = 0

    global x_pos
    x_pos = 0
    global y_pos
    y_pos = 0
    global range_min
    range_min = 0
    

    pose = PoseStamped()
    pose.pose.position.x = x_pos
    pose.pose.position.y = y_pos
    pose.pose.position.z = targetHeight

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
            offboard_started_time = rospy.get_rostime()
        prev_state = current_state

        now = rospy.get_rostime()
        
        os.system('clear')
	    
        ######################################################################################################################
        ############# READY MODE ##############################################################################################
        #######################################################################################################################
        if input_str == "ready":
            print("Ready for mode")

            reset_pos = True

            rate.sleep()

        #######################################################################################################################
        ############# TAKEOFF MODE ############################################################################################
        #######################################################################################################################
        if input_str == "takeoff":
            print("Takeoff")

            if reset_pos:
                pose = PoseStamped()
                pose.pose.position.x = x_pos
                pose.pose.position.y = y_pos
                pose.pose.position.z = targetHeight

                set_geo_pub.publish(geo_pos)


            if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
                arming_client(True)
                set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now

            local_pos_pub.publish(pose)

            reset_pos = False


            rate.sleep()


        #######################################################################################################################
        ############# LAND MODE ###############################################################################################
        #######################################################################################################################
        elif input_str == "land":
            print("Landing")

            reset_pos = True

            if current_state.mode != "AUTO.LAND" and (now - last_request > rospy.Duration(5.)):
                    set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
                    last_request = now

            rate.sleep()


        #######################################################################################################################
        ############# DISARM  #################################################################################################
        #######################################################################################################################
        elif input_str == "disarm":
                print("Disarming")

                arming_client(False)


                rate.sleep()

        #######################################################################################################################
        ############# HOVER MODE ##############################################################################################
        #######################################################################################################################
        elif input_str == "hover":
            print("Hovering")
	    
            twist=Twist()

            xvel = x_accel / 2
            yvel = y_accel / 2

            # Maintain Altitude
            zvel = (targetHeight - rel_alt)*2
            '''
            if altitude < targetHeight:
                zvel = 0.2
            else:
                zvel = 0
            '''
    
            print(zvel)

            twist.linear.x = xvel
            twist.linear.y = yvel
            twist.linear.z = zvel
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0

            body_vel_pub.publish(twist)

	    

            rate.sleep()


        #######################################################################################################################
        ############# OBSTACLE AVOIDANCE MODE #################################################################################
        #######################################################################################################################
        elif input_str == "avoid":
            obstacle_angle = ranges.index(range_min)
            obstacle_distance = ranges[obstacle_angle]

            # Remove drone legs as obstacles
            if obstacle_angle > 45 and obstacle_angle < 55:
                range_min = 3
            elif obstacle_angle > 108 and obstacle_angle < 118:
                range_min = 3
            elif obstacle_angle > 238 and obstacle_angle < 248:
                range_min = 3
            elif obstacle_angle > 302 and obstacle_angle < 312:
                range_min = 3

            # If obstacle is detected, Move in opposite direction
            if range_min < danger_zone:
                print("Obstacle Detected\n")

                twist = Twist()

                x_direction = -math.cos(obstacle_angle*(math.pi/180))
                y_direction = -math.sin(obstacle_angle*(math.pi/180))

                xvel = -x_direction * (danger_zone - obstacle_distance) * 10
                yvel = y_direction * (danger_zone - obstacle_distance) * 10

                # Maintain Altitude
            	zvel = (targetHeight - rel_alt)*2

                twist.linear.x = xvel
                twist.linear.y = yvel
                twist.linear.z = zvel
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0

                body_vel_pub.publish(twist)


            # Else, hover in place
            else:
                print("Hovering")

                twist = Twist()

                # Maintain Altitude
            	zvel = (targetHeight - rel_alt)*2
                '''
                if altitude < targetHeight:
                    zvel = 0.2
                else:
                    zvel = 0
                '''

                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = zvel
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0

                body_vel_pub.publish(twist)

            rate.sleep()


        #######################################################################################################################
        ############# LINE FOLLOW MODE ########################################################################################
        #######################################################################################################################
        elif input_str == "track":
            print("Following Line\n")

            recent_angles.append(angle)
            if len(recent_angles) > 5:
                    del recent_angles[0]
            recent_displacements.append(displacement)
            if len(recent_displacements) > 5:
                    del recent_displacements[0]


            #if the average of the recent angles is not within tolerance, adjust the angle.
            #if it's within tolerance, instead check if the average of displacements is within tolerance; if not, adjust the displacement.
            #if both are good, then just go forward. Also, if the altitude is not at target height, then increase altitude.
            twist = Twist()
##            if math.fabs(angle) > 3:
##                twist.linear.x = 0
##                twist.linear.y = 0
##                twist.angular.x = angle
##            elif math.fabs(displacement) > 50:
##                twist.linear.x = 0
##                twist.linear.y = displacement/50
##                twist.angular.x = 0
##            else:
##                twist.linear.x = 0.1
##                twist.linear.y = 0
##                twist.angular.x = 0
##
##                twist.angular.y = 0
##                twist.angular.z = 0

            xvel = (1.0-0.4*(math.fabs(displacement)))*math.cos(angle)
            twist.linear.x = xvel

            yvel = 1.0*math.sin(angle)
            twist.linear.y = yvel

            twist.angular.x = 0
            twist.angular.y = 0

            try:
                twist.angular.z = -(0.5*displacement)*(xvel*math.cos(angle)/angle - yvel*math.sin(angle)/angle)-0.5*(angle)
            except:
                twist.angular.z = 0
            
            print("angle: ")
            print(angle)
            print("\n")
            print("displacement")
            print(displacement)
            print("\n")
            # Maintain Altitude
            zvel = (targetHeight - rel_alt)*2
            '''
            if altitude < targetHeight:
                zvel = 0.2
            else:
                zvel = 0
            '''

            twist.linear.z = zvel

            body_vel_pub.publish(twist)
            rate.sleep()

 
if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
	pass
