#!/usr/bin/env python3

"""
BTMH4443 Robotic System and Design Laboratory (ROS Assignment/ SLAM)
    by Louis Lew Zun Kang
       Ooi Yao Sheng
"""

import rospy
import time
import math
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion

ROBOT_SPEED = 0.2
RPY = [0., 0., 0.]
Yaw = 0.
Angle = 0.
Complete = True
State = 0

def push_cmd_vel(lin_vel, ang_vel):
    msg = Twist()
    msg.linear.x = lin_vel
    msg.angular.z = ang_vel
    pub.publish(msg)

def continue_task():
    msg = Twist()
    if State == 1:
        if RPY[2] >= Angle:
            push_cmd_vel(0, 0.1)
        else:
            Complete = True
            push_cmd_vel(0, 0)
    elif State == 2:
        if RPY[2] <= Angle:
            push_cmd_vel(0, -0.1)
        else:
            Complete = True
            push_cmd_vel(0, 0)
    msg.linear.x = Angle
    imu_pub.publish(msg)

def turn(direction): #-1 left, 1 right
    global RPY, Angle, Complete, State
    if direction == -1:
        Angle = RPY[2]- (math.pi/2)
        State = 1
    elif direction == 1:
        Angle = RPY[2] + (math.pi/2)
        State = 2

def laser_callback(msg):
    left_side = 0.0
    right_side = 0.0
    regions = {
        'left':min(min(msg.ranges[68:112]),10),
        'fleft':min(min(msg.ranges[113:157]),10),
        'front':min(min(msg.ranges[158:202]),10),
        'fright':min(min(msg.ranges[203:247]),10),
        'right':min(min(msg.ranges[248:292]),10),
    }
    mapping(regions)

def mapping(regions):
    global Complete

    if Complete == False:
        continue_task()
        return
    else:
    # if front and front right detected 
        if regions['front'] < 0.5:
            push_cmd_vel(0, 0)
            turn(-1)
            Complete = False
        else:
            push_cmd_vel(ROBOT_SPEED, 0)



def odom_callback(msg):
    global RPY
    orientation = [0., 0., 0., 0.]
    orientation[0] = msg.pose.pose.orientation.x
    orientation[1] = msg.pose.pose.orientation.y
    orientation[2] = msg.pose.pose.orientation.z
    orientation[3] = msg.pose.pose.orientation.w
    RPY = euler_from_quaternion(orientation)

    

def map_callback(msg):

    map_width = msg.info.width
    map_height = msg.info.height

    map_origin_x = msg.info.origin.position.x
    map_origin_y = msg.info.origin.position.y
    map_orientation = math.acos(msg.info.origin.orientation.z)


rospy.init_node('obstacle_avoidance') #Iniitate node
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)
map_sub = rospy.Subscriber('/map', OccupancyGrid, map_callback) #Create subsriber
scan_sub = rospy.Subscriber('/scan', LaserScan, laser_callback) #Create subsriber
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
imu_pub = rospy.Publisher('/imu', Twist, queue_size=10)

rospy.spin()