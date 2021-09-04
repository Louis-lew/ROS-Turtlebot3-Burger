#!/usr/bin/env python3

"""
BTMH4443 Robotic System and Design Laboratory (ROS Assignment/ SLAM)
    by Louis Lew Zun Kang
       Ooi Yao Sheng
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback_laser(msg):
    region = {
        'right'     : min(min(msg.ranges[0:53]), 10),
        'frontright': min(min(msg.ranges[54:107]), 10),
        'frontleft' : min(min(msg.ranges[108:161]),10),
        'front'     : min(min(msg.ranges[162:215]), 10),
        'left'      : min(min(msg.ranges[216:270]), 10),
    }

    avoidance(region)


def avoidance(region):
    msg = Twist()

    if region['front'] <= 0.5:
        lin_vel = 0.0
        ang_vel = 0.0
    else:
        lin_vel = 0.22
        ang_vel = 0.0

    msg.linear.x = lin_vel
    msg.angular.z = ang_vel
    pub.publish(msg)


rospy.init_node('obstacle_avoidance', anonymous=True)
sub = rospy.Subscriber('/scan', LaserScan, callback_laser)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

rospy.spin()
