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

import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #

def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

def euclidean_distance(self, goal_pose):
    """Euclidean distance between current pose and the goal."""
    return math.sqrt(pow((goal_pose.x - self.pose.x), 2) +
                pow((goal_pose.y - self.pose.y), 2))



