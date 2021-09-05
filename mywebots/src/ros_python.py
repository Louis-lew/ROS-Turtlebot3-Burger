#!/usr/bin/env python3

"""
BTMH4443 Robotic System and Design Laboratory (ROS Assignment/ SLAM)
    by Louis Lew Zun Kang
"""

import tf
import tf2_ros
import rospy
import roslib
#import turtlesim.msg
# from rospy.core import loginfo
# from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from controller import Robot
import os
import math

FRAME_ID = "base_link" # or base_footprint

NAME = "Burger"
WHEEL_RADIUS = 0.033          # meter
WHEEL_SEPARATION = 0.160      # meter (BURGER : 0.160, WAFFLE : 0.287)
TURNING_RADIUS = 0.080        # meter (BURGER : 0.080, WAFFLE : 0.1435)
ROBOT_RADIUS = 0.105          # meter (BURGER : 0.105, WAFFLE : 0.220)
ENCODER_MIN = -2147483648     # raw
ENCODER_MAX = 2147483648      # raw
# m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
MAX_LINEAR_VELOCITY = (WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60)
MAX_ANGULAR_VELOCITY = (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       # rad/s
MIN_LINEAR_VELOCITY = -MAX_LINEAR_VELOCITY
MIN_ANGULAR_VELOCITY = -MAX_ANGULAR_VELOCITY

# webots max motor speed is 2*PI one rotation
WEBOTS_MOTOR_MAX_VELOCITY = 2 * math.pi
INFINITY = float('+inf')

X = 0.0
Y = 0.0
TH = 0.0
VX = 0.0
VY = 0.0
VTH = 0.0
lin_vel = 0.0
ang_vel = 0.0

encLeft = [0.0, 0.0] #current, last
encRight = [0.0, 0.0]

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

imu = robot.getDevice("inertial unit")
imu.enable(TIME_STEP)

lidar = robot.getDevice("LDS-01")
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

lidar_main_motor = robot.getDevice("LDS-01_main_motor")
lidar_secondary_motor = robot.getDevice("LDS-01_secondary_motor")
lidar_main_motor.setPosition(INFINITY)
lidar_secondary_motor.setPosition(INFINITY)
lidar_main_motor.setVelocity(30.0)
lidar_secondary_motor.setVelocity(60.0)

right_motor = robot.getDevice("right wheel motor")
left_motor = robot.getDevice("left wheel motor")
right_motor.setPosition(INFINITY)
left_motor.setPosition(INFINITY)
right_motor.setVelocity(0.0)
left_motor.setVelocity(0.0)

left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

# lidar_width = lidar.getHorizontalResolution()
# lidar_max_range = lidar.getMaxRange()

# velocity = 0
message = ''

rospy.loginfo('Initializing ROS: connecting to ' +
              os.environ['ROS_MASTER_URI'])
robot.step(TIME_STEP)

# Listener
def speedConstrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

#https://navigation.ros.org/setup_guides/odom/setup_odom.html
def cmdVelCallback(data):
    global message
    global left_motor
    global right_motor
    global lin_vel
    global ang_vel

    lin_vel = speedConstrain(
        data.linear.x,  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)
    ang_vel = speedConstrain(
        data.angular.z, MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)

    left_wheel_lin_vel = lin_vel - (ang_vel * WHEEL_SEPARATION / 2)
    right_wheel_lin_vel = lin_vel + (ang_vel * WHEEL_SEPARATION / 2)
    
    left_wheel_ang_vel = speedConstrain(
        left_wheel_lin_vel / WHEEL_RADIUS, -WEBOTS_MOTOR_MAX_VELOCITY, WEBOTS_MOTOR_MAX_VELOCITY) 
    right_wheel_ang_vel = speedConstrain(
        right_wheel_lin_vel / WHEEL_RADIUS, -WEBOTS_MOTOR_MAX_VELOCITY, WEBOTS_MOTOR_MAX_VELOCITY)

    message = f'left_wheel_lin_vel: {str(left_wheel_lin_vel)} angular velocity: {str(left_wheel_ang_vel)}'

    left_motor.setVelocity(left_wheel_ang_vel)
    right_motor.setVelocity(right_wheel_ang_vel)

# Publisher
def readLidar(lidar):
    laserData = LaserScan()
    laserData.header.stamp = rospy.Time.now()
    laserData.header.frame_id = 'base_scan'
    laserData.angle_increment = 2*math.pi / lidar.getHorizontalResolution()
    laserData.scan_time = 0
    laserData.time_increment = 0
    laserData.angle_min = 0
    laserData.angle_max = 2 * math.pi - laserData.angle_increment
    laserData.range_min = lidar.getMinRange()
    laserData.range_max = lidar.getMaxRange()
    laserData.ranges = lidar.getRangeImage()
    laserData.intensities = [0]*360
    return laserData

# Encoder & IMU
def readEnc(current_time, last_time, init_state):
    global encLeft, encRight
    diff = [0.0, 0.0]

    if init_state:
        encLeft[1]= left_encoder.getValue()
        encRight[1] = right_encoder.getValue()

    #current position in rad
    encLeft[0] = left_encoder.getValue()
    encRight[0] = right_encoder.getValue()

    diff[0] = encLeft[0] - encLeft[1]
    diff[1] = encRight[0] - encRight[1]

    step_time = current_time-last_time

    if diff[0] == 0 and diff[1] == 0:
        enc_lin_vel =0.
        enc_ang_vel = 0.
    else:
        left_enc_lin_vel =  WHEEL_RADIUS*diff[0]/step_time.to_sec()
        right_enc_lin_vel = WHEEL_RADIUS*diff[1]/step_time.to_sec()
        enc_lin_vel = (left_enc_lin_vel+right_enc_lin_vel)/2
        enc_ang_vel = (left_enc_lin_vel-right_enc_lin_vel)/WHEEL_SEPARATION

    encLeft[1] = left_encoder.getValue()
    encRight[1] = right_encoder.getValue()

    return enc_lin_vel, enc_ang_vel

# Publisher
def sendodom(init=False):
    global X, Y, TH, VX, VTH
    global CURRENT_TIME, LAST_TIME

    # compute odometry in a typical way given the velocities of the robot
    CURRENT_TIME = rospy.Time.now()
    VX, VTH = readEnc(CURRENT_TIME, LAST_TIME, init)
    dt = (CURRENT_TIME-LAST_TIME).to_sec()
    delta_x = (VX * math.cos(TH) - VY * math.sin(TH)) * dt
    delta_y = (VX * math.sin(TH) + VY * math.cos(TH)) * dt
    delta_th = VTH * dt

    X += delta_x
    Y += delta_y
    TH += delta_th

    odom_quat = imu.getQuaternion()
    #odom_quat = quaternion_from_euler(0, 0, TH)
    #publish transform over tf
    odom_broadcaster.sendTransform((X, Y, 0.),
        odom_quat, CURRENT_TIME, "base_footprint", "odom")

    odom = Odometry()
    odom.header.stamp = CURRENT_TIME
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_footprint"

    #set position
    odom.pose.pose = Pose(Point(X, Y, 0.), Quaternion(*odom_quat))
    
    #set velocity
    odom.child_frame_id = FRAME_ID
    odom.twist.twist = Twist(Vector3(VX, VY, 0), Vector3(0, 0, VTH))
    
    #publish the message
    pub_odom.publish(odom)
    LAST_TIME = CURRENT_TIME

# Static Transform
def handle_turtle_static_pose(trans, rot, stamp, frame_child, frame_header):
    stf = TransformStamped()
    stf.header.stamp = stamp
    stf.header.frame_id = frame_header
    stf.child_frame_id = frame_child
    stf.transform.translation.x = trans[0]
    stf.transform.translation.y = trans[1]
    stf.transform.translation.z = trans[2]
    stf.transform.rotation.x = rot[0]
    stf.transform.rotation.y = rot[1]
    stf.transform.rotation.z = rot[2]
    stf.transform.rotation.w = rot[3]
    turtle_TF.sendTransform(stf)

def handle_turtle_pose():
    #turtle_TF.sendTransform((0., 0., 0.), (0, 0 ,0 ,1), rospy.Time.now(), FRAME_ID, "base_footprint")
    #turtle_TF.sendTransform((0., 0., 0.), (0, 1 ,0 ,0), rospy.Time.now(), "base_scan", FRAME_ID)
    #turtle_TF.sendTransform((0., 0.08, 0.023), quaternion_from_euler(-1.57, 0, 0), rospy.Time.now(), "wheel_left_link", FRAME_ID)
    #turtle_TF.sendTransform((0., -0.08, 0.033), quaternion_from_euler(-1.57, 0, 0), rospy.Time.now(), "wheel_right_link", FRAME_ID)

    handle_turtle_static_pose((0., 0., 0.), (quaternion_from_euler(0, 0, 1.57)), rospy.Time.now(), FRAME_ID, "base_footprint")
    handle_turtle_static_pose((0., 0., 0.), (0, 1 ,0 ,0), rospy.Time.now(), "base_scan", FRAME_ID)
    handle_turtle_static_pose((0., 0.08, 0.023), quaternion_from_euler(-1.57, 0, 0), rospy.Time.now(), "wheel_left_link", FRAME_ID)
    handle_turtle_static_pose((0., -0.08, 0.023), quaternion_from_euler(-1.57, 0, 0), rospy.Time.now(), "wheel_right_link", FRAME_ID)

rospy.init_node('listener', anonymous=True)
rospy.loginfo('Subscribing to "/cmd_vel" topic')
rospy.Subscriber('/cmd_vel', Twist, cmdVelCallback)

CURRENT_TIME = rospy.Time.now()
LAST_TIME = rospy.Time.now()
#turtle_TF = tf.TransformBroadcaster()
turtle_TF = tf2_ros.StaticTransformBroadcaster()

pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
rospy.loginfo('Publishing the topic "odom".')
odom_broadcaster = tf.TransformBroadcaster()

pub_lidar = rospy.Publisher('scan', LaserScan, queue_size=10)
rospy.loginfo('Publishing the topic "scan"')
rospy.loginfo('Running the control loop')

handle_turtle_pose()
sendodom(True)


r = rospy.Rate(1/TIME_STEP)
while robot.step(TIME_STEP) != -1 and not rospy.is_shutdown():
    handle_turtle_pose()
    sendodom()

    lidar_values = readLidar(lidar)
    pub_lidar.publish(lidar_values)
    # print('Published sensor value: ', sensor.getValue())

    if message:
        # print(message)
        # rospy.loginfo(message)
        message = ''
    # left.setVelocity(velocity)
    # right.setVelocity(velocity)
    r.sleep()
