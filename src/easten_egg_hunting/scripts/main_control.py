#!/usr/bin/python

import rospy, sys
import numpy as np
import glob, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from std_msgs.msg import String


twist = Twist()
twist_last = Twist()

pt_on_right_mc = 0
pt_on_left_mc = 0
learning_rate = 0.9
in_front = False
is_stop = False
is_far_away = True
new_born = True
fast_fail_num = 0
is_on_operation = False

mode_set = ["nav","found","docking","undocking"]
mode = "nav"

def pose_callback(pose_msg):
    global mode

    print pose_msg.data

def detection_callback(detection_msg):
    global mode

    # print(detection_msg.data)
    if detection_msg.data=="True" and mode=="nav":
        detected = True
        mode = "found"

        goal_command = Twist()
        distance = 0
        angle = -(math.pi / 2)
        goal_command.linear.x = distance
        goal_command.angular.z = angle
        precise_cmd_pub.publish(goal_command)

def precise_cmd_callback(precise_cmd_feedback_msg):
    print precise_cmd_feedback_msg.data

""" ros node configs """
detection_sub = rospy.Subscriber('detector', String, detection_callback)

cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
precise_cmd_pub = rospy.Publisher('control/precise_command',
                                   Twist, queue_size=1)
precise_cmd_feedback_sub = rospy.Subscriber('control/precise_command/feedback',
                                   String, precise_cmd_callback)

rospy.init_node('main_control_node')
rate = rospy.Rate(10)
while not rospy.is_shutdown():

    """ simulate navigation """
    if mode=="nav":
        command = Twist()
        command.linear.x = 0.5
        cmd_vel_pub.publish(command)

    rate.sleep()
