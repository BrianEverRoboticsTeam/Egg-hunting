#!/usr/bin/python

import rospy, sys
import numpy as np
import glob, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


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

image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, image_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel_safe', Twist, queue_size=1)
percise_cmd_pub = rospy.Publisher('control/precise_command',
                                   Twist, queue_size=1)

rospy.init_node('vision_docking_node')
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    rate.sleep()

cv2.destroyAllWindows()
