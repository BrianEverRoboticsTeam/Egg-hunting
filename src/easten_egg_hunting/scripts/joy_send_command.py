#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import math

def joy_callback(msg):
    goal_command = Twist()
    distance = None
    angle = None

    if msg.buttons[3]==1:
        distance = 1.7
        angle = 0
        # mode = "forward"
    elif msg.buttons[2]==1:
        distance = 0
        angle = (math.pi / 2)
        # mode = "turn_left"
    elif msg.buttons[0]==1:
        distance = 0
        angle = 0
        # mode = "stop"
    elif msg.buttons[1]==1:
        distance = 0
        angle = -(math.pi / 2)
        # mode = "forward"
    # else:
    #     distance = 0
    #     angle = 0

    # print(msg.buttons)

    if (distance!=None and angle!=None):
        goal_command.linear.x = distance
        goal_command.angular.z = angle
        percise_cmd_pub.publish(goal_command)

if __name__ == '__main__':

    rospy.init_node('joy_send_command_node')
    joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
    percise_cmd_pub = rospy.Publisher('control/precise_command',
                                       Twist, queue_size=1)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()
