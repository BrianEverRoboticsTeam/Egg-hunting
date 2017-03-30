#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math



""" global variables """
last_odom_time = None
twist_last = Twist()
twist = Twist()

goal_distance = 1.7 # in meters
goal_angle = 0#2*math.pi # in radians
goal_angle_dir = 1

mode = "stop"

def deactivate():
    global mode, twist

    mode = "stop"
    twist = Twist()
    feedback_pub.publish(mode)
    cmd_vel_pub.publish(twist)

def odom_callback(msg):
    global last_odom_time, twist_last, twist, goal_distance, goal_angle
    global goal_angle_dir, mode

    """ operation of times """
    current_odom_time = rospy.Time.now().to_nsec()
    if (last_odom_time==None):
        last_odom_time = current_odom_time
        duration = 0
    else:
        duration = current_odom_time - last_odom_time
        duration = duration/1000000000.0
        last_odom_time = current_odom_time
        # print(duration/1000000000.0)


    """ find angle and distance differents """
    # distance
    v_last = twist_last.linear.x
    v_current = msg.twist.twist.linear.x
    distance_i = ((v_current + v_last) * duration) / 2.0

    # angle
    d_last = twist_last.angular.z
    d_current = msg.twist.twist.angular.z
    d_angle_i = ((d_current + d_last) * duration) / 2.0

    # update last twist
    twist_last.linear.x = v_current
    twist_last.angular.z = d_current

    """ define control commands """
    if(mode == "forwarding"):
        goal_distance = goal_distance - distance_i
        if (goal_distance > 0.2):
            twist.linear.x = 0.8#goal_distance + 0.5
        elif (goal_distance > 0):
            twist.linear.x = goal_distance + 0.3
        else:
            twist.linear.x = 0
            last_odom_time = None
            deactivate()

        print("Distance to target:", goal_distance)

    if(mode == "backwarding"):
        goal_distance = goal_distance - abs(distance_i)
        if (goal_distance > 0.2):
            twist.linear.x = -0.8#goal_distance + 0.5
        elif (goal_distance > 0):
            twist.linear.x = -(goal_distance + 0.3)
        else:
            twist.linear.x = 0
            last_odom_time = None
            deactivate()

        print("Distance to target:", goal_distance)

    if(mode == "turning" or goal_angle > 0):
        goal_angle = goal_angle - abs(d_angle_i)

        """ acurrate version"""
        if (goal_angle > (math.pi/3)*2): # acurate
            twist.angular.z = 2*math.pi # acurate
        elif (goal_angle > 0):
            # twist.angular.z = (goal_angle / (math.pi / 4)) * math.pi + 0.5# too strong
            # twist.angular.z = goal_angle*2 # acurate
            twist.angular.z = goal_angle*2 + 0.5# good for small goal angle
        else:
            twist.angular.z = 0
            last_odom_time = None
            deactivate()

        """ aggrasive but not accurate version """
        """
        if (goal_angle > (math.pi)): # aggrasive
            twist.angular.z = 4*math.pi + 1.3553 # close to max
        elif (goal_angle > 0):
            # twist.angular.z = (goal_angle / (math.pi / 4)) * math.pi # too strong
            twist.angular.z = goal_angle*2 # acurate
        else:
            twist.angular.z = 0
            deactivate()
        """

        twist.angular.z = twist.angular.z * goal_angle_dir

        print("Angle to target:", goal_angle)

    if(mode != "stop"):
        cmd_vel_pub.publish(twist)


"""
This function take the input
"""
def command_callback(msg):
    global goal_distance, goal_angle
    global goal_angle_dir, mode

    if (msg.linear.x!=0):
        if msg.linear.x > 0:
            mode = "forwarding"
        elif msg.linear.x < 0:
            mode = "backwarding"
        goal_distance = abs(msg.linear.x)
        """ test only """
        if (msg.angular.z!=0):
            goal_angle = abs(msg.angular.z)
            goal_angle = goal_angle #- 0.05 # end point tunning
            goal_angle_dir = msg.angular.z / goal_angle
        else:
            goal_angle = 0
    elif (msg.angular.z!=0):
        mode = "turning"
        goal_angle = abs(msg.angular.z)
        goal_angle = goal_angle - 0.05 # end point tunning
        goal_angle_dir = msg.angular.z / abs(msg.angular.z)
    else:
        mode = "stop"


if __name__ == '__main__':
    rospy.init_node('percise_control_node')

    odom_sub = rospy.Subscriber('odom',Odometry, odom_callback)
    command_sub = rospy.Subscriber('control/precise_command',Twist, command_callback)
    cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    feedback_pub = rospy.Publisher('control/precise_command/feedback',
                                       String, queue_size=1)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()
