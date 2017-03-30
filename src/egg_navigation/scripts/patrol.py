#!/usr/bin/env python

import rospy
import actionlib
import tf
import os
import time
import numpy as np

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped

from geometry_msgs.msg import Twist
from ramp import Movement, getTimeSafe

# 90 degree turns:
# waypoints = [
    # [(-2.15, 2.58, 0.0), (0.0, 0.0, -0.71, 0.70)],
    # [(-2.71, -1.21, 0.0), (0.0, 0.0, -0.00, 0.99)],
    # [(7.99, -2.18, 0.0), (0.0, 0.0, 0.69, 0.72)],
    # [(8.10, 2.21, 0.0), (0.0, 0.0, 0.99, 0.01)]
# ]

# 45 degree turns:
# waypoints = [
    # [(-1.96, 2.97, 0.0), (0.0, 0.0, -0.91, 0.42)],
    # [(-2.63, -1.78, 0.0), (0.0, 0.0, -0.40, 0.92)],
    # [(7.78, -2.17, 0.0), (0.0, 0.0, 0.37, 0.93)],
    # [(8.46, 2.28, 0.0), (0.0, 0.0, 0.95, 0.32)]
# ]

# 45 degree turns smaller area:
waypoints = [
    [(-2.12, 2.88, 0.0), (0.0, 0.0, -0.93, 0.36)],
    [(-2.45, -1.25, 0.0), (0.0, 0.0, -0.36, 0.93)],
    [(8.01, -2.03, 0.0), (0.0, 0.0, 0.36, 0.93)],
    [(8.20, 2.26, 0.0), (0.0, 0.0, 0.92, 0.39)]
]


client = None
force_stop = False
current_goal = None
pause = True
global_localization = None
cmd_vel_pub = None
command_pub = None
restart = False


def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    return goal_pose


# def joy_callback(msg):
    # global trig
    # global client
    # if msg.buttons[2]==1:
        # trig = not trig
        # print 'trigger =', trig
    # elif msg.buttons[1]==1:
        # print 'Exit...'
        # client.cancel_all_goals()
        # force_stop = True
        # # os._exit(0)


# def detector_callback(msg):
    # global client, command_pub, force_stop
    # # time.sleep(2)
    # # client.cancel_goal()
    # # force_stop = True

    # if msg.data == 'True':
        # client.cancel_goal()
        # while pause:
            # time.sleep(0.1)
        # client.send_goal(current_goal)
Relocate_is_on_going = False

def command_callback(msg):
    global pause, client, cmd_vel_pub, command_pub, Relocate_is_on_going
    if msg.data == 'Start':
        pause = False
    elif msg.data == 'Stop':
        if client != None:
            client.cancel_goal()
        pause = True
    elif msg.data == 'Relocate':
        if not Relocate_is_on_going:
            Relocate_is_on_going = True

            if client != None:
                client.cancel_goal()
            global_localization()
            timelimit = getTimeSafe() + rospy.Duration(8)
            tw = Twist()
            while getTimeSafe() < timelimit:
                tw.angular.z = 0.8
                cmd_vel_pub.publish(tw)
            command_pub.publish('Done')
            pause = False
            restart = True

            Relocate_is_on_going = False


def pose_callback(msg):
    # print msg.pose
    pass


def main():
    global trig, current_goal, client, global_localization, cmd_vel_pub, command_pub, restart

    rospy.init_node('patrol')

    # detector_sub = rospy.Subscriber('detector', String, detector_callback)
    pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, pose_callback)
    command_sub = rospy.Subscriber('command_to_navi', String, command_callback)
    command_pub = rospy.Publisher('command_to_navi_feedback', String, queue_size=1)
    cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Reset AMCL Localizer:
    rospy.wait_for_service('global_localization')
    global_localization = rospy.ServiceProxy('global_localization', Empty)
    #global_localization() # reset pose to start amcl

    while pause:
        time.sleep(0.1)

    # Do some simple movements to find the location

    # move = Movement()
    # tw = Twist()
    # # move.start()
    # # move.updateTarget(tw)
    # print 'turning...'
    # timelimit = getTimeSafe() + rospy.Duration(8)
    # while getTimeSafe() < timelimit:
    #     tw.angular.z = 0.8
    #     cmd_vel_pub.publish(tw)
    #
    # print 'stopped.'
    # print 'navigation..'
    #
    # command_pub.publish('Done')


    while not rospy.is_shutdown():
        for pose in waypoints:
            print 'send goal pose', pose
            goal = goal_pose(pose)
            current_goal = goal
            client.send_goal(goal)
            client.wait_for_result()

            if pause:
                while pause:
                    time.sleep(0.1)
                client.send_goal(goal)

            if restart:
                restart = False
                break


if __name__ == '__main__':
    main()
