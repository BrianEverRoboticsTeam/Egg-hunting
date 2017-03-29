#!/usr/bin/env python

import rospy
import actionlib
import tf
import os

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy


# Counterclockwise safe:
waypoints = [
    [(6.29, -1.80, 0.0), (0.0, 0.0, 0.63, 0.78)],
    [(6.31, 1.23, 0.0), (0.0, 0.0, 0.99, 0.09)],
    [(0.63, 1.43, 0.0), (0.0, 0.0, -0.78, 0.62)],
    [(0.49, -1.39, 0.0), (0.0, 0.0, -0.17, 0.98)]
]


client = None
curr_pose = None
running = False


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


def joy_callback(msg):
    global running
    global client
    if msg.buttons[2]==1:
        running = not running
    elif msg.buttons[1]==1:
        print 'Exit...'
        client.cancel_goal()
        os._exit(0)


if __name__ == '__main__':
    rospy.init_node('patrol')
    joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # <3>
    client.wait_for_server()

    # Go to the start point:
    client.send_goal(goal_pose(waypoints[0]))
    client.wait_for_result()

    print 'ready'
    while not running:
        pass
    
    while not rospy.is_shutdown():
    # for _ in range(3):
        for pose in waypoints:
            curr_pose = pose
            goal = goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()
