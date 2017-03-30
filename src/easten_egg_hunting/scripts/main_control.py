#!/usr/bin/python

import rospy, sys, cv2
import numpy as np
import glob, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from std_msgs.msg import String
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from kobuki_msgs.msg import Sound


twist = Twist()
twist_last = Twist()

pt_on_right_mc = 0
pt_on_left_mc = 0
learning_rate = 0.9
in_front = False
is_stop = False
is_far_away = True
fast_fail_num = 0

new_born = True
docking_is_on_operation = False
precise_cmd_in_operation = False

mode_set = ["nav","found","docking","undocking"]
mode = "nav"

def send_sound(value):
    sound = Sound()
    if value=="Found":
        sound.value = 0
    else:
        sound.value = 1

    sound_pub.publish(sound)

def do_undocking_sequence():
    global precise_cmd_in_operation, mode, docking_is_on_operation
    goal_command = Twist()
    distance = None
    angle = None

    # backwarding
    distance = -1.0
    angle = 0
    goal_command.linear.x = distance
    goal_command.angular.z = angle
    precise_cmd_in_operation = True
    precise_cmd_pub.publish(goal_command)

    while precise_cmd_in_operation:
        pass

    #turn_left
    distance = 0
    angle = 3*(math.pi / 4)

    goal_command.linear.x = distance
    goal_command.angular.z = angle
    precise_cmd_in_operation = True
    precise_cmd_pub.publish(goal_command)

    while precise_cmd_in_operation:
        pass

    mode="nav"
    nav_pub.publish("Start")
    docking_is_on_operation = False


def ar_pose_callback(pose_msg):
    global mode, docking_is_on_operation, precise_cmd_in_operation

    if mode=="docking" and not docking_is_on_operation:
        docking_is_on_operation = True

        # print pose_msg.data
        tvecs = np.array([[pose_msg.data[0]],[pose_msg.data[1]],[pose_msg.data[2]]])
        rvecs = np.array([[pose_msg.data[3]],[pose_msg.data[4]],[pose_msg.data[5]]])
        print("tvecs:\n", tvecs)
        print("rvecs:\n", rvecs)

        percise_cmd_pub = precise_cmd_pub

        rmtx = cv2.Rodrigues(rvecs)[0]
        rmtx = np.append(rmtx,tvecs,axis=1)
        rmtx = np.append(rmtx,np.array([[0,0,0,1]]),axis=0)
        print(rmtx)

        target_position = np.matrix(rmtx) * np.matrix([[0.0],[0.0],[-1.0],[1.0]])
        print(target_position)

        # dinominator shouldn't be 0
        delta_z = target_position[2]
        delta_x = target_position[0]
        tan_c_o = -delta_x/delta_z
        angle_bt_target = math.degrees(math.atan(tan_c_o))
        # angle_to_turn = (90 - abs(angle_bt_target)) * angle_bt_target/abs(angle_bt_target)
        angle_to_turn = angle_bt_target
        print(angle_to_turn)
        print(tan_c_o)

        percise_command = Twist()
        percise_command.angular.z = math.radians(angle_to_turn)
        turn_dir = percise_command.angular.z / abs(percise_command.angular.z)
        precise_cmd_in_operation = True
        percise_cmd_pub.publish(percise_command)

        # delay a bit
        while (precise_cmd_in_operation):
            pass

        percise_command.linear.x = math.sqrt(delta_z*delta_z + delta_x*delta_x) #* 0.025
        #percise_command.angular.z = 0
        percise_command.linear.x -= 0.5
        precise_cmd_in_operation = True
        percise_cmd_pub.publish(percise_command)
        print("forward1:", percise_command.linear.x)

        while (precise_cmd_in_operation):
            pass

        send_sound("Found")
        # docking_is_on_operation = False
        # mode = "undocking"

def logo_pose_callback(pose_msg):
    global mode, docking_is_on_operation, precise_cmd_in_operation

    if mode=="docking" and not docking_is_on_operation:
        docking_is_on_operation = True

        # print pose_msg.data
        tvecs = np.array([[pose_msg.data[0]],[pose_msg.data[1]],[pose_msg.data[2]]])
        rvecs = np.array([[pose_msg.data[3]],[pose_msg.data[4]],[pose_msg.data[5]]])
        print("tvecs:\n", tvecs)
        print("rvecs:\n", rvecs)

        percise_cmd_pub = precise_cmd_pub

        rmtx = cv2.Rodrigues(rvecs)[0]
        rmtx = np.append(rmtx,tvecs,axis=1)
        rmtx = np.append(rmtx,np.array([[0,0,0,1]]),axis=0)
        print(rmtx)

        target_position = np.matrix(rmtx) * np.matrix([[0.0],[0.0],[-1.0],[1.0]])
        print(target_position)

        # dinominator shouldn't be 0
        delta_z = target_position[2]
        delta_x = target_position[0]
        tan_c_o = -delta_x/delta_z
        angle_bt_target = math.degrees(math.atan(tan_c_o))
        # angle_to_turn = (90 - abs(angle_bt_target)) * angle_bt_target/abs(angle_bt_target)
        angle_to_turn = angle_bt_target
        print(angle_to_turn)
        print(tan_c_o)

        percise_command = Twist()
        percise_command.angular.z = math.radians(angle_to_turn)
        turn_dir = percise_command.angular.z / abs(percise_command.angular.z)
        precise_cmd_in_operation = True
        percise_cmd_pub.publish(percise_command)

        # delay a bit
        while (precise_cmd_in_operation):
            pass

        percise_command.linear.x = math.sqrt(delta_z*delta_z + delta_x*delta_x) #* 0.025
        #percise_command.angular.z = 0
        percise_command.linear.x -= 1.0
        precise_cmd_in_operation = True
        percise_cmd_pub.publish(percise_command)
        print("forward1:", percise_command.linear.x)

        while (precise_cmd_in_operation):
            pass

        send_sound("Found")
        mode = "undocking"
        # docking_is_on_operation = False

def detection_callback(detection_msg):
    global mode, new_born

    # print(detection_msg.data)
    if detection_msg.data=="True" and mode=="nav" and not new_born:
        detected = True
        nav_pub.publish("Stop") # stop navigation
        mode = "found"

        goal_command = Twist()
        distance = 0
        angle = -(math.pi / 2)
        goal_command.linear.x = distance
        goal_command.angular.z = angle
        precise_cmd_in_operation = True
        precise_cmd_pub.publish(goal_command)

        while (precise_cmd_in_operation):
            pass

        mode = "docking"

def nav_callback(nav_msg):
    global mode
    print(nav_msg.data)
    if mode=="nav":
        pass

def logo_approching_guide_callback(guide_msg):
    global mode, docking_is_on_operation, precise_cmd_in_operation

    #print(type(guide_msg.data))
    if mode=="docking" and not docking_is_on_operation:
        command = Twist()
        # command.linear.x = 0.1
        command.angular.z = math.radians(30) * (320 - guide_msg.data) / 320.0
        # cmd_vel_pub.publish(command)
        precise_cmd_in_operation = True
        precise_cmd_pub.publish(command)
        print("UA logo approching:", command.angular.z)
        print(320 - guide_msg.data)

        while (precise_cmd_in_operation):
            pass

        while not docking_is_on_operation:
            command.linear.x = 0.2
            command.angular.z = 0
            cmd_vel_pub.publish(command)

def precise_cmd_callback(precise_cmd_feedback_msg):
    global mode, precise_cmd_in_operation
    # print precise_cmd_feedback_msg.data
    if mode=="found" and precise_cmd_feedback_msg.data=="stop":
        mode = "docking"
    elif mode=="docking" and precise_cmd_feedback_msg.data=="stop":
        precise_cmd_in_operation = False

def nav_feedback_callback(nav_feedback_msg):
    global mode, new_born

    if mode=="nav" and nav_feedback_msg.data=="Done":
        new_born = False

""" ros node configs """
rospy.init_node('main_control_node')

ar_pose_sub = rospy.Subscriber('ar_pose', numpy_msg(Floats), ar_pose_callback)
logo_pose_sub = rospy.Subscriber('logo_pose', numpy_msg(Floats), logo_pose_callback)
logo_approching_guide_sub = rospy.Subscriber('detector_x', Float32, logo_approching_guide_callback)
detection_sub = rospy.Subscriber('detector', String, detection_callback)
# nav_sub = rospy.Subscriber('egg_navigation/raw_cmd_vel', Twist, nav_callback)

nav_pub = rospy.Publisher('command_to_navi', String, queue_size = 1)
sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size = 1)
cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
precise_cmd_pub = rospy.Publisher('control/precise_command',
                                   Twist, queue_size=1)
precise_cmd_feedback_sub = rospy.Subscriber('control/precise_command/feedback',
                                   String, precise_cmd_callback)
nav_feedback_sub = rospy.Subscriber('command_to_navi_feedback',
                                   String, nav_feedback_callback)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    if mode=="nav":
        """ simulate navigation """
        # command = Twist()
        # command.linear.x = 0.5
        # cmd_vel_pub.publish(command)

        """ real navigation """
        if new_born:
            nav_pub.publish("Relocate")

    elif mode=="undocking":
        do_undocking_sequence()

    rate.sleep()
