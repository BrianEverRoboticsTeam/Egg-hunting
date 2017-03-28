#!/usr/bin/python

import rospy, cv2, cv_bridge, sys
import numpy as np
import glob, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Load previously saved data
import os
root = os.path.dirname(os.path.abspath(__file__))
# print(root)
npzfile = np.load(root + '/test.npz')
mtx = npzfile['arr_0']
dist = npzfile['arr_1']

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5) # blue
    cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5) # green
    cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5) # red
    #print(corner, tuple(imgpts[2].ravel()))
    return img


def image_callback(msg):
    global pt_on_right_mc, pt_on_left_mc, learning_rate, in_front, is_stop, fast_fail_num, is_far_away
    global new_born, percise_cmd_pub
    global is_on_operation

    if is_on_operation:
        return

    img = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # ret, corners = cv2.findChessboardCorners(gray, (8,6), cv2.cv.CV_CALIB_CB_ADAPTIVE_THRESH, cv2.CALIB_CB_FAST_CHECK)
    ret, corners = cv2.findChessboardCorners(gray, (8,6), None, cv2.CALIB_CB_FAST_CHECK)
    # ret, corners = cv2.findChessboardCorners(gray, (8,6), None)

    if(ret==False and fast_fail_num>30):
        fast_fail_num = 0
        try_hard_ret, corners = cv2.findChessboardCorners(gray, (8,6), None)


        if try_hard_ret == True:
            rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners, mtx, dist)

            shipped_imgpts, shipped_jac = cv2.projectPoints(shipped_axis, rvecs, tvecs, mtx, dist)
            # shipped_draw(img,shipped_imgpts)

            twist.linear.x = 0.1 # slow approching
            if twist_last.angular.z == 0:
                twist.angular.z = - math.radians(58) * (tuple(shipped_imgpts[3].ravel())[0] - 320)/320
            else:
                twist.angular.z = 0
            print("Far from target")
            is_far_away = True
            new_born = False
            percise_cmd_pub.publish(twist)

        else:
            pass


    # cv2.drawChessboardCorners(img,(8,6),corners,ret)

    elif ret == True:
        fast_fail_num = 0
        is_far_away = False
        new_born = False
        is_on_operation = True

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

        print(type(corners))

        rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners, mtx, dist)
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
        draw(img,corners,imgpts)

        """ find robot's angle to turn """
        rmtx = cv2.Rodrigues(rvecs)[0]
        # print(rmtx)
        cameraPosition = -np.matrix(rmtx).T * np.matrix(tvecs)
        # print(cameraPosition)

        # dinominator shouldn't be 0
        delta_z = cameraPosition[2]
        delta_y = cameraPosition[1]
        tan_c_o = delta_y/delta_z
        angle_bt_target = math.degrees(math.atan(tan_c_o))
        angle_to_turn = (90 - abs(angle_bt_target)) * angle_bt_target/abs(angle_bt_target)

        percise_command = Twist()
        percise_command.angular.z = math.radians(angle_to_turn)
        turn_dir = percise_command.angular.z / abs(percise_command.angular.z)
        percise_cmd_pub.publish(percise_command)

        # delay a bit
        state_change_time = rospy.Time.now() + rospy.Duration(2)
        while (rospy.Time.now() < state_change_time):
            pass

        percise_command.linear.x = delta_y * 0.025
        percise_command.angular.z = 0
        percise_cmd_pub.publish(percise_command)
        print("forward1:", percise_command.linear.x)

        state_change_time = rospy.Time.now() + rospy.Duration(2)
        while (rospy.Time.now() < state_change_time):
            pass

        percise_command.linear.x = 0
        percise_command.angular.z =  - turn_dir * math.pi / 2
        percise_cmd_pub.publish(percise_command)

        state_change_time = rospy.Time.now() + rospy.Duration(5)
        while (rospy.Time.now() < state_change_time):
            pass

        percise_command.linear.x = abs(delta_z * 0.025) - 0.3
        percise_command.angular.z = 0
        percise_cmd_pub.publish(percise_command)
        print("forward2:", percise_command.linear.x)

        state_change_time = rospy.Time.now() + rospy.Duration(2)
        while (rospy.Time.now() < state_change_time):
            pass






        """ find target space post """
        """
        if( (box_imgpts[0].ravel() - box_imgpts[4].ravel())[0]<0 and
            (box_imgpts[1].ravel() - box_imgpts[5].ravel())[0]<0 ): # target on right (robot on target's left)
            twist.angular.z = -0.5 # turn right
            twist_last.angular.z = -0.5
            # print("target on right")
        elif((box_imgpts[0].ravel() - box_imgpts[4].ravel())[0]>0 and
             (box_imgpts[1].ravel() - box_imgpts[5].ravel())[0]>0): # target on left (robot on taget's right)
            twist.angular.z = 0.5 # turn left
            twist_last.angular.z = 0.5
            # print("target on left")
        else: # target on center (robot in front of target)
            twist.angular.x = 0.05 # slow approching
            twist.angular.z = ( 320 - tuple(shipped_imgpts[3].ravel())[0] - z_vector(shipped_imgpts))/80
            # print("in front of target")
        """

        """ find target posision on frame """
        """
        if(tuple(shipped_imgpts[3].ravel())[0] > 320 ): # target on right of the frame
            twist_last.angular.z = 0.5
            # print("target on right of the frame")
        else: # target on left of the frame
            twist_last.angular.z = -0.5
            # print("target on left of the frame")
        """

        """ distance check """
        """
        # find the botton left point
        if (box_imgpts[8].ravel())[1] > (box_imgpts[9].ravel())[1]:
            botton_left_stop_point = box_imgpts[10].ravel()
        else:
            botton_left_stop_point = box_imgpts[11].ravel()

        # if( ((box_imgpts[8].ravel())[0] > 640 or (box_imgpts[8].ravel())[0] < 0 or
        #      (box_imgpts[8].ravel())[1] > 480 or (box_imgpts[8].ravel())[1] < 0 ) and
        #     ((box_imgpts[9].ravel())[0] > 640 or (box_imgpts[9].ravel())[0] < 0 or
        #      (box_imgpts[9].ravel())[1] > 480 or (box_imgpts[9].ravel())[1] < 0) ):
        if( (botton_left_stop_point[0] > 640 or botton_left_stop_point[0] < 0 or
             botton_left_stop_point[1] > 480 or botton_left_stop_point[1] < 0 ) ):
            is_stop = True
            new_born = True
            is_far_away = True
            twist.angular.z = 0
            twist.linear.x = 0
            # print("stop")
        """
        percise_cmd_pub.publish(twist)



    else:

        fast_fail_num += 1
        if (is_far_away):
            if(new_born):
                twist.linear.x = 0
                # new_born = False
            else:
                twist.linear.x = 0
            # twist.angular.z = (0 - twist_last.angular.z ) / 8 # turn to follow
        else:
            twist.linear.x = 0
            # twist.angular.z = (0 - twist_last.angular.z ) *0.9 # turn to opposite dir

        in_front == False




    cv2.imshow("window", img)
    cv2.waitKey(10)

from sensor_msgs.msg import Joy


""" main function starts here """
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
shipped_axis = np.float32([[6.5,2.5,0],
                           [3.5,5.5,0],
                           [3.5,2.5,-3],
                           [3.5,2.5,0]]).reshape(-1,3)
box_points = np.float32([[0,0,0], [0,5,0], [7,5,0], [7,0,0],
                         [0,0,-5], [0,5,-5], [7,5,-5], [7,0,-5],
                         [0,0,-10], [7,5,-10], [0,2.5,-8.5], [7,2.5,-8.5]]).reshape(-1,3)


bridge = cv_bridge.CvBridge()
cv2.namedWindow("window", 1)
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
percise_cmd_pub = rospy.Publisher('control/percise_command',
                                   Twist, queue_size=1)

rospy.init_node('demo5')
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    rate.sleep()

cv2.destroyAllWindows()
