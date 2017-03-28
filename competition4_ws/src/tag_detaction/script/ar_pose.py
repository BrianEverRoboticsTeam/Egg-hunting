#!/usr/bin/python

import rospy, cv2, cv_bridge, sys, math, tf
import numpy as np
import glob
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
#import cv2.aruco as aruco

# Load previously saved data
npzfile = np.load('test.npz')
mtx = npzfile['arr_0']
dist = npzfile['arr_1']

def draw(img, imgpts):
    corner = tuple(imgpts[3].ravel())
    cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5) # blue
    cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5) # green
    cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5) # red
    return img

def image_callback(msg):
    global tvecs, rvecs, detect

    img = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    if detect == True:
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
        draw(img,imgpts)

    cv2.imshow('ar_track', img)
    cv2.waitKey(1)

def pose_callback(msg):
    global tvecs, rvecs, detect
    if len(msg.markers)>=1:
        for i in range(len(msg.markers)):
            if msg.markers[i].id == 11:
                break
        if msg.markers[i].id != 11:
            return

        print("-------------------------")
        print("AR pose reading")

        if msg.markers[i].pose.pose.position.z < 0.8:
            print("Caution: too close")

        print ("id: "+str(msg.markers[i].id))
        print ("distance: "+str(msg.markers[i].pose.pose.position.z))
        print("-------------------------")
        tvecs = np.array([[msg.markers[i].pose.pose.position.x],
                          [msg.markers[i].pose.pose.position.y],
                          [msg.markers[i].pose.pose.position.z]])
        #print("tvecs:")
        #print(tvecs)
        #print("-------------------------")
        qx = msg.markers[i].pose.pose.orientation.x
        qy = msg.markers[i].pose.pose.orientation.y
        qz = msg.markers[i].pose.pose.orientation.z
        qw = msg.markers[i].pose.pose.orientation.w
        X = qx/math.sqrt(1-qw*qw)
        Y = qy/math.sqrt(1-qw*qw)
        Z = qz/math.sqrt(1-qw*qw)
        angle = 2*math.acos(qw)
        ratio = math.sqrt(X*X+Y*Y+Z*Z)
        X = -X/ratio*angle
        Y = -Y/ratio*angle
        Z = Z/ratio*angle
        rvecs = np.array([[X],[Y],[Z]])
        #print("rvecs:")
        #print(rvecs)
        #print("================================================")
        detect = True

        #pose = np.array([tvecs, rvecs], dtype=np.float32)
        pose = np.array([tvecs[0],tvecs[1],tvecs[2],rvecs[0],rvecs[1],rvecs[2]], dtype=np.float32)
        # print("tvecs:")
        # print(tvecs)
        # print("rvecs:")
        # print(rvecs)
        # print("pose:")
        # print(pose)
        pub.publish(pose)

    else:
        detect = False

global detect
detect = False
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#objp = np.zeros((6*8,3), np.float32)
#objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

axis = np.float32([[0.15,0,0], [0,0.15,0], [0,0,-0.15], [0,0,0]]).reshape(-1,3)

bridge = cv_bridge.CvBridge()
rospy.init_node('ar_pose_reader')
# cv2.namedWindow("window", 1)
pub = rospy.Publisher('ar_pose', numpy_msg(Floats), queue_size=10)
#image_sub = rospy.Subscriber('usb_cam/image_raw', Image, image_callback)
image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, image_callback)
pose_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, pose_callback)
# rate = rospy.Rate(10)
# while not rospy.is_shutdown():
#     rate.sleep()
rospy.spin()

cv2.destroyAllWindows()
