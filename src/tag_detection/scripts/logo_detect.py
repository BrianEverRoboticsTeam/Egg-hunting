#!/usr/bin/python

import numpy as np
import rospy, cv2, cv_bridge
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

# Load previously saved data
import os
root = os.path.dirname(os.path.abspath(__file__))
npzfile = np.load(root + '/test.npz')
mtx = npzfile['arr_0']
dist = npzfile['arr_1']

def draw(img, imgpts):
    corner = tuple(imgpts[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[3].ravel()), (0,0,255), 5)
    return img

def prepare_solvePnP(good_src_pts):
    src_pts_array = np.zeros((good_src_pts.shape[0],1,good_src_pts.shape[2]+1), np.float32)
    for i in range(len(good_src_pts)):
        src_pts_array[i][0][0] = good_src_pts[i][0][0]
        src_pts_array[i][0][1] = good_src_pts[i][0][1]
        src_pts_array[i][0][2] = 0.0
    return src_pts_array

def image_callback(msg):
    global detect, img1, kp1, des1

    img2 = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    kp2, des2 = orb.detectAndCompute(img2,None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # initial error handling
    matches = bf.match(des1, des1)
    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                    singlePointColor = None,
                    matchesMask = None, # draw only inliers
                    flags = 2)
    output = cv2.drawMatches(img1,kp1,img2,[],None,None,None)

    # start matching
    if(len(kp2)!=0):
        matches = bf.match(des1, des2)
    else:
        print("Vision blocked!")
        cv2.imshow('output', output)
        if cv2.waitKey(1) & 0xff == ord('q'):
            return
        return

    if len(matches)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in matches ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in matches ]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()

        good_matches = []
        good_src_pts = []
        good_dst_pts = []
        count = 0
        for i in range(len(mask)):
            if mask[i][0] == 1:
                count += 1
                good_matches.append(matches[i])
                good_src_pts.append(kp1[matches[i].queryIdx].pt)
                good_dst_pts.append(kp2[matches[i].trainIdx].pt)
        good_src_pts = np.float32(good_src_pts).reshape(-1,1,2)
        good_dst_pts = np.float32(good_dst_pts).reshape(-1,1,2)

        sorted_good_matches = sorted(good_matches, key=lambda x:x.distance)
        perfect_matches = sorted_good_matches[:int(count*2/4)]

        perfect_src_pts = np.float32([ kp1[m.queryIdx].pt for m in perfect_matches ]).reshape(-1,1,2)
        perfect_dst_pts = np.float32([ kp2[m.trainIdx].pt for m in perfect_matches ]).reshape(-1,1,2)

        try:
            M2, mask2 = cv2.findHomography(perfect_src_pts, perfect_dst_pts, cv2.RANSAC,5.0)
            matchesMask2 = mask2.ravel().tolist()
        except:
            perfect_matches = matches
        else:
            M = M2
            matchesMask = matchesMask2

        print("inliers num: ", count)

        # bounding box
        h,w,_ = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,M)

        img2 = cv2.polylines(img2,[np.int32(dst)],True,(255,255,0),3, cv2.LINE_AA)

        # pts = np.float32(good_matches).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(good_src_pts,M)


        src_pts_array = prepare_solvePnP(good_src_pts)

        # _, rvecs, tvecs, inliers = cv2.solvePnPRansac(src_pts_array, dst, mtx, dist)
        ret, rvecs, tvecs = cv2.solvePnP(src_pts_array, dst, mtx, dist)
        imgpts, shipped_jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
        if count > 20:
            img2 = draw(img2, imgpts)

            tvecs = tvecs / (np.ones((3,1)) * WORLD_RATIO)
            pose = np.array([tvecs[0],tvecs[1],tvecs[2],rvecs[0],rvecs[1],rvecs[2]], dtype=np.float32)
            print("tvecs:")
            print(tvecs)
            print("rvecs:")
            print(rvecs)
            # print("pose:")
            # print(pose)
            pub.publish(pose)

            #print "rvecs:\n", rvecs, "\n", "tvecs:\n", tvecs / (np.ones((3,1)) * WORLD_RATIO), "\n===========\n"

        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                        singlePointColor = None,
                        matchesMask = matchesMask, # draw only inliers
                        flags = 2)
        output = cv2.drawMatches(img1,kp1,img2,kp2,perfect_matches,None,**draw_params)
        # output = cv2.drawMatches(img1,good_kp1,img2,good_kp2,good_matches,None,**draw_params)


    else:
        print "Not enough matches are found - %d/%d" % (len(matches),MIN_MATCH_COUNT)
        matchesMask = None

    cv2.imshow('logo_track', output)
    cv2.waitKey(1)




global detect, img1, kp1, des1
detect = False

# There is a bug in opencv3.0> with failure in orb.compute, this is a
# temp fix:
cv2.ocl.setUseOpenCL(False)

cam = cv2.VideoCapture(0)
MIN_MATCH_COUNT = 10

WORLD_RATIO = 135.5/0.0555

img1 = cv2.imread(root+'/logo.png')   # queryImage
orb = cv2.ORB_create()#nfeatures=50, nlevels=1)
# orb = cv2.ORB()
kp1, des1 = orb.detectAndCompute(img1,None)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
axis = np.float32([[0,0,0], [100,0,0], [0,100,0], [0,0,-100]]).reshape(-1,3)
bridge = cv_bridge.CvBridge()

rospy.init_node('logo_pose_reader')
pub = rospy.Publisher('logo_pose', numpy_msg(Floats), queue_size=10)
#image_sub = rospy.Subscriber('usb_cam/image_raw', Image, image_callback)
image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, image_callback)

rospy.spin()

cv2.destroyAllWindows()
