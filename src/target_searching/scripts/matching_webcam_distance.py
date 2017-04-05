#!/usr/bin/python
import rospy
import cv2
import numpy as np
import cv_bridge
from scipy.misc import imresize
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cPickle as pickle
import os

showVideo = False
root = os.path.dirname(os.path.abspath(__file__))

class LogoDetector:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.ultrasonic_sub = rospy.Subscriber('ultrasonic', String,
                self.ultrasonic_callback)
        self.image_sub = rospy.Subscriber('usb_cam/image_raw', Image,
                self.image_callback)
        self.pub = rospy.Publisher('detector', String, queue_size=0)
        self.bridge = cv_bridge.CvBridge()
        self.dist = None
        self.image = None

        # Prepare templates at multiple scales:
        self.template_original = cv2.imread(root+'/logo.png',0)

        self.logo = cv2.imread(root+'/logo.png', 0)
        self.ar_tag = cv2.imread(root+'/ar_tag.png', 0)

        self.logo = imresize(self.logo, 0.4)
        self.ar_tag = imresize(self.ar_tag, 0.4)

        self.logo = cv2.GaussianBlur(self.logo, (9,9), 0)
        self.ar_tag = cv2.GaussianBlur(self.ar_tag, (9,9), 0)

        with open(root+'/param/webcam_logo.bin', 'rb') as f:
            self.kh, self.kw = pickle.load(f)

        with open(root+'/param/webcam_ar.bin', 'rb') as f:
            self.kh_ar, self.kw_ar = pickle.load(f)

    def ultrasonic_callback(self, msg):
        self.dist = int(msg.data)

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def detection(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # gray = cv2.GaussianBlur(gray, (9,9), 0)

        # All the 6 methods for comparison in a list
        # methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
                    # 'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
        # methods = ['cv2.TM_CCOEFF_NORMED']


        method = eval('cv2.TM_CCOEFF_NORMED')

        # Ultrasonic reading distance:
        dist = self.dist
        # print 'dist:', dist

        foundTarget = False

        # Calculate the size of UA logo template based on distance:
        w = int(round(self.kw / dist))
        h = int(round(self.kh / dist))

        # Apply template matching of ua logo:
        logoThreshold = 0.45
        if h < gray.shape[0] and w < gray.shape[1] and h > 20 and w > 20:
            logo = imresize(self.logo, (h, w))
            res_logo = cv2.matchTemplate(gray, logo, method)
            if res_logo.max() > logoThreshold:
                self.pub.publish('True')
                foundTarget = True
            if showVideo:
                loc_logo = np.where(res_logo > logoThreshold)
                for pt in zip(*loc_logo[::-1]):
                    cv2.rectangle(frame, pt, (pt[0]+w, pt[1]+h), (0,0,255), 2)

        # Calculate the size of AR tag template based on distance:
        w_ar = int(round(self.kw_ar / dist))
        h_ar = int(round(self.kh_ar / dist))

        # Apply template matching of AR Tag:
        arThreshold = 0.75
        if h_ar < gray.shape[0] and w_ar < gray.shape[1] and h_ar > 20 and w_ar > 20:
            ar_tag = imresize(self.ar_tag, (h_ar, w_ar))
            res_ar = cv2.matchTemplate(gray, ar_tag, method)
            if res_ar.max() > arThreshold:
                self.pub.publish('True')
                foundTarget = True
            if showVideo:
                loc_ar = np.where(res_ar > arThreshold)
                for pt in zip(*loc_ar[::-1]):
                    cv2.rectangle(frame, pt, (pt[0]+w_ar, pt[1]+h_ar), (0,128,255), 2)

        if not foundTarget:
            self.pub.publish('False')

        # Distance rectangle:
        if showVideo:
            tx0 = int(round(320-w/2))
            tx1 = int(round(320+w/2))
            ty0 = int(round(240-h/2))
            ty1 = int(round(240+h/2))
            tp0 = tx0, ty0
            tp1 = tx1, ty1
            cv2.rectangle(frame, tp0, tp1, (0,255,0), 2)

        # template = imresize(template, (h, w))
        # logo = imresize(self.logo, (h, w))
        # ar_tag = imresize(self.ar_tag, (h_ar, w_ar))

        # res_logo = cv2.matchTemplate(gray, logo, method)
        # res_ar = cv2.matchTemplate(gray, ar_tag, method)

        # Draw target rectangles:
        if showVideo:
            cv2.imshow('side detector', frame)


        # print loc_logo
        # if res_logo.max() > 0.45 or res_ar.max() > 0.75:
            # self.pub.publish('True')
        # else:
            # self.pub.publish('False')



    def spin(self):
        while not rospy.is_shutdown():
            if self.dist == None:
                continue
            if self.image == None:
                continue

            frame = self.image.copy()
            self.detection(frame)

            if showVideo:
                if cv2.waitKey(1) & 0xff == ord('q'):
                    break
            self.rate.sleep()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('logo_detector')
    logo = LogoDetector()
    logo.spin()
