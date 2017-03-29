#!/usr/bin/python
import rospy
import cv2
import numpy as np
from scipy.misc import imresize
from std_msgs.msg import String
import cPickle as pickle

class LogoDetector:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.ultrasonic_sub = rospy.Subscriber('ultrasonic', String,
                self.ultrasonic_callback)
        self.dist = None
        self.cam = cv2.VideoCapture(0)
        self.image = None

        # Prepare templates at multiple scales:
        self.template_original = cv2.imread('logo.png',0)
        self.template_original = imresize(self.template_original, 0.8)
        self.th, self.tw = self.template_original.shape
        self.template_original = cv2.GaussianBlur(self.template_original, (9,9), 0)

        #cv2.imshow('original template', self.template_original)


        with open('param/khw.bin', 'rb') as f:
            self.kh, self.kw = pickle.load(f)

    def ultrasonic_callback(self, msg):
        self.dist = int(msg.data)

    def detection(self, frame):
        global detect
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        methods = ['cv2.TM_CCOEFF_NORMED']
        for meth in methods:
            method = eval(meth)

            template = self.template_original

            dist = self.dist
            #print 'dist:', dist

            w = int(round(self.kw / dist))
            h = int(round(self.kh / dist))

            if h >= gray.shape[0] or w >= gray.shape[1] or h < 20 or w < 20:
                #cv2.imshow(meth, frame)
                continue

            # test rectangle:
            tx0 = int(round(320-w/2))
            tx1 = int(round(320+w/2))
            ty0 = int(round(240-h/2))
            ty1 = int(round(240+h/2))
            tp0 = tx0, ty0
            tp1 = tx1, ty1
            cv2.rectangle(frame, tp0, tp1, (0,255,0), 2)

            template = imresize(template, (h, w))
            res = cv2.matchTemplate(gray, template, method)
            loc = np.where(res > 0.45)
            # for pt in zip(*loc[::-1]):
            #     cv2.rectangle(frame, pt, (pt[0]+w, pt[1]+h), (0,0,255), 2)
            # cv2.imshow(meth, frame)

            if len(loc) > 0:
                print("logo detected")
                print 'dist:', dist
                detect = "True"
            else:
                detect = "False"
        pub.publish(detect)


    def spin(self):
        while True:
            if self.dist == None:
                continue

            ret, self.image = self.cam.read()
            assert ret == True

            frame = self.image.copy()
            self.detection(frame)

            if cv2.waitKey(1) & 0xff == ord('q'):
                break

        self.cam.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('logo_detector')
    pub = rospy.Publisher('detector', String, queue_size=10)
    logo = LogoDetector()
    logo.spin()
