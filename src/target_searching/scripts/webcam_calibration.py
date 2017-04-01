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
        # self.template_original = cv2.imread('logo.png',0)
        self.template_original = cv2.imread('ar_tag.png',0)
        self.template_original = imresize(self.template_original, 0.3)
        self.th, self.tw = self.template_original.shape
        self.template_original = cv2.GaussianBlur(self.template_original, (9,9), 0)
        # self.template_original = cv2.blur(self.template_original, (9,9))
        cv2.imshow('original template', self.template_original)
        scales = np.linspace(0.1, 0.8, 20)
        self.templates = []
        for scale in scales:
            self.templates.append(imresize(self.template_original, scale))

        self.kh_table = []
        self.kw_table = []

    def ultrasonic_callback(self, msg):
        self.dist = int(msg.data)

    def detection(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # gray = cv2.GaussianBlur(gray, (9,9), 0)

        # All the 6 methods for comparison in a list
        # methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
                    # 'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
        # methods = ['cv2.TM_CCOEFF']
        methods = ['cv2.TM_CCOEFF_NORMED']
        for meth in methods:
            method = eval(meth)



            # Apply template Matching
            for template in self.templates:
                w, h = template.shape[::-1]
                res = cv2.matchTemplate(gray,template,method)

                # normalize for imshow:
                # resimg = res / np.max(np.abs(res)) * 127.5 + 127.5
                # resimg = resimg.astype(np.uint8)
                # cv2.imshow('resimg'+str(scale), resimg)

                found = False
                loc = np.where(res > 0.7)
                for pt in zip(*loc[::-1]):
                    dist = self.dist
                    # kw = w / dist
                    # kh = h / dist
                    kw = w * dist
                    kh = h * dist
                    self.kw_table.append(kw)
                    self.kh_table.append(kh)
                    print 'dist:%f w:%d h:%d kw:%f kh:%f\n'%(dist, w, h, kw, kh)
                    cv2.rectangle(frame, pt, (pt[0]+w, pt[1]+h), (0,0,255), 2)
                    found = True
                if found:
                    break
            cv2.imshow(meth, frame)
                


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


        kh_array = np.array(self.kh_table, dtype=np.float32)
        kw_array = np.array(self.kw_table, dtype=np.float32)
        kh_mean = np.mean(kh_array)
        kh_median = np.median(kh_array)
        kw_mean = np.mean(kw_array)
        kw_median = np.median(kw_array)

        with open('param/webcam_ar.bin', 'wb') as f:
            pickle.dump([kh_median, kw_median], f, pickle.HIGHEST_PROTOCOL)

        print 'Summary:'
        print 'kh_mean:%f kh_median:%f'%(kh_mean, kh_median)
        print 'kw_mean:%f kw_median:%f'%(kw_mean, kw_median)
        print 'the data has been saved'

        self.cam.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('logo_detector')
    logo = LogoDetector()
    logo.spin()
