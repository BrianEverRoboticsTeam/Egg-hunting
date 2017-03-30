#!/usr/bin/python
import rospy, os
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv_bridge
import cv2
import numpy as np
from matplotlib import pyplot as plt
from scipy.misc import imresize
import scipy

class LogoDetector:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('camera/depth/image_raw',
                Image, self.depth_callback)
        self.x_pub = rospy.Publisher('detector_x', Float32, queue_size=1)

        self.bridge = cv_bridge.CvBridge()
        self.depth_bridge = cv_bridge.CvBridge()
        self.image = None
        self.depth = None

        # Prepare templates at multiple scales:
        root = os.path.dirname(os.path.abspath(__file__))
        self.template_original = cv2.imread(root+'/logo.png',0)
        self.template_original = imresize(self.template_original, 0.8)
        self.template_original = cv2.GaussianBlur(self.template_original, (9,9), 0)
        # self.template_original = cv2.blur(self.template_original, (9,9))
        # cv2.imshow('original template', self.template_original)

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        self.depth = self.depth_bridge.imgmsg_to_cv2(msg)

    def detection(self, frame, p0, p1):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # gray = cv2.GaussianBlur(gray, (9,9), 0)

        methods = ['cv2.TM_CCOEFF_NORMED']
        for meth in methods:
            method = eval(meth)

            template = self.template_original
            x0, y0 = p0
            x1, y1 = p1
            depth = self.depth.copy()
            region = depth[y0:y1, x0:x1]
            dist = np.nanmean(region.astype(np.float32))

            if np.isnan(dist) or dist < 1.0:
                self.x_pub.publish(-1.0)
                continue

            # print 'dist:', dist

            w = int(round(94895.492 / dist))
            h = int(round(119687.266 / dist))

            if h >= gray.shape[0] or w >= gray.shape[1] or h < 10 or w < 10:
                # cv2.imshow(meth, frame)
                self.x_pub.publish(-1.0)
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
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            pt = min_loc
            if max_val > 0.45:
                # cv2.rectangle(frame, pt, (pt[0]+w, pt[1]+h), (0,0,255), 2)
                x = pt[0] + w/2
                self.x_pub.publish(x)
            else:
                self.x_pub.publish(-1.0)
            # loc = np.where(res > 0.45)
            # for pt in zip(*loc[::-1]):
                # cv2.rectangle(frame, pt, (pt[0]+w, pt[1]+h), (0,0,255), 2)
            # cv2.imshow(meth, frame)





    def spin(self):
        while not rospy.is_shutdown():
            if self.image == None:
                continue
            if self.depth == None:
                continue

            p0 = (300, 220) # (x, y)
            p1 = (340, 240) # (x, y)

            # Show depth image:
            # depth = self.depth.copy()
            # cv2.normalize(depth, depth, 0, 255, cv2.NORM_MINMAX)
            # depth = depth.astype(np.uint8)
            # depth = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
            # grid_y = np.linspace(200, 280, 2)
            # grid_x = np.linspace(0, 639, 3)
            # cv2.rectangle(depth, p0, p1, (0,0,255), 4)
            # cv2.imshow('depth', depth)

            frame = self.image.copy()
            self.detection(frame, p0, p1)
            # cv2.imshow('frame', frame)

            # if cv2.waitKey(1) & 0xff == ord('q'):
                # break
            self.rate.sleep()


        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('logo_detector')
    logo = LogoDetector()
    logo.spin()
