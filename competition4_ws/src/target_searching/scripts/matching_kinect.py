import rospy
from sensor_msgs.msg import Image
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
        self.bridge = cv_bridge.CvBridge()
        self.depth_bridge = cv_bridge.CvBridge()
        self.image = None
        self.depth = None

        # Prepare templates at multiple scales:
        self.template_original = cv2.imread('logo.png',0)
        self.template_original = imresize(self.template_original, 0.8)
        self.template_original = cv2.GaussianBlur(self.template_original, (9,9), 0)
        # self.template_original = cv2.blur(self.template_original, (9,9))
        cv2.imshow('original template', self.template_original)
        # scales = [0.1]
        # scales = [x*0.001 for x in range(100,500,30)]
        scales = np.linspace(0.02, 0.6, 20)
        self.templates = []
        for scale in scales:
            self.templates.append(imresize(self.template_original, scale))
        # template = imresize(template, 0.2)
        # w, h = template.shape[::-1]


        self.kh_table = []
        self.kw_table = []

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        self.depth = self.depth_bridge.imgmsg_to_cv2(msg)

    def detection(self, frame, p0, p1):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # gray = cv2.GaussianBlur(gray, (9,9), 0)

        # All the 6 methods for comparison in a list
        # methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
                    # 'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
        # methods = ['cv2.TM_CCOEFF']
        methods = ['cv2.TM_CCOEFF_NORMED']

        for meth in methods:
            method = eval(meth)





            template = self.template_original
            x0, y0 = p0
            x1, y1 = p1
            depth = self.depth.copy()
            region = depth[y0:y1, x0:x1]
            dist = np.nanmean(region.astype(np.float32))
            if dist < 10:
                cv2.imshow(meth, frame)
                continue

            print 'dist:', dist

            # w = int(round(101674.69 / dist))
            # h = int(round(128121.20 / dist))

            # w = int(round(104947.70 / dist))
            # h = int(round(132350.72 / dist))

            w = int(round(94895.492 / dist))
            h = int(round(119687.266 / dist))

            if h >= gray.shape[0] or w >= gray.shape[1] or h < 2 or w < 2:
                cv2.imshow(meth, frame)
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
            for pt in zip(*loc[::-1]):
                cv2.rectangle(frame, pt, (pt[0]+w, pt[1]+h), (0,0,255), 2)
            cv2.imshow(meth, frame)



            # Apply template Matching
            # for template in self.templates:
                # w, h = template.shape[::-1]
                # res = cv2.matchTemplate(gray,template,method)

                # # normalize for imshow:
                # # resimg = res / np.max(np.abs(res)) * 127.5 + 127.5
                # # resimg = resimg.astype(np.uint8)
                # # cv2.imshow('resimg'+str(scale), resimg)

                # found = False
                # loc = np.where(res > 0.8)
                # for pt in zip(*loc[::-1]):
                    # depth = self.depth.copy()
                    # # region = depth[pt[0]:pt[0]+w, pt[1]:pt[1]+h]
                    # region = depth[pt[1]:pt[1]+h, pt[0]:pt[0]+w]
                    # dist = np.nanmean(region.astype(np.float64))
                    # # kw = w / dist
                    # # kh = h / dist
                    # kw = w * dist
                    # kh = h * dist
                    # self.kw_table.append(kw)
                    # self.kh_table.append(kh)
                    # print 'dist:%f w:%d h:%d kw:%f kh:%f\n'%(dist, w, h, kw, kh)
                    # cv2.rectangle(frame, pt, (pt[0]+w, pt[1]+h), (0,0,255), 2)
                    # found = True
                # if found:
                    # break
            # cv2.imshow(meth, frame)
                





                # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                # force_bounding_box = False
                # if max_val > 0.5 or force_bounding_box:
                    # # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
                    # if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                        # top_left = min_loc
                    # else:
                        # top_left = max_loc
                    # bottom_right = (top_left[0] + w, top_left[1] + h)

                    # cv2.rectangle(frame,top_left, bottom_right, (0,0,255), 6)
                    # break


    def spin(self):
        while True:
            if self.image == None:
                continue
            if self.depth == None:
                continue

            # Show depth image:
            depth = self.depth.copy()
            cv2.normalize(depth, depth, 0, 255, cv2.NORM_MINMAX)
            depth = depth.astype(np.uint8)
            depth = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
            p0 = (300, 220) # (x, y)
            p1 = (340, 240) # (x, y)
            grid_y = np.linspace(200, 280, 2)
            grid_x = np.linspace(0, 639, 3)
            cv2.rectangle(depth, p0, p1, (0,0,255), 4)
            cv2.imshow('depth', depth)

            frame = self.image.copy()
            self.detection(frame, p0, p1)
            # cv2.imshow('frame', frame)

            if cv2.waitKey(1) & 0xff == ord('q'):
                break
            self.rate.sleep()

        # kh_array = np.array(self.kh_table, dtype=np.float32)
        # kw_array = np.array(self.kw_table, dtype=np.float32)
        # kh_mean = np.mean(kh_array)
        # kh_median = np.median(kh_array)
        # kw_mean = np.mean(kw_array)
        # kw_median = np.median(kw_array)
        # print 'Summary:'
        # print 'kh_mean:%f kh_median:%f'%(kh_mean, kh_median)
        # print 'kw_mean:%f kw_median:%f'%(kw_mean, kw_median)

        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('logo_detector')
    logo = LogoDetector()
    logo.spin()
