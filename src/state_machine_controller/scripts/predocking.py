#!/usr/bin/env python
import rospy, time, math
from smach import State, StateMachine
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
import numpy as np

class PreDocking(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist,
                queue_size = 1)
        self.logo_dir_sub = rospy.Subscriber(
            'detector_x', Float32, self.logo_approching_guide_callback
        )
        self.ar_dir_sub = rospy.Subscriber(
            'ar_detector_x', Float32, self.ar_approching_guide_callback
        )
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.controller = rospy.Publisher('control/precise_command', Twist,
                queue_size=1)
        self.controller_feedback = rospy.Subscriber(
            'control/precise_command/feedback', String, self.controller_callback)
        self.stopped = False
        self.is_precise_done = False
        self.target_type = None
        self.ar_dir_detection_count = 0
        self.logo_dir_detection_count = 0
        self.tw = Twist()
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        # print("[Debug] preDocking is running")
        self.target_type = None
        self.ar_dir_detection_count = 0
        self.logo_dir_detection_count = 0
        self.stopped = False
        tw = Twist()
        tw.angular.z = -math.pi/2
        self.controller.publish(tw)
        print("[Debug] preDocking precise command send")
        self.is_precise_done = False
        while not self.is_precise_done:
            if self.stopped:
                break
            self.rate.sleep()
        print("[Debug] preDocking precise command done")

        time.sleep(2)
        self.stopped = False
        while not self.stopped:
            self.twist_pub.publish(self.tw)
            self.rate.sleep()
        return 'success'

    def logo_approching_guide_callback(self, msg):
        if self.target_type=='AR_TAG':
            return
        if msg.data>-1:
            self.logo_dir_detection_count += 0
            if self.target_type==None and self.logo_dir_detection_count > 50:
                self.target_type = 'UA_LOGO'
            angle_in_degree = 30 * (320 - msg.data) / 320.0
            if abs(angle_in_degree) < 1:
                # self.stopped = True
                pass
            else:
                self.stopped = False
                self.tw.angular.z = 6*math.radians(angle_in_degree)
                self.tw.linear.x = 0.1
        else:
            # self.stopped = True
            self.tw.linear.x = 0.1
            self.tw.angular.z = 0

    def ar_approching_guide_callback(self, msg):
        if self.target_type=='UA_LOGO':
            return
        self.ar_dir_detection_count += 0
        if self.target_type==None and self.ar_dir_detection_count > 50:
            self.target_type = 'AR_TAG'
        angle_in_degree = 30 * (320 - msg.data) / 320.0
        if abs(angle_in_degree) < 1:
            # self.stopped = True
            pass
        else:
            self.stopped = False
            self.tw.angular.z = 2*math.radians(angle_in_degree)
            self.tw.linear.x = 0.1

    def controller_callback(self, msg):
        if msg.data == 'stop':
            self.is_precise_done = True
        else:
            self.is_precise_done = False

    def scan_callback(self, msg):
        scan_data = msg.ranges
        valid_scan_data = []

        for distance in scan_data:
            if not np.isnan(distance):
                valid_scan_data.append(distance)

        if min(valid_scan_data) < 0.5 or len(valid_scan_data)<200:
            self.stopped = True
            # print "[Debug]", min(valid_scan_data)
            # print "[Debug]", len(valid_scan_data)


""" This main function is for testing only """
if __name__ == '__main__':
    rospy.init_node('predocking_state_test')
    sm = StateMachine(outcomes=['success'])
    with sm:
        StateMachine.add('PreDocking', PreDocking(), transitions={'success': 'success'})
        time.sleep(0.6)

    sm.execute()
