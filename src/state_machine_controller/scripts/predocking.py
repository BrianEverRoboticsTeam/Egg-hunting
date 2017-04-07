#!/usr/bin/env python
import rospy, time, math
from smach import State, StateMachine
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from kobuki_msgs.msg import Sound
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class SoundController():
    def __init__(self):
        self.sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound,
            queue_size = 1)

    def send_sound(self, type):
        sound = Sound()

        soundhandle = SoundClient()
        rospy.sleep(0.5)
        import os
        root = os.path.dirname(os.path.abspath(__file__))

        #
        # # sound_src = "/home/jimmy/Documents/CMPUT412/Egg-hunting/smb2_grow.wav"
        # sound_src = "/home/jimmy/Documents/CMPUT412/Egg-hunting/super-mario-bros.wav"
        # soundhandle.playWave(sound_src)
        # rospy.sleep(1)

        if type=="UA_LOGO":
            # sound_src = "/home/jimmy/Documents/CMPUT412/Egg-hunting/smb2_grow.wav"
            sound_src = root + "/smb2_grow.wav"
            soundhandle.playWave(sound_src)

            sound.value = 1
            self.sound_pub.publish(sound)
            time.sleep(0.3)
            sound.value = 1
            self.sound_pub.publish(sound)
            time.sleep(0.3)
            sound.value = 0
            self.sound_pub.publish(sound)
            time.sleep(1)

        elif type=="AR_TAG":
            # sound_src = "/home/jimmy/Documents/CMPUT412/Egg-hunting/smb2_1up.wav"
            sound_src = root + "/smb2_1up.wav"
            soundhandle.playWave(sound_src)

            sound.value = 0
            self.sound_pub.publish(sound)
            time.sleep(0.3)
            sound.value = 0
            self.sound_pub.publish(sound)
            time.sleep(0.3)
            sound.value = 1
            self.sound_pub.publish(sound)
            time.sleep(1)


class PreDocking(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed'])
        self.twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist,
            queue_size = 1)
        self.logo_dir_sub = rospy.Subscriber(
            'detector_x', Float32, self.logo_approching_guide_callback
        )
        self.ar_dir_sub = rospy.Subscriber(
            'ar_detector_x', Float32, self.ar_approching_guide_callback
        )
        self.logo_pose_guide_sub = rospy.Subscriber(
            'logo_pose', numpy_msg(Floats), self.logo_pose_guide_callback
        )
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.controller = rospy.Publisher('control/precise_command', Twist,
                queue_size=1)
        self.controller_feedback = rospy.Subscriber(
            'control/precise_command/feedback', String, self.controller_callback)
        self.sound_control = SoundController()
        self.stopped = False
        self.is_precise_done = False
        self.target_type = None
        self.ar_dir_detection_count = 0
        self.logo_dir_detection_count = 0
        self.min_distance_ahead = 10
        self.tw = Twist()
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        # print("[Debug] preDocking is running")
        self.target_type = None
        self.ar_dir_detection_count = 0
        self.logo_dir_detection_count = 0
        self.min_distance_ahead = 0.5
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

        time.sleep(1)
        self.stopped = False
        while not self.stopped:
            if self.min_distance_ahead > 3:
                self.tw.linear.x = 0.7
                self.tw.angular.z = 0
            elif self.min_distance_ahead > 1.4:
                self.tw.linear.x = self.min_distance_ahead/7
                self.tw.angular.z = self.tw.angular.z/2
            else:
                self.tw.linear.x = 0.08
            self.twist_pub.publish(self.tw)
            self.rate.sleep()
        print(self.target_type)
        time.sleep(0.5)
        if self.target_type!=None:
            self.sound_control.send_sound(self.target_type)
            return 'success'
        else:
            return 'failed'

    def logo_approching_guide_callback(self, msg):
        if self.target_type=='AR_TAG':
            return
        if msg.data>-1:
            self.logo_dir_detection_count += 1
            if self.target_type==None and self.logo_dir_detection_count > 20:
                self.target_type = 'UA_LOGO'
            angle_in_degree = 30 * (320 - msg.data) / 320.0
            if abs(angle_in_degree) < 1:
                # self.stopped = True
                self.tw.angular.z = 0.0
                pass
            else:
                self.stopped = False
                self.tw.angular.z = 2*math.radians(angle_in_degree)
                # self.tw.linear.x = 0.1
        else:
            # self.stopped = True
            # self.tw.linear.x = 0.1
            self.tw.angular.z = 0

    def ar_approching_guide_callback(self, msg):
        if self.target_type=='UA_LOGO':
            return
        self.ar_dir_detection_count += 1
        if self.target_type==None and self.ar_dir_detection_count > 5:
            self.target_type = 'AR_TAG'
        angle_in_degree = 30 * (320 - msg.data) / 320.0
        if abs(angle_in_degree) < 1:
            # self.stopped = True
            self.tw.angular.z = 0.0
            pass
        else:
            self.stopped = False
            self.tw.angular.z = 2*math.radians(angle_in_degree)
            # self.tw.linear.x = 0.1

    def logo_pose_guide_callback(self, msg):
        self.target_type = 'UA_LOGO'
        # self.stopped = True

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

        if  len(valid_scan_data)<190 or min(valid_scan_data) < 0.5:
            self.min_distance_ahead = 0.5
            self.stopped = True
            # print "[Debug]", min(valid_scan_data)
            # print "[Debug]", len(valid_scan_data)
        else:
            self.min_distance_ahead = min(valid_scan_data)



""" This main function is for testing only """
from simulated_explore import SimulatedExplore
if __name__ == '__main__':
    rospy.init_node('predocking_state_test')

    """ state test """
    # sm = StateMachine(outcomes=['success', 'failed'])
    # with sm:
    #     StateMachine.add('SimulatedExplore', SimulatedExplore(), transitions={'success':'PreDocking'})
    #     StateMachine.add('PreDocking', PreDocking(), transitions={'success': 'success', 'failed':'failed'})
    #     time.sleep(0.6)
    #
    # sm.execute()

    """ sound test """
    sound_control = SoundController()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        raw_input()
        # sound_control.send_sound("UA_LOGO")
        # sound_control.send_sound("AR_TAG")
        sound_control.send_sound(None)
        rate.sleep()
