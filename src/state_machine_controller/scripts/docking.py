#!/usr/bin/env python
import rospy
import time
from smach import State, StateMachine
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import pi
from kobuki_msgs.msg import Sound
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class Docking(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.controller = rospy.Publisher('control/precise_command', Twist,
                queue_size=1)
        self.controller_feedback = rospy.Subscriber(
            'control/precise_command/feedback', String, self.controller_callback)
        self.sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size = 1)
        self.stopped = False
        self.rate = rospy.Rate(10)

    def execute(self, userdata):

        soundhandle = SoundClient()
        rospy.sleep(0.5)
        sound_src = "/home/jimmy/Documents/CMPUT412/Egg-hunting/smb2_grow.wav"
        # sound_src = "/home/jimmy/Documents/CMPUT412/Egg-hunting/super-mario-bros.wav"
        soundhandle.playWave(sound_src)
        rospy.sleep(0.5)
        return 'success'

    def controller_callback(self, msg):
        if msg.data == 'stop':
            self.stopped = True
        else:
            self.stopped = False


""" This main function is for testing only """
if __name__ == '__main__':
    rospy.init_node('docking_state_test')
    sm = StateMachine(outcomes=['success', 'finished'])
    with sm:
        StateMachine.add('Docking', Docking(), transitions={'success': 'finished'})
        time.sleep(0.6)

    sm.execute()
