#!/usr/bin/env python
import rospy
import time
from smach import State, StateMachine
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import pi
from kobuki_msgs.msg import Sound

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

        # tw = Twist()
        # tw.linear.x = 0.5
        # self.controller.publish(tw)
        # self.stopped = False
        # while not self.stopped:
        #     self.rate.sleep()

        # time.sleep(5)
        # sound = Sound()
        # sound.value = 0
        # self.sound_pub.publish(sound)
        time.sleep(2)
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
