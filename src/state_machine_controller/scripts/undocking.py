import rospy
import time
from smach import State, StateMachine
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import pi

class UnDocking(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.controller = rospy.Publisher('control/precise_command', Twist,
                queue_size=1)
        self.controller_feedback = rospy.Subscriber(
            'control/precise_command/feedback', String, self.controller_callback)
        self.stopped = False
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        self.stopped = False
        tw = Twist()
        tw.linear.x = -0.23
        self.controller.publish(tw)
        while not self.stopped:
            self.rate.sleep()

        print 'turn'

        tw = Twist()
        tw.angular.z = 3*pi/4
        self.controller.publish(tw)
        self.stopped = False
        while not self.stopped:
            self.rate.sleep()

        # print 'forward'
        #
        # tw = Twist()
        # tw.linear.x = 0.8
        # self.controller.publish(tw)
        # self.stopped = False
        # while not self.stopped:
        #     self.rate.sleep()

        return 'success'

    def controller_callback(self, msg):
        if msg.data == 'stop':
            self.stopped = True
        else:
            self.stopped = False
