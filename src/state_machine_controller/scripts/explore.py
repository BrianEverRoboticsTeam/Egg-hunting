import rospy
import time
from smach import State, StateMachine
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import pi

class Explore(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.value = 0
        self.side_detector = rospy.Subscriber('detector', String,
                self.side_detector_callback)
        self.twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist,
                queue_size = 1)
        self.found_target = False
        self.tw = Twist()
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        while not self.found_target:
            self.tw.linear.x = 0.2
            self.twist_pub.publish(self.tw)
            self.rate.sleep()
        self.found_target = False
        return 'success'

    def side_detector_callback(self, msg):
        if msg.data == 'True':
            self.found_target = True
        else:
            self.found_target = False
