import rospy
import time
from smach import State, StateMachine
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SimulatedExplore(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.side_detector = rospy.Subscriber('detector', String,
                self.side_detector_callback)
        self.twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist,
                queue_size = 1)
        self.stopped = False
        self.tw = Twist()
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        self.stopped = False
        while not self.stopped:
            self.tw.linear.x = 0.6
            self.twist_pub.publish(self.tw)
            self.rate.sleep()
        return 'success'

    def side_detector_callback(self, msg):
        if msg.data=="True":
            self.stopped = True
        else:
            self.stopped = False
