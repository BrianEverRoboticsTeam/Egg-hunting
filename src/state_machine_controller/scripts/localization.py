import rospy
import time
from smach import State, StateMachine
from geometry_msgs.msg import Twist
from math import pi

def getTimeSafe():
    while True:
        # rospy may returns zero, so we loop until get a non-zero value.
        time = rospy.Time.now()
        if time != rospy.Time(0):
            return time

class Localization(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist,
                queue_size = 1)
        self.rate = rospy.Rate(10)

        rospy.wait_for_service('global_localization')
        self.global_localization = rospy.ServiceProxy('global_localization',
                Empty)

    def execute(self, userdata):
        # reset AMCL localizer
        self.global_localization()
        time.sleep(0.5) # wait for system to process

        # self turning
        duration = 12
        speed = 0.8
        tw = Twist()
        timelimit = getTimeSafe() + rospy.Duration(duration)
        while getTimeSafe() < timelimit:
            tw.angular.z = speed
            cmd_vel_pub.publish(tw)

        return 'success'
