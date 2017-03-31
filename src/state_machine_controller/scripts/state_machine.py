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


class Docking(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.value = 0
        self.controller = rospy.Publisher('control/precise_command', Twist,
                queue_size=1)
        self.controller_feedback = rospy.Subscriber(
            'control/precise_command/feedback', String, self.controller_callback)
        self.stopped = False
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        self.stopped = False
        tw = Twist()
        tw.angular.z = -pi/2
        self.controller.publish(tw)
        while not self.stopped:
            self.rate.sleep()


        tw = Twist()
        tw.linear.x = 0.5
        self.controller.publish(tw)
        self.stopped = False
        while not self.stopped:
            self.rate.sleep()
        return 'success'

    def controller_callback(self, msg):
        if msg.data == 'stop':
            self.stopped = True
        else:
            self.stopped = False


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
        tw.linear.x = -0.5
        self.controller.publish(tw)
        while not self.stopped:
            self.rate.sleep()

        print 'turn'

        tw = Twist()
        tw.angular.z = pi/2
        self.controller.publish(tw)
        self.stopped = False
        while not self.stopped:
            self.rate.sleep()

        print 'forward'

        tw = Twist()
        tw.linear.x = 0.8
        self.controller.publish(tw)
        self.stopped = False
        while not self.stopped:
            self.rate.sleep()

        return 'success'

    def controller_callback(self, msg):
        if msg.data == 'stop':
            self.stopped = True
        else:
            self.stopped = False

if __name__ == '__main__':
    rospy.init_node('state_machine_controller')
    sm = StateMachine(outcomes=['success'])
    with sm:
        StateMachine.add('Explore', Explore(), transitions={'success': 'Docking'})
        StateMachine.add('Docking', Docking(), transitions={'success': 'UnDocking'})
        StateMachine.add('UnDocking', UnDocking(), transitions={'success': 'Explore'})

    sm.execute()
