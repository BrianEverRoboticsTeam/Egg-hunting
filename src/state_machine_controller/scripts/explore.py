import rospy
import time
from smach import State, StateMachine
from std_msgs.msg import String

# 45 degree turns smaller area:
waypoints = [
    [(-2.12, 2.88, 0.0), (0.0, 0.0, -0.93, 0.36)],
    [(-2.45, -1.25, 0.0), (0.0, 0.0, -0.36, 0.93)],
    [(8.01, -2.03, 0.0), (0.0, 0.0, 0.36, 0.93)],
    [(8.20, 2.26, 0.0), (0.0, 0.0, 0.92, 0.39)]
]

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    return goal_pose


class Explore(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.side_detector = rospy.Subscriber('detector', String,
                self.side_detector_callback)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.i = 0

    def execute(self, userdata):
        while not rospy.is_shutdown():
            while self.i < len(waypoints):
                pose = waypoints[self.i]
                self.client.send_goal(goal_pose(pose))
                self.client.wait_for_result()
                self.i += 1
            self.i = 0

    def side_detector_callback(self, msg):
        if msg.data == 'True':
            return 'success'
