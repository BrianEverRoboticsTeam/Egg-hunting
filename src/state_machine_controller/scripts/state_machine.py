#!/usr/bin/env python
import rospy
from smach import State, StateMachine
from explore import Explore
from docking import Docking
from undocking import UnDocking

if __name__ == '__main__':
    rospy.init_node('state_machine_controller')
    sm = StateMachine(outcomes=['success'])
    with sm:
        StateMachine.add('Explore', Explore(), transitions={'success': 'Docking'})
        StateMachine.add('Docking', Docking(), transitions={'success': 'UnDocking'})
        StateMachine.add('UnDocking', UnDocking(), transitions={'success': 'Explore'})

    sm.execute()
