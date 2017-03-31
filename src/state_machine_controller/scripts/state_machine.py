#!/usr/bin/env python
import rospy
import time
from smach import State, StateMachine
from localization import Localization
from explore import Explore
from predocking import PreDocking
from docking import Docking
from undocking import UnDocking

if __name__ == '__main__':
    rospy.init_node('state_machine_controller')
    sm = StateMachine(outcomes=['success'])
    with sm:
        StateMachine.add('Localization', Localization(),
                transitions={'success': 'Explore'})

        StateMachine.add('Explore', Explore(),
                transitions={'success': 'PreDocking', 'lost': 'Localization'})

        StateMachine.add('PreDocking', PreDocking(),
                transitions={'success', 'Docking'})

        StateMachine.add('Docking', Docking(),
                transitions={'success': 'UnDocking'})

        StateMachine.add('UnDocking', UnDocking(),
                transitions={'success': 'Explore'})

    time.sleep(1)
    sm.execute()
