import rospy
import time
from smach import State, StateMachine

class Foo(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'],
                input_keys=['docking_position'])

    def execute(self, userdata):
        try:
            print 'Foo: userdata.docking_position =', userdata.docking_position
        except KeyError:
            print 'Foo: data not avilable'
        time.sleep(1)
        return 'success'

class Bar(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'],
                output_keys=['docking_position'])

    def execute(self, userdata):
        userdata.docking_position = '1, 4'
        time.sleep(1)
        return 'success'

if __name__ == '__main__':
    rospy.init_node('state_machine_controller')
    sm = StateMachine(outcomes=['success'])
    with sm:
        StateMachine.add('Foo', Foo(),
                transitions={'success': 'Bar'})

        StateMachine.add('Bar', Bar(),
                transitions={'success': 'Foo'})

    time.sleep(1)
    sm.execute()
