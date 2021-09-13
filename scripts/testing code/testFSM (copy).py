#!/usr/bin/env python

import roslib #; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool

#define a state

class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2', 'outcome3'])
        self.counter=0



    def execute(self, userdata):
        rospy.loginfo('Executing initial state')
        if self.counter == 3:
            return 'outcome2'
        else:
            self.counter = self.counter + 1
            return 'outcome3'

#define lighting state
class Light(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state lighting')
        return 'outcome2'


def main():
    rospy.init_node('smach_example_state_machine', anonymous=True)

    #Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container

    with sm:
        #add states to the counter
        smach.StateMachine.add('InitState', InitState(), transitions={'outcome1':'Light', 'outcome2':'outcome4' , 'outcome3':'InitState'})
        smach.StateMachine.add('Light', Light(), transitions={'outcome1':'InitState', 'outcome2':'Light'})

    #Execute SMACH plan

    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




