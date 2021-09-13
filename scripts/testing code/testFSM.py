#!/usr/bin/env python

import roslib #; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool

#define a state

""" class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2', 'outcome3'])
        self.counter=0



    def execute(self, userdata):
        rospy.loginfo('Executing initial state')
        if self.counter == 3:
            return 'outcome2'
        else:
            self.counter = self.counter + 1
            return 'outcome3' """

#define lighting state
class Light(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'], input_keys=['lightControl'])

    def execute(self, userdata):
        rospy.loginfo('Inside the lighting state')
        #return 'outcome1'

        if userdata.lightControl == False:
            rospy.loginfo('Lights OFF')
            return 'outcome1'
        if userdata.lightControl == True:
            rospy.loginfo('Lights ON')
            return 'outcome1'


#based on the source code this state is needed
def monitor_cb(ud, msg):
    #ud.humanDetected2=False
    ud.humanDetected2=msg.data
    
    return False

def main():
    rospy.init_node('smach_example_state_machine', anonymous=True)

    #Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container

    with sm:
        #add states to the counter

        smach.StateMachine.add('monitor', smach_ros.MonitorState("/humanDetected", Bool, monitor_cb, output_keys=['humanDetected2']), transitions={'invalid':'Light', 'valid':'monitor', 'preempted':'monitor'}, remapping={'humanDetected2':'humanDetected'})


        #smach.StateMachine.add('InitState', InitState(), transitions={'outcome1':'Light', 'outcome2':'outcome4' , 'outcome3':'InitState'})
        smach.StateMachine.add('Light', Light(), transitions={'outcome1':'monitor'}, remapping={'lightControl':'humanDetected'})

    #Execute SMACH plan

    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




