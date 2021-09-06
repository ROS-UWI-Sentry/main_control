#!/usr/bin/env python

import roslib #; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
#import required data types for messages
from std_msgs.msg import Bool

#this machine templates how to subscribe/monitor a ROS topic

#state class definitions:

#define a state that performs actions when a message is received: 
class Subscribe(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'], input_keys=['inputData'])

    def execute(self, userdata):
        #can print the name of the state you're in, using rospy.loginfo but its not necessary    
        
        #example code checking the data received and exiting the state based on it:
        if userdata.inputData == False:
            rospy.loginfo('Lights OFF')
            return 'outcome1'
        if userdata.inputData == True:
            rospy.loginfo('Lights ON')
            return 'outcome1'


#based on the source code this state is needed, ud is the userdata, msg is the message received
def monitor_cb(ud, msg):
    
    ud.dataReceived=msg.data
    
    return False

def main():
    rospy.init_node('smach_example_state_machine', anonymous=True)

    #Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container

    with sm:
        #add states to the counter
        #change "/humanDetected" to topic name and Bool to required data type
        smach.StateMachine.add('monitor', smach_ros.MonitorState("/humanDetected", Bool, monitor_cb, output_keys=['dataReceived']), transitions={'invalid':'Subscribe', 'valid':'monitor', 'preempted':'monitor'}, remapping={'dataReceived':'messageData'})

        smach.StateMachine.add('Subscribe', Subscribe(), transitions={'outcome1':'monitor'}, remapping={'inputData':'messageData'})

        #there are names sounding similar, such as messageData, inputData, messageData, 
        #these are just to connect inputs to outputs to send data where needed

    #Execute SMACH plan

    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




