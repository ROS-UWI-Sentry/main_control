#!/usr/bin/env python

import roslib #; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from smach import CBState

#define a state

#define lighting state
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['done'])
def publish_cb1( user_data):
    data=False
    rospy.loginfo('Inside the Light OFF State')
    pub = rospy.Publisher('humanDetected', Bool, queue_size=1)
    pub.publish(data)
    rospy.sleep(1)
    
    return 'done'

@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['done'])
def publish_cb2( user_data):
    data=True
    rospy.loginfo('Inside the Light ON State')
    pub = rospy.Publisher('humanDetected', Bool, queue_size=1)
    pub.publish(data)
    rospy.sleep(1)
    
    return 'done'




def main():
    rospy.init_node('smach_publisher', anonymous=True)

    #Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container

    with sm:
        #add states to the counter

        smach.StateMachine.add('PublishFalse', CBState(publish_cb1), {'done':'PublishTrue'})
        smach.StateMachine.add('PublishTrue', CBState(publish_cb2), {'done':'outcome4'})
    #Execute SMACH plan

    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
