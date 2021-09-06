#!/usr/bin/env python

import roslib #; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool

#define a state

#define lighting state
class Publish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        self.pub = rospy.Publisher('/humanDetected', Bool, queue_size=10)
        self.rate = rospy.Rate(1)
        self.data=False

    def execute(self, userdata):
        rospy.loginfo('Executing state lighting')
        #return 'outcome1'

        self.data = not(self.data)
       
        
        if (self.pub.get_num_connections()) > 0 :
            self.pub.publish(self.data)
            rospy.loginfo(self.data)
        self.rate.sleep()
        return 'outcome1'

# class Empty(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['outcome2'])
#     def execute(self, userdata):
#         rospy.loginfo('in empty state')
#         return 'outcome2'


def main():
    rospy.init_node('smach_publisher', anonymous=True)

    #Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container

    with sm:
        #add states to the counter

        smach.StateMachine.add('Publish', Publish(), transitions={'outcome1':'Publish'})
        #smach.StateMachine.add('Empty', Empty(), transitions={'outcome2':'Publish'})

    #Execute SMACH plan

    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




