#!/usr/bin/env python

import roslib #; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
#import required data types for messages
from std_msgs.msg import Bool

#this machine infinitely publishes the value data
#can be extended to work with userdata

#state class definitions:
class Publish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        #define publisher object:
        self.pub = rospy.Publisher(x,y,z)
        #where x= '/topic_name'
        #y = message data time
        #z= queue_size= int value
        self.rate = rospy.Rate(1)   #sets the frequency of publishing the mesage
        #can initialize any other variables needed over here
        self.data=1

    def execute(self, userdata):
        #can print the name of the state you're in, using rospy.loginfo but its not necessary    
        #need this if statement to publish only after subscribers are connected
        if (self.pub.get_num_connections()) >0 :
            self.pub.publish(self.data)
        self.rate.sleep()
        return 'outcome'

def main():
    rospy.init_node('smach_publisher', anonymous=True)

    #Create a SMACH state machine:
    #this outcome will exit the machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    #open the container

    with sm:
        #add states to the counter
        smach.StateMachine.add('Publish', Publish()), transitions={'outcome1':'Publish'})
        #Execute SMACH plan
        outcome = sm.execute()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass