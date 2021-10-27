#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## This code is to publish a message to the broswer so that it can respond
#and the ROS on the jetson would know that it is connected 

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from multiprocessing import Process, Pipe
import thread

#This node is a combination of a subscriber and publisher


reset = False
running = False
#percent = 0


#this callback function gets called whenever a message is recieved
#it pushes the message to the terminal for us to confirm what happened
#and it checks the value of the message and performs actions based on it
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' I heard: %s', data.data)
    global reset, running #percent 

    
    #self reset timer 

    # if (data.data=="timer_complete"):
    #     keepCounting = False
    #     reset = True

    # elif (data.data=="turn_off_sentry"):
    #     keepCounting = False
    #     reset = True
        
    if (data.data=="connected"): #this also continues timing

        rospy.loginfo("conencted")
        
        running = False
    


    else:
        rospy.logwarn("Incorrect data received.")
        
#this function is for subscribing to messages
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #check for message on Topic

    global reset, running #percent

    #this call creates a subscriber and defines message type and which topic it publishes to
    #whenever a message is received it calls the callback function
    rospy.Subscriber('/connection_test_rx', String, callback)

    #this value is a sleep value
    rate = rospy.Rate(1) #1Hz
    

    #while ROS is not shutdown via terminal etc, if the conditions are met:
    i = 0
    t = 30
    
    while not rospy.is_shutdown():
   
        # if (reset):
        #     i = 0
        #     reset = False
        #     running=0


        #elif (i==0)
        
        #    i = i + 1


        if (i<t):
            now = rospy.get_time()  
            rospy.loginfo(now)
            rospy.loginfo(i)
            pub.publish(i)
            i = i + 1
        elif (i==t and running==False):
            i = 0 
            running = True
            pub_heartbeat_ros_remote.publish("connection_test")          
        elif (i==t and running==True):
            pub_heartbeat_state_machine.publish("turn_off_sentry")
        rate.sleep()


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    #create a unique node
    rospy.init_node('heartbeat_ros_remote', anonymous=True)
    #create a publisher object and defines which topic it subscribes to
    pub_heartbeat_ros_remote = rospy.Publisher('connection_test_tx', String, queue_size=10)

    pub_heartbeat_state_machine = rospy.Publisher('sentry_control_topic', String, queue_size=10)
    
    #start the subscribing and publishing process
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    

