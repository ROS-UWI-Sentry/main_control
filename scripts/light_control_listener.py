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


                        ###note###
# ____________________________________________________
#|This is a simple node that listens to commands from |
#|the state machine and changes the output of the pin.|
#|See accompanying flowchart for overall operation    |
#|____________________________________________________|




import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Bool


####### Pin Definitions ######
output_pin = 32  # BOARD pin 32


#this function is called whenever a new message comes in on the subscribed topic
def callback(data):
    global curr_value

    rospy.loginfo(rospy.get_caller_id() + 'Node heard: %s', data.data)
    if data.data == True:
        #set the output pin true if the message received is true
        curr_value= GPIO.HIGH
        GPIO.output(output_pin, curr_value)
    elif data.data == False:
        #set the output pin true if the message received is true        
        curr_value= GPIO.LOW
        GPIO.output(output_pin, curr_value)

def listener():
    global curr_value

    # Pin Setup:
    GPIO.setmode(GPIO.BOARD)  # using the BOARD pin-numbering scheme
    # set pin as an output pin with optional initial state of LOW:
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)

    print("Starting demo now! Press CTRL+C to exit")
    curr_value = GPIO.LOW




    #not ananymous node so only one node can control pins at a time.
    rospy.init_node('light_control_listener')

    rospy.Subscriber('light_control', Bool, callback)






    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    try:
        listener()  
    except KeyboardInterrupt:
        GPIO.cleanup()
