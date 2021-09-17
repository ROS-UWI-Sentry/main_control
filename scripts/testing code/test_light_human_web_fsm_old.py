#!/usr/bin/env python

import roslib #; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from std_msgs.msg import String
import roslaunch

#this is testing the ability to turn on the pin from the webbrowser as well as human detection



class Control_data_processing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn_off', 'monitor'], input_keys=['control_data'])
        #initialize publisher
        self.pub_light=rospy.Publisher('light_control', Bool, queue_size=500)
        self.last_state = False

    def execute(self, userdata):
        rospy.loginfo('Inside the Control data processing state')

        if userdata.control_data== "human_detected_true":
            #publish turn off lights
            self.pub_light.publish(False)
            rospy.loginfo('OFF')
            return 'turn_off'
        elif userdata.control_data== "human_detected_false":
            #publish turn on lights
            #self.pub_light.publish(True)
            #rospy.loginfo('ON')
            return 'monitor'
        elif userdata.control_data== "stop_sanitization":
            #publish turn off lights
            if self.last_state!=False:
                self.pub_light.publish(False)
                self.last_state=False
                rospy.loginfo('OFF')
                return 'monitor'
            else: 
                return 'monitor'
        elif userdata.control_data== "turn_off_sentry":
            #turn off lights
            self.pub_light.publish(False)
            rospy.loginfo('OFF')
            return 'turn_off'
        elif userdata.control_data== "pause_sanitization":
            #turn off lights
            if self.last_state!=False:
                self.pub_light.publish(False)
                self.last_state=False
                rospy.loginfo('OFF')
                return 'monitor'
            else:
                return 'monitor'
        elif userdata.control_data== "start_sanitization":
            #turn on lights
            if self.last_state!=True:
                self.pub_light.publish(True)
                self.last_state=True
                rospy.loginfo('ON')
                return 'monitor'
            else:
                return 'monitor'
        else :
            rospy.loginfo('no idea what data was sent')
            return 'turn_off'

class Start_rosbridge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next_state', 'turn_off'])

        self.init_var = 0

    def execute(self, userdata):
        if self.init_var==0:
            self.package = 'rosbridge_server'
            self.executable = 'rosbridge_websocket'
            self.node = roslaunch.core.Node(self.package, self.executable)
            self.launch = roslaunch.scriptapi.ROSLaunch()
            self.launch.start()
            self.process=self.launch.launch(self.node)
            rospy.loginfo(self.process.is_alive())
            self.init_var = self.init_var + 1
            return 'next_state'
        elif self.init_var >= 1:
            self.process.stop()
            return 'turn_off'



class Start_human_detection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next_state', 'turn_off'])
        #launch yolov5 here from launching code
        self.init_var = 0

    def execute(self, userdata):
        #exit yolov5 from here from launching code
        if self.init_var==0:
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi/catkin_ws/src/human_detection/launch/launch_detector2.launch"])
            self.launch.start()
            rospy.loginfo("detector started")
            self.init_var = self.init_var + 1
            return 'next_state'
        elif self.init_var >= 1:
            self.launch.shutdown()
            return 'turn_off'

class Turn_off(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn_off'])

    def execute(self, userdata):
        return 'turn_off'


def monitor_cb_human_detection_starting(ud, msg):
    if msg.data=="human_detected_false":
        return False
    elif msg.data=="human_detected_true":
        return True

def monitor_cb_control(ud, msg):
    ud.control_data=msg.data
    return False


def main():
    rospy.init_node('smach_example_state_machine', anonymous=True)

    #Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exitSM'])

    # Open the container

    with sm:
        #add states to the counter

        smach.StateMachine.add('Start_rosbridge', Start_rosbridge(), transitions={'next_state':'Start_human_detection','turn_off':'Turn_off'})

        smach.StateMachine.add('Start_human_detection', Start_human_detection(), transitions={'next_state':'monitor_for_human_detection_starting', 'turn_off':'Start_rosbridge'})

        smach.StateMachine.add('monitor_for_human_detection_starting', smach_ros.MonitorState("/sentry_control_topic", String, monitor_cb_human_detection_starting), transitions={'invalid':'monitor_control', 'valid':'Start_human_detection', 'preempted':'monitor_for_human_detection_starting'})

        smach.StateMachine.add('monitor_control', smach_ros.MonitorState("/sentry_control_topic", String, monitor_cb_control, output_keys=['control_data']), transitions={'invalid':'Control_data_processing', 'valid':'monitor_control', 'preempted':'monitor_control'}, remapping={'control_data':'control_data'})


        
        smach.StateMachine.add('Control_data_processing', Control_data_processing(), transitions={'turn_off':'Start_human_detection','monitor':'monitor_control'}, remapping={'control_data':'control_data'})
        smach.StateMachine.add('Turn_off', Turn_off(),transitions={'turn_off':'exitSM'})

    #Execute SMACH plan

    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




