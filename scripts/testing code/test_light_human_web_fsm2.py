#!/usr/bin/env python

import roslib #; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from std_msgs.msg import String
import roslaunch

#this is testing the ability to turn on the pin from the webbrowser as well as human detection

last_state = 0
pub_light = rospy.Publisher('light_control', Bool, queue_size=500)
has_human_detection_been_started=False

# class Pre_data_processing(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['turn_off', 'monitor', 'monitor_for_human'], input_keys=['received_data'])
#         has_human_detection_been_started=False

#     def execute(self, userdata):
#         rospy.loginfo('Inside the Control data processing state')

#         if userdata.received_data== "human_detected_true":
#             return 'turn_off'
#         elif userdata.received_data== "human_detected_false":
#             has_human_detection_been_started = True
#             return 'monitor_for_human'
#         elif userdata.received_data== "stop_sanitization" and has_human_detection_been_started = True:
#             return 'monitor_for_human'
#         elif userdata.received_data== "turn_off_sentry":
#             return 'turn_off'
#         elif userdata.received_data== "pause_sanitization" and has_human_detection_been_started = True:
#             return 'monitor_for_human'
#         elif userdata.received_data== "start_sanitization" and has_human_detection_been_started = True:
#             return 'monitor'
#         else :
#             rospy.loginfo('no idea what data was sent')
#             return 'turn_off'

class Control_rosbridge(smach.State):
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



class Control_human_detection(smach.State):
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
            #self.launch.shutdown()

            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi/catkin_ws/src/human_detection/launch/end_detector.launch"])
            self.launch.start()
            rospy.loginfo("detector ending")
        
            rospy.sleep(5)
            self.launch.shutdown()
            return 'turn_off'

class Turn_off(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn_off'])

    def execute(self, userdata):
        return 'turn_off'


# def monitor_cb_human_detection_starting(ud, msg):

#     ud.data_received=msg.data
#     return False


    

def monitor_cb_control(ud, msg):
    #ud.control_data=msg.data
    global last_state, pub_light, has_human_detection_been_started

    if msg.data=="human_detected_true":
        pub_light.publish(False)
        rospy.loginfo('OFF')
        return False
    elif msg.data== "human_detected_false":
        has_human_detection_been_started =True
        return True
    elif msg.data== "stop_sanitization" and has_human_detection_been_started == True:
        if last_state != False:
            pub_light.publish(False)
            last_state=False
            rospy.loginfo('OFF')
            return True
        else :
            return True
    elif msg.data== "stop_sanitization" and has_human_detection_been_started == False:
        return True
    elif msg.data== "turn_off_sentry":
        pub_light.publish(False)
        rospy.loginfo('OFF')
        return False
    elif msg.data=="pause_sanitization" and has_human_detection_been_started == True:
        if last_state!=False:
            pub_light.publish(False)
            last_state=False
            rospy.loginfo('OFF')
            return True
        else:
            return True
    elif msg.data== "pause_sanitization" and has_human_detection_been_started == False:
        return True
    elif msg.data== "start_sanitization" and has_human_detection_been_started == True:
        if last_state!=True:
            pub_light.publish(True)
            last_state=True
            rospy.loginfo('ON')
            return True
        else:
            return True
    elif msg.data== "start_sanitization" and has_human_detection_been_started == False:
        return True            
    else:
        rospy.loginfo('Incorrect data')
        return False


def main():
    rospy.init_node('smach_example_state_machine')

    #Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exitSM'])

    # Open the container

    with sm:
        #add states to the counter

        smach.StateMachine.add('Control_rosbridge', Control_rosbridge(),\
         transitions={'next_state':'Control_human_detection','turn_off':'Turn_off'})

        smach.StateMachine.add('Control_human_detection', Control_human_detection(),\
         transitions={'next_state':'monitor_control', 'turn_off':'Control_rosbridge'})

        # smach.StateMachine.add('monitor_for_human_detection_starting',\
        #  smach_ros.MonitorState("/sentry_control_topic", String, monitor_cb_human_detection_starting, \
        #  output_keys=['data_received']), \
        #  transitions={'invalid':'Pre_data_processing','valid':'monitor_for_human_detection_starting',\
        #  'preempted':'monitor_for_human_detection_starting'})

        smach.StateMachine.add('monitor_control', smach_ros.MonitorState("/sentry_control_topic", \
        String, monitor_cb_control,\
         output_keys=['control_data']),\
          transitions={'invalid':'Control_human_detection', 'valid':'monitor_control',\
           'preempted':'monitor_control'}, remapping={'control_data':'control_data'})


        
        # smach.StateMachine.add('Pre_data_processing', Pre_data_processing(), transitions=\
        # {'turn_off':'Control_human_detection','monitor':'monitor_control',\
        #  'monitor_for_human':'monitor_for_human_detection_starting'},\
        #   remapping={'received_data':'data_received'})
        
        smach.StateMachine.add('Turn_off', Turn_off(),transitions={'turn_off':'exitSM'})

    #Execute SMACH plan

    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




