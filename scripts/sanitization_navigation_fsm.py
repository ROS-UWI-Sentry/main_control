#!/usr/bin/env python

import roslib #; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from std_msgs.msg import String
import roslaunch

#this is testing the ability to turn on the pin from the webbrowser as well as human detection

last_state = False
pub_light = rospy.Publisher('light_control', Bool, queue_size=500)
pub_nav_control = rospy.Publisher('nav_control', String, queue_size=500)
pub_timer_control = rospy.Publisher('timer_control_topic', String, queue_size=500)
pub_status_browser = rospy.Publisher('status', String, queue_size=500, latch=True)

has_human_detection_been_started=False

#state to initialize ROSBRIDGE
class setup_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next_state', 'error', 'turn_off'])
        #setup publisher
        global pub_timer_control

        #to reset the timer to zero
        pub_timer_control.publish("stop_timer")       

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

            #check amount of cameras attatched
            #use the previous bash script 
            
            #publish this to the browser under camera topic maybe

            #if no webcams found
                #return 'error'            
            return 'next_state'
        elif self.init_var >= 1:

            self.process.stop()

            
            return 'turn_off'


def monitor_cb_start_pressed(ud, msg):

    #ud.msg_data=msg.data
    #return False
    
    if msg.data=="start_sanitization":
        ud.msg_data="start_human_detection"
        return False
    elif msg.data=="turn_off_sentry":
        ud.msg_data=msg.data
        return False
    else:    
        ud.msg_data=msg.data
        return False


#Starts up/shuts down human detection and checks for other cases 
class Control_human_detection(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=['userdata_input'], output_keys=['userdata_output'], outcomes=['monitor_for_human_detection_started', 'Control_navigation', 'monitor_control', 'turn_off', 'error'])


    def execute(self, userdata):
        global pub_timer_control, pub_light, pub_status_browser
        
   
        if userdata.userdata_input == "start_human_detection":
            pub_status_browser.publish("Starting up Human Detection")
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi/catkin_ws/src/human_detection/launch/launch_detector.launch"])
            self.launch.start()
            rospy.loginfo("detector started")
            return 'monitor_for_human_detection_started'


        elif userdata.userdata_input == "turn_off_human_detection":
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi/catkin_ws/src/human_detection/launch/end_detector.launch"])
            self.launch.start()
            rospy.loginfo("detector ending")
        
            rospy.sleep(5)
            self.launch.shutdown()
            userdata.userdata_output="start_navigation"
            return 'Control_navigation'

        elif userdata.userdata_input == "turn_off_sentry":
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi/catkin_ws/src/human_detection/launch/end_detector.launch"])
            self.launch.start()
            rospy.loginfo("detector ending")
        
            rospy.sleep(5)
            self.launch.shutdown()
            return 'turn_off'   

        elif userdata.userdata_input == "human_detected_true":
            pub_light.publish(False)

            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi/catkin_ws/src/human_detection/launch/end_detector.launch"])
            self.launch.start()
            rospy.loginfo("detector ending")
        
            rospy.sleep(5)
            self.launch.shutdown()
            return 'turn_off'  


        elif userdata.userdata_input == "human_detected_false":
            #here the light it turnt on because its after human detection
            #has started and also the timer must begin
            pub_light.publish(True)
            pub_timer_control.publish("start_timer")
            return 'monitor_control'
        
        else:
            rospy.logwarn("Incorrect data received, error occured!")
            return 'error'



def monitor_cb_human_detection_started(ud, msg):
    #this state waits until you get a msg from the human detection module
    #so you know it is running before turning on the lights

    ud.msg_data=msg.data
    return False

    # if msg.data=="human_detected_true":
    #     ud.msg_data="turn_off_sentry"
    #     pub_light.publish(False)
    #     return False
    # elif msg.data=="human_detected_false":
    #     ud.msg_data=msg.data
    #     return False
    # else:    
    #     rospy.logwarn("data out of scope")
    #     return True

    
#this function is called whenever the /sentry_control_topic receives a message
#it then checks what the message is and performs an action
def monitor_cb_control(ud, msg):
    
    global last_state, pub_light, pub_timer_control



    if msg.data=="human_detected_true":
        pub_light.publish(False)
        rospy.loginfo('Found a human, shutting down')
        ud.msg_data="turn_off_sentry"
        pub_timer_control.publish("stop_timer")  
        return False
    elif msg.data== "human_detected_false": 
        #has_human_detection_been_started =True
        return True
    elif msg.data== "stop_sanitization":
        pub_timer_control.publish("stop_timer")
        if last_state != False:
            pub_light.publish(False)
            last_state=False
            rospy.loginfo('OFF')
            ud.msg_data="turn_off_sentry"
            return False
        else:
            ud.msg_data="turn_off_sentry"
            return False

    elif msg.data== "timer_complete":
        pub_timer_control.publish("stop_timer")
        if last_state != False:
            pub_light.publish(False)
            last_state=False
            rospy.loginfo('OFF')
            ud.msg_data="turn_off_human_detection"
            
            return False
        else:
            ud.msg_data="turn_off_human_detection"
            
            return False   
  
    elif msg.data== "turn_off_sentry":
        pub_light.publish(False)
        rospy.loginfo('OFF')
        ud.msg_data="turn_off_sentry"
        pub_timer_control.publish("stop_timer")
        return False

    elif msg.data=="pause_sanitization":
        pub_timer_control.publish("pause_timer")
        if last_state!=False:
            pub_light.publish(False)
            last_state=False
            rospy.loginfo('OFF')
            return True
        else:
            return True

    elif msg.data== "start_sanitization":
        pub_timer_control.publish("start_timer")
        if last_state!=True:
            pub_light.publish(True)
            last_state=True
            rospy.loginfo('ON')
            return True
        else:
            return True         
    else:
        rospy.loginfo('Incorrect data')
        rospy.loginfo(msg.data)
        pub_timer_control.publish("stop_timer")
        ud.msg_data=msg.data
        return False


class Control_navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys= ['userdata_input'],\
         output_keys = ['userdata_output'], \
         outcomes=['monitor_navigation','Control_human_detection', 'turn_off', 'error'])
        self.init_var=0

    def execute(self, userdata):
        global pub_status_browser

        #exectute commands to start the navigation module using code similar to setup_state
        
        pub_status_browser.publish("Navigating the room")

        #launch the launch file
        if userdata.userdata_input=="start_navigation":
            #launch the launch file

            return 'monitor_navigation'

        elif userdata.userdata_input=="goal_reached":
            #shut down the node that was launched
            userdata.userdata_output="start_human_detection"
            return 'Control_human_detection'
       
        elif userdata.userdata_input=="turn_off_sentry":
            #shut down the node that was launched
            return 'turn_off'

        elif userdata.userdata_input=="all_goals_reached":
            #shut down the node
            pub_status_browser.publish("Sanitization Complete")
            return 'turn_off'
        
        else:
            rospy.logwarn("Incorrect data received, error occured!")
            #shutdown node
            return 'error'


def monitor_cb_navigation(ud, msg):
    
    ud.msg_data=msg.data
    return False

    # if msg.data=="goal_reached":
    #     ud.msg_data=msg.data
    #     return False
    # elif msg.data=="turn_off_sentry":
    #     ud.msg_data=msg.data
    #     return False
    # elif msg.data=="all_goals_reached":
    #     ud.msg_data=msg.data
    #     return False
    # else:    
    #     rospy.logwarn("data out of scope")
    #     return True


class Turn_off(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn_off'])

    def execute(self, userdata):
        return 'turn_off'

class Error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error'])

    def execute(self, userdata):
        return 'error'



def main():
    rospy.init_node('smach_example_state_machine')

    #Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exitSM'])

    # Open the container

    with sm:
        #add states to the container
        #this is the standard way from the tutorials

        smach.StateMachine.add('setup_state', setup_state(),\
         transitions={\
         'next_state':'monitor_for_start',\
         'error':'Error',\
         'turn_off':'Turn_off'})

        smach.StateMachine.add('monitor_for_start', smach_ros.MonitorState("/sentry_control_topic", \
        String, monitor_cb_start_pressed,\
        output_keys=['msg_data']),\
         transitions={\
         'invalid':'Control_human_detection',\
         'valid':'monitor_for_start',\
         'preempted':'monitor_for_start'})

        smach.StateMachine.add('Control_human_detection', Control_human_detection(),\
         transitions={\
         'monitor_for_human_detection_started':'monitor_control',\
         'Control_navigation':'Control_navigation',\
         'monitor_control':'monitor_control',\
         'turn_off':'setup_state',\
         'error':'Error'},\
         remapping={\
         'userdata_input':'msg_data',\
         'userdata_output':'control_navigation_human_detection'})

        smach.StateMachine.add('monitor_for_human_detection_started', smach_ros.MonitorState("/sentry_control_topic", \
        String, monitor_cb_start_pressed,\
        output_keys=['msg_data']),\
         transitions={\
         'invalid':'Control_human_detection',\
         'valid':'monitor_for_human_detection_started',\
         'preempted':'monitor_for_human_detection_started'})

        smach.StateMachine.add('monitor_control', smach_ros.MonitorState("/sentry_control_topic", \
        String, monitor_cb_control,\
         output_keys=['msg_data']),\
         transitions={\
         'invalid':'Control_human_detection',\
         'valid':'monitor_control',\
         'preempted':'monitor_control'})

        smach.StateMachine.add('Control_navigation', Control_navigation(),\
         transitions={\
         'monitor_navigation':'monitor_navigation',\
         'Control_human_detection':'Control_human_detection',\
         'turn_off':'setup_state',\
         'error':'Error'},\
         remapping={\
         'userdata_input':'msg_data',\
         'userdata_output':'control_navigation_human_detection'})     

        smach.StateMachine.add('monitor_navigation', smach_ros.MonitorState("/sentry_control_topic", \
        String, monitor_cb_navigation,\
         output_keys=['msg_data']),\
         transitions={'invalid':'Control_navigation', 'valid':'monitor_navigation',\
         'preempted':'monitor_navigation'})


        smach.StateMachine.add('Turn_off', Turn_off(),transitions={'turn_off':'exitSM'})

        smach.StateMachine.add('Error', Error(),transitions={'error':'exitSM'})

    #Execute SMACH plan
    
    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



