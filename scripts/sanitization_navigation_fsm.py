#!/usr/bin/env python

import roslib #; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from std_msgs.msg import String
import roslaunch


                        ###note###
# ____________________________________________________
#|see accompanying flowchart for overall operation    |
#|____________________________________________________|



##########VARIABLES##########

#last_state is what the output to the lights was
#used to prevent updating the value if it 
#the same value as it was before
last_state = False
#to know if a human_has been detection so
#it won't publish it constantly to the remote
human_detected_once=False
##########publisher variables declaration##############
pub_light = rospy.Publisher('light_control', Bool, queue_size=500)
pub_nav_control = rospy.Publisher('nav_control', String, queue_size=500)
pub_timer_control = rospy.Publisher('timer_control_topic', String, queue_size=500)
pub_status_remote = rospy.Publisher('status', String, queue_size=500) #removed latching so that if remote restarts it doesn't see the last value

has_human_detection_been_started=False



##########STATE DECLARATIONS###########

#state to startup ROSBRIDGE
#Entering the first time starts up the launch file
#Entering a second time shuts down the launch file
class setup_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next_state', 'Error', 'Turn_off'])
        #setup publisher
        global pub_timer_control

        #to reset the timer to zero
        pub_timer_control.publish("stop_timer")       

        #to ensure the light is off
        pub_light.publish(False)

        #this variable allows to use this state differently
        #depending on if its the first or second time you enter it
        self.init_var = 0

    def execute(self, userdata):


        if self.init_var==0:
        #     #Taken from the ros launch tutorial on running ros launch files in scripts
        #     self.package = 'rosbridge_server'
        #     self.executable = 'rosbridge_websocket'
        #     self.node = roslaunch.core.Node(self.package, self.executable)
        #     self.launch = roslaunch.scriptapi.ROSLaunch()
        #     self.launch.start()
        #     self.process=self.launch.launch(self.node)
        #     rospy.loginfo(self.process.is_alive())
            self.init_var = self.init_var + 1
         
            return 'next_state'
        elif self.init_var >= 1:
        #     self.launch.shutdown()
        #     self.process.stop()
            return 'Turn_off'


#This state starts up/shuts down human detection and checks for other cases 
class Control_human_detection(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=['userdata_input'], output_keys=['userdata_output'], outcomes=['monitor_for_start', 'monitor_for_human_detection_started', 'Control_navigation', 'monitor_control', 'Turn_off', 'Error'])


    def execute(self, userdata):
        global pub_timer_control, pub_light, pub_status_remote
        
        #userdata.userdata_input is the value 
        #of the data sent into the state from another state
        #based on this value, different code is ran


        if userdata.userdata_input == "start_human_detection":

            #Taken from the ros launch tutorial on running ros launch files in scripts

            #Launching a script to determine the amount of cameras attatched to the system
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi-sentry-agx/catkin_ws/src/main_control/launch/launch_webcam_counter.launch"])
            self.launch.start()
            rospy.sleep(1)
            self.launch.shutdown()

            #Here the amount of cameras attatched to the system is checked

            with open("/home/uwi-sentry-agx/catkin_ws/src/main_control/launch/webcam_detected_result.txt") as f:
                temp=f.readlines()
            
            #if no cameras are attatched the system performs this
            if len(temp) == 0:
                rospy.logwarn(rospy.get_caller_id() +" No cameras detected, turning off")
                pub_status_remote.publish("NO CAMERAS DETECTED, SHUTTING DOWN")
                return 'Turn_off'
            #if cameras are detected the human detector is started
            else:
                pub_status_remote.publish("Starting up Human Detection")
                self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(self.uuid)
                self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi-sentry-agx/catkin_ws/src/human_detection/launch/launch_detector.launch"])
                self.launch.start()
                rospy.loginfo(rospy.get_caller_id() + "detector started")
                return 'monitor_for_human_detection_started'


        elif userdata.userdata_input == "turn_off_human_detection":
            #this starts a script to pkill the human detector for a clean exit
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi-sentry-agx/catkin_ws/src/human_detection/launch/end_detector.launch"])
            self.launch.start()
           
        
            rospy.sleep(5)
            self.launch.shutdown()
            userdata.userdata_output="start_navigation"
            rospy.loginfo(rospy.get_caller_id() + "detector ended")
            return 'Control_navigation'

        elif userdata.userdata_input == "turn_off_sentry":
            #this starts a script to pkill the human detector for a clean exit            
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi-sentry-agx/catkin_ws/src/human_detection/launch/end_detector.launch"])
            self.launch.start()

        
            rospy.sleep(5)
            self.launch.shutdown()
            rospy.loginfo(rospy.get_caller_id() + "detector ended")            
            return 'Turn_off'   

        elif userdata.userdata_input == "turn_off_sanitization":
            #this starts a script to pkill the human detector for a clean exit            
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi-sentry-agx/catkin_ws/src/human_detection/launch/end_detector.launch"])
            self.launch.start()

        
            rospy.sleep(5)
            self.launch.shutdown()
            rospy.loginfo(rospy.get_caller_id() + "detector ended")            
            return 'monitor_for_start'   

        elif userdata.userdata_input == "monitor_for_start":
            #this starts a script to pkill the human detector for a clean exit            
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi-sentry-agx/catkin_ws/src/human_detection/launch/end_detector.launch"])
            self.launch.start()

        
            rospy.sleep(5)
            self.launch.shutdown()
            rospy.loginfo(rospy.get_caller_id() + "detector ended")            
            return 'monitor_for_start' 

        elif userdata.userdata_input == "error_received":
            #this starts a script to pkill the human detector for a clean exit            
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi-sentry-agx/catkin_ws/src/human_detection/launch/end_detector.launch"])
            self.launch.start()

        
            rospy.sleep(5)
            self.launch.shutdown()
            rospy.loginfo(rospy.get_caller_id() + "error occured")            
            return 'Error'   

        elif userdata.userdata_input == "go_to_monitor_control":
            return 'monitor_control'
        
        else:
            rospy.logwarn(rospy.get_caller_id() +" Incorrect data received, error occured!")

            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi-sentry-agx/catkin_ws/src/human_detection/launch/end_detector.launch"])
            self.launch.start()


            rospy.sleep(5)
            self.launch.shutdown()
            
            rospy.loginfo(rospy.get_caller_id() + "detector ended")            

            return 'Error'


#This state starts up/shuts down navigation and checks for other cases
class Control_navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys= ['userdata_input'],\
         output_keys = ['userdata_output'], \
         outcomes=['monitor_navigation','Control_human_detection', 'Turn_off', 'Error'])
        self.init_var=0

    def execute(self, userdata):
        global pub_status_remote

        #exectute commands to start the navigation module using code similar to human_detection state
        
        pub_status_remote.publish("Navigating the room")

        #launch the launch file
        if userdata.userdata_input=="start_navigation":
            #launch the launch file
            pub_nav_control.publish("navigation_active")

            return 'monitor_navigation'

        elif userdata.userdata_input=="goal_reached":
            #shut down the node that was launched
            pub_nav_control.publish("navigation_inactive")
            userdata.userdata_output="start_human_detection"
            return 'Control_human_detection'
       
        elif userdata.userdata_input=="turn_off_sentry":
            #shut down the node that was launched
            pub_nav_control.publish("navigation_inactive")
            return 'Turn_off'

        elif userdata.userdata_input=="all_goals_reached":
            #shut down the node
            pub_nav_control.publish("navigation_inactive")
            pub_status_remote.publish("Sanitization Complete")
            return 'Turn_off'
        
        
        elif userdata.userdata_input=="error_received":
            #shut down the node that was launched
            pub_nav_control.publish("navigation_inactive")
            return 'Error'        


        else:
            rospy.logwarn(rospy.get_caller_id() +" Incorrect data received, error occured!")
            #shutdown node
            return 'Error'


class Turn_off(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Turn_off'])

    def execute(self, userdata):
        return 'Turn_off'

class Error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Error'])

    def execute(self, userdata):
        return 'Error'


################FUNCTION DECLARATIONS#############

#this function is called whenever a message is seen on the topic
#it waits until the user selects start sanitization on the remote 
#or selects shut down
def monitor_cb_start_pressed(ud, msg):

    #ud.msg_data=msg.data
    #return False
    
    if msg.data=="start_sanitization":
        ud.msg_data="start_human_detection"
        return False
    elif msg.data=="turn_off_sentry":
        ud.msg_data=msg.data
        return False
    elif msg.data=="error_received":
        ud.msg_data=msg.data
        return False
    elif msg.data=="turn_off_sanitization":
        return True
    else:    
        rospy.logwarn(rospy.get_caller_id() +" data either corrupt or ignored for this state")
        return True

#this state waits until you get a msg from the human detection module
#so you know it is running before turning on the lights
def monitor_cb_human_detection_started(ud, msg):

    global pub_light, pub_timer_control
    
    if msg.data=="human_detected_true":
        #we need to ensure that the lights are off
        #and to monitor for the resume signal we change states
        pub_light.publish(False)
        rospy.loginfo(rospy.get_caller_id() + 'UV lights OFF,Found a human')
        #update the remote
        pub_status_remote.publish("human_detected_true")
        pub_timer_control.publish("pause_timer") 
        ud.msg_data="go_to_monitor_control"
        return False
    elif msg.data=="human_detected_false":
        #here the light is turned on because its required after human detection
        #has started and also the timer must begin at the same time        
        pub_light.publish(True)
        rospy.loginfo(rospy.get_caller_id() + 'UV lights ON')
        pub_timer_control.publish("start_timer")        
        ud.msg_data="go_to_monitor_control"
        return False
    elif msg.data=="error_received":
        ud.msg_data=msg.data
        pub_light.publish(False)
        rospy.loginfo(rospy.get_caller_id() + 'UV lights OFF')
        pub_timer_control.publish("stop_timer") 
        return False
    else:    
        rospy.logwarn(rospy.get_caller_id() +" data either corrupt or ignored for this state")
        return True


# #this state is transitioned into when a human is detected
# #so that yolov5 doesn't have to be restarted
# #and the remote isn't constantly remined that a human is in the room 
# def monitor_for_restart_after_human_detected(ud, msg):

#     global pub_light, pub_timer_control
    
#     if msg.data=="human_detected_true":
#         #we need to ensure that the lights are off
#         #and to monitor for the resume signal we change states
#         pub_light.publish(False)
#         rospy.loginfo(rospy.get_caller_id() + 'UV lights OFF,Found a human')
#         #update the remote
#         pub_status_remote.publish("human_detected_true")
#         pub_timer_control.publish("pause_timer") 
#         ud.msg_data="go_to_monitor_control"
#         return False
#     elif msg.data=="human_detected_false":
#         #here the light is turned on because its required after human detection
#         #has started and also the timer must begin at the same time        
#         pub_light.publish(True)
#         rospy.loginfo(rospy.get_caller_id() + 'UV lights ON')
#         pub_timer_control.publish("start_timer")        
#         ud.msg_data="go_to_monitor_control"
#         return False
#     elif msg.data=="error_received":
#         ud.msg_data=msg.data
#         pub_light.publish(False)
#         rospy.loginfo(rospy.get_caller_id() + 'UV lights OFF')
#         pub_timer_control.publish("stop_timer") 
#         return False
#     else:    
#         rospy.logwarn("data either corrupt or ignored for this state")
#         return True
   
#this function is called whenever the /sentry_control_topic receives a message
#it then checks what the message is and performs an action
def monitor_cb_control(ud, msg):
    
    global last_state, pub_light, pub_timer_control, human_detected_once

    if msg.data=="human_detected_true":
        #ensure that the lights are off
        pub_light.publish(False)
        rospy.logwarn(rospy.get_caller_id() +' Found a human, awaiting response!')
        #pause the timer
        pub_timer_control.publish("pause_timer")
        if (not human_detected_once):
            #update the remote:
            pub_status_remote.publish("human_detected_true")
            pub_timer_control.publish("pause_timer")
            #so that this is published once
            #not every time this message comes in,
            human_detected_once=True
        return True
    elif msg.data== "human_detected_false": 
        #so that after the human is removed from the room
        #we can now resend the signal to the remote
        #if another human is detected
        if(human_detected_once):
            pub_status_remote.publish("human_detected_false")
            human_detected_once=False
        return True
    elif msg.data== "stop_sanitization":
        pub_timer_control.publish("stop_timer")
        #to ensure that the lights are off
        pub_light.publish(False)
        rospy.loginfo(rospy.get_caller_id() + 'UV lights OFF')
        ud.msg_data="monitor_for_start"
        last_state=False
        return False


        # pub_timer_control.publish("stop_timer")
        # if last_state != False:
        #     pub_light.publish(False)
        #     rospy.loginfo(rospy.get_caller_id() + 'UV lights OFF')
        #     ud.msg_data="turn_off_sentry"
        #     last_state=False
        #     return False
        # else:
        #     rospy.loginfo(rospy.get_caller_id() + 'UV lights OFF')
        #     ud.msg_data="turn_off_sentry"
        #     return False

    elif msg.data== "timer_complete":
        pub_timer_control.publish("stop_timer")
        #to ensure that the lights are off
        pub_light.publish(False)
        rospy.loginfo(rospy.get_caller_id() + 'UV lights OFF')
        ud.msg_data="turn_off_human_detection"   
        last_state=False  
        return False




        # if last_state != False:
        #     pub_light.publish(False)
        #     rospy.loginfo(rospy.get_caller_id() + 'UV lights OFF')
        #     ud.msg_data="turn_off_human_detection"   
        #     last_state=False  
        #     return False
        # else:
        #     rospy.loginfo(rospy.get_caller_id() + 'UV lights OFF')
        #     ud.msg_data="turn_off_human_detection"
        #     return False   
  

    elif msg.data== "turn_off_sanitization":
        pub_timer_control.publish("stop_timer")
        pub_light.publish(False)
        rospy.loginfo(rospy.get_caller_id() + 'UV lights OFF, turn_off_sanitization')
        ud.msg_data="turn_off_sanitization"
        human_detected_once=False #to prevent this variable remaining true
        return False

    elif msg.data== "turn_off_sentry":
        pub_timer_control.publish("stop_timer")
        pub_light.publish(False)
        rospy.loginfo(rospy.get_caller_id() + 'UV lights OFF, turn_off_sentry')
        ud.msg_data="turn_off_sentry"
        human_detected_once=False #to prevent this variable remaining true
        return False

    elif msg.data== "error_received":
        pub_timer_control.publish("stop_timer")
        pub_light.publish(False)
        rospy.loginfo(rospy.get_caller_id() + 'UV lights OFF')
        ud.msg_data="error_received"
        return False

    elif msg.data=="pause_sanitization":
        #This if statement is in the event the user presses
        #pause multiple times, so the lights and timer
        #will be turned on only once until it has to be stopped        
        if last_state!=False:
            pub_timer_control.publish("pause_timer")
            pub_light.publish(False)
            rospy.loginfo(rospy.get_caller_id() + 'UV lights OFF')
            last_state=False
            return True
        else:
            rospy.loginfo(rospy.get_caller_id() + 'UV lights OFF')
            return True

    elif msg.data== "start_sanitization":
        #This if statement is in the event the user presses
        #start multiple times, so the lights and timer
        #will be turned on only once until it has to be stopped
        if last_state!=True:
            pub_timer_control.publish("start_timer")
            pub_light.publish(True)
            rospy.loginfo(rospy.get_caller_id() + 'UV lights ON')
            last_state=True
            return True
        else:
            rospy.loginfo(rospy.get_caller_id() + 'UV lights ON')
            return True         

    elif msg.data=="all_goals_reached":
        rospy.logwarn(rospy.get_caller_id() +"nav data")
        return True
    elif msg.data=="goal_reached":
        rospy.logwarn(rospy.get_caller_id() +"nav data")
        return True
    else:
        #if none of the data is recognised return incorrect data to control human detection
        pub_timer_control.publish("stop_timer")
        pub_light.publish(False)
        rospy.loginfo(rospy.get_caller_id() +' UV lights OFF')
        rospy.logwarn(rospy.get_caller_id() +' Incorrect data')
        rospy.loginfo(rospy.get_caller_id() + msg.data)
        ud.msg_data=msg.data
        return False


#this function is called whenever the /sentry_navigation_topic receives a message
#it then checks what the message is and performs an action
def monitor_cb_navigation(ud, msg):
    

    if msg.data=="goal_reached":
        ud.msg_data=msg.data
        return False
    elif msg.data=="turn_off_sentry":
        ud.msg_data=msg.data
        return False
    elif msg.data=="stop_sanitization":
        ud.msg_data="turn_off_sentry"
        return False
    elif msg.data=="all_goals_reached":
        ud.msg_data=msg.data
        return False
    else:    
        rospy.logwarn(rospy.get_caller_id() +" data out of scope")
        return True



def main():
    rospy.init_node('smach_example_state_machine')

    #Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exitSM'])

    # Open the container

    with sm:
        #The following block add states to the container
        #This is the standard way from the tutorials

        smach.StateMachine.add('setup_state', setup_state(),\
         transitions={\
         'next_state':'monitor_for_start',\
         'Error':'Error',\
         'Turn_off':'Turn_off'})

        #this is a monitor state
        #it is a template from the smach tutorials
        smach.StateMachine.add('monitor_for_start', smach_ros.MonitorState("/sentry_control_topic", \
        String, monitor_cb_start_pressed,\
        output_keys=['msg_data']),\
         transitions={\
         'invalid':'Control_human_detection',\
         'valid':'monitor_for_start',\
         'preempted':'monitor_for_start'})

        smach.StateMachine.add('Control_human_detection', Control_human_detection(),\
         transitions={\
         'monitor_for_start':'monitor_for_start',\
         'monitor_for_human_detection_started':'monitor_for_human_detection_started',\
         'Control_navigation':'Control_navigation',\
         'monitor_control':'monitor_control',\
         'Turn_off':'setup_state',\
         'Error':'Error'},\
         remapping={\
         'userdata_input':'msg_data',\
         'userdata_output':'msg_data'})

        #this is a monitor state
        #it is a template from the smach tutorials
        smach.StateMachine.add('monitor_for_human_detection_started', smach_ros.MonitorState("/sentry_control_topic", \
        String, monitor_cb_human_detection_started,\
        output_keys=['msg_data']),\
         transitions={\
         'invalid':'Control_human_detection',\
         'valid':'monitor_for_human_detection_started',\
         'preempted':'monitor_for_human_detection_started'})

        # #this is a monitor state
        # #it is a template from the smach tutorials
        # smach.StateMachine.add('monitor_for_restart_after_human_detected', smach_ros.MonitorState("/sentry_control_topic", \
        # String, monitor_cb_restart_after_human_detected,\
        # output_keys=['msg_data']),\
        #  transitions={\
        #  'invalid':'Control_human_detection',\
        #  'valid':'monitor_for_restart_after_human_detected',\
        #  'preempted':'monitor_for_restart_after_human_detected'})        

        #this is a monitor state
        #it is a template from the smach tutorials
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
         'Turn_off':'setup_state',\
         'Error':'Error'},\
         remapping={\
         'userdata_input':'msg_data',\
         'userdata_output':'msg_data'})     

        #this is a monitor state
        #it is a template from the smach tutorials
        smach.StateMachine.add('monitor_navigation', smach_ros.MonitorState("/sentry_control_topic", \
        String, monitor_cb_navigation,\
         output_keys=['msg_data']),\
         transitions={'invalid':'Control_navigation', 'valid':'monitor_navigation',\
         'preempted':'monitor_navigation'})


        smach.StateMachine.add('Turn_off', Turn_off(),transitions={'Turn_off':'exitSM'})

        smach.StateMachine.add('Error', Error(),transitions={'Error':'setup_state'})

    #Execute the SMACH itself
    
    outcome = sm.execute()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




