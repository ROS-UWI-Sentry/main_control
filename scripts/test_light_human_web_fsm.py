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
pub_timer_control = rospy.Publisher('brwsrButtons', String, queue_size=500)
pub_status_browser = rospy.Publisher('status', String, queue_size=500)

has_human_detection_been_started=False


class setup_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next_state', 'error', 'turn_off'])
        #setup publisher


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





class sanitization_master(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next_state','turn_off', 'error'])
        #setup publisher
        self.pub_nav_startup = rospy.Publisher('nav_startup', Bool, queue_size=500)
        self.pub_nav_startup.publish(False)

    def execute(self, userdata):

        #exectute commands to start the navigation module
        #or 
        #publish message to start the navigation module

        self.pub_nav_startup.publish(True)
        
        pub_status_browser.publish("Starting navigation stack")
        
        ###publish message to request goal infromation    
        ###"Calculate the sanitization point"
        
        ###receive the goal info and send it to the browser OR
        ###maybe the nav module can publish this to the browser
        ###itself since this state cannot receive anything 
    

        ###publish move to next goal message to nav module
        ###how do i know if the nav module is running?

        # go into a monitor state to look for goal reached confirmation



        return 'next_state'



#monitor callback




def monitor_cb_goal_reached(ud, msg):
    
    if msg.data=="turn_off":
        ud.msg_data=msg.data
        return False
    elif msg.data=="nav_module_started":
        #publish message to request goal infromation from nav module    
        #publish "Calculate the sanitization points" 
        #let nav module send the result data to the remote?
        pub_nav_control.publish("calculate_sanitization_points")
        return True
    elif msg.data=="goal_information_sent":
        #publish move to next goal message to nav module
        #publish "move to next goal"
        pub_nav_control.publish("move_to_next_goal")
        return True
    elif msg.data=="goal_reached":
        ud.msg_data=msg.data
        return False
    elif msg.data=="all_goals_reached":
        ud.msg_data=msg.data
        return False
    else:    
        rospy.logwarn("data out of scope")
        return True


# class Control_rosbridge(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['next_state', 'turn_off'])

#         self.init_var = 0

#     def execute(self, userdata):
#         if self.init_var==0:
#             self.package = 'rosbridge_server'
#             self.executable = 'rosbridge_websocket'
#             self.node = roslaunch.core.Node(self.package, self.executable)
#             self.launch = roslaunch.scriptapi.ROSLaunch()
#             self.launch.start()
#             self.process=self.launch.launch(self.node)
#             rospy.loginfo(self.process.is_alive())
#             self.init_var = self.init_var + 1
#             return 'next_state'
#         elif self.init_var >= 1:
#             self.process.stop()
#             return 'turn_off'


#Startup human detection
class Control_human_detection(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=['msg_data'], outcomes=['next_state', 'previous_state', 'turn_off'])
        #launch yolov5 here from launching code
        self.init_var = 0

    def execute(self, userdata):
        #exit yolov5 from here from launching code

        if userdata.msg_data == "goal_reached":
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi/catkin_ws/src/human_detection/launch/launch_detector.launch"])
            self.launch.start()
            rospy.loginfo("detector started")
            self.init_var = self.init_var + 1
            return 'next_state'

            #if self.init_var==0:

            #elif self.init_var >= 1:
                #self.launch.shutdown()



        elif userdata.msg_data == "all_goals_reached":

            return 'turn_off'

        elif userdata.msg_data == "turn_off_human_detection":
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi/catkin_ws/src/human_detection/launch/end_detector.launch"])
            self.launch.start()
            rospy.loginfo("detector ending")
        
            rospy.sleep(5)
            self.launch.shutdown()
            return 'previous_state'

        elif userdata.msg_data == "turn_off":
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/uwi/catkin_ws/src/human_detection/launch/end_detector.launch"])
            self.launch.start()
            rospy.loginfo("detector ending")
        
            rospy.sleep(5)
            self.launch.shutdown()
            return 'turn_off'
        
        else:
            rospy.logwarn("Incorrect data received, turning off")
            return 'turn_off'

class Turn_off(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn_off'])

    def execute(self, userdata):
        return 'turn_off'

class Error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Error'])

    def execute(self, userdata):
        return 'Error'

# def monitor_cb_human_detection_starting(ud, msg):

#     ud.data_received=msg.data
#     return False


    

def monitor_cb_control(ud, msg):
    #ud.control_data=msg.data
    global last_state, pub_light, has_human_detection_been_started, pub_nav_control
    #has_human_detection_been_started waits until you get a msg from that module
    #so you know it is running before turning on the lights

    if msg.data=="human_detected_true":
        pub_light.publish(False)
        rospy.loginfo('OFF')
        ud.msg_data="turn_off"
        return False
    elif msg.data== "human_detected_false": #and has_human_detection_been_started==False:
        has_human_detection_been_started =True
        return True
    elif msg.data== "stop_sanitization" and has_human_detection_been_started == True:
        if last_state != False:
            pub_light.publish(False)
            last_state=False
            rospy.loginfo('OFF')
            ud.msg_data="turn_off"
            return False
        else:
            ud.msg_data="turn_off"
            return False
    elif msg.data== "stop_sanitization" and has_human_detection_been_started == False:
        return True
    elif msg.data== "timer_complete" and has_human_detection_been_started==True:
        if last_state != False:
            pub_light.publish(False)
            last_state=False
            rospy.loginfo('OFF')
            ud.msg_data="turn_off_human_detection"
            pub_nav_control.publish("move_to_next_goal")
            #pub_timer_control("Reset Timer")           
            return False
        else:
            ud.msg_data="turn_off_human_detection"
            pub_nav_control.publish("move_to_next_goal")
            #pub_timer_control("reset_timer")
            return False   
    elif msg.data== "timer_complete" and has_human_detection_been_started==False:

        return True     
    elif msg.data== "turn_off_sentry":
        pub_light.publish(False)
        rospy.loginfo('OFF')
        ud.msg_data="turn_off"
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
        rospy.loginfo(msg.data)
        ud.msg_data="turn_off"
        return False


def main():
    rospy.init_node('smach_example_state_machine')

    #Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exitSM'])

    # Open the container

    with sm:
        #add states to the counter

        smach.StateMachine.add('setup_state', setup_state(),\
         transitions={'next_state':'sanitization_master','error':'Error', 'turn_off':'Turn_off'})


        smach.StateMachine.add('sanitization_master', sanitization_master(),\
         transitions={'next_state':'monitor_goal_reached','error':'Error','turn_off':'Turn_off'})


        smach.StateMachine.add('monitor_goal_reached', smach_ros.MonitorState("/sentry_navigation_topic", \
        String, monitor_cb_goal_reached,\
        output_keys=['msg_data']),\
         transitions={'invalid':'Control_human_detection', 'valid':'monitor_goal_reached',\
         'preempted':'monitor_goal_reached'})

        smach.StateMachine.add('Control_human_detection', Control_human_detection(),\
         transitions={'next_state':'monitor_control', 'previous_state':'monitor_goal_reached', 'turn_off':'setup_state'},\
         remapping={'msg_data':'msg_data'})



        smach.StateMachine.add('monitor_control', smach_ros.MonitorState("/sentry_control_topic", \
        String, monitor_cb_control,\
         output_keys=['msg_data']),\
         transitions={'invalid':'Control_human_detection', 'valid':'monitor_control',\
         'preempted':'monitor_control'}, remapping={'control_data':'control_data'})


        #smach.StateMachine.add('Control_rosbridge', Control_rosbridge(),\
        # transitions={'next_state':'Control_human_detection','turn_off':'Turn_off'})         

        smach.StateMachine.add('Turn_off', Turn_off(),transitions={'turn_off':'exitSM'})

        smach.StateMachine.add('Error', Error(),transitions={'Error':'exitSM'})

    #Execute SMACH plan

    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




