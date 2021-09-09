#!/usr/bin/env python
import roslaunch
import rospy
import rosnode

package = 'rosbridge_server'
executable = 'rosbridge_websocket'

node=roslaunch.core.Node(package, executable)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()
process = launch.launch(node)
print process.is_alive()


rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/uwi/catkin_ws/src/human_detection/launch/launch_detector2.launch"])

launch.start()

rospy.loginfo("started")

rospy.sleep(60)
# 3 seconds later

launch.shutdown()


#rosnode.kill_nodes(['/listener'])



rospy.sleep(10)
process.stop()


""" rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/uwi/catkin_ws/src/human_detection/launch/launch_detector.launch"])

launch.start()

rospy.loginfo("started")

rospy.sleep(30)
# 3 seconds later

launch.shutdown() """
