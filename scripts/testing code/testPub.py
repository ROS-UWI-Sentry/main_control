#!/usr/bin/env python


import rospy
from std_msgs.msg import Bool


def talker():
    pub = rospy.Publisher('humanDetected', Bool, queue_size=10)
    rospy.init_node('testPub', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    data = False
    while not rospy.is_shutdown():
        data = not(data)
       
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
