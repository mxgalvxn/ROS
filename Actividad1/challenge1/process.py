#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callTime(msg):
    rospy.loginfo( " I heard %s", msg.data)

def callSignal(msg):
    rospy.loginfo( " I heard %s", msg.data)
    
if __name__ == '__main__':
    rospy.init_node("process")
    rospy.Subscriber("time", String, callTime)
    rospy.Subscriber("signal", String, callSignal)

    rospy.spin()
