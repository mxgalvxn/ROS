#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Float32

signal_data = 0

def callTime(msg):
    rospy.loginfo( " Time: %s", msg.data)

def callSignal1(msg):
    rospy.loginfo( " Sine: %s", msg.data)

def callSignal(msg):
    global signal_data
    signal_data = msg.data + 10
    signal_data /=2
    rospy.loginfo( " Sine_modified %s", signal_data)
    
if __name__ == '__main__':
    rospy.init_node("process")
    rospy.Subscriber("signal", Float32,callSignal1 )
    rospy.Subscriber("signal", Float32, callSignal)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        
       rate.sleep()
