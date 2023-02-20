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
    signal_data = msg.data
    rospy.loginfo( " Sine_modified %s", signal_data)
    
if __name__ == '__main__':
    rospy.init_node("process")
    signal_processed = rospy.Subscriber("signal", Float32, callSignal)
    pub = rospy.Publisher("signal_processed", Float32, queue_size = 10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if signal_data < 0:
            signal_data = signal_data*-1
        signal_data/=2
        pub.publish(signal_data)  
        rate.sleep()
