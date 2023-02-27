#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import String

if __name__ == '__main__':
    pub = rospy.Publisher("time", Float32, queue_size = 10)
    pub1 = rospy.Publisher("signal", Float32, queue_size = 10)
    rospy.init_node("signal_generator")
    rate = rospy.Rate(10)
    time = 0

    while not rospy.is_shutdown():
       time = rospy.get_time()
       pub.publish(time)
       signal = np.sin(time)
       pub1.publish(signal)  
       rate.sleep() 