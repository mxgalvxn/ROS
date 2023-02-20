#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String

if __name__ == '__main__':
    pub = rospy.Publisher("chatter", String, queue_size = 10)
    rospy.init_node("signal_generator")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        for i in range (10):
            hello_str = np.sin(t)   
            pub.publish(hello_str)  
            rate.sleep()    
