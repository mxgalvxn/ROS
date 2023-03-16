#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16, Float32
from final.msg import sender
import math

set_point = 0.0
_time = 0.0
ref = 10
CMD = {
    "none": 0,
    "step": 1,
    "square": 2,
    "sine": 3
}

def define_command(command):
    global set_point, _time, ref
    if command == CMD["step"]:
        set_point = 1 * ref if _time > 0 else 0
    elif command == CMD["square"]:
        set_point = 1 * ref if math.fmod(_time*0.5,2) > 1 else -1 * ref
    elif command == CMD["sine"]:
        set_point = math.sin(_time*2) * ref

def main():
    global set_point, _time
    rospy.init_node("sender")
    nodeRate = 100
    signalPub = rospy.Publisher("/set_point", sender, queue_size=10)
    rate = rospy.Rate(nodeRate)
    set_point_out = sender()
    cmd = CMD["none"]
    set_point_out.set_point_data = 0.0
    while not rospy.is_shutdown():
        cmd = rospy.get_param("/sp_type", CMD["none"])
        _time = rospy.get_time()
        define_command(cmd)
        set_point_out.set_point_data = set_point
        signalPub.publish(set_point_out)
        rospy.spin_once()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
