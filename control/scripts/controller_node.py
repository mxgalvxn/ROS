#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from control.msg import motor_output
from control.msg import motor_input

motor_out = 0
motor_init = False
ref = 0

def receive_feedback(output):
    motor_out = output
    motor_init = True

def receive_setpoint(data):
    ref = data

    
if __name__ == '__main__':
    rospy.init_node("controller_node")

    rate = rospy.Rate(1000)
    kp = rospy.get_param("/Kp")
    Ti = rospy.get_param("/Ti")
    Td = rospy.get_param("/Td")
    ref = rospy.get_param("/set_point")

    systemfeedback = rospy.Subscriber("/motor_out",Float32, queue_size=10)
    controllerOutput = rospy.Publisher("/setpoint",Float32, queue_size=10)
    setPointSubscr = rospy.Subscriber("/motor_out",Float32, queue_size=10)
    handler=rospy.Subscriber("/setpoint",Float32, queue_size=10)
    time = rospy.Time.now().to_sec()
    lastError = 0.0
    cumError = 0.0
    motor_in = 0.0
    controllerOutput.publish(controllerOutput)
    while rospy.is_shutdown():
        dTime = rospy.Time.now().to_sec() - time
        if dTime > 0 and motor_init:
            time = rospy.Time.now().toSec()
            error = ref - motor_out
            cumError += error*dTime
            rateError = (error - lastError) / dTime
            lastError = error
            motor_in.input = (kp*error)+(Td*rateError)+(Ti*cumError)
            controllerOutput.publish(motor_in)
        else:
            motor_in = 0
            controllerOutput.publish(motor_in)
    rospy.spin()
    rate.sleep()



