#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from control.msg import motor_input
from control.msg import motor_output


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

    rate = rospy.get_param("/system_node_rate")
    kp = rospy.get_param("/Kp")
    Ti = rospy.get_param("/Ti")
    Td = rospy.get_param("/Td")
    ref = rospy.get_param("/set_point")

    systemfeedback = rospy.Subscriber("/motor_out",Float32, receive_feedback)
    controllerOutput = rospy.Publisher("/motor_input",Float32, queue_size=10)
    setPointSubscr = rospy.Subscriber("/motor_out",Float32,receive_setpoint)
    handler=rospy.Subscriber("/setpoint",Float32, queue_size=10)
    init_time = rospy.get_time()
    lastError = 0.0
    cumError = 0.0
    motor_in = 0.0
    controllerOutput.publish(motor_in)
    while not rospy.is_shutdown():
        time = rospy.get_time()-init_time
        dTime = rospy.Time.now().to_sec() - time
        if dTime > 0 and motor_init:
            time = rospy.get_time()-init_time
            error = ref - motor_out
            cumError += error*dTime
            rateError = (error - lastError) / dTime
            lastError = error
            motor_in = (kp*error)+(Td*rateError)+(Ti*cumError)
            controllerOutput.publish(motor_in)
        else:
            motor_in = 0
            controllerOutput.publish(motor_in)
    rospy.spin()
    rate.sleep()



