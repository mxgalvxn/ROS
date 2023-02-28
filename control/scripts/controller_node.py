#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from control.msg import motor_input
from control.msg import motor_output


motor_out = 0
motor_init = False
ref = 0

def receive_feedback(output):
    global motor_out
    global motor_init
    motor_out = output
    motor_init = True
    

def receive_setpoint(msg):
    global ref
    ref = msg.data

    
if __name__ == '__main__':
    rospy.init_node("controller_node")

    systemfeedback = rospy.Subscriber("motor_output", motor_output, receive_feedback)
    setPointSubscr = rospy.Subscriber("setpoint", Float32, receive_setpoint)

    controllerOutput = rospy.Publisher("motor_input", motor_input, queue_size=10)

    rateParam = rospy.get_param("/system_node_rate")
    rate = rospy.Rate(rateParam)

    kp = rospy.get_param("/Kp")
    Ti = rospy.get_param("/Ti")
    Td = rospy.get_param("/Td")
    ref = rospy.get_param("/set_point")

    init_time = rospy.Time.now().to_sec()
    lastError = 0.0
    totError = 0.0
    
    motor_in = 0.0

    controllerOutput.publish(motor_in, 0)
    rospy.loginfo("The motor_in value is: %f", motor_in)
    
    
    while not rospy.is_shutdown():
        time = rospy.Time.now().to_sec() - init_time
        
        if time > 0 and motor_init:
            time = rospy.Time.now().to_sec()
            error = ref - motor_out.output
            totError += error*time
            lastError = error
            rateError = (error - lastError) / time

            motor_in = (kp*error)+(Td*rateError)+(Ti*totError)
            rospy.loginfo("The motor_in value is: %f", motor_in)
            controllerOutput.publish(motor_in, init_time)
            
        else:
            motor_in = 0
            controllerOutput.publish(motor_in, init_time)
        
        rate.sleep()
