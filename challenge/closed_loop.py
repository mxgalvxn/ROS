#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float64

KP = 0.1
KI = 0.01
KD = 0.01

MIN_PWM = 0.0
MAX_PWM = 1.0

MIN_SPEED = 0.0
MAX_SPEED = 10.0

target_speed = 0.0
current_speed = 0.0
error = 0.0
last_error = 0.0
integral = 0.0
derivative = 0.0
output = 0.0

node_rate = 100

last_time = None

def speed_callback(msg):
    global current_speed
    current_speed = msg.data

def control_loop(event):
    global error, last_error, integral, derivative, output, last_time

    dt = (event.current_real - last_time).to_sec()

    # Calculate error
    error = target_speed - current_speed

    # Calculate integral and derivative
    integral += error * dt
    derivative = (error - last_error) / dt

    # Calculate output
    output = KP * error + KI * integral + KD * derivative

    # Apply saturation limits
    if output > MAX_PWM:
        output = MAX_PWM
    elif output < MIN_PWM:
        output = MIN_PWM

    # Publish output
    pwm_out = Float32()
    pwm_out.data = output
    signal_pub.publish(pwm_out)

    # Store error for next iteration
    last_error = error

    # Store time for next iteration
    last_time = event.current_real

if __name__ == '__main__':
    rospy.init_node('sender')

    signal_pub = rospy.Publisher('/motor_input', Float32, queue_size=10)
    speed_sub = rospy.Subscriber('/motor_speed', Float64, speed_callback)

    timer = rospy.Timer(rospy.Duration(1.0 / node_rate), control_loop)

    last_time = rospy.Time.now()

    target_speed = rospy.get_param('/ref')

    rospy.spin()
