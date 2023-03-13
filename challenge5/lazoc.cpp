#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

const double KP = 0.1;
const double KI = 0.01;
const double KD = 0.01;

const double MIN_PWM = 0.0;
const double MAX_PWM = 1.0;

const double MIN_SPEED = 0.0;
const double MAX_SPEED = 10.0;

double target_speed = 0.0;
double current_speed = 0.0;
double error = 0.0;
double last_error = 0.0;
double integral = 0.0;
double derivative = 0.0;
double output = 0.0;

int nodeRate = 100;

ros::Time last_time;

void speed_callback(const std_msgs::Float64::ConstPtr& msg) {
    current_speed = msg->data;
}

void control_loop(const ros::TimerEvent& event) {
    double dt = (event.current_real - last_time).toSec();

    // Calculate error
    error = target_speed - current_speed;

    // Calculate integral and derivative
    integral += error * dt;
    derivative = (error - last_error) / dt;

    // Calculate output
    output = KP * error + KI * integral + KD * derivative;

    // Apply saturation limits
    if (output > MAX_PWM) {
        output = MAX_PWM;
    } else if (output < MIN_PWM) {
        output = MIN_PWM;
    }

    // Publish output
    std_msgs::Float32 pwmOut;
    pwmOut.data = output;
    signalPub.publish(pwmOut);

    // Store error for next iteration
    last_error = error;

    // Store time for next iteration
    last_time = event.current_real;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sender");
    ros::NodeHandle handler;

    ros::Subscriber speedSub = handler.subscribe("/motor_speed", 10, speed_callback);
    ros::Publisher signalPub = handler.advertise<std_msgs::Float32>("/motor_input", 10);

    ros::Timer timer = handler.createTimer(ros::Duration(1.0 / nodeRate), control_loop);

    last_time = ros::Time::now();

    ros::param::get("/ref", target_speed);

    ros::spin();

    return 0;
}
