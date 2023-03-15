#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "final/motor_input.h"
#include "final/motor_output.h"
#include "final/sender.h"

const double Kp = .05;
const double Ki = 0.1;
const double Kd = 0;

double target_speed = 0.0;
double current_speed = 0.0;
double error = 0.0;
double last_error = 0.0;
double integral = 0.0;
double derivative = 0.0;
double output = 0.0;

int nodeRate = 100;
bool motor_init = false;

void speed_callback(const final::motor_output::ConstPtr& msg)
{
    current_speed = msg->data;
    motor_init = true;
}

void set_point_callback(const final::sender::ConstPtr& setpoint)
{
    target_speed = setpoint->set_point_data;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle handler;
    ros::Rate rate(nodeRate); 

    double last_time = ros::Time::now().toSec();

    ros::Subscriber speedSub = handler.subscribe("/motor_output", 10, speed_callback);
    ros::Subscriber setPointSub = handler.subscribe("/sender", 10, set_point_callback);

    ros::Publisher signalPub = handler.advertise<final::motor_input>("/motor_input", 10);

    final::motor_input pwmOut;
    pwmOut.input = 0.0;
    signalPub.publish(pwmOut);
    while (ros::ok()) {
        ros::param::get("/ref", target_speed);
        if(motor_init){
            double dt = ros::Time::now().toSec() - last_time;
            if(dt > 0) {
                error = target_speed - current_speed;
                integral += error*dt;
                derivative = (error - last_error) / dt;
                last_error = error;
                pwmOut.input = (Kp*error)+(Kd*derivative)+(Ki*integral);
                double result = abs(pwmOut.input);
                if(result>1){
                    pwmOut.input = pwmOut.input/result;
                    signalPub.publish(pwmOut);
                }else{
                    signalPub.publish(pwmOut); 
                }
            }
        }
        last_time = ros::Time::now().toSec();
        ros::spinOnce();
        rate.sleep();
    }
}
