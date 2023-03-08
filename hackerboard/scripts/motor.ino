#include <ros.h>
#include <std_msgs/Float32.h>

#define PWM_PIN 4
#define MOTOR_PIN_1 18
#define MOTOR_PIN_2 15

ros::NodeHandle node_handle;

void PwmCallback(const std_msgs::Float32 &pwm_msg) {
  ledcWrite(0, abs((int)(pwm_msg.data * 255)));
  if (pwm_msg.data > 0) {
    digitalWrite(MOTOR_PIN_1, HIGH);
    digitalWrite(MOTOR_PIN_2, LOW);
  } else {
    digitalWrite(MOTOR_PIN_1, LOW);
    digitalWrite(MOTOR_PIN_2, HIGH);
  }
}

ros::Subscriber<std_msgs::Float32> subscriber("/pwm", PwmCallback);

void setup() {
  ledcSetup(0, 980, 8);
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  ledcAttachPin(PWM_PIN, 0);
  node_handle.initNode();
  node_handle.subscribe(subscriber);
}

void loop() {
  node_handle.spinOnce();
  delay(1);
}
