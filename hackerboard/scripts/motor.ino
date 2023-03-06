#include <ros.h>
#include <std_msgs/Float32.h>

#define pwm 4
#define mot1 18
#define mot2 15


ros::NodeHandle handler;

void pwmCallback(const std_msgs::Float32 &pwmMsg) {
  ledcWrite(0, abs((int)(pwmMsg.data*255)));
  if (pwmMsg.data > 0) {
    digitalWrite(mot1, 1);
    digitalWrite(mot2, 0);
  } else {
    digitalWrite(mot1, 0);
    digitalWrite(mot2, 1);
  }
}

ros::Subscriber<std_msgs::Float32> sub("/pwm", pwmCallback);

void setup() {
  ledcSetup(0, 980, 8);
  pinMode(mot1, OUTPUT);
  pinMode(mot2, OUTPUT);
  pinMode(pwm, OUTPUT);
  ledcAttachPin(pwm, 0);
  handler.initNode();
  handler.subscribe(sub);
}

void loop() {
  handler.spinOnce();
  delay(1);
}
