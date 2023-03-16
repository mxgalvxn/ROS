#include <ros.h>
#include <final/motor_output.h>
#include <final/motor_input.h>

#define MOTOR_ADE 18
#define MOTOR_AT 15
#define PWM 4

#define ENC_A 34
#define ENC_B 36
#define PI 3.14159265358979

const int POLLING_TIME = 100;
const double TICKS_PER_REV = 374;
const double RADS_PER_TICK = (2 * PI) / TICKS_PER_REV;

final::motor_output encB_vel;

long currentPos = 0;
long oldPos = 0;
long currTime = millis();

ros::NodeHandle handler;

void enc() {
  if (digitalRead(ENC_B))
    currentPos++;
  else
    currentPos--;
}

void pwmCallback(const final::motor_input &pwmIn) {
  ledcWrite(0, abs((int)(pwmIn.input * 180)));
  if (pwmIn.input > 0) {
    digitalWrite(MOTOR_ADE, 180);
    digitalWrite(MOTOR_AT, 0);
  } else {
    digitalWrite(MOTOR_ADE, 0);
    digitalWrite(MOTOR_AT, 180);
  }
}

void calculateSpeed(int Pos) {
  float angular_speed = float(((RADS_PER_TICK * (Pos - oldPos)) * 1000) / POLLING_TIME);
  encB_vel.output = float(((RADS_PER_TICK * (Pos - oldPos)) * 1000) / POLLING_TIME); // Rads per second
  oldPos = Pos;
}

ros::Subscriber<final::motor_input> pwm_receiver("/motor_input", pwmCallback);
ros::Publisher motor_velocity("/motor_output", &encB_vel);

void setup() {
  final::motor_output motor_out;
  // Encoder initial setup
  attachInterrupt(digitalPinToInterrupt(ENC_A), enc, RISING);
  // MOTOR_RIGHT config
  ledcSetup(0, 980, 8);
  pinMode(MOTOR_ADE, OUTPUT);
  pinMode(MOTOR_AT, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  // ROS Init
  ledcAttachPin(PWM, 0);
  handler.initNode();
  handler.subscribe(pwm_receiver);
  handler.advertise(motor_velocity);
}

void loop() {
  if (millis() - currTime >= POLLING_TIME) {
    calculateSpeed(currentPos);
    motor_velocity.publish(&encB_vel);
    currTime = millis();
  }
  handler.spinOnce();
}
