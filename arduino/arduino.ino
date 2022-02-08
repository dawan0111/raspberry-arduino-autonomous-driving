#include <TimerFive.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "MotorPID.h"
#define MOTOR1_EN_A 18
#define MOTOR1_EN_B 19
#define MOTOR1_DIR 4
#define MOTOR1_PWM 5

#define MOTOR2_EN_A 3
#define MOTOR2_EN_B 2
#define MOTOR2_PWM 6
#define MOTOR2_DIR 7

#define PI 3.1415926535897932384626433832795

MotorPID motor1PID(0.1, 0.1, 2, 13, 100, 50);
MotorPID motor2PID(0.1, 0.1, 2, 13, 100, 50);

void motor1EnAISR(){
  motor1PID.encoderAInterrupt(digitalRead(MOTOR1_EN_A), digitalRead(MOTOR1_EN_B));
}

void motor1EnBISR(){
  motor1PID.encoderBInterrupt(digitalRead(MOTOR1_EN_A), digitalRead(MOTOR1_EN_B));
}

void motor2EnAISR(){
  motor2PID.encoderAInterrupt(digitalRead(MOTOR2_EN_A), digitalRead(MOTOR2_EN_B));
}

void motor2EnBISR(){
  motor2PID.encoderBInterrupt(digitalRead(MOTOR2_EN_A), digitalRead(MOTOR2_EN_B));
}

ros::NodeHandle nh;

void messageCb( const geometry_msgs::Twist& goalTwist){
  float linearSpeed = goalTwist.linear.x;
  float angularSpeed = goalTwist.angular.z;

  float motor1Speed = (float) linearSpeed + (float) (angularSpeed * 0.1075);
  float motor2Speed = (float) linearSpeed + (float) (angularSpeed * -0.1075);

  float motor1RPM = (float) motor1Speed / 0.066 / PI * 60.0;
  float motor2RPM = (float) motor2Speed / 0.066 / PI * 60.0;

  motor1PID.setGoalSpeed(motor1RPM);
  motor2PID.setGoalSpeed(motor2RPM);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

void setup() {
  Timer5.initialize(50000);
  Timer5.attachInterrupt(timer5_ISR);
  
  pinMode(MOTOR1_EN_A, INPUT_PULLUP);
  pinMode(MOTOR1_EN_B, INPUT_PULLUP);
  pinMode(MOTOR2_EN_A, INPUT_PULLUP);
  pinMode(MOTOR2_EN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(MOTOR2_EN_A), motor2EnAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_EN_B), motor2EnBISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_EN_A), motor1EnAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_EN_B), motor1EnBISR, CHANGE);

  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);
  
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);

  Serial.begin(115200);

  motor1PID.setGoalSpeed(0);
  motor2PID.setGoalSpeed(0);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  Serial.print(motor1PID.getCurrentSpeed(), 10);
  Serial.println(motor2PID.getCurrentSpeed(), 10);
  nh.spinOnce();
}

void timer5_ISR(){
  int motor1Value[2] = {0, 0};
  motor1PID.control(motor1Value);
  
  int motor2Value[2] = {0, 0};
  motor2PID.control(motor2Value);
  
  digitalWrite(MOTOR1_DIR, motor1Value[0] == 1 ? HIGH : LOW);
  analogWrite(MOTOR1_PWM, motor1Value[1]);
  digitalWrite(MOTOR2_DIR, motor2Value[0] == 1 ? HIGH : LOW);
  analogWrite(MOTOR2_PWM, motor2Value[1]);
}
