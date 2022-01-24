#include <TimerFive.h>
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include "MotorPID.h"
#define MOTOR1_EN_A 18
#define MOTOR1_EN_B 19
#define MOTOR1_DIR 4
#define MOTOR1_PWM 5

#define MOTOR2_EN_A 3
#define MOTOR2_EN_B 2
#define MOTOR2_PWM 6
#define MOTOR2_DIR 7

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

void messageCb( const std_msgs::Int8& toggle_msg){
  motor2PID.setGoalSpeed(toggle_msg.data);
}


std_msgs::Float32MultiArray pubMotorSpeed;
ros::Subscriber<std_msgs::Int8> sub("toggle_led", &messageCb );
ros::Publisher odometryPub("odometry", &pubMotorSpeed);

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

  Serial.begin(57600);

  // motor2PID.setGoalSpeed(15);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(odometryPub);

  pubMotorSpeed.data = (float*)malloc(sizeof(float) * 2);
  pubMotorSpeed.data_length = 2;
}

void loop() {;
   nh.spinOnce();
}

void timer5_ISR(){
  int motor2Value[2] = {0, 0};
  motor2PID.control(motor2Value);

  pubMotorSpeed.data[0] = motor2PID.getCurrentSpeed();
  pubMotorSpeed.data[1] = motor2PID.getCurrentSpeed();
  odometryPub.publish(&pubMotorSpeed);
  
  digitalWrite(MOTOR2_DIR, motor2Value[0] == 1 ? HIGH : LOW);
  analogWrite(MOTOR2_PWM, motor2Value[1]); 
}
