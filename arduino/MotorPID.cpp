#include "MotorPID.h"

MotorPID::MotorPID(
  float kp, float ki, float kd,
  int plusePerRev, int gearRatio, int dletaTime
): kp(kp), ki(ki), kd(kd), plusePerRev(plusePerRev), gearRatio(gearRatio), dletaTime(dletaTime),
encoderPosition(0), dletaEncoderPosition(0), prevEncoderPosition(0), currentSpeed(0), errorSpeed(0), prevErrorSpeed(0), errorSpeedSum(0)
{
}

void MotorPID::setGoalSpeed(int goalSpeed)
{
  this->goalSpeed = goalSpeed;
}

void MotorPID::setPIDGain(float kp, float ki, float kd)
{
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

void MotorPID::encoderAInterrupt(bool encoderAValue, bool encoderBValue)
{
  if(encoderAValue == true) { 
    if(encoderBValue == false) encoderPosition = encoderPosition + 1;
    else encoderPosition = encoderPosition - 1;
  }
  else { 
    if(encoderBValue == true) encoderPosition = encoderPosition + 1;
    else encoderPosition = encoderPosition - 1;
  }
}

void MotorPID::encoderBInterrupt(bool encoderAValue, bool encoderBValue)
{
  if(encoderBValue == true) { 
    if(encoderAValue == true) encoderPosition = encoderPosition + 1;
    else encoderPosition = encoderPosition - 1;
  }
  else { 
    if(encoderAValue == false) encoderPosition = encoderPosition + 1;
    else encoderPosition = encoderPosition - 1;
  }
}

float MotorPID::getCurrentSpeed()
{
  return currentSpeed;
}


long MotorPID::getEncoderPosition()
{
  return encoderPosition;
}

void MotorPID::control(int arr[])
{
  dletaEncoderPosition = encoderPosition - prevEncoderPosition;
  prevEncoderPosition = encoderPosition;

  currentSpeed = dletaEncoderPosition * (60000 / ((float) dletaTime * plusePerRev * 4 * gearRatio));

  long up=0, ui=0, ud=0, u_val=0;
  float dletaErrorSpeed = 0;
 
  errorSpeed = goalSpeed - currentSpeed;
  errorSpeedSum = errorSpeedSum + errorSpeed;
  dletaErrorSpeed = errorSpeed - prevErrorSpeed;
  prevErrorSpeed = errorSpeed;

  up = kp * (float) errorSpeed;
  ui = ki * (float) errorSpeedSum;
  kd = kd * (float) dletaErrorSpeed;
  
  u_val = up + ui + ud;

  if(u_val<0){
    u_val = -u_val;
    if(u_val>255) u_val=255;
    arr[0] = 0;
    arr[1] = u_val;   
  }
  else{
    if(u_val>255) u_val=255;
    arr[0] = 1;
    arr[1] = u_val;
  }
}