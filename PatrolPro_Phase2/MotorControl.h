#pragma once

#include <Arduino.h>

class MotorControl
{
public:
  void begin();
  void driveForward(int pwm);
  void driveBackward(int pwm);
  void rotateLeft(int pwm);
  void rotateRight(int pwm);
  void stop();

private:
  int runtimeTrim_[4] = {0, 0, 0, 0};

  int scaleMotionPwm(int basePwm, float ratio) const;
  void drive4(int m1, int m2, int m3, int m4);
  void driveTank(int leftPwm, int rightPwm);
  void writeMotorByIndex(uint8_t index, int signedPwm);
  void writeMotorRaw(uint8_t pwmPin, uint8_t in1, uint8_t in2, int signedPwm);
};
