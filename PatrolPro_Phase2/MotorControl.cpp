#include "MotorControl.h"

#include "Config.h"

void MotorControl::begin()
{
  for (size_t i = 0; i < Config::kMotorCount; ++i)
  {
    pinMode(Config::kMotors[i].pwm, OUTPUT);
    pinMode(Config::kMotors[i].in1, OUTPUT);
    pinMode(Config::kMotors[i].in2, OUTPUT);
  }
  stop();
}

void MotorControl::driveForward(int pwm)
{
  drive4(
    scaleMotionPwm(pwm, Config::kForwardRatio[0]),
    scaleMotionPwm(pwm, Config::kForwardRatio[1]),
    scaleMotionPwm(pwm, Config::kForwardRatio[2]),
    scaleMotionPwm(pwm, Config::kForwardRatio[3]));
}

void MotorControl::driveBackward(int pwm)
{
  drive4(
    -scaleMotionPwm(pwm, Config::kBackwardRatio[0]),
    -scaleMotionPwm(pwm, Config::kBackwardRatio[1]),
    -scaleMotionPwm(pwm, Config::kBackwardRatio[2]),
    -scaleMotionPwm(pwm, Config::kBackwardRatio[3]));
}

void MotorControl::rotateLeft(int pwm)
{
  driveTank(pwm, -pwm);
}

void MotorControl::rotateRight(int pwm)
{
  driveTank(-pwm, pwm);
}

void MotorControl::stop()
{
  drive4(0, 0, 0, 0);
}

int MotorControl::scaleMotionPwm(int basePwm, float ratio) const
{
  return constrain((int)((float)basePwm * ratio + 0.5f), 0, 255);
}

void MotorControl::drive4(int m1, int m2, int m3, int m4)
{
  if (!Config::kMotorDriveEnabled)
  {
    writeMotorByIndex(0, 0);
    writeMotorByIndex(1, 0);
    writeMotorByIndex(2, 0);
    writeMotorByIndex(3, 0);
    return;
  }

  writeMotorByIndex(0, m1);
  writeMotorByIndex(1, m2);
  writeMotorByIndex(2, m3);
  writeMotorByIndex(3, m4);
}

void MotorControl::driveTank(int leftPwm, int rightPwm)
{
  drive4(
    leftPwm - Config::kSideBalance,
    rightPwm + Config::kSideBalance,
    leftPwm - Config::kSideBalance,
    rightPwm + Config::kSideBalance);
}

void MotorControl::writeMotorByIndex(uint8_t index, int signedPwm)
{
  if (index >= Config::kMotorCount)
  {
    return;
  }

  if (signedPwm != 0)
  {
    const int direction = (signedPwm > 0) ? 1 : -1;
    const int magnitude = constrain(
      abs(signedPwm) + Config::kMotorTrim[index] + runtimeTrim_[index],
      0,
      255);
    signedPwm = direction * magnitude;
  }

  signedPwm *= Config::kMotorSign[index];
  writeMotorRaw(Config::kMotors[index].pwm, Config::kMotors[index].in1, Config::kMotors[index].in2, signedPwm);
}

void MotorControl::writeMotorRaw(uint8_t pwmPin, uint8_t in1, uint8_t in2, int signedPwm)
{
  signedPwm = constrain(signedPwm, -255, 255);

  if (signedPwm > 0)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, signedPwm);
  }
  else if (signedPwm < 0)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -signedPwm);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}
