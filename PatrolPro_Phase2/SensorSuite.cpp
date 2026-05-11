#include "SensorSuite.h"

#include <string.h>

void SensorSuite::begin()
{
  for (size_t i = 0; i < Config::kDualSensorCount; ++i)
  {
    pinMode(Config::kDualSensors[i].digitalPin, INPUT);
    pinMode(Config::kDualSensors[i].analogPin, INPUT);
  }

  for (size_t i = 0; i < Config::kUltrasonicCount; ++i)
  {
    pinMode(Config::kUltrasonicSensors[i].trigPin, OUTPUT);
    pinMode(Config::kUltrasonicSensors[i].echoPin, INPUT);
    digitalWrite(Config::kUltrasonicSensors[i].trigPin, LOW);
  }
}

bool SensorSuite::update(unsigned long nowMs, bool forceRead)
{
  if (!forceRead && (nowMs - lastHazardSampleMs_) < Config::kHazardSampleIntervalMs)
  {
    return false;
  }

  lastHazardSampleMs_ = nowMs;

  for (size_t i = 0; i < Config::kDualSensorCount; ++i)
  {
    snapshot_.analog[i] = readAnalogAverage(Config::kDualSensors[i].analogPin);
    snapshot_.digital[i] = digitalRead(Config::kDualSensors[i].digitalPin);
  }

  if (snapshot_.analog[Config::kGas] >= Config::kGasThreshold)
  {
    ++gasCount_;
  }
  else
  {
    gasCount_ = 0;
  }
  snapshot_.gasAlert = gasCount_ >= Config::kGasStableCount;

  const bool anyFrontActive =
    frontActive(snapshot_.analog[Config::kIr5_1]) ||
    frontActive(snapshot_.analog[Config::kIr5_2]) ||
    frontActive(snapshot_.analog[Config::kIr5_3]) ||
    frontActive(snapshot_.analog[Config::kIr5_4]) ||
    frontActive(snapshot_.analog[Config::kIr5_5]);

  if (anyFrontActive)
  {
    ++frontCount_;
  }
  else
  {
    frontCount_ = 0;
  }
  snapshot_.frontAlert = frontCount_ >= Config::kFrontStableCount;

  snapshot_.direction = getOverallDirection();
  if (!snapshot_.frontAlert && strncmp(snapshot_.direction, "FRONT", 5) == 0)
  {
    snapshot_.direction = "NONE";
  }

  snapshot_.hazardDetected = (strcmp(snapshot_.direction, "NONE") != 0) || snapshot_.gasAlert;
  snapshot_.hazardType = getHazardType(snapshot_.hazardDetected, snapshot_.gasAlert, snapshot_.direction);
  return true;
}

const SensorSuite::Snapshot& SensorSuite::snapshot() const
{
  return snapshot_;
}

float SensorSuite::readPatrolLeftCm() const
{
  return readDistanceMedian(Config::kPatrolLeftTrig, Config::kPatrolLeftEcho);
}

float SensorSuite::readPatrolRightCm() const
{
  return readDistanceMedian(Config::kPatrolRightTrig, Config::kPatrolRightEcho);
}

float SensorSuite::readPatrolRearCm() const
{
  return readDistanceMedian(Config::kPatrolRearTrig, Config::kPatrolRearEcho);
}

float SensorSuite::readPatrolFrontCm() const
{
  return readDistanceMedian(Config::kPatrolFrontTrig, Config::kPatrolFrontEcho);
}

float SensorSuite::readDistanceMedian(uint8_t trigPin, uint8_t echoPin) const
{
  float a = readDistanceCm(trigPin, echoPin);
  delay(8);
  float b = readDistanceCm(trigPin, echoPin);
  delay(8);
  float c = readDistanceCm(trigPin, echoPin);

  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

void SensorSuite::printSnapshot(Stream& out)
{
  update(millis(), true);

  out.println(F("------------------------------------------------------------"));
  out.println(F("[Analog + Digital sensors]"));

  for (size_t i = 0; i < Config::kDualSensorCount; ++i)
  {
    const Config::DualSensor& sensor = Config::kDualSensors[i];
    out.print(sensor.name);
    out.print(F(": A="));
    out.print(snapshot_.analog[i]);
    out.print(F(" D="));
    out.print(snapshot_.digital[i]);
    out.print(F("  ("));
    out.print(sensor.digitalLabel);
    out.print(F(", "));
    out.print(sensor.analogLabel);
    out.println(F(")"));
  }

  out.println(F(""));
  out.println(F("[Ultrasonic sensors]"));
  bool obstacleWarning = false;
  for (size_t i = 0; i < Config::kUltrasonicCount; ++i)
  {
    const Config::UltrasonicSensor& sensor = Config::kUltrasonicSensors[i];
    const float distanceCm = readDistanceMedian(sensor.trigPin, sensor.echoPin);

    out.print(sensor.name);
    out.print(F(": "));
    if (distanceCm < 0.0f)
    {
      out.print(F("NO_ECHO"));
    }
    else
    {
      out.print(distanceCm, 1);
      out.print(F(" cm"));
      if (distanceCm <= Config::kObstacleLedCm)
      {
        obstacleWarning = true;
      }
    }
    out.print(F("  (trig "));
    out.print(sensor.trigLabel);
    out.print(F(", echo "));
    out.print(sensor.echoLabel);
    out.println(F(")"));
    delay(20);
  }

  out.println(F(""));
  out.println(F("[Hazard summary]"));
  out.print(F("Direction="));
  out.print(snapshot_.direction);
  out.print(F(" | HazardType="));
  out.print(snapshot_.hazardType);
  out.print(F(" | GasAlert="));
  out.print(snapshot_.gasAlert ? F("YES") : F("NO"));
  out.print(F(" | HazardDetected="));
  out.println(snapshot_.hazardDetected ? F("YES") : F("NO"));

  if (obstacleWarning)
  {
    out.println(F("[ULTRASONIC] obstacle within warning distance"));
  }
}

int SensorSuite::readAnalogAverage(uint8_t analogPin, int samples) const
{
  long total = 0;
  for (int i = 0; i < samples; ++i)
  {
    total += analogRead(analogPin);
    delay(2);
  }
  return (int)(total / samples);
}

float SensorSuite::readDistanceCm(uint8_t trigPin, uint8_t echoPin) const
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const unsigned long duration = pulseIn(echoPin, HIGH, Config::kUltraTimeoutUs);
  if (duration == 0UL)
  {
    return -1.0f;
  }

  const float distanceCm = duration * 0.0343f * 0.5f;
  if (distanceCm < Config::kUltraMinCm || distanceCm > Config::kUltraMaxCm)
  {
    return -1.0f;
  }
  return distanceCm;
}

bool SensorSuite::sideFlameTriggered(int value) const
{
  return value < Config::kSideFlameThreshold;
}

bool SensorSuite::frontActive(int value) const
{
  return value > Config::kFrontActiveThreshold;
}

bool SensorSuite::frontStrong(int value) const
{
  return value > Config::kFrontStrongThreshold;
}

const char* SensorSuite::getFrontDirection() const
{
  const int f1 = snapshot_.analog[Config::kIr5_1];
  const int f2 = snapshot_.analog[Config::kIr5_2];
  const int f3 = snapshot_.analog[Config::kIr5_3];
  const int f4 = snapshot_.analog[Config::kIr5_4];
  const int f5 = snapshot_.analog[Config::kIr5_5];

  const bool a1 = frontActive(f1);
  const bool a2 = frontActive(f2);
  const bool a3 = frontActive(f3);
  const bool a4 = frontActive(f4);
  const bool a5 = frontActive(f5);

  const bool s1 = frontStrong(f1);
  const bool s2 = frontStrong(f2);
  const bool s3 = frontStrong(f3);
  const bool s4 = frontStrong(f4);
  const bool s5 = frontStrong(f5);

  if (s3 && !s1 && !s5) return "FRONT_CENTER";
  if (s1 || s2) return s3 ? "FRONT_LEFT_CENTER" : "FRONT_LEFT";
  if (s4 || s5) return s3 ? "FRONT_RIGHT_CENTER" : "FRONT_RIGHT";
  if (a3) return "FRONT_CENTER";
  if ((a1 || a2) && (a4 || a5)) return "FRONT_WIDE";
  return "NONE";
}

const char* SensorSuite::getOverallDirection() const
{
  const bool leftTrig = sideFlameTriggered(snapshot_.analog[Config::kIr1]);
  const bool rightTrig = sideFlameTriggered(snapshot_.analog[Config::kIr2]);
  const bool backTrig = sideFlameTriggered(snapshot_.analog[Config::kIrBack]);

  if (leftTrig && !rightTrig && !backTrig) return "LEFT";
  if (rightTrig && !leftTrig && !backTrig) return "RIGHT";
  if (backTrig && !leftTrig && !rightTrig) return "REAR";
  if (leftTrig && rightTrig && !backTrig) return "BOTH_SIDE";
  if (leftTrig && backTrig && !rightTrig) return "LEFT_REAR";
  if (rightTrig && backTrig && !leftTrig) return "RIGHT_REAR";
  if (leftTrig && rightTrig && backTrig) return "MULTI_SIDE_REAR";
  return getFrontDirection();
}

const char* SensorSuite::getHazardType(bool hazard, bool gasAlert, const char* direction) const
{
  if (!hazard && !gasAlert && strcmp(direction, "NONE") == 0) return "NONE";
  if (strcmp(direction, "NONE") != 0 && !gasAlert) return "FLAME";
  if (strcmp(direction, "NONE") == 0 && gasAlert) return "SMOKE_GAS";
  return "BOTH";
}
