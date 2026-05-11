#pragma once

#include <Arduino.h>

#include "Config.h"

class SensorSuite
{
public:
  struct Snapshot
  {
    int analog[Config::kDualSensorCount] = {0};
    int digital[Config::kDualSensorCount] = {0};
    bool gasAlert = false;
    bool frontAlert = false;
    bool hazardDetected = false;
    const char* direction = "NONE";
    const char* hazardType = "NONE";
  };

  void begin();
  bool update(unsigned long nowMs, bool forceRead = false);
  const Snapshot& snapshot() const;
  float readPatrolLeftCm() const;
  float readPatrolRightCm() const;
  float readPatrolRearCm() const;
  float readPatrolFrontCm() const;
  float readDistanceMedian(uint8_t trigPin, uint8_t echoPin) const;
  void printSnapshot(Stream& out);

private:
  Snapshot snapshot_;
  int gasCount_ = 0;
  int frontCount_ = 0;
  unsigned long lastHazardSampleMs_ = 0;

  int readAnalogAverage(uint8_t analogPin, int samples = Config::kAnalogSamples) const;
  float readDistanceCm(uint8_t trigPin, uint8_t echoPin) const;
  bool sideFlameTriggered(int value) const;
  bool frontActive(int value) const;
  bool frontStrong(int value) const;
  const char* getFrontDirection() const;
  const char* getOverallDirection() const;
  const char* getHazardType(bool hazard, bool gasAlert, const char* direction) const;
};
