#pragma once

#include <Arduino.h>

enum class SerialCommandType : uint8_t
{
  None,
  Help,
  Snapshot,
  ToggleStream,
  Beep,
  LedTest,
  ServoSweep,
  ServoCenter,
  ExternalLedSet,
  ExternalLedRange,
  ExternalLedAll,
  ExternalLedOff,
  DriveForward,
  DriveBackward,
  TurnLeft,
  TurnRight,
  DriveStop,
  EmergencyStop,
  AutoPatrol,
  ManualDrive,
  DriveThreeMeters,
  Status,
  PersonDetected,
  FaceVerified,
  FaceUnknown,
  FaceTimeout,
  ArrivalReached,
  Heartbeat,
};

struct SerialCommand
{
  SerialCommandType type = SerialCommandType::None;
  char arg[128] = {0};
};

class SerialProtocol
{
public:
  bool read(Stream& serial, SerialCommand& command);
  void printHelp(Stream& serial) const;

private:
  char buffer_[128] = {0};
  uint8_t length_ = 0;

  SerialCommand parseLine(char* line) const;
  void trim(char* text) const;
  bool equalsIgnoreCase(const char* a, const char* b) const;
};
