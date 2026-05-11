#include "SerialProtocol.h"

#include <ctype.h>
#include <string.h>

bool SerialProtocol::read(Stream& serial, SerialCommand& command)
{
  while (serial.available() > 0)
  {
    const char c = (char)serial.read();
    if (c == '\r')
    {
      continue;
    }

    if (c == '\n')
    {
      buffer_[length_] = '\0';
      length_ = 0;
      command = parseLine(buffer_);
      return command.type != SerialCommandType::None;
    }

    if (length_ < sizeof(buffer_) - 1)
    {
      buffer_[length_++] = c;
    }
    else
    {
      length_ = 0;
    }
  }

  return false;
}

void SerialProtocol::printHelp(Stream& serial) const
{
  serial.println(F(""));
  serial.println(F("=== PatrolPro Phase2 Arduino ==="));
  serial.println(F("Serial Monitor baud: 115200"));
  serial.println(F("Utility commands:"));
  serial.println(F("  h  : print help"));
  serial.println(F("  o  : print one sensor snapshot"));
  serial.println(F("  p  : pause/resume sensor streaming"));
  serial.println(F("  b  : short buzzer beep"));
  serial.println(F("  l  : LED color test"));
  serial.println(F("  v  : servo sweep test"));
  serial.println(F("  c  : center camera servo"));
  serial.println(F("  E:n:r:g:b      : set external LED n, 1-200"));
  serial.println(F("  ER:a:b:r:g:b   : set external LED range a-b"));
  serial.println(F("  EA:r:g:b       : set all external LEDs"));
  serial.println(F("  EOFF           : turn off external LEDs"));
  serial.println(F(""));
  serial.println(F("Drive commands:"));
  serial.println(F("  MANUAL         : stop auto patrol, accept timed drive commands"));
  serial.println(F("  AUTO           : resume automatic patrol"));
  serial.println(F("  STOP           : stop wheels"));
  serial.println(F("  EMERGENCY_STOP : latch motors off until AUTO or reset"));
  serial.println(F("  F 1000         : forward for 1000 ms"));
  serial.println(F("  B 500          : backward for 500 ms, white reverse lights"));
  serial.println(F("  L 800          : rotate left for 800 ms"));
  serial.println(F("  R 800          : rotate right for 800 ms"));
  serial.println(F("  D3             : approximate 3 meter forward drive"));
  serial.println(F(""));
  serial.println(F("Friend Jetson commands:"));
  serial.println(F("  STATUS:CLEAR"));
  serial.println(F("  STATUS:T0:SCANNING:LEFT,T1:VERIFIED:Name,T2:UNKNOWN"));
  serial.println(F("  PERSON_DETECTED"));
  serial.println(F("  FACE_VERIFIED:<name>"));
  serial.println(F("  FACE_UNKNOWN"));
  serial.println(F("  FACE_TIMEOUT"));
  serial.println(F("  ARRIVAL_REACHED"));
  serial.println(F("  EMERGENCY_STOP"));
  serial.println(F("  HEARTBEAT"));
  serial.println(F(""));
}

SerialCommand SerialProtocol::parseLine(char* line) const
{
  SerialCommand command;
  trim(line);
  if (line[0] == '\0')
  {
    return command;
  }

  if (strcmp(line, "h") == 0) { command.type = SerialCommandType::Help; return command; }
  if (strcmp(line, "o") == 0) { command.type = SerialCommandType::Snapshot; return command; }
  if (strcmp(line, "p") == 0) { command.type = SerialCommandType::ToggleStream; return command; }
  if (strcmp(line, "b") == 0) { command.type = SerialCommandType::Beep; return command; }
  if (strcmp(line, "l") == 0) { command.type = SerialCommandType::LedTest; return command; }
  if (strcmp(line, "v") == 0) { command.type = SerialCommandType::ServoSweep; return command; }
  if (strcmp(line, "c") == 0) { command.type = SerialCommandType::ServoCenter; return command; }
  if (equalsIgnoreCase(line, "EOFF")) { command.type = SerialCommandType::ExternalLedOff; return command; }
  if (equalsIgnoreCase(line, "STOP")) { command.type = SerialCommandType::DriveStop; return command; }
  if (equalsIgnoreCase(line, "EMERGENCY_STOP") || equalsIgnoreCase(line, "ESTOP"))
  {
    command.type = SerialCommandType::EmergencyStop;
    return command;
  }
  if (equalsIgnoreCase(line, "AUTO")) { command.type = SerialCommandType::AutoPatrol; return command; }
  if (equalsIgnoreCase(line, "MANUAL")) { command.type = SerialCommandType::ManualDrive; return command; }
  if (equalsIgnoreCase(line, "D3")) { command.type = SerialCommandType::DriveThreeMeters; return command; }

  if (strncmp(line, "E:", 2) == 0)
  {
    command.type = SerialCommandType::ExternalLedSet;
    strncpy(command.arg, line + 2, sizeof(command.arg) - 1);
    return command;
  }
  if (strncmp(line, "ER:", 3) == 0)
  {
    command.type = SerialCommandType::ExternalLedRange;
    strncpy(command.arg, line + 3, sizeof(command.arg) - 1);
    return command;
  }
  if (strncmp(line, "EA:", 3) == 0)
  {
    command.type = SerialCommandType::ExternalLedAll;
    strncpy(command.arg, line + 3, sizeof(command.arg) - 1);
    return command;
  }
  if (strncmp(line, "STATUS:", 7) == 0)
  {
    command.type = SerialCommandType::Status;
    strncpy(command.arg, line + 7, sizeof(command.arg) - 1);
    trim(command.arg);
    return command;
  }

  const char driveShortcut = (char)toupper((unsigned char)line[0]);
  if ((driveShortcut == 'F' || driveShortcut == 'B' || driveShortcut == 'L' || driveShortcut == 'R') &&
      (line[1] == '\0' || line[1] == ' ' || line[1] == ':' || isdigit((unsigned char)line[1])))
  {
    switch (driveShortcut)
    {
      case 'F': command.type = SerialCommandType::DriveForward; break;
      case 'B': command.type = SerialCommandType::DriveBackward; break;
      case 'L': command.type = SerialCommandType::TurnLeft; break;
      case 'R': command.type = SerialCommandType::TurnRight; break;
    }

    const char* arg = line + 1;
    while (*arg == ' ' || *arg == ':')
    {
      ++arg;
    }
    strncpy(command.arg, arg, sizeof(command.arg) - 1);
    trim(command.arg);
    return command;
  }

  if (equalsIgnoreCase(line, "PERSON_DETECTED"))
  {
    command.type = SerialCommandType::PersonDetected;
    return command;
  }
  if (equalsIgnoreCase(line, "FACE_UNKNOWN"))
  {
    command.type = SerialCommandType::FaceUnknown;
    return command;
  }
  if (equalsIgnoreCase(line, "FACE_TIMEOUT"))
  {
    command.type = SerialCommandType::FaceTimeout;
    return command;
  }
  if (equalsIgnoreCase(line, "ARRIVAL_REACHED") || equalsIgnoreCase(line, "ARRIVED"))
  {
    command.type = SerialCommandType::ArrivalReached;
    return command;
  }
  if (equalsIgnoreCase(line, "HEARTBEAT"))
  {
    command.type = SerialCommandType::Heartbeat;
    return command;
  }

  if (strncmp(line, "FACE_VERIFIED:", 14) == 0)
  {
    command.type = SerialCommandType::FaceVerified;
    strncpy(command.arg, line + 14, sizeof(command.arg) - 1);
    trim(command.arg);
    return command;
  }

  command.type = SerialCommandType::None;
  return command;
}

void SerialProtocol::trim(char* text) const
{
  if (!text)
  {
    return;
  }

  char* start = text;
  while (*start && isspace((unsigned char)*start))
  {
    ++start;
  }

  if (start != text)
  {
    memmove(text, start, strlen(start) + 1);
  }

  size_t len = strlen(text);
  while (len > 0 && isspace((unsigned char)text[len - 1]))
  {
    text[--len] = '\0';
  }
}

bool SerialProtocol::equalsIgnoreCase(const char* a, const char* b) const
{
  while (*a && *b)
  {
    if (toupper((unsigned char)*a) != toupper((unsigned char)*b))
    {
      return false;
    }
    ++a;
    ++b;
  }
  return *a == '\0' && *b == '\0';
}
