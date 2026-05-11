#include "Config.h"
#include "MotorControl.h"
#include "PatrolController.h"
#include "SensorSuite.h"
#include "SerialProtocol.h"
#include "StatusOutputs.h"

#include <stdio.h>

MotorControl motors;
SensorSuite sensors;
StatusOutputs outputs;
SerialProtocol protocol;
PatrolController controller(motors, sensors, outputs, Serial, Serial3);

bool streamEnabled = false;
unsigned long lastStreamMs = 0;

bool validByteValue(int value)
{
  return value >= 0 && value <= 255;
}

CRGB makeColor(int red, int green, int blue)
{
  return CRGB((uint8_t)red, (uint8_t)green, (uint8_t)blue);
}

void handleExternalLedSet(const char* arg)
{
  int led = 0;
  int red = 0;
  int green = 0;
  int blue = 0;

  if (sscanf(arg, "%d:%d:%d:%d", &led, &red, &green, &blue) != 4 ||
      !validByteValue(red) || !validByteValue(green) || !validByteValue(blue) ||
      !outputs.setExternalLed((uint16_t)led, makeColor(red, green, blue)))
  {
    Serial.println(F("[LED] invalid command. Use E:index:r:g:b, index 1-200, RGB 0-255"));
    return;
  }

  Serial.print(F("[LED] external #"));
  Serial.print(led);
  Serial.println(F(" updated"));
}

void handleExternalLedRange(const char* arg)
{
  int first = 0;
  int last = 0;
  int red = 0;
  int green = 0;
  int blue = 0;

  if (sscanf(arg, "%d:%d:%d:%d:%d", &first, &last, &red, &green, &blue) != 5 ||
      !validByteValue(red) || !validByteValue(green) || !validByteValue(blue) ||
      !outputs.setExternalLedRange((uint16_t)first, (uint16_t)last, makeColor(red, green, blue)))
  {
    Serial.println(F("[LED] invalid command. Use ER:first:last:r:g:b, LED 1-200, RGB 0-255"));
    return;
  }

  Serial.print(F("[LED] external range "));
  Serial.print(first);
  Serial.print(F("-"));
  Serial.print(last);
  Serial.println(F(" updated"));
}

void handleExternalLedAll(const char* arg)
{
  int red = 0;
  int green = 0;
  int blue = 0;

  if (sscanf(arg, "%d:%d:%d", &red, &green, &blue) != 3 ||
      !validByteValue(red) || !validByteValue(green) || !validByteValue(blue))
  {
    Serial.println(F("[LED] invalid command. Use EA:r:g:b, RGB 0-255"));
    return;
  }

  outputs.setExternalLedAll(makeColor(red, green, blue));
  Serial.println(F("[LED] all external LEDs updated"));
}

void handleCommand(const SerialCommand& command, unsigned long nowMs)
{
  switch (command.type)
  {
    case SerialCommandType::Help:
      protocol.printHelp(Serial);
      break;

    case SerialCommandType::Snapshot:
      sensors.printSnapshot(Serial);
      break;

    case SerialCommandType::ToggleStream:
      streamEnabled = !streamEnabled;
      Serial.print(F("[STREAM] "));
      Serial.println(streamEnabled ? F("ON") : F("OFF"));
      break;

    case SerialCommandType::Beep:
      outputs.beep(2200, 150);
      Serial.println(F("[BUZZER] short beep"));
      break;

    case SerialCommandType::LedTest:
      outputs.runLedTest();
      outputs.setLedWhite();
      outputs.showNormal();
      break;

    case SerialCommandType::ServoSweep:
      outputs.runServoSweep();
      break;

    case SerialCommandType::ServoCenter:
      outputs.centerServo();
      Serial.print(F("[SERVO] centered at "));
      Serial.print(Config::kServoCenterDeg);
      Serial.println(F(" deg on D39"));
      break;

    case SerialCommandType::ExternalLedSet:
      handleExternalLedSet(command.arg);
      break;

    case SerialCommandType::ExternalLedRange:
      handleExternalLedRange(command.arg);
      break;

    case SerialCommandType::ExternalLedAll:
      handleExternalLedAll(command.arg);
      break;

    case SerialCommandType::ExternalLedOff:
      outputs.clearExternalLeds();
      Serial.println(F("[LED] external LEDs off"));
      break;

    default:
      controller.handleCommand(command, nowMs);
      break;
  }
}

void setup()
{
  Serial.begin(Config::kSerialBaud);
  Serial3.begin(Config::kBluetoothBaud);

  sensors.begin();
  motors.begin();
  outputs.begin();

  delay(300);
  outputs.runLedTest();
  outputs.setLedWhite();
  outputs.showNormal();

  sensors.update(millis(), true);
  protocol.printHelp(Serial);
  sensors.printSnapshot(Serial);

  controller.begin(millis());
}

void loop()
{
  const unsigned long nowMs = millis();

  SerialCommand command;
  while (protocol.read(Serial, command))
  {
    handleCommand(command, nowMs);
  }

  outputs.update(nowMs);
  controller.update(nowMs);

  if (streamEnabled && (nowMs - lastStreamMs) >= Config::kStreamIntervalMs)
  {
    lastStreamMs = nowMs;
    sensors.printSnapshot(Serial);
  }
}
