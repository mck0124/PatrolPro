#pragma once

#include <Arduino.h>
#include <FastLED.h>

#include "Config.h"

class StatusOutputs
{
public:
  void begin();
  void update(unsigned long nowMs);

  void setLed(const CRGB& color);
  void setLedWhite();
  void setLedBlack();
  void setLedRed();
  void setLedOrange();
  void setLedGreen();
  void startLedScanner();
  void setExternalLedBoards(const CRGB& firstBoardColor, const CRGB& secondBoardColor);
  bool setExternalLed(uint16_t ledNumber, const CRGB& color);
  bool setExternalLedRange(uint16_t firstLedNumber, uint16_t lastLedNumber, const CRGB& color);
  void setExternalLedAll(const CRGB& color);
  void clearExternalLeds();
  void showTailLights();
  void showBrakeLights();
  void showReverseLights();
  void showLeftTurnSignal(bool signalOn);
  void showRightTurnSignal(bool signalOn);
  void showTopPatrol();
  void showTopScanning();
  void showTopVerified();
  void showTopUnknown();
  void showTopFire();

  void beep(uint16_t frequencyHz, unsigned long durationMs);
  void silenceBuzzer();

  void showNormal();
  void showFireAlert(bool visible, const char* hazardType, const char* direction);
  void showVerification();
  void showSecurityAlert(bool visible);
  void showVerified(const char* name);
  void showArrivalReached();
  void showEmergencyStop();

  void centerServo();
  void tiltServoTo(int angle, bool holdAfterMove = false);
  void detachServo();
  bool servoAtTarget() const;

  void runLedTest();
  void runServoSweep();
  bool oledReady() const;

private:
  bool oledReady_ = false;
  uint8_t activeOledAddress_ = 0;

  bool servoAttached_ = false;
  bool servoHoldAfterMove_ = false;
  int currentServoDeg_ = Config::kServoCenterDeg;
  int targetServoDeg_ = Config::kServoCenterDeg;
  unsigned long lastServoStepMs_ = 0;
  unsigned long servoTargetReachedMs_ = 0;

  bool beginOledAtAddress(uint8_t address);
  bool isI2cPresent(uint8_t address);
  void attachServoIfNeeded();
  void writeServoNow(int angle);
};
