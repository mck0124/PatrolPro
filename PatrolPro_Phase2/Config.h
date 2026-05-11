#pragma once

#include <Arduino.h>

namespace Config
{
constexpr unsigned long kSerialBaud = 115200UL;
constexpr unsigned long kBluetoothBaud = 9600UL;
constexpr bool kStartInPatrol = true;   // start autonomous patrol after boot/reset

constexpr unsigned long kStreamIntervalMs = 500UL;
constexpr int kAnalogSamples = 6;
constexpr bool kMotorDriveEnabled = true;

constexpr unsigned long kUltraTimeoutUs = 25000UL;
constexpr float kUltraMinCm = 2.0f;
constexpr float kUltraMaxCm = 400.0f;
constexpr float kObstacleLedCm = 20.0f;

constexpr uint8_t kBuzzerPin = 41;    // PG0
constexpr uint8_t kLedDataPin = 24;   // WS2812B data
constexpr uint8_t kLedCount = 24;
constexpr uint8_t kLedBrightness = 96;
constexpr uint8_t kExternalLedDataPin = 17;
constexpr uint16_t kExternalLedCount = 200;
constexpr uint16_t kExternalLedBoardSize = 100;  // LED 1-100 first board, 101-200 second board
constexpr uint8_t kExternalLedNormalLevel = 8;   // keep 200 LEDs from browning out USB power
constexpr uint8_t kExternalLedAlertLevel = 48;

constexpr uint8_t kServoPin = 39;
constexpr int kServoCenterDeg = 100;
constexpr int kServoUpDeg = 140;
constexpr int kServoVerifyMaxDeg = 130;
constexpr int kServoStepDeg = 2;
constexpr unsigned long kServoStepDelayMs = 55UL;
constexpr unsigned long kServoDetachDelayMs = 700UL;

constexpr uint8_t kOledWidth = 128;
constexpr uint8_t kOledHeight = 64;
constexpr int8_t kOledReset = -1;
constexpr uint8_t kOledPrimaryAddress = 0x3C;
constexpr uint8_t kOledFallbackAddress = 0x3D;

constexpr int kSideFlameThreshold = 300;  // side/back IR: lower = flame
constexpr int kFrontActiveThreshold = 300;
constexpr int kFrontStrongThreshold = 400;
constexpr int kGasThreshold = 150;
constexpr int kGasStableCount = 3;
constexpr int kFrontStableCount = 8;
constexpr unsigned long kHazardSampleIntervalMs = 100UL;
constexpr unsigned long kAlertBlinkMs = 50UL;
constexpr unsigned long kTurnSignalBlinkMs = 400UL;
constexpr unsigned long kStatusScanFrameMs = 70UL;
constexpr unsigned long kRearPatrolFrameMs = 180UL;
constexpr unsigned long kRearScanFrameMs = 120UL;
constexpr unsigned long kRearFireBlinkMs = 140UL;
constexpr unsigned long kTopSpinnerFrameMs = 80UL;
constexpr unsigned long kTopEyeFrameMs = 250UL;

constexpr unsigned long kFirePreTurnSettleMs = 500UL;
constexpr unsigned long kFireSnapshotSettleMs = 1000UL;
constexpr unsigned long kFirePostSnapshotHoldMs = 2000UL;
constexpr unsigned long kFireClearStableMs = 700UL;
constexpr int kPwmFireTurn = 38;
constexpr unsigned long kFireLeftSideTurnMs = 3600UL;
constexpr unsigned long kFireRightSideTurnMs = 3800UL;
constexpr unsigned long kFireRearTurnMs = 7400UL;
constexpr unsigned long kSecurityAlertDurationMs = 10000UL;
constexpr unsigned long kVerificationTimeoutMs = 8000UL;
constexpr unsigned long kVerificationBackupMs = 1400UL;
constexpr int kPwmVerificationBackup = 28;
constexpr float kVerificationBackupRearClearCm = 25.0f;
constexpr unsigned long kVerificationServoTriggerMs = 3000UL;
constexpr unsigned long kVerificationAudioWaitMs = 3000UL;
constexpr unsigned long kVerificationServoHoldMs = 3000UL;
constexpr unsigned long kVerifiedPauseMs = 2000UL;

constexpr uint8_t kM1Pwm = 12;
constexpr uint8_t kM1In1 = 34;
constexpr uint8_t kM1In2 = 35;
constexpr uint8_t kM2Pwm = 8;
constexpr uint8_t kM2In1 = 37;
constexpr uint8_t kM2In2 = 36;
constexpr uint8_t kM3Pwm = 9;
constexpr uint8_t kM3In1 = 43;
constexpr uint8_t kM3In2 = 42;
constexpr uint8_t kM4Pwm = 5;
constexpr uint8_t kM4In1 = A4;
constexpr uint8_t kM4In2 = A5;

constexpr int kMotorSign[4] = {-1, +1, -1, +1};
constexpr int kMotorTrim[4] = {0, 0, 0, 0};
constexpr int kSideBalance = 0;

constexpr int kPwmForward = 34;
constexpr int kPwmSlowForward = 24;
constexpr int kPwmBackup = 28;
constexpr int kPwmTurn90 = 28;
constexpr float kForwardRatio[4] = {0.9650f, 0.8462f, 0.9650f, 0.8462f};
constexpr float kBackwardRatio[4] = {1.0000f, 0.8462f, 1.0000f, 0.8462f};
constexpr unsigned long kTurnLeft90Ms = 2400UL;
constexpr unsigned long kTurnRight90Ms = 2550UL;
constexpr unsigned long kStopSettleMs = 150UL;
constexpr unsigned long kAvoidanceBackupMs = 500UL;
constexpr unsigned long kManualDefaultDriveMs = 1000UL;
constexpr unsigned long kManualMaxDriveMs = 10000UL;
constexpr unsigned long kDriveThreeMetersMs = 8500UL;  // time-based estimate; tune on floor

constexpr uint8_t kPatrolLeftTrig = 33;   // U1
constexpr uint8_t kPatrolLeftEcho = 32;
constexpr uint8_t kPatrolRightTrig = 28;  // U2
constexpr uint8_t kPatrolRightEcho = 25;
constexpr uint8_t kPatrolRearTrig = 46;   // U3
constexpr uint8_t kPatrolRearEcho = 13;
constexpr uint8_t kPatrolFrontTrig = 30;  // U4
constexpr uint8_t kPatrolFrontEcho = 29;
constexpr float kFrontImmediateStopCm = 20.0f;
constexpr float kFrontSlowCm = 35.0f;
constexpr float kRearBackupClearCm = 25.0f;
constexpr float kSideTurnClearCm = 25.0f;
constexpr float kSideChoiceMarginCm = 5.0f;
constexpr uint8_t kObstacleStableCount = 2;

struct MotorPins
{
  uint8_t pwm;
  uint8_t in1;
  uint8_t in2;
};

struct DualSensor
{
  const char* name;
  const char* digitalLabel;
  const char* analogLabel;
  uint8_t digitalPin;
  uint8_t analogPin;
};

struct UltrasonicSensor
{
  const char* name;
  const char* trigLabel;
  const char* echoLabel;
  uint8_t trigPin;
  uint8_t echoPin;
};

enum DualSensorIndex : uint8_t
{
  kIr1 = 0,
  kIr2,
  kIr5_1,
  kIr5_2,
  kIr5_3,
  kIr5_4,
  kIr5_5,
  kIrBack,
  kGas,
};

static const MotorPins kMotors[4] = {
  {kM1Pwm, kM1In1, kM1In2},
  {kM2Pwm, kM2In1, kM2In2},
  {kM3Pwm, kM3In1, kM3In2},
  {kM4Pwm, kM4In1, kM4In2},
};

constexpr size_t kMotorCount = sizeof(kMotors) / sizeof(kMotors[0]);
constexpr size_t kDualSensorCount = 9;
constexpr size_t kUltrasonicCount = 4;

static const DualSensor kDualSensors[kDualSensorCount] = {
  {"IR_1", "PL1/D48", "A2", 48, A2},
  {"IR_2", "PL2/D47", "A3", 47, A3},
  {"IR_5_1", "PB5/D11", "A8", 11, A8},
  {"IR_5_2", "PB4/D10", "A9", 10, A9},
  {"IR_5_3", "PH4/D7", "A10", 7, A10},
  {"IR_5_4", "PH3/D6", "A11", 6, A11},
  {"IR_5_5", "PG5/D4", "A12", 4, A12},
  {"IR_BACK", "PL4/D45", "A6", 45, A6},
  {"GAS", "PL5/D44", "A0", 44, A0},
};

static const UltrasonicSensor kUltrasonicSensors[kUltrasonicCount] = {
  {"U1", "PC4/D33", "PC5/D32", 33, 32},
  {"U2", "PA6/D28", "PA3/D25", 28, 25},
  {"U3", "PL3/D46", "PB7/D13", 46, 13},
  {"U4", "PC7/D30", "PA7/D29", 30, 29},
};
}  // namespace Config
