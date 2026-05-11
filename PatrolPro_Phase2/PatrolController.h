#pragma once

#include <Arduino.h>

#include "MotorControl.h"
#include "SensorSuite.h"
#include "SerialProtocol.h"
#include "StatusOutputs.h"

class PatrolController
{
public:
  PatrolController(MotorControl& motors, SensorSuite& sensors, StatusOutputs& outputs, Stream& serialOut, Stream& bluetoothOut);

  void begin(unsigned long nowMs);
  void update(unsigned long nowMs);
  void handleCommand(const SerialCommand& command, unsigned long nowMs);
  const char* modeName() const;

private:
  enum class Mode : uint8_t
  {
    Patrol,
    FireAlert,
    Verification,
    SecurityAlert,
    VerifiedPause,
    ManualDrive,
    ArrivalStop,
    EmergencyStop,
  };

  enum class PatrolSubState : uint8_t
  {
    Forward,
    Backup,
    Turning,
  };

  enum class TurnDirection : uint8_t
  {
    Left,
    Right,
  };

  enum class ManualMotion : uint8_t
  {
    Idle,
    Forward,
    Backward,
    Left,
    Right,
  };

  enum class AlignmentDirection : uint8_t
  {
    Center,
    Left,
    Right,
  };

  enum class VerificationStage : uint8_t
  {
    BackingUp,
    BackupSettle,
    InitialWait,
    AudioWait,
    ServoRaising,
    ServoHolding,
    ServoReturning,
    PostScanWait,
  };

  enum class FireStage : uint8_t
  {
    PreTurnSettle,
    Orienting,
    Holding,
    Restoring,
  };

  enum class FireTurn : uint8_t
  {
    None,
    Left90,
    Right90,
    Right180,
  };

  MotorControl& motors_;
  SensorSuite& sensors_;
  StatusOutputs& outputs_;
  Stream& serialOut_;
  Stream& bluetoothOut_;

  Mode mode_ = Mode::Patrol;
  Mode preFireMode_ = Mode::Patrol;
  PatrolSubState patrolSubState_ = PatrolSubState::Forward;
  VerificationStage verificationStage_ = VerificationStage::InitialWait;
  FireStage fireStage_ = FireStage::Holding;
  FireTurn fireTurn_ = FireTurn::None;

  unsigned long modeStartMs_ = 0;
  unsigned long stageStartMs_ = 0;
  unsigned long patrolBackupStartMs_ = 0;
  unsigned long patrolTurnStartMs_ = 0;
  unsigned long manualMotionEndMs_ = 0;
  unsigned long fireStageStartMs_ = 0;
  unsigned long fireTurnMs_ = 0;
  unsigned long fireClearStartMs_ = 0;
  unsigned long fireSnapshotAtMs_ = 0;
  unsigned long lastBlinkMs_ = 0;
  bool blinkOn_ = false;
  bool verificationPendingResolution_ = false;
  bool arrivalStopPending_ = false;
  bool fireSnapshotSent_ = false;
  bool fireClearCandidate_ = false;
  uint8_t frontImmediateCount_ = 0;
  TurnDirection avoidanceTurnDirection_ = TurnDirection::Right;
  ManualMotion manualMotion_ = ManualMotion::Idle;
  AlignmentDirection verificationAlignDirection_ = AlignmentDirection::Center;

  char verifiedName_[40] = {0};
  char alertHazardType_[16] = "NONE";
  char alertDirection_[24] = "NONE";

  void enterPatrol(unsigned long nowMs);
  void enterFireAlert(unsigned long nowMs, const char* hazardType, const char* direction);
  void enterVerification(unsigned long nowMs, AlignmentDirection alignment = AlignmentDirection::Center);
  void enterSecurityAlert(unsigned long nowMs);
  void enterVerifiedPause(unsigned long nowMs, const char* name);
  void enterManualDrive(unsigned long nowMs);
  void enterArrivalStop(unsigned long nowMs);
  void enterEmergencyStop(unsigned long nowMs);
  void resumeVerification(unsigned long nowMs);

  void runPatrol(unsigned long nowMs);
  void runFireAlert(unsigned long nowMs);
  void runVerification(unsigned long nowMs);
  void runSecurityAlert(unsigned long nowMs);
  void runVerifiedPause(unsigned long nowMs);
  void runManualDrive(unsigned long nowMs);
  void runArrivalStop(unsigned long nowMs);
  void runEmergencyStop(unsigned long nowMs);

  void startAvoidance(unsigned long nowMs);
  void startAvoidanceTurn(unsigned long nowMs, TurnDirection direction);
  TurnDirection chooseTurnDirection(float leftCm, float rightCm) const;
  FireTurn chooseFireTurn(const char* direction) const;
  FireTurn inverseFireTurn(FireTurn turn) const;
  unsigned long fireTurnDurationMs(FireTurn turn) const;
  void startFireTurn(FireTurn turn, bool restoring);
  void enterFireHolding(unsigned long nowMs);
  bool fireClearStable(unsigned long nowMs);
  void resumeAfterFire(unsigned long nowMs);
  void startVerificationPreparation(unsigned long nowMs);
  void startVerificationScanTimer(unsigned long nowMs);
  bool isClearDistance(float distanceCm, float thresholdCm) const;
  bool isValidDistance(float distanceCm) const;
  void startManualMotion(ManualMotion motion, unsigned long durationMs, unsigned long nowMs);
  unsigned long parseDriveDurationMs(const char* arg, unsigned long fallbackMs) const;

  void sendMode(const char* mode);
  void sendPlay(const char* clip);
  void sendBluetoothFire(unsigned long nowMs);
  void sendBluetoothSecurity(unsigned long nowMs);
  void handleStatusPayload(const char* payload, unsigned long nowMs);
  AlignmentDirection parseScanningAlignment(const char* payload) const;
  bool buildVerifiedNameList(const char* payload, char* dest, size_t destSize) const;
  void printHazardSummary();
  void copyText(char* dest, size_t destSize, const char* src);
};
