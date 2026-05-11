#include "PatrolController.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "Config.h"

PatrolController::PatrolController(
  MotorControl& motors,
  SensorSuite& sensors,
  StatusOutputs& outputs,
  Stream& serialOut,
  Stream& bluetoothOut)
  : motors_(motors),
    sensors_(sensors),
    outputs_(outputs),
    serialOut_(serialOut),
    bluetoothOut_(bluetoothOut)
{
}

void PatrolController::begin(unsigned long nowMs)
{
  if (Config::kStartInPatrol)
  {
    enterPatrol(nowMs);
  }
  else
  {
    enterManualDrive(nowMs);
  }
}

void PatrolController::update(unsigned long nowMs)
{
  sensors_.update(nowMs, false);
  const SensorSuite::Snapshot& snap = sensors_.snapshot();

  if (mode_ != Mode::FireAlert &&
      mode_ != Mode::ArrivalStop &&
      mode_ != Mode::EmergencyStop &&
      snap.hazardDetected)
  {
    enterFireAlert(nowMs, snap.hazardType, snap.direction);
    return;
  }

  switch (mode_)
  {
    case Mode::Patrol:        runPatrol(nowMs); break;
    case Mode::FireAlert:     runFireAlert(nowMs); break;
    case Mode::Verification:  runVerification(nowMs); break;
    case Mode::SecurityAlert: runSecurityAlert(nowMs); break;
    case Mode::VerifiedPause: runVerifiedPause(nowMs); break;
    case Mode::ManualDrive:   runManualDrive(nowMs); break;
    case Mode::ArrivalStop:   runArrivalStop(nowMs); break;
    case Mode::EmergencyStop: runEmergencyStop(nowMs); break;
  }
}

void PatrolController::handleCommand(const SerialCommand& command, unsigned long nowMs)
{
  if (command.type == SerialCommandType::EmergencyStop)
  {
    enterEmergencyStop(nowMs);
    return;
  }

  if (mode_ == Mode::EmergencyStop)
  {
    if (command.type == SerialCommandType::AutoPatrol)
    {
      enterPatrol(nowMs);
    }
    else if (command.type != SerialCommandType::Heartbeat &&
             command.type != SerialCommandType::Status)
    {
      serialOut_.println(F("[ESTOP] locked; send AUTO to resume"));
    }
    return;
  }

  switch (command.type)
  {
    case SerialCommandType::Status:
      handleStatusPayload(command.arg, nowMs);
      break;

    case SerialCommandType::PersonDetected:
      serialOut_.println(F("[JETSON] PERSON_DETECTED"));
      if (mode_ == Mode::Patrol)
      {
        enterVerification(nowMs);
      }
      break;

    case SerialCommandType::FaceVerified:
      serialOut_.print(F("[JETSON] FACE_VERIFIED: "));
      serialOut_.println(command.arg);
      if (mode_ == Mode::Verification)
      {
        enterVerifiedPause(nowMs, command.arg);
      }
      break;

    case SerialCommandType::FaceUnknown:
      serialOut_.println(F("[JETSON] FACE_UNKNOWN"));
      if (mode_ == Mode::Verification)
      {
        enterSecurityAlert(nowMs);
      }
      break;

    case SerialCommandType::FaceTimeout:
      serialOut_.println(F("[JETSON] FACE_TIMEOUT"));
      if (mode_ == Mode::Verification)
      {
        enterSecurityAlert(nowMs);
      }
      break;

    case SerialCommandType::ArrivalReached:
      serialOut_.println(F("[JETSON] ARRIVAL_REACHED"));
      enterArrivalStop(nowMs);
      break;

    case SerialCommandType::Heartbeat:
      break;

    case SerialCommandType::ManualDrive:
      enterManualDrive(nowMs);
      break;

    case SerialCommandType::AutoPatrol:
      enterPatrol(nowMs);
      break;

    case SerialCommandType::DriveStop:
      enterManualDrive(nowMs);
      motors_.stop();
      outputs_.showBrakeLights();
      serialOut_.println(F("[DRIVE] STOP"));
      break;

    case SerialCommandType::DriveForward:
      startManualMotion(ManualMotion::Forward, parseDriveDurationMs(command.arg, Config::kManualDefaultDriveMs), nowMs);
      break;

    case SerialCommandType::DriveBackward:
      startManualMotion(ManualMotion::Backward, parseDriveDurationMs(command.arg, Config::kManualDefaultDriveMs), nowMs);
      break;

    case SerialCommandType::TurnLeft:
      startManualMotion(ManualMotion::Left, parseDriveDurationMs(command.arg, Config::kManualDefaultDriveMs), nowMs);
      break;

    case SerialCommandType::TurnRight:
      startManualMotion(ManualMotion::Right, parseDriveDurationMs(command.arg, Config::kManualDefaultDriveMs), nowMs);
      break;

    case SerialCommandType::DriveThreeMeters:
      startManualMotion(ManualMotion::Forward, Config::kDriveThreeMetersMs, nowMs);
      break;

    default:
      break;
  }
}

const char* PatrolController::modeName() const
{
  switch (mode_)
  {
    case Mode::Patrol:        return "PATROL";
    case Mode::FireAlert:     return "FIRE_ALERT";
    case Mode::Verification:  return "VERIFICATION";
    case Mode::SecurityAlert: return "SECURITY_ALERT";
    case Mode::VerifiedPause: return "VERIFIED";
    case Mode::ManualDrive:   return "MANUAL";
    case Mode::ArrivalStop:   return "ARRIVED";
    case Mode::EmergencyStop: return "EMERGENCY_STOP";
  }
  return "UNKNOWN";
}

void PatrolController::enterPatrol(unsigned long nowMs)
{
  const bool wasScanning = mode_ == Mode::Verification ||
                           mode_ == Mode::VerifiedPause ||
                           preFireMode_ == Mode::Verification;
  mode_ = Mode::Patrol;
  verificationPendingResolution_ = false;
  arrivalStopPending_ = false;
  manualMotion_ = ManualMotion::Idle;
  manualMotionEndMs_ = 0;
  frontImmediateCount_ = 0;
  modeStartMs_ = nowMs;
  patrolSubState_ = PatrolSubState::Forward;
  blinkOn_ = false;
  outputs_.silenceBuzzer();
  outputs_.setLedWhite();
  if (wasScanning)
  {
    outputs_.tiltServoTo(Config::kServoCenterDeg, true);
  }
  preFireMode_ = Mode::Patrol;
  outputs_.showTailLights();
  outputs_.showTopPatrol();
  outputs_.showNormal();
  serialOut_.println(F("[MODE] PATROL"));
  sendMode("PATROL");
}

void PatrolController::enterFireAlert(unsigned long nowMs, const char* hazardType, const char* direction)
{
  preFireMode_ = mode_;
  mode_ = Mode::FireAlert;
  manualMotion_ = ManualMotion::Idle;
  manualMotionEndMs_ = 0;
  modeStartMs_ = nowMs;
  fireStageStartMs_ = nowMs;
  fireTurn_ = chooseFireTurn(direction);
  fireTurnMs_ = fireTurnDurationMs(fireTurn_);
  fireSnapshotSent_ = false;
  fireSnapshotAtMs_ = 0;
  fireClearCandidate_ = false;
  fireClearStartMs_ = 0;
  lastBlinkMs_ = 0;
  blinkOn_ = false;
  motors_.stop();
  outputs_.detachServo();
  outputs_.showBrakeLights();
  outputs_.showTopFire();
  copyText(alertHazardType_, sizeof(alertHazardType_), hazardType);
  copyText(alertDirection_, sizeof(alertDirection_), direction);

  serialOut_.println(F("[MODE] FIRE_ALERT"));
  sendMode("FIRE_ALERT");
  printHazardSummary();
  sendPlay("alert_fire");
  sendBluetoothFire(nowMs);

  if (fireTurn_ == FireTurn::None)
  {
    enterFireHolding(nowMs);
  }
  else
  {
    fireStage_ = FireStage::PreTurnSettle;
    serialOut_.println(F("[FIRE] stopped before orientation turn"));
  }
}

void PatrolController::enterVerification(unsigned long nowMs, AlignmentDirection alignment)
{
  mode_ = Mode::Verification;
  manualMotion_ = ManualMotion::Idle;
  manualMotionEndMs_ = 0;
  verificationPendingResolution_ = false;
  arrivalStopPending_ = false;
  verificationAlignDirection_ = alignment;
  modeStartMs_ = nowMs;
  stageStartMs_ = nowMs;
  verificationStage_ = VerificationStage::InitialWait;
  motors_.stop();
  outputs_.showBrakeLights();
  outputs_.showTopScanning();
  outputs_.silenceBuzzer();
  outputs_.centerServo();
  outputs_.startLedScanner();
  outputs_.showVerification();

  serialOut_.println(F("[MODE] VERIFICATION"));
  sendMode("VERIFICATION");
  startVerificationPreparation(nowMs);
}

void PatrolController::enterSecurityAlert(unsigned long nowMs)
{
  mode_ = Mode::SecurityAlert;
  manualMotion_ = ManualMotion::Idle;
  manualMotionEndMs_ = 0;
  verificationPendingResolution_ = false;
  modeStartMs_ = nowMs;
  lastBlinkMs_ = 0;
  blinkOn_ = false;
  motors_.stop();
  outputs_.showBrakeLights();
  outputs_.showTopUnknown();
  outputs_.centerServo();
  outputs_.detachServo();
  serialOut_.println(F("[MODE] SECURITY_ALERT"));
  sendMode("SECURITY_ALERT");
  sendPlay("alert_intruder");
  sendBluetoothSecurity(nowMs);
}

void PatrolController::enterVerifiedPause(unsigned long nowMs, const char* name)
{
  mode_ = Mode::VerifiedPause;
  manualMotion_ = ManualMotion::Idle;
  manualMotionEndMs_ = 0;
  verificationPendingResolution_ = false;
  modeStartMs_ = nowMs;
  motors_.stop();
  outputs_.showBrakeLights();
  outputs_.showTopVerified();
  outputs_.silenceBuzzer();
  outputs_.setLedGreen();
  outputs_.tiltServoTo(Config::kServoCenterDeg, true);
  copyText(verifiedName_, sizeof(verifiedName_), name && name[0] ? name : "registered");
  outputs_.showVerified(verifiedName_);
  serialOut_.println(F("[MODE] VERIFIED"));
  sendMode("VERIFIED");
  serialOut_.print(F("PLAY:verified_"));
  serialOut_.println(verifiedName_);
}

void PatrolController::enterManualDrive(unsigned long nowMs)
{
  mode_ = Mode::ManualDrive;
  verificationPendingResolution_ = false;
  arrivalStopPending_ = false;
  manualMotion_ = ManualMotion::Idle;
  manualMotionEndMs_ = 0;
  modeStartMs_ = nowMs;
  lastBlinkMs_ = nowMs;
  blinkOn_ = false;
  motors_.stop();
  outputs_.showBrakeLights();
  outputs_.showTopPatrol();
  outputs_.showNormal();
  serialOut_.println(F("[MODE] MANUAL"));
  sendMode("MANUAL");
}

void PatrolController::enterArrivalStop(unsigned long nowMs)
{
  mode_ = Mode::ArrivalStop;
  preFireMode_ = Mode::ArrivalStop;
  verificationPendingResolution_ = false;
  arrivalStopPending_ = false;
  manualMotion_ = ManualMotion::Idle;
  manualMotionEndMs_ = 0;
  modeStartMs_ = nowMs;
  motors_.stop();
  outputs_.silenceBuzzer();
  outputs_.showBrakeLights();
  outputs_.showTopVerified();
  outputs_.setLedGreen();
  outputs_.centerServo();
  outputs_.detachServo();
  outputs_.showArrivalReached();
  serialOut_.println(F("[MODE] ARRIVED"));
  serialOut_.println(F("[ARRIVAL] route marker reached; send AUTO to resume"));
  sendMode("ARRIVED");
}

void PatrolController::enterEmergencyStop(unsigned long nowMs)
{
  mode_ = Mode::EmergencyStop;
  verificationPendingResolution_ = false;
  arrivalStopPending_ = false;
  manualMotion_ = ManualMotion::Idle;
  manualMotionEndMs_ = 0;
  modeStartMs_ = nowMs;
  lastBlinkMs_ = nowMs;
  blinkOn_ = true;
  motors_.stop();
  outputs_.detachServo();
  outputs_.silenceBuzzer();
  outputs_.showBrakeLights();
  outputs_.showTopFire();
  outputs_.setLedRed();
  outputs_.showEmergencyStop();
  serialOut_.println(F("[MODE] EMERGENCY_STOP"));
  serialOut_.println(F("[ESTOP] motors locked off; send AUTO to resume"));
  sendMode("EMERGENCY_STOP");
}

void PatrolController::resumeVerification(unsigned long nowMs)
{
  mode_ = Mode::Verification;
  modeStartMs_ = nowMs;
  stageStartMs_ = nowMs;
  verificationStage_ = VerificationStage::PostScanWait;
  motors_.stop();
  outputs_.showBrakeLights();
  outputs_.showTopScanning();
  outputs_.silenceBuzzer();
  outputs_.centerServo();
  outputs_.startLedScanner();
  outputs_.showVerification();
  serialOut_.println(F("[MODE] VERIFICATION resumed after fire alert"));
  sendMode("VERIFICATION");
}

void PatrolController::runPatrol(unsigned long nowMs)
{
  const SensorSuite::Snapshot& snap = sensors_.snapshot();
  if (snap.hazardDetected)
  {
    enterFireAlert(nowMs, snap.hazardType, snap.direction);
    return;
  }

  if (patrolSubState_ == PatrolSubState::Forward)
  {
    const float frontCm = sensors_.readPatrolFrontCm();
    if (isValidDistance(frontCm) && frontCm <= Config::kFrontImmediateStopCm)
    {
      motors_.stop();
      outputs_.showBrakeLights();
      if (frontImmediateCount_ < 255)
      {
        ++frontImmediateCount_;
      }

      if (frontImmediateCount_ >= Config::kObstacleStableCount)
      {
        startAvoidance(nowMs);
      }
    }
    else if (isValidDistance(frontCm) && frontCm <= Config::kFrontSlowCm)
    {
      frontImmediateCount_ = 0;
      motors_.driveForward(Config::kPwmSlowForward);
      outputs_.showTailLights();
    }
    else
    {
      frontImmediateCount_ = 0;
      motors_.driveForward(Config::kPwmForward);
      outputs_.showTailLights();
    }
  }
  else if (patrolSubState_ == PatrolSubState::Backup)
  {
    const float rearCm = sensors_.readPatrolRearCm();
    if (isValidDistance(rearCm) && rearCm <= Config::kFrontImmediateStopCm)
    {
      motors_.stop();
      outputs_.showBrakeLights();
      startAvoidanceTurn(nowMs, avoidanceTurnDirection_);
      return;
    }

    if (nowMs - patrolBackupStartMs_ >= Config::kAvoidanceBackupMs)
    {
      motors_.stop();
      outputs_.showBrakeLights();
      startAvoidanceTurn(nowMs, avoidanceTurnDirection_);
    }
  }
  else
  {
    if (nowMs - lastBlinkMs_ >= Config::kTurnSignalBlinkMs)
    {
      lastBlinkMs_ = nowMs;
      blinkOn_ = !blinkOn_;
      if (avoidanceTurnDirection_ == TurnDirection::Left)
      {
        outputs_.showLeftTurnSignal(blinkOn_);
      }
      else
      {
        outputs_.showRightTurnSignal(blinkOn_);
      }
    }

    const unsigned long turnMs = avoidanceTurnDirection_ == TurnDirection::Left
      ? Config::kTurnLeft90Ms
      : Config::kTurnRight90Ms;

    if (nowMs - patrolTurnStartMs_ >= turnMs)
    {
      motors_.stop();
      outputs_.showTailLights();
      outputs_.showTopPatrol();
      patrolSubState_ = PatrolSubState::Forward;
      frontImmediateCount_ = 0;
    }
  }
}

void PatrolController::startAvoidance(unsigned long nowMs)
{
  const float leftCm = sensors_.readPatrolLeftCm();
  const float rightCm = sensors_.readPatrolRightCm();
  const float rearCm = sensors_.readPatrolRearCm();
  avoidanceTurnDirection_ = chooseTurnDirection(leftCm, rightCm);

  serialOut_.print(F("[NAV] obstacle: left="));
  serialOut_.print(leftCm, 1);
  serialOut_.print(F(" right="));
  serialOut_.print(rightCm, 1);
  serialOut_.print(F(" rear="));
  serialOut_.println(rearCm, 1);

  if (isClearDistance(rearCm, Config::kRearBackupClearCm))
  {
    patrolSubState_ = PatrolSubState::Backup;
    patrolBackupStartMs_ = nowMs;
    motors_.driveBackward(Config::kPwmBackup);
    outputs_.showReverseLights();
    serialOut_.println(F("[NAV] BACKUP before turn"));
  }
  else
  {
    startAvoidanceTurn(nowMs, avoidanceTurnDirection_);
  }
}

void PatrolController::startAvoidanceTurn(unsigned long nowMs, TurnDirection direction)
{
  patrolSubState_ = PatrolSubState::Turning;
  avoidanceTurnDirection_ = direction;
  patrolTurnStartMs_ = nowMs;
  lastBlinkMs_ = nowMs;
  blinkOn_ = true;

  if (avoidanceTurnDirection_ == TurnDirection::Left)
  {
    outputs_.showLeftTurnSignal(blinkOn_);
    motors_.rotateLeft(Config::kPwmTurn90);
    serialOut_.println(F("[NAV] TURN LEFT"));
  }
  else
  {
    outputs_.showRightTurnSignal(blinkOn_);
    motors_.rotateRight(Config::kPwmTurn90);
    serialOut_.println(F("[NAV] TURN RIGHT"));
  }
}

PatrolController::TurnDirection PatrolController::chooseTurnDirection(float leftCm, float rightCm) const
{
  const bool leftClear = isClearDistance(leftCm, Config::kSideTurnClearCm);
  const bool rightClear = isClearDistance(rightCm, Config::kSideTurnClearCm);

  if (leftClear && rightClear)
  {
    return leftCm > rightCm + Config::kSideChoiceMarginCm ? TurnDirection::Left : TurnDirection::Right;
  }
  if (leftClear)
  {
    return TurnDirection::Left;
  }
  return TurnDirection::Right;
}

PatrolController::FireTurn PatrolController::chooseFireTurn(const char* direction) const
{
  if (!direction)
  {
    return FireTurn::None;
  }
  if (strcmp(direction, "LEFT") == 0 || strcmp(direction, "LEFT_REAR") == 0)
  {
    return FireTurn::Left90;
  }
  if (strcmp(direction, "RIGHT") == 0 || strcmp(direction, "RIGHT_REAR") == 0)
  {
    return FireTurn::Right90;
  }
  if (strcmp(direction, "REAR") == 0)
  {
    return FireTurn::Right180;
  }
  return FireTurn::None;
}

PatrolController::FireTurn PatrolController::inverseFireTurn(FireTurn turn) const
{
  switch (turn)
  {
    case FireTurn::Left90:   return FireTurn::Right90;
    case FireTurn::Right90:  return FireTurn::Left90;
    case FireTurn::Right180: return FireTurn::Right180;
    case FireTurn::None:    return FireTurn::None;
  }
  return FireTurn::None;
}

unsigned long PatrolController::fireTurnDurationMs(FireTurn turn) const
{
  switch (turn)
  {
    case FireTurn::Left90:   return Config::kFireLeftSideTurnMs;
    case FireTurn::Right90:  return Config::kFireRightSideTurnMs;
    case FireTurn::Right180: return Config::kFireRearTurnMs;
    case FireTurn::None:    return 0UL;
  }
  return 0UL;
}

void PatrolController::startFireTurn(FireTurn turn, bool restoring)
{
  switch (turn)
  {
    case FireTurn::Left90:
      outputs_.showLeftTurnSignal(true);
      motors_.rotateLeft(Config::kPwmFireTurn);
      serialOut_.println(restoring ? F("[FIRE] restoring from left inspection") : F("[FIRE] orient toward left side"));
      break;

    case FireTurn::Right90:
      outputs_.showRightTurnSignal(true);
      motors_.rotateRight(Config::kPwmFireTurn);
      serialOut_.println(restoring ? F("[FIRE] restoring from right inspection") : F("[FIRE] orient toward right side"));
      break;

    case FireTurn::Right180:
      outputs_.showRightTurnSignal(true);
      motors_.rotateRight(Config::kPwmFireTurn);
      serialOut_.println(restoring ? F("[FIRE] restoring from rear inspection") : F("[FIRE] orient toward rear"));
      break;

    case FireTurn::None:
      motors_.stop();
      outputs_.showBrakeLights();
      serialOut_.println(F("[FIRE] no orientation turn"));
      break;
  }
}

void PatrolController::enterFireHolding(unsigned long nowMs)
{
  fireStage_ = FireStage::Holding;
  fireStageStartMs_ = nowMs;
  fireClearCandidate_ = false;
  fireClearStartMs_ = 0;
  motors_.stop();
  outputs_.showBrakeLights();
  serialOut_.println(F("[FIRE] holding until hazard clears"));
}

bool PatrolController::fireClearStable(unsigned long nowMs)
{
  const SensorSuite::Snapshot& snap = sensors_.snapshot();
  if (snap.hazardDetected)
  {
    fireClearCandidate_ = false;
    fireClearStartMs_ = 0;
    return false;
  }

  if (!fireClearCandidate_)
  {
    fireClearCandidate_ = true;
    fireClearStartMs_ = nowMs;
    return false;
  }

  return nowMs - fireClearStartMs_ >= Config::kFireClearStableMs;
}

void PatrolController::resumeAfterFire(unsigned long nowMs)
{
  outputs_.silenceBuzzer();
  fireSnapshotSent_ = false;
  fireSnapshotAtMs_ = 0;
  fireClearCandidate_ = false;
  fireClearStartMs_ = 0;

  if (preFireMode_ == Mode::Verification)
  {
    resumeVerification(nowMs);
  }
  else if (preFireMode_ == Mode::ManualDrive)
  {
    enterManualDrive(nowMs);
  }
  else if (arrivalStopPending_ || preFireMode_ == Mode::ArrivalStop)
  {
    enterArrivalStop(nowMs);
  }
  else
  {
    enterPatrol(nowMs);
  }
}

void PatrolController::startVerificationPreparation(unsigned long nowMs)
{
  const float rearCm = sensors_.readPatrolRearCm();
  const bool rearBlocked = isValidDistance(rearCm) && rearCm < Config::kVerificationBackupRearClearCm;

  if (rearBlocked)
  {
    serialOut_.print(F("[VERIF] backup skipped; rear="));
    serialOut_.print(rearCm, 1);
    serialOut_.println(F(" cm"));
    startVerificationScanTimer(nowMs);
    return;
  }

  verificationStage_ = VerificationStage::BackingUp;
  stageStartMs_ = nowMs;
  motors_.driveBackward(Config::kPwmVerificationBackup);
  outputs_.showReverseLights();
  serialOut_.println(F("[VERIF] backing up before face scan"));
}

void PatrolController::startVerificationScanTimer(unsigned long nowMs)
{
  verificationStage_ = VerificationStage::InitialWait;
  modeStartMs_ = nowMs;
  stageStartMs_ = nowMs;
  motors_.stop();
  outputs_.showBrakeLights();
  outputs_.showTopScanning();
  outputs_.startLedScanner();
  serialOut_.println(F("[VERIF] starting face scan timer"));
}

bool PatrolController::isClearDistance(float distanceCm, float thresholdCm) const
{
  return isValidDistance(distanceCm) && distanceCm >= thresholdCm;
}

bool PatrolController::isValidDistance(float distanceCm) const
{
  return distanceCm > 0.0f;
}

void PatrolController::startManualMotion(ManualMotion motion, unsigned long durationMs, unsigned long nowMs)
{
  if (mode_ != Mode::ManualDrive)
  {
    enterManualDrive(nowMs);
  }

  manualMotion_ = motion;
  manualMotionEndMs_ = nowMs + durationMs;
  lastBlinkMs_ = nowMs;
  blinkOn_ = true;

  switch (manualMotion_)
  {
    case ManualMotion::Forward:
      outputs_.showTailLights();
      motors_.driveForward(Config::kPwmForward);
      serialOut_.print(F("[DRIVE] FORWARD ms="));
      break;

    case ManualMotion::Backward:
      outputs_.showReverseLights();
      motors_.driveBackward(Config::kPwmBackup);
      serialOut_.print(F("[DRIVE] BACKWARD ms="));
      break;

    case ManualMotion::Left:
      outputs_.showLeftTurnSignal(blinkOn_);
      motors_.rotateLeft(Config::kPwmTurn90);
      serialOut_.print(F("[DRIVE] LEFT ms="));
      break;

    case ManualMotion::Right:
      outputs_.showRightTurnSignal(blinkOn_);
      motors_.rotateRight(Config::kPwmTurn90);
      serialOut_.print(F("[DRIVE] RIGHT ms="));
      break;

    case ManualMotion::Idle:
      motors_.stop();
      outputs_.showBrakeLights();
      serialOut_.print(F("[DRIVE] IDLE ms="));
      break;
  }
  serialOut_.println(durationMs);
}

void PatrolController::runManualDrive(unsigned long nowMs)
{
  if (manualMotion_ == ManualMotion::Idle)
  {
    return;
  }

  if (manualMotion_ == ManualMotion::Left || manualMotion_ == ManualMotion::Right)
  {
    if (nowMs - lastBlinkMs_ >= Config::kTurnSignalBlinkMs)
    {
      lastBlinkMs_ = nowMs;
      blinkOn_ = !blinkOn_;
      if (manualMotion_ == ManualMotion::Left)
      {
        outputs_.showLeftTurnSignal(blinkOn_);
      }
      else
      {
        outputs_.showRightTurnSignal(blinkOn_);
      }
    }
  }

  if (nowMs >= manualMotionEndMs_)
  {
    manualMotion_ = ManualMotion::Idle;
    manualMotionEndMs_ = 0;
    motors_.stop();
    outputs_.showBrakeLights();
    serialOut_.println(F("[DRIVE] timed command complete"));
  }
}

void PatrolController::runArrivalStop(unsigned long nowMs)
{
  (void)nowMs;
  motors_.stop();
}

void PatrolController::runEmergencyStop(unsigned long nowMs)
{
  motors_.stop();
  outputs_.showBrakeLights();

  if (nowMs - lastBlinkMs_ >= Config::kAlertBlinkMs)
  {
    lastBlinkMs_ = nowMs;
    blinkOn_ = !blinkOn_;
    if (blinkOn_)
    {
      outputs_.setLedRed();
    }
    else
    {
      outputs_.setLedBlack();
    }
  }
}

unsigned long PatrolController::parseDriveDurationMs(const char* arg, unsigned long fallbackMs) const
{
  if (!arg || arg[0] == '\0')
  {
    return fallbackMs;
  }

  const unsigned long parsed = strtoul(arg, NULL, 10);
  if (parsed == 0)
  {
    return fallbackMs;
  }
  return min(parsed, Config::kManualMaxDriveMs);
}

void PatrolController::runFireAlert(unsigned long nowMs)
{
  if (nowMs - lastBlinkMs_ >= Config::kAlertBlinkMs)
  {
    lastBlinkMs_ = nowMs;
    blinkOn_ = !blinkOn_;
    if (blinkOn_)
    {
      outputs_.setLedRed();
      outputs_.beep(2200, Config::kAlertBlinkMs - 5);
    }
    else
    {
      outputs_.setLedBlack();
    }
    outputs_.showFireAlert(blinkOn_, alertHazardType_, alertDirection_);
  }

  switch (fireStage_)
  {
    case FireStage::PreTurnSettle:
      motors_.stop();
      outputs_.showBrakeLights();
      if (nowMs - fireStageStartMs_ >= Config::kFirePreTurnSettleMs)
      {
        fireStage_ = FireStage::Orienting;
        fireStageStartMs_ = nowMs;
        fireTurnMs_ = fireTurnDurationMs(fireTurn_);
        startFireTurn(fireTurn_, false);
      }
      break;

    case FireStage::Orienting:
      if (nowMs - fireStageStartMs_ >= fireTurnMs_)
      {
        motors_.stop();
        outputs_.showBrakeLights();
        enterFireHolding(nowMs);
      }
      break;

    case FireStage::Holding:
      motors_.stop();
      outputs_.showBrakeLights();

      if (!fireSnapshotSent_ && nowMs - fireStageStartMs_ >= Config::kFireSnapshotSettleMs)
      {
        serialOut_.println(F("SNAP:fire"));
        serialOut_.println(F("[FIRE] snapshot requested after orientation"));
        fireSnapshotSent_ = true;
        fireSnapshotAtMs_ = nowMs;
      }

      if (fireSnapshotSent_ &&
          nowMs - fireSnapshotAtMs_ >= Config::kFirePostSnapshotHoldMs &&
          fireClearStable(nowMs))
      {
        const FireTurn restoreTurn = inverseFireTurn(fireTurn_);
        if (restoreTurn == FireTurn::None)
        {
          resumeAfterFire(nowMs);
        }
        else
        {
          fireStage_ = FireStage::Restoring;
          fireStageStartMs_ = nowMs;
          fireTurnMs_ = fireTurnDurationMs(restoreTurn);
          fireClearCandidate_ = false;
          startFireTurn(restoreTurn, true);
        }
      }
      break;

    case FireStage::Restoring:
      if (nowMs - fireStageStartMs_ >= fireTurnMs_)
      {
        motors_.stop();
        outputs_.showBrakeLights();
        resumeAfterFire(nowMs);
      }
      break;
  }
}

void PatrolController::runVerification(unsigned long nowMs)
{
  switch (verificationStage_)
  {
    case VerificationStage::BackingUp:
    {
      const float rearCm = sensors_.readPatrolRearCm();
      const bool rearBlocked = isValidDistance(rearCm) && rearCm < Config::kVerificationBackupRearClearCm;
      if (rearBlocked || nowMs - stageStartMs_ >= Config::kVerificationBackupMs)
      {
        motors_.stop();
        outputs_.showBrakeLights();
        verificationStage_ = VerificationStage::BackupSettle;
        stageStartMs_ = nowMs;
        if (rearBlocked)
        {
          serialOut_.println(F("[VERIF] backup stopped; rear obstacle"));
        }
      }
      break;
    }

    case VerificationStage::BackupSettle:
      if (nowMs - stageStartMs_ >= Config::kStopSettleMs)
      {
        startVerificationScanTimer(nowMs);
      }
      break;

    case VerificationStage::InitialWait:
      if (nowMs - modeStartMs_ >= Config::kVerificationServoTriggerMs)
      {
        sendPlay("approach_camera");
        verificationStage_ = VerificationStage::AudioWait;
        stageStartMs_ = nowMs;
      }
      break;

    case VerificationStage::AudioWait:
      if (nowMs - stageStartMs_ >= Config::kVerificationAudioWaitMs)
      {
        outputs_.tiltServoTo(Config::kServoVerifyMaxDeg, true);
        verificationStage_ = VerificationStage::ServoRaising;
      }
      break;

    case VerificationStage::ServoRaising:
      if (outputs_.servoAtTarget())
      {
        verificationStage_ = VerificationStage::ServoHolding;
        stageStartMs_ = nowMs;
      }
      break;

    case VerificationStage::ServoHolding:
      if (nowMs - stageStartMs_ >= Config::kVerificationServoHoldMs)
      {
        outputs_.tiltServoTo(Config::kServoCenterDeg, true);
        verificationStage_ = VerificationStage::ServoReturning;
      }
      break;

    case VerificationStage::ServoReturning:
      if (outputs_.servoAtTarget())
      {
        serialOut_.println(F("[VERIF] scan done; waiting for Jetson response"));
        verificationStage_ = VerificationStage::PostScanWait;
        stageStartMs_ = nowMs;
      }
      break;

    case VerificationStage::PostScanWait:
      if (nowMs - stageStartMs_ >= Config::kVerificationTimeoutMs)
      {
        serialOut_.println(F("[VERIF] timeout; no Jetson response after scan"));
        enterSecurityAlert(nowMs);
      }
      break;
  }
}

void PatrolController::runSecurityAlert(unsigned long nowMs)
{
  if (nowMs - lastBlinkMs_ >= Config::kAlertBlinkMs)
  {
    lastBlinkMs_ = nowMs;
    blinkOn_ = !blinkOn_;
    if (blinkOn_)
    {
      outputs_.setLedRed();
      outputs_.beep(3500, Config::kAlertBlinkMs - 5);
    }
    else
    {
      outputs_.setLedBlack();
    }
    outputs_.showSecurityAlert(blinkOn_);
  }

  if (nowMs - modeStartMs_ >= Config::kSecurityAlertDurationMs)
  {
    outputs_.silenceBuzzer();
    if (arrivalStopPending_)
    {
      enterArrivalStop(nowMs);
    }
    else
    {
      enterPatrol(nowMs);
    }
  }
}

void PatrolController::runVerifiedPause(unsigned long nowMs)
{
  if (nowMs - modeStartMs_ >= Config::kVerifiedPauseMs && outputs_.servoAtTarget())
  {
    enterPatrol(nowMs);
  }
}

void PatrolController::sendMode(const char* mode)
{
  serialOut_.print(F("MODE:"));
  serialOut_.println(mode);
}

void PatrolController::sendPlay(const char* clip)
{
  serialOut_.print(F("PLAY:"));
  serialOut_.println(clip);
}

void PatrolController::sendBluetoothFire(unsigned long nowMs)
{
  bluetoothOut_.print(F("{\"type\":\"fire\",\"subtype\":\""));
  bluetoothOut_.print(alertHazardType_);
  bluetoothOut_.print(F("\",\"dir\":\""));
  bluetoothOut_.print(alertDirection_);
  bluetoothOut_.print(F("\",\"ts\":"));
  bluetoothOut_.print(nowMs);
  bluetoothOut_.println(F("}"));
}

void PatrolController::sendBluetoothSecurity(unsigned long nowMs)
{
  bluetoothOut_.print(F("{\"type\":\"intruder\",\"ts\":"));
  bluetoothOut_.print(nowMs);
  bluetoothOut_.println(F("}"));
}

void PatrolController::handleStatusPayload(const char* payload, unsigned long nowMs)
{
  if (!payload || payload[0] == '\0')
  {
    return;
  }

  if (strcmp(payload, "CLEAR") == 0)
  {
    if (mode_ == Mode::Verification)
    {
      if (verificationPendingResolution_)
      {
        serialOut_.println(F("[STATUS] CLEAR: unresolved person left scene"));
        enterSecurityAlert(nowMs);
      }
      else
      {
        serialOut_.println(F("[STATUS] CLEAR: verification complete"));
        enterPatrol(nowMs);
      }
    }
    return;
  }

  const bool anyUnknown = strstr(payload, ":UNKNOWN") != NULL;
  const bool anyScanning = strstr(payload, ":SCANNING") != NULL;
  const AlignmentDirection scanningAlignment = parseScanningAlignment(payload);

  serialOut_.print(F("[STATUS] "));
  serialOut_.println(payload);

  switch (mode_)
  {
    case Mode::Patrol:
      if (anyUnknown)
      {
        enterSecurityAlert(nowMs);
      }
      else if (anyScanning)
      {
        enterVerification(nowMs, scanningAlignment);
        verificationPendingResolution_ = true;
      }
      // All VERIFIED while already in PATROL means the scene is already clean.
      // Do not re-enter VERIFIED repeatedly while the registered person remains in view.
      break;

    case Mode::Verification:
      if (anyUnknown)
      {
        enterSecurityAlert(nowMs);
      }
      else if (anyScanning)
      {
        verificationPendingResolution_ = true;
      }
      else
      {
        char names[sizeof(verifiedName_)] = {0};
        verificationPendingResolution_ = false;
        if (buildVerifiedNameList(payload, names, sizeof(names)))
        {
          enterVerifiedPause(nowMs, names);
        }
        else
        {
          enterPatrol(nowMs);
        }
      }
      break;

    case Mode::FireAlert:
    case Mode::SecurityAlert:
    case Mode::VerifiedPause:
    case Mode::ManualDrive:
    case Mode::ArrivalStop:
    case Mode::EmergencyStop:
      break;
  }
}

PatrolController::AlignmentDirection PatrolController::parseScanningAlignment(const char* payload) const
{
  if (!payload)
  {
    return AlignmentDirection::Center;
  }
  if (strstr(payload, ":SCANNING:LEFT") != NULL)
  {
    return AlignmentDirection::Left;
  }
  if (strstr(payload, ":SCANNING:RIGHT") != NULL)
  {
    return AlignmentDirection::Right;
  }
  return AlignmentDirection::Center;
}

bool PatrolController::buildVerifiedNameList(const char* payload, char* dest, size_t destSize) const
{
  if (!payload || !dest || destSize == 0)
  {
    return false;
  }

  dest[0] = '\0';
  size_t used = 0;
  uint8_t count = 0;
  const char* cursor = payload;
  const char marker[] = ":VERIFIED:";
  const size_t markerLen = strlen(marker);

  while ((cursor = strstr(cursor, marker)) != NULL)
  {
    cursor += markerLen;
    const char* end = strchr(cursor, ',');
    if (!end)
    {
      end = cursor + strlen(cursor);
    }

    const char* prefix = count == 0 ? "" : "_and_";
    for (const char* p = prefix; *p && used + 1 < destSize; ++p)
    {
      dest[used++] = *p;
    }

    for (const char* p = cursor; p < end && used + 1 < destSize; ++p)
    {
      dest[used++] = (*p == ' ') ? '_' : *p;
    }
    dest[used] = '\0';
    ++count;
    cursor = end;
  }

  return count > 0;
}

void PatrolController::printHazardSummary()
{
  const SensorSuite::Snapshot& snap = sensors_.snapshot();
  serialOut_.println(F(""));
  serialOut_.println(F("[HAZARD TRIGGER]"));
  serialOut_.print(F("  type="));
  serialOut_.print(alertHazardType_);
  serialOut_.print(F(" dir="));
  serialOut_.println(alertDirection_);
  serialOut_.print(F("  gas="));
  serialOut_.print(snap.gasAlert ? F("YES") : F("NO"));
  serialOut_.print(F(" front="));
  serialOut_.println(snap.frontAlert ? F("YES") : F("NO"));
  serialOut_.println(F(""));
}

void PatrolController::copyText(char* dest, size_t destSize, const char* src)
{
  if (!dest || destSize == 0)
  {
    return;
  }
  if (!src)
  {
    src = "";
  }
  strncpy(dest, src, destSize - 1);
  dest[destSize - 1] = '\0';
}
