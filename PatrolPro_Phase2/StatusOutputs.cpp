#include "StatusOutputs.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <Wire.h>
#include <string.h>

#include "Config.h"

namespace
{
CRGB gStatusLeds[Config::kLedCount];
CRGB gExternalLeds[Config::kExternalLedCount];
Servo gCameraServo;
Adafruit_SSD1306 gDisplay(Config::kOledWidth, Config::kOledHeight, &Wire, Config::kOledReset);

constexpr uint8_t kMatrixSize = 10;
constexpr uint8_t kRearDisplayMinY = 0;
constexpr uint8_t kRearDisplayMaxY = 5;  // rear LEDs 41-100 only; 1-30 stay brake/turn only
const uint16_t kRightBrakeLeds[] = {1, 2, 11, 12, 21, 22};
const uint16_t kLeftBrakeLeds[] = {9, 10, 19, 20, 29, 30};

enum TopDisplayMode : uint8_t
{
  TOP_NONE = 0,
  TOP_PATROL_SPINNER,
  TOP_SCANNING_EYE,
  TOP_VERIFIED_SMILE,
  TOP_UNKNOWN_QUESTION,
  TOP_FIRE_EXCLAMATION,
};

enum RearDisplayMode : uint8_t
{
  REAR_NONE = 0,
  REAR_PATROL,
  REAR_SCANNING,
  REAR_VERIFIED,
  REAR_UNKNOWN,
  REAR_FIRE,
};

enum StatusLedMode : uint8_t
{
  STATUS_SOLID = 0,
  STATUS_SCANNER,
};

TopDisplayMode gTopMode = TOP_NONE;
uint8_t gTopFrame = 0;
unsigned long gLastTopFrameMs = 0;
RearDisplayMode gRearMode = REAR_NONE;
uint8_t gRearFrame = 0;
unsigned long gLastRearFrameMs = 0;
StatusLedMode gStatusLedMode = STATUS_SOLID;
uint8_t gStatusScanIndex = 0;
unsigned long gLastStatusScanMs = 0;

CRGB externalSafeColor(const CRGB& color)
{
  const uint8_t maxChannel = max(color.r, max(color.g, color.b));
  if (maxChannel == 0)
  {
    return CRGB::Black;
  }

  const bool isWhiteLike = color.r > 0 && color.g > 0 && color.b > 0;
  const uint8_t cap = isWhiteLike ? Config::kExternalLedNormalLevel : Config::kExternalLedAlertLevel;
  return CRGB(
    (uint8_t)((uint16_t)color.r * cap / maxChannel),
    (uint8_t)((uint16_t)color.g * cap / maxChannel),
    (uint8_t)((uint16_t)color.b * cap / maxChannel));
}

void setExternalLedRaw(uint16_t ledNumber, const CRGB& color)
{
  if (ledNumber >= 1 && ledNumber <= Config::kExternalLedCount)
  {
    gExternalLeds[ledNumber - 1] = color;
  }
}

uint16_t rearLedNumber(uint8_t x, uint8_t y)
{
  if (x >= kMatrixSize || y >= kMatrixSize)
  {
    return 0;
  }

  // Rear board: bottom-right is 1, bottom-left is 10, rows increase upward.
  return (uint16_t)((kMatrixSize - 1 - y) * kMatrixSize + (kMatrixSize - x));
}

uint16_t topLedNumber(uint8_t x, uint8_t y)
{
  if (x >= kMatrixSize || y >= kMatrixSize)
  {
    return 0;
  }

  // Top board is mounted 180 degrees rotated. Logical front-left maps to physical back-right.
  const uint8_t physicalX = kMatrixSize - 1 - x;
  const uint8_t physicalY = kMatrixSize - 1 - y;
  return (uint16_t)(Config::kExternalLedBoardSize + physicalY * kMatrixSize + physicalX + 1);
}

void setRearPixel(uint8_t x, uint8_t y, const CRGB& color)
{
  setExternalLedRaw(rearLedNumber(x, y), color);
}

void setTopPixel(uint8_t x, uint8_t y, const CRGB& color)
{
  setExternalLedRaw(topLedNumber(x, y), color);
}

void clearRearBoard()
{
  fill_solid(gExternalLeds, Config::kExternalLedBoardSize, CRGB::Black);
}

void clearRearDisplayArea()
{
  for (uint8_t y = kRearDisplayMinY; y <= kRearDisplayMaxY; ++y)
  {
    for (uint8_t x = 0; x < kMatrixSize; ++x)
    {
      setRearPixel(x, y, CRGB::Black);
    }
  }
}

void clearRearSignalArea()
{
  for (uint16_t ledNumber = 1; ledNumber <= 40; ++ledNumber)
  {
    setExternalLedRaw(ledNumber, CRGB::Black);
  }
}

void clearTopBoard()
{
  fill_solid(
    gExternalLeds + Config::kExternalLedBoardSize,
    Config::kExternalLedCount - Config::kExternalLedBoardSize,
    CRGB::Black);
}

void setLedGroup(const uint16_t* leds, size_t count, const CRGB& color)
{
  for (size_t i = 0; i < count; ++i)
  {
    setExternalLedRaw(leds[i], color);
  }
}

void setBrakeGroups(const CRGB& rightColor, const CRGB& leftColor)
{
  setLedGroup(kRightBrakeLeds, sizeof(kRightBrakeLeds) / sizeof(kRightBrakeLeds[0]), rightColor);
  setLedGroup(kLeftBrakeLeds, sizeof(kLeftBrakeLeds) / sizeof(kLeftBrakeLeds[0]), leftColor);
}

void drawRearPattern(const char* const rows[kMatrixSize], const CRGB& color)
{
  clearRearDisplayArea();
  const CRGB safeColor = externalSafeColor(color);
  for (uint8_t y = kRearDisplayMinY; y <= kRearDisplayMaxY; ++y)
  {
    for (uint8_t x = 0; x < kMatrixSize; ++x)
    {
      if (rows[y][x] != ' ')
      {
        setRearPixel(x, y, safeColor);
      }
    }
  }
  FastLED.show();
}

void drawRearPatrol(uint8_t frame)
{
  clearRearDisplayArea();
  const uint8_t positions[] = {1, 2, 3, 4, 5, 6, 7, 8, 7, 6, 5, 4, 3, 2};
  const uint8_t centerX = positions[frame % (sizeof(positions) / sizeof(positions[0]))];
  const CRGB core = externalSafeColor(CRGB(0, 120, 180));
  const CRGB innerTrail = CRGB(0, 22, 36);
  const CRGB outerTrail = CRGB(0, 8, 14);

  for (uint8_t y = kRearDisplayMinY; y <= kRearDisplayMaxY; ++y)
  {
    if (centerX >= 2)
    {
      setRearPixel(centerX - 2, y, outerTrail);
    }
    if (centerX >= 1)
    {
      setRearPixel(centerX - 1, y, innerTrail);
    }
    setRearPixel(centerX, y, core);
    if (centerX + 1 < kMatrixSize)
    {
      setRearPixel(centerX + 1, y, innerTrail);
    }
    if (centerX + 2 < kMatrixSize)
    {
      setRearPixel(centerX + 2, y, outerTrail);
    }
  }
  FastLED.show();
}

void drawRearScanning(uint8_t frame)
{
  clearRearDisplayArea();
  const uint8_t positions[] = {4, 5, 6, 7, 8, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 1, 2, 3};
  const uint8_t x = positions[frame % (sizeof(positions) / sizeof(positions[0]))];
  const CRGB dimAmber = CRGB(18, 6, 0);
  const CRGB brightAmber = externalSafeColor(CRGB(255, 80, 0));

  for (uint8_t y = kRearDisplayMinY + 1; y < kRearDisplayMaxY; ++y)
  {
    setRearPixel(4, y, dimAmber);
    setRearPixel(5, y, dimAmber);
    setRearPixel(x, y, brightAmber);
  }
  FastLED.show();
}

void drawRearVerified()
{
  const char* const rows[kMatrixSize] = {
    "   XXXX   ",
    " XX    XX ",
    "X        X",
    "X        X",
    " XX    XX ",
    "   XXXX   ",
    "          ",
    "          ",
    "          ",
    "          ",
  };
  drawRearPattern(rows, CRGB::Green);
}

void drawRearUnknown()
{
  const char* const rows[kMatrixSize] = {
    "XX      XX",
    "  XX  XX  ",
    "   XXXX   ",
    "   XXXX   ",
    "  XX  XX  ",
    "XX      XX",
    "          ",
    "          ",
    "          ",
    "          ",
  };
  drawRearPattern(rows, CRGB::Red);
}

void drawRearFire(bool visible)
{
  if (!visible)
  {
    clearRearDisplayArea();
    FastLED.show();
    return;
  }

  const char* const rows[kMatrixSize] = {
    "    XX    ",
    "    XX    ",
    "    XX    ",
    "          ",
    "    XX    ",
    "    XX    ",
    "          ",
    "          ",
    "          ",
    "          ",
  };
  drawRearPattern(rows, CRGB::Red);
}

void renderRearDisplay()
{
  switch (gRearMode)
  {
    case REAR_PATROL:
      drawRearPatrol(gRearFrame);
      break;
    case REAR_SCANNING:
      drawRearScanning(gRearFrame);
      break;
    case REAR_VERIFIED:
      drawRearVerified();
      break;
    case REAR_UNKNOWN:
      drawRearUnknown();
      break;
    case REAR_FIRE:
      drawRearFire((gRearFrame % 2) == 0);
      break;
    case REAR_NONE:
    default:
      break;
  }
}

void setRearMode(RearDisplayMode mode)
{
  gRearMode = mode;
  gRearFrame = 0;
  gLastRearFrameMs = 0;
  renderRearDisplay();
}

void updateRearDisplay(unsigned long nowMs)
{
  unsigned long frameMs = 0;
  if (gRearMode == REAR_PATROL)
  {
    frameMs = Config::kRearPatrolFrameMs;
  }
  else if (gRearMode == REAR_SCANNING)
  {
    frameMs = Config::kRearScanFrameMs;
  }
  else if (gRearMode == REAR_FIRE)
  {
    frameMs = Config::kRearFireBlinkMs;
  }
  else
  {
    return;
  }

  if (gLastRearFrameMs == 0 || nowMs - gLastRearFrameMs >= frameMs)
  {
    gLastRearFrameMs = nowMs;
    ++gRearFrame;
    renderRearDisplay();
  }
}

void drawStatusScanner()
{
  fill_solid(gStatusLeds, Config::kLedCount, CRGB::Black);
  gStatusLeds[gStatusScanIndex % Config::kLedCount] = CRGB(255, 120, 0);
  FastLED.show();
}

void updateStatusScanner(unsigned long nowMs)
{
  if (gStatusLedMode != STATUS_SCANNER)
  {
    return;
  }

  if (gLastStatusScanMs == 0 || nowMs - gLastStatusScanMs >= Config::kStatusScanFrameMs)
  {
    gLastStatusScanMs = nowMs;
    gStatusScanIndex = (uint8_t)((gStatusScanIndex + 1) % Config::kLedCount);
    drawStatusScanner();
  }
}

void drawTopPattern(const char* const rows[kMatrixSize], const CRGB& color)
{
  clearTopBoard();
  const CRGB safeColor = externalSafeColor(color);
  for (uint8_t y = 0; y < kMatrixSize; ++y)
  {
    for (uint8_t x = 0; x < kMatrixSize; ++x)
    {
      if (rows[y][x] != ' ')
      {
        setTopPixel(x, y, safeColor);
      }
    }
  }
  FastLED.show();
}

void drawTopSpinner(uint8_t frame)
{
  clearTopBoard();
  const CRGB dimTeal = CRGB(0, 3, 6);
  const CRGB trail4 = CRGB(0, 7, 12);
  const CRGB trail3 = CRGB(0, 12, 20);
  const CRGB trail2 = CRGB(0, 20, 32);
  const CRGB trail1 = CRGB(0, 34, 54);
  const CRGB brightTeal = externalSafeColor(CRGB(0, 120, 180));
  const uint8_t points[][2] = {
    {3, 1}, {4, 1}, {5, 1}, {6, 1}, {7, 2},
    {8, 3}, {8, 4}, {8, 5}, {8, 6}, {7, 7},
    {6, 8}, {5, 8}, {4, 8}, {3, 8}, {2, 7},
    {1, 6}, {1, 5}, {1, 4}, {1, 3}, {2, 2},
  };
  const uint8_t pointCount = sizeof(points) / sizeof(points[0]);
  const uint8_t active = frame % pointCount;
  const uint8_t t1 = (active + pointCount - 1) % pointCount;
  const uint8_t t2 = (active + pointCount - 2) % pointCount;
  const uint8_t t3 = (active + pointCount - 3) % pointCount;
  const uint8_t t4 = (active + pointCount - 4) % pointCount;

  for (uint8_t i = 0; i < pointCount; ++i)
  {
    setTopPixel(points[i][0], points[i][1], dimTeal);
  }
  setTopPixel(points[t4][0], points[t4][1], trail4);
  setTopPixel(points[t3][0], points[t3][1], trail3);
  setTopPixel(points[t2][0], points[t2][1], trail2);
  setTopPixel(points[t1][0], points[t1][1], trail1);
  setTopPixel(points[active][0], points[active][1], brightTeal);
  FastLED.show();
}

void drawTopEye(bool closed)
{
  if (closed)
  {
    const char* const rows[kMatrixSize] = {
      "          ",
      "          ",
      "          ",
      "  XXXXXX  ",
      " XX    XX ",
      "  XXXXXX  ",
      "          ",
      "          ",
      "          ",
      "          ",
    };
    drawTopPattern(rows, CRGB(255, 80, 0));
    return;
  }

  const char* const rows[kMatrixSize] = {
    "          ",
    "   XXXX   ",
    " XX    XX ",
    "X   XX   X",
    "X  XXXX  X",
    "X   XX   X",
    " XX    XX ",
    "   XXXX   ",
    "          ",
    "          ",
  };
  drawTopPattern(rows, CRGB(255, 80, 0));
}

void drawTopSmile()
{
  const char* const rows[kMatrixSize] = {
    "          ",
    "  XX  XX  ",
    "  XX  XX  ",
    "          ",
    "          ",
    " X      X ",
    "  X    X  ",
    "   XXXX   ",
    "          ",
    "          ",
  };
  drawTopPattern(rows, CRGB::Green);
}

void drawTopQuestion()
{
  const char* const rows[kMatrixSize] = {
    "  XXXXX   ",
    " XX   XX  ",
    "      XX  ",
    "     XX   ",
    "    XX    ",
    "   XX     ",
    "   XX     ",
    "          ",
    "   XX     ",
    "   XX     ",
  };
  drawTopPattern(rows, CRGB::Red);
}

void drawTopExclamation()
{
  const char* const rows[kMatrixSize] = {
    "    XX    ",
    "    XX    ",
    "    XX    ",
    "    XX    ",
    "    XX    ",
    "    XX    ",
    "          ",
    "    XX    ",
    "    XX    ",
    "          ",
  };
  drawTopPattern(rows, CRGB::Red);
}

void renderTopDisplay()
{
  switch (gTopMode)
  {
    case TOP_PATROL_SPINNER:
      drawTopSpinner(gTopFrame);
      break;
    case TOP_SCANNING_EYE:
      drawTopEye((gTopFrame % 5) == 4);
      break;
    case TOP_VERIFIED_SMILE:
      drawTopSmile();
      break;
    case TOP_UNKNOWN_QUESTION:
      drawTopQuestion();
      break;
    case TOP_FIRE_EXCLAMATION:
      drawTopExclamation();
      break;
    case TOP_NONE:
    default:
      break;
  }
}

void setTopMode(TopDisplayMode mode)
{
  gTopMode = mode;
  gTopFrame = 0;
  gLastTopFrameMs = 0;
  renderTopDisplay();
}

void updateTopDisplay(unsigned long nowMs)
{
  unsigned long frameMs = 0;
  if (gTopMode == TOP_PATROL_SPINNER)
  {
    frameMs = Config::kTopSpinnerFrameMs;
  }
  else if (gTopMode == TOP_SCANNING_EYE)
  {
    frameMs = Config::kTopEyeFrameMs;
  }
  else
  {
    return;
  }

  if (gLastTopFrameMs == 0 || nowMs - gLastTopFrameMs >= frameMs)
  {
    gLastTopFrameMs = nowMs;
    ++gTopFrame;
    renderTopDisplay();
  }
}
}

void StatusOutputs::begin()
{
  pinMode(Config::kBuzzerPin, OUTPUT);
  silenceBuzzer();

  FastLED.addLeds<WS2812B, Config::kLedDataPin, GRB>(gStatusLeds, Config::kLedCount);
  FastLED.addLeds<WS2812B, Config::kExternalLedDataPin, GRB>(gExternalLeds, Config::kExternalLedCount);
  FastLED.setBrightness(Config::kLedBrightness);
  setLedWhite();

  currentServoDeg_ = Config::kServoCenterDeg;
  targetServoDeg_ = Config::kServoCenterDeg;
  servoTargetReachedMs_ = 0;
  attachServoIfNeeded();
  writeServoNow(Config::kServoCenterDeg);

  Wire.begin();
  Wire.setClock(100000UL);
  oledReady_ = beginOledAtAddress(Config::kOledPrimaryAddress);
  if (!oledReady_)
  {
    oledReady_ = beginOledAtAddress(Config::kOledFallbackAddress);
  }

  if (!oledReady_)
  {
    Serial.println(F("[OLED] SSD1306 init failed. Check 0x3C/0x3D, SDA=20, SCL=21."));
  }
  else
  {
    gDisplay.clearDisplay();
    gDisplay.display();
  }

  showNormal();
}

void StatusOutputs::update(unsigned long nowMs)
{
  if (servoAttached_ &&
      currentServoDeg_ != targetServoDeg_ &&
      nowMs - lastServoStepMs_ >= Config::kServoStepDelayMs)
  {
    lastServoStepMs_ = nowMs;
    if (currentServoDeg_ < targetServoDeg_)
    {
      currentServoDeg_ = min(currentServoDeg_ + Config::kServoStepDeg, targetServoDeg_);
    }
    else
    {
      currentServoDeg_ = max(currentServoDeg_ - Config::kServoStepDeg, targetServoDeg_);
    }
    gCameraServo.write(currentServoDeg_);
    servoTargetReachedMs_ = 0;
  }

  if (servoAttached_ && currentServoDeg_ == targetServoDeg_)
  {
    if (targetServoDeg_ == Config::kServoCenterDeg)
    {
      if (servoTargetReachedMs_ == 0)
      {
        servoTargetReachedMs_ = nowMs;
      }
      else if (nowMs - servoTargetReachedMs_ >= Config::kServoDetachDelayMs)
      {
        detachServo();
      }
    }
    else
    {
      servoTargetReachedMs_ = 0;
    }
  }

  updateStatusScanner(nowMs);
  updateRearDisplay(nowMs);
  updateTopDisplay(nowMs);
}

void StatusOutputs::setLed(const CRGB& color)
{
  gStatusLedMode = STATUS_SOLID;
  fill_solid(gStatusLeds, Config::kLedCount, color);
  FastLED.show();
}

void StatusOutputs::setLedWhite()
{
  setLed(CRGB::White);
}

void StatusOutputs::setLedBlack()
{
  setLed(CRGB::Black);
}

void StatusOutputs::setLedRed()
{
  setLed(CRGB::Red);
}

void StatusOutputs::setLedOrange()
{
  setLed(CRGB(255, 80, 0));
}

void StatusOutputs::setLedGreen()
{
  setLed(CRGB::Green);
}

void StatusOutputs::startLedScanner()
{
  gStatusLedMode = STATUS_SCANNER;
  gStatusScanIndex = 0;
  gLastStatusScanMs = 0;
  drawStatusScanner();
}

void StatusOutputs::setExternalLedBoards(const CRGB& firstBoardColor, const CRGB& secondBoardColor)
{
  fill_solid(gExternalLeds, Config::kExternalLedBoardSize, externalSafeColor(firstBoardColor));
  fill_solid(
    gExternalLeds + Config::kExternalLedBoardSize,
    Config::kExternalLedCount - Config::kExternalLedBoardSize,
    externalSafeColor(secondBoardColor));
  FastLED.show();
}

bool StatusOutputs::setExternalLed(uint16_t ledNumber, const CRGB& color)
{
  if (ledNumber < 1 || ledNumber > Config::kExternalLedCount)
  {
    return false;
  }

  gExternalLeds[ledNumber - 1] = color;
  FastLED.show();
  return true;
}

bool StatusOutputs::setExternalLedRange(uint16_t firstLedNumber, uint16_t lastLedNumber, const CRGB& color)
{
  if (firstLedNumber < 1 || lastLedNumber < 1 ||
      firstLedNumber > Config::kExternalLedCount ||
      lastLedNumber > Config::kExternalLedCount ||
      firstLedNumber > lastLedNumber)
  {
    return false;
  }

  fill_solid(gExternalLeds + firstLedNumber - 1, lastLedNumber - firstLedNumber + 1, externalSafeColor(color));
  FastLED.show();
  return true;
}

void StatusOutputs::setExternalLedAll(const CRGB& color)
{
  fill_solid(gExternalLeds, Config::kExternalLedCount, externalSafeColor(color));
  FastLED.show();
}

void StatusOutputs::clearExternalLeds()
{
  gRearMode = REAR_NONE;
  gTopMode = TOP_NONE;
  fill_solid(gExternalLeds, Config::kExternalLedCount, CRGB::Black);
  FastLED.show();
}

void StatusOutputs::showTailLights()
{
  clearRearSignalArea();
  const CRGB tailRed = CRGB(8, 0, 0);
  setBrakeGroups(tailRed, tailRed);
  FastLED.show();
}

void StatusOutputs::showBrakeLights()
{
  clearRearSignalArea();
  const CRGB brakeRed = externalSafeColor(CRGB::Red);
  setBrakeGroups(brakeRed, brakeRed);
  FastLED.show();
}

void StatusOutputs::showReverseLights()
{
  clearRearSignalArea();
  const CRGB reverseWhite = externalSafeColor(CRGB::White);
  setBrakeGroups(reverseWhite, reverseWhite);
  FastLED.show();
}

void StatusOutputs::showLeftTurnSignal(bool signalOn)
{
  clearRearSignalArea();
  const CRGB tailRed = CRGB(8, 0, 0);
  const CRGB signalAmber = signalOn ? externalSafeColor(CRGB(255, 80, 0)) : tailRed;
  setBrakeGroups(signalAmber, tailRed);
  FastLED.show();
}

void StatusOutputs::showRightTurnSignal(bool signalOn)
{
  clearRearSignalArea();
  const CRGB tailRed = CRGB(8, 0, 0);
  const CRGB signalAmber = signalOn ? externalSafeColor(CRGB(255, 80, 0)) : tailRed;
  setBrakeGroups(tailRed, signalAmber);
  FastLED.show();
}

void StatusOutputs::showTopPatrol()
{
  setTopMode(TOP_PATROL_SPINNER);
  setRearMode(REAR_PATROL);
}

void StatusOutputs::showTopScanning()
{
  setTopMode(TOP_SCANNING_EYE);
  setRearMode(REAR_SCANNING);
}

void StatusOutputs::showTopVerified()
{
  setTopMode(TOP_VERIFIED_SMILE);
  setRearMode(REAR_VERIFIED);
}

void StatusOutputs::showTopUnknown()
{
  setTopMode(TOP_UNKNOWN_QUESTION);
  setRearMode(REAR_UNKNOWN);
}

void StatusOutputs::showTopFire()
{
  setTopMode(TOP_FIRE_EXCLAMATION);
  setRearMode(REAR_FIRE);
}

void StatusOutputs::beep(uint16_t frequencyHz, unsigned long durationMs)
{
  tone(Config::kBuzzerPin, frequencyHz, durationMs);
}

void StatusOutputs::silenceBuzzer()
{
  noTone(Config::kBuzzerPin);
}

void StatusOutputs::showNormal()
{
  if (!oledReady_) return;

  gDisplay.clearDisplay();
  gDisplay.setTextColor(SSD1306_WHITE);
  gDisplay.setTextSize(2);
  gDisplay.setCursor(10, 8);
  gDisplay.println(F("PATROL"));
  gDisplay.setCursor(25, 36);
  gDisplay.println(F("SAFE"));
  gDisplay.display();
}

void StatusOutputs::showFireAlert(bool visible, const char* hazardType, const char* direction)
{
  if (!oledReady_) return;

  gDisplay.clearDisplay();
  if (!visible)
  {
    gDisplay.display();
    return;
  }

  gDisplay.setTextColor(SSD1306_WHITE);
  gDisplay.setTextSize(1);
  gDisplay.setCursor(28, 0);
  gDisplay.println(F("!!! ALERT !!!"));
  gDisplay.drawLine(0, 10, Config::kOledWidth - 1, 10, SSD1306_WHITE);
  gDisplay.setTextSize(2);

  if (strcmp(hazardType, "BOTH") == 0)
  {
    gDisplay.setCursor(10, 18);
    gDisplay.println(F("FIRE+GAS"));
    gDisplay.setTextSize(1);
    gDisplay.setCursor(22, 44);
    gDisplay.println(F("DETECTED"));
  }
  else if (strcmp(hazardType, "SMOKE_GAS") == 0)
  {
    gDisplay.setCursor(22, 18);
    gDisplay.println(F("GAS"));
    gDisplay.setCursor(4, 42);
    gDisplay.println(F("DETECTED"));
  }
  else
  {
    gDisplay.setCursor(18, 18);
    gDisplay.println(F("FIRE"));
    gDisplay.setCursor(4, 42);
    gDisplay.println(F("DETECTED"));
  }

  gDisplay.setTextSize(1);
  gDisplay.setCursor(0, 56);
  gDisplay.print(F("DIR: "));
  gDisplay.println(direction);
  gDisplay.display();
}

void StatusOutputs::showVerification()
{
  if (!oledReady_) return;

  gDisplay.clearDisplay();
  gDisplay.setTextColor(SSD1306_WHITE);
  gDisplay.setTextSize(1);
  gDisplay.setCursor(20, 0);
  gDisplay.println(F("[ PERSON DETECTED ]"));
  gDisplay.drawLine(0, 10, Config::kOledWidth - 1, 10, SSD1306_WHITE);
  gDisplay.setTextSize(2);
  gDisplay.setCursor(10, 18);
  gDisplay.println(F("SCANNING"));
  gDisplay.setTextSize(1);
  gDisplay.setCursor(10, 52);
  gDisplay.println(F("Please face camera"));
  gDisplay.display();
}

void StatusOutputs::showSecurityAlert(bool visible)
{
  if (!oledReady_) return;

  gDisplay.clearDisplay();
  if (!visible)
  {
    gDisplay.display();
    return;
  }

  gDisplay.setTextColor(SSD1306_WHITE);
  gDisplay.setTextSize(1);
  gDisplay.setCursor(28, 0);
  gDisplay.println(F("!!! ALERT !!!"));
  gDisplay.drawLine(0, 10, Config::kOledWidth - 1, 10, SSD1306_WHITE);
  gDisplay.setTextSize(2);
  gDisplay.setCursor(4, 18);
  gDisplay.println(F("INTRUDER"));
  gDisplay.setCursor(4, 42);
  gDisplay.println(F("DETECTED"));
  gDisplay.display();
}

void StatusOutputs::showEmergencyStop()
{
  if (!oledReady_) return;

  gDisplay.clearDisplay();
  gDisplay.setTextColor(SSD1306_WHITE);
  gDisplay.setTextSize(1);
  gDisplay.setCursor(15, 0);
  gDisplay.println(F("[ EMERGENCY ]"));
  gDisplay.drawLine(0, 10, Config::kOledWidth - 1, 10, SSD1306_WHITE);
  gDisplay.setTextSize(2);
  gDisplay.setCursor(22, 21);
  gDisplay.println(F("STOP"));
  gDisplay.setTextSize(1);
  gDisplay.setCursor(6, 52);
  gDisplay.println(F("Send AUTO to resume"));
  gDisplay.display();
}

void StatusOutputs::showVerified(const char* name)
{
  if (!oledReady_) return;

  gDisplay.clearDisplay();
  gDisplay.setTextColor(SSD1306_WHITE);
  gDisplay.setTextSize(1);
  gDisplay.setCursor(24, 0);
  gDisplay.println(F("[ VERIFIED ]"));
  gDisplay.drawLine(0, 10, Config::kOledWidth - 1, 10, SSD1306_WHITE);
  gDisplay.setTextSize(2);
  gDisplay.setCursor(0, 24);
  gDisplay.println(name);
  gDisplay.display();
}

void StatusOutputs::showArrivalReached()
{
  if (!oledReady_) return;

  gDisplay.clearDisplay();
  gDisplay.setTextColor(SSD1306_WHITE);
  gDisplay.setTextSize(1);
  gDisplay.setCursor(26, 0);
  gDisplay.println(F("[ ARRIVED ]"));
  gDisplay.drawLine(0, 10, Config::kOledWidth - 1, 10, SSD1306_WHITE);
  gDisplay.setTextSize(2);
  gDisplay.setCursor(8, 20);
  gDisplay.println(F("ROUTE"));
  gDisplay.setCursor(8, 42);
  gDisplay.println(F("DONE"));
  gDisplay.display();
}

void StatusOutputs::centerServo()
{
  attachServoIfNeeded();
  writeServoNow(Config::kServoCenterDeg);
  delay(350);
  detachServo();
  Serial.print(F("[SERVO] centered at "));
  Serial.print(Config::kServoCenterDeg);
  Serial.println(F(" deg on D39, detached"));
}

void StatusOutputs::tiltServoTo(int angle, bool holdAfterMove)
{
  targetServoDeg_ = constrain(angle, 0, 180);
  servoHoldAfterMove_ = holdAfterMove;
  servoTargetReachedMs_ = 0;
  attachServoIfNeeded();
}

void StatusOutputs::detachServo()
{
  if (servoAttached_)
  {
    gCameraServo.detach();
    servoAttached_ = false;
    servoTargetReachedMs_ = 0;
  }
}

bool StatusOutputs::servoAtTarget() const
{
  return currentServoDeg_ == targetServoDeg_;
}

void StatusOutputs::runLedTest()
{
  Serial.println(F("[LED] FastLED color test on D24"));
  setLedRed();
  delay(180);
  setLedGreen();
  delay(180);
  setLed(CRGB::Blue);
  delay(180);
  setLedWhite();
  delay(180);
  setLedBlack();
  delay(120);
}

void StatusOutputs::runServoSweep()
{
  Serial.println(F("[SERVO] SG90 sweep on D39"));
  attachServoIfNeeded();
  for (int angle = Config::kServoCenterDeg; angle <= Config::kServoUpDeg; angle += Config::kServoStepDeg)
  {
    writeServoNow(angle);
    delay(Config::kServoStepDelayMs);
  }
  delay(200);
  for (int angle = Config::kServoUpDeg; angle >= Config::kServoCenterDeg; angle -= Config::kServoStepDeg)
  {
    writeServoNow(angle);
    delay(Config::kServoStepDelayMs);
  }
  writeServoNow(Config::kServoCenterDeg);
  delay(350);
  detachServo();
  Serial.println(F("[SERVO] sweep done, centered and detached"));
}

bool StatusOutputs::oledReady() const
{
  return oledReady_;
}

bool StatusOutputs::beginOledAtAddress(uint8_t address)
{
  if (!isI2cPresent(address))
  {
    Serial.print(F("[OLED] No I2C ACK at 0x"));
    Serial.println(address, HEX);
    return false;
  }

  if (!gDisplay.begin(SSD1306_SWITCHCAPVCC, address))
  {
    return false;
  }

  activeOledAddress_ = address;
  Serial.print(F("[OLED] SSD1306 found at 0x"));
  Serial.println(address, HEX);
  return true;
}

bool StatusOutputs::isI2cPresent(uint8_t address)
{
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void StatusOutputs::attachServoIfNeeded()
{
  if (!servoAttached_)
  {
    gCameraServo.attach(Config::kServoPin);
    servoAttached_ = true;
    gCameraServo.write(currentServoDeg_);
    delay(20);
  }
}

void StatusOutputs::writeServoNow(int angle)
{
  currentServoDeg_ = constrain(angle, 0, 180);
  targetServoDeg_ = currentServoDeg_;
  servoTargetReachedMs_ = 0;
  gCameraServo.write(currentServoDeg_);
}
