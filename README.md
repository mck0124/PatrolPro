# PatrolPro

PatrolPro is a security patrol robot project built around a Jetson Nano and an Arduino Mega.  
The Jetson handles person detection, face recognition, event logging, and the optional web dashboard.  
The Arduino owns the robot state machine and controls the motors, LEDs, OLED, buzzer, servo, and hazard sensors.

## What the system does

- Detects people with YOLO-based vision on the Jetson side
- Verifies registered faces before allowing patrol to continue
- Marks unknown visitors as security events
- Detects fire and gas hazards and prioritizes those alerts over person verification
- Drives in patrol mode with obstacle-aware speed control and basic avoidance
- Logs snapshots and events for later review

## Repository layout

- `detect_people.py`  
  Main Jetson runtime for person detection, tracking, face recognition, serial messaging, snapshots, and dashboard updates.

- `face_id.py`  
  Face detection and recognition pipeline.

- `arduino_link.py`  
  Serial bridge between Jetson and Arduino.

- `web_dashboard.py`  
  Lightweight browser dashboard for live status and event review.

- `PatrolPro_Phase2/`  
  Current modular Arduino Mega firmware.

- `hazard_monitor/`  
  Earlier integrated Arduino sketch kept for reference.

- `wall_bounce/`  
  Standalone patrol-drive prototype used during motion tuning.

## Current Arduino behavior

The active Arduino sketch is:

```text
PatrolPro_Phase2/PatrolPro_Phase2.ino
```

The robot boots in a safe stopped state.  
Automatic patrol starts only after sending `AUTO` over the serial monitor or through the Jetson bridge.

### Drive highlights

- Safe boot in manual stop mode
- Timed manual drive commands for testing
- Front obstacle slowing between 20 cm and 35 cm
- Immediate stop at 20 cm or less
- Short reverse move before turning when rear space is available
- Left/right turn choice based on side ultrasonic distance
- White reverse lights while backing up

### Person verification highlights

- Jetson sends compact `STATUS:` packets to Arduino
- Arduino enters verification mode when a stable person track appears
- If the person is far to the left or right of frame, Arduino applies a short 300 ms heading correction before face scan
- Servo-based face scan starts only after that alignment step is complete

## Setup notes

This repo contains code and documentation, but some runtime paths in the Jetson scripts are still environment-specific.  
Before running on a fresh Jetson, check:

- TensorRT engine paths in `detect_people.py`
- Face enrollment / database paths in `face_id.py`, `enroll.py`, and `capture_enroll.py`
- Arduino serial port in `arduino_link.py` and `detect_people.py`

## Quick start

### Arduino

1. Open `PatrolPro_Phase2/PatrolPro_Phase2.ino`
2. Upload to Arduino Mega
3. Open Serial Monitor at `115200`
4. Use `AUTO` to start patrol, or `F 1000`, `B 500`, `L 800`, `R 800` for timed drive tests

### Jetson

1. Prepare the TensorRT engines and face database
2. Connect the camera and Arduino
3. Run:

```bash
python3 detect_people.py --web
```

If you do not want the local OpenCV window:

```bash
python3 detect_people.py --web --no-local-display
```

## Notes

- `CODE_NOTES.md` is a technical reference for the codebase
- `PatrolPro_Phase2/README_Phase2.md` documents the modular Arduino firmware in more detail
- Local planning files and editor config are intentionally excluded from version control
