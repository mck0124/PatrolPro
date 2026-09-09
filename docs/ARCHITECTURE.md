# PatrolPro Architecture

This document describes the current PatrolPro hardware/software split, runtime responsibilities, and safety-oriented control flow.

## Design Goal

PatrolPro separates **perception** from **physical control**:

- The **Jetson Nano** performs camera processing, person detection, tracking, face recognition, event capture, and dashboard serving.
- The **Arduino Mega** owns the patrol state machine, sensor interpretation, motor control, and physical status outputs.

This division prevents GPU-heavy vision processing from becoming the authority for real-time movement and hazard handling.

## High-Level Architecture

```mermaid
flowchart TB
    subgraph Jetson[Jetson Nano - Perception Layer]
        CAM[CSI / USB Camera]
        YOLO[TensorRT Person Detection]
        TRACK[IoU Multi-Person Tracker]
        FACE[Face Recognition]
        SNAP[Snapshot + Event Logger]
        WEB[Local Web Dashboard]
        LINK[Arduino Serial Bridge]

        CAM --> YOLO
        YOLO --> TRACK
        TRACK --> FACE
        YOLO --> SNAP
        FACE --> SNAP
        SNAP --> WEB
        FACE --> LINK
    end

    subgraph Arduino[Arduino Mega - Control Layer]
        SERIAL[SerialProtocol]
        CTRL[PatrolController]
        SENSOR[SensorSuite]
        MOTOR[MotorControl]
        STATUS[StatusOutputs]

        SERIAL --> CTRL
        SENSOR --> CTRL
        CTRL --> MOTOR
        CTRL --> STATUS
    end

    LINK <--> SERIAL

    FLAME[Flame Sensor] --> SENSOR
    GAS[Gas Sensor] --> SENSOR
    ULTRA[Ultrasonic Sensors] --> SENSOR
    IR[IR Sensors] --> SENSOR

    MOTOR --> DRIVE[Drive Motors]
    STATUS --> OLED[OLED]
    STATUS --> LED[Status / External LEDs]
    STATUS --> BUZZ[Buzzer]
    STATUS --> SERVO[Camera Servo]
```

## Jetson Runtime

### `detect_people.py`

The main Jetson process coordinates:

- TensorRT-based person detection
- persistent track IDs using IoU matching
- face-verification scheduling
- Jetson ↔ Arduino serial messaging
- snapshot/event logging
- browser dashboard updates
- graceful shutdown around CUDA work

The current detector configuration uses a TensorRT engine with a 320×320 network input. Camera capture is configured for a 1280×720 stream at 30 FPS, while face identification is intentionally performed less frequently than person detection to reduce GPU load.

### Tracking

`PersonTracker` maintains persistent track IDs across frames and removes tracks after a configurable disappearance window. Verification state is tied to the track lifetime, so a person who leaves and re-enters must be verified again.

### Face Recognition

`face_id.py` performs the face identification stage. The main runtime only attempts verification after a person track is sufficiently stable, reducing false triggers from short-lived detections.

### Event Logging and Dashboard

`snapshot_writer.py` persists event data and snapshots. `web_dashboard.py` exposes a lightweight local HTTP interface using Python's standard-library HTTP server plus OpenCV, avoiding a Flask dependency on the Jetson environment.

## Arduino Runtime

The active firmware is located in `PatrolPro_Phase2/` and is split into modules rather than a single monolithic sketch.

| Module | Responsibility |
| --- | --- |
| `Config.h` | Pin mappings, thresholds, servo positions, motor tuning |
| `MotorControl.*` | Forward, reverse, turning, and stop primitives |
| `SensorSuite.*` | Sensor reads and hazard evaluation |
| `StatusOutputs.*` | LEDs, OLED, buzzer, camera servo |
| `SerialProtocol.*` | Serial command parsing and Jetson message handling |
| `PatrolController.*` | High-level state transitions and behavior |

## State and Priority Model

The system is structured so normal patrol can be interrupted by higher-priority events.

```mermaid
stateDiagram-v2
    [*] --> ManualStop
    ManualStop --> Patrol: AUTO
    Patrol --> ManualStop: MANUAL / STOP

    Patrol --> HazardAlert: fire / gas
    Patrol --> PersonVerification: stable person detected
    Patrol --> ObstacleResponse: obstacle threshold reached

    PersonVerification --> Verified: registered face
    PersonVerification --> Unknown: unknown / timeout
    Verified --> Patrol: completion delay

    Unknown --> ManualStop
    HazardAlert --> ManualStop
    ObstacleResponse --> Patrol: path available
```

The exact controller contains more implementation detail, but the important design principle is that hazard and stop conditions can override normal patrol behavior.

## Obstacle Handling

The current firmware includes the following behaviors:

- reduce speed when a front obstacle is approximately 20–35 cm away
- stop at approximately 20 cm or less
- reverse briefly when rear space permits
- choose a turn direction based on side ultrasonic distance
- expose timed manual movement commands for subsystem testing

## Person Verification Flow

```mermaid
sequenceDiagram
    participant Camera
    participant Jetson
    participant Arduino
    participant Outputs

    Camera->>Jetson: video frame
    Jetson->>Jetson: detect + track person
    Jetson->>Arduino: STATUS:Tn:SCANNING:<direction>
    Arduino->>Outputs: stop + scanning indicators

    alt person is off-center
        Arduino->>Outputs: short heading correction
    end

    Jetson->>Jetson: face recognition

    alt registered face
        Jetson->>Arduino: STATUS:Tn:VERIFIED:<name>
        Arduino->>Outputs: green / name / verified audio
    else unknown or timeout
        Jetson->>Arduino: STATUS:Tn:UNKNOWN or FACE_TIMEOUT
        Arduino->>Outputs: stop / red alert / buzzer
    end
```

## Safety-Oriented Behaviors

- Boot defaults to a stopped/manual state rather than immediately moving.
- Autonomous motion requires an explicit `AUTO` command.
- Environmental hazard sensing is handled on the Arduino side.
- Vision and motor control are separated across devices.
- Unknown-person and hazard states produce explicit physical alerts.
- Serial testing commands allow subsystems to be exercised without entering the full autonomous loop.

## Prototype History

The repository keeps several earlier subsystem experiments for engineering traceability:

- `hazard_monitor/` — earlier integrated Arduino prototype
- `wall_bounce/` — motion-control / obstacle-response prototype
- `IR_test/` — IR sensor test
- `Servo_test/` — camera-servo test

These are not the active production firmware. The active Arduino implementation is `PatrolPro_Phase2/PatrolPro_Phase2.ino`.
