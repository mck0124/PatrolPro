# Jetson ↔ Arduino Serial Protocol

PatrolPro uses a lightweight text protocol over USB serial to coordinate the Jetson Nano perception layer with the Arduino Mega control layer.

Default baud rate: **115200**

## Design Principles

- Human-readable messages simplify hardware debugging through Serial Monitor.
- The Arduino remains the authority for patrol state and physical outputs.
- Jetson messages describe perception state rather than directly commanding motor PWM.
- Manual movement commands are retained for controlled subsystem testing.

## Jetson → Arduino Status Messages

### Clear state

```text
STATUS:CLEAR
```

Indicates that no tracked person currently requires verification.

### Scanning / alignment

```text
STATUS:T0:SCANNING:LEFT
STATUS:T0:SCANNING:CENTER
STATUS:T0:SCANNING:RIGHT
```

`T0` is the track identifier. The direction allows the Arduino to perform a short heading correction when the person is substantially off-center before face scanning.

### Verified identity

```text
STATUS:T0:VERIFIED:Noah
```

The Arduino transitions to the verified-user output state and can request the corresponding audio clip.

### Unknown identity

```text
STATUS:T0:UNKNOWN
```

Triggers the unknown/intruder response state.

### Multiple tracks

Status packets may contain more than one track:

```text
STATUS:T0:VERIFIED:Noah,T1:UNKNOWN
```

### Face timeout

```text
FACE_TIMEOUT
```

Used when face verification cannot complete within the expected window.

### Heartbeat

```text
HEARTBEAT
```

Used as a lightweight liveness message between devices.

## Arduino → Jetson Messages

### Mode update

```text
MODE:<state>
```

Reports the Arduino's current high-level operating state.

### Audio playback request

```text
PLAY:<clip_name>
```

Examples:

```text
PLAY:alert_fire
PLAY:alert_intruder
PLAY:verified_noah
```

The Jetson-side audio service maps the requested clip name to the available audio asset.

## Serial Monitor / Manual Control Commands

These commands are intended for calibration, integration testing, and controlled manual operation.

| Command | Function |
| --- | --- |
| `h` | Print help |
| `o` | Print sensor values once |
| `p` | Toggle automatic sensor output |
| `b` | Buzzer test |
| `l` | LED test |
| `v` | Servo sweep test |
| `c` | Return camera servo to default angle |
| `MANUAL` | Enter manual stopped mode |
| `AUTO` | Start autonomous patrol |
| `STOP` | Immediate stop |
| `F 1000` | Drive forward for 1000 ms |
| `B 500` | Reverse for 500 ms |
| `L 800` | Turn left for 800 ms |
| `R 800` | Turn right for 800 ms |
| `D3` | Approximate time-based 3 m forward drive |

## External LED Commands

The Arduino firmware also exposes commands for individually addressing or grouping the external LED array.

```text
E:n:r:g:b
ER:a:b:r:g:b
EA:r:g:b
EOFF
```

Examples:

```text
E:1:255:0:0
E:101:0:0:255
ER:1:100:0:255:0
ER:101:200:255:80:0
EA:10:10:10
EOFF
```

Where:

- `E` controls one LED.
- `ER` controls an inclusive LED range.
- `EA` sets the entire external LED array.
- `EOFF` disables the external LED array.

## Example Verification Exchange

```text
Jetson  -> Arduino: STATUS:T0:SCANNING:LEFT
Arduino -> Jetson:  MODE:VERIFYING

Jetson  -> Arduino: STATUS:T0:VERIFIED:Noah
Arduino -> Jetson:  PLAY:verified_noah
Arduino -> Jetson:  MODE:PATROL
```

## Implementation References

- Jetson serial bridge: [`arduino_link.py`](../arduino_link.py)
- Main Jetson runtime: [`detect_people.py`](../detect_people.py)
- Arduino parser: [`PatrolPro_Phase2/SerialProtocol.cpp`](../PatrolPro_Phase2/SerialProtocol.cpp)
- Patrol state machine: [`PatrolPro_Phase2/PatrolController.cpp`](../PatrolPro_Phase2/PatrolController.cpp)
