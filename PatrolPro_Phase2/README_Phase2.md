# PatrolPro Phase 2 — Arduino Firmware

This directory contains the current modular Arduino Mega firmware for PatrolPro.

Active sketch:

```text
PatrolPro_Phase2/PatrolPro_Phase2.ino
```

## Module Structure

| Module | Responsibility |
| --- | --- |
| `Config.h` | Pin mappings, sensor thresholds, servo angles, motor PWM tuning |
| `MotorControl.*` | Four-motor forward, reverse, left, right, and stop control |
| `SensorSuite.*` | IR, gas, and ultrasonic sensor reads plus fire/gas hazard evaluation |
| `StatusOutputs.*` | LEDs, buzzer, OLED, and camera servo |
| `SerialProtocol.*` | Serial Monitor commands and Jetson message parsing |
| `PatrolController.*` | Patrol, fire alert, person verification, and intruder state machine |

The earlier integrated Arduino implementation is preserved in `../hazard_monitor/hazard_monitor.ino` for reference. The Phase 2 firmware separates the active implementation into modules to make tuning and integration easier to reason about.

## LED Layout

### Existing status LEDs

- Arduino pin: `24`
- LED count: `24`

These front status LEDs act as an orange scanning animation during face verification and switch to green after a registered user is verified.

### External LED array

- Arduino pin: `17`
- LED count: `200`
- LEDs `1–100`: first board
- LEDs `101–200`: second board

The 24 status LEDs and the 200 external LEDs operate independently.

### Top panel

The top-panel logical coordinates are rotated 180° in software to match the physical installation orientation before being mapped into the global `101–200` range.

Mode-dependent visuals include:

- Patrol: cyan loading-circle animation
- Face verification: blinking eye
- Verified user: smile
- Unknown user: question mark
- Hazard: red exclamation mark

### Rear panel

Rear-panel LEDs `41–100` act as the main status area:

- Patrol: cyan scanner line
- Face verification: orange sweep
- Verified user: green circle
- Unknown visitor: red X
- Hazard: blinking red exclamation mark

Rear-panel LEDs `1–40` are reserved for vehicle-style signals. Brake/indicator groups are defined on both sides of the panel.

During normal operation these groups remain dim red as tail lights. They become brighter red during slowdown/stop states such as person verification, fire, unknown-person alerts, or obstacle stopping. Turn indicators blink orange while turning.

## Boot and Patrol Behavior

The active configuration currently starts autonomous patrol after boot/reset:

```cpp
constexpr bool kStartInPatrol = true;
```

For bench testing or staged hardware integration, set `kStartInPatrol` to `false`; the controller will remain stopped until it receives `AUTO`.

Regardless of startup mode, the firmware accepts explicit `MANUAL`, `AUTO`, and `STOP` commands for operator control.

## Serial Monitor Commands

Default baud rate: **115200**

```text
h              print help
o              print sensor values once
p              toggle continuous sensor output
b              buzzer test
l              LED test
v              servo sweep test
c              return camera servo to its default angle
E:n:r:g:b      set one external LED, n = 1–200
ER:a:b:r:g:b   set inclusive external LED range a–b
EA:r:g:b       set all external LEDs
EOFF           switch all external LEDs off
MANUAL         enter manual stopped mode
AUTO           start autonomous patrol
STOP           immediate stop
F 1000         drive forward for 1 second
B 500          reverse for 0.5 seconds
L 800          turn left for 0.8 seconds
R 800          turn right for 0.8 seconds
D3             approximate time-based 3 m forward drive
```

Example LED commands:

```text
E:1:255:0:0
E:101:0:0:255
ER:1:100:0:255:0
ER:101:200:255:80:0
EA:10:10:10
EOFF
```

## Jetson → Arduino Messages

The current firmware follows the compact `STATUS:` packet format used by `../detect_people.py`. Legacy individual events remain available for manual integration testing.

```text
STATUS:CLEAR
STATUS:T0:SCANNING:LEFT
STATUS:T0:VERIFIED:Noah,T1:UNKNOWN
FACE_TIMEOUT
HEARTBEAT
```

## Arduino → Jetson Messages

```text
MODE:<state>
PLAY:<clip_name>
```

The Jetson runtime uses these messages to track control state and trigger the corresponding audio output.

See [`../docs/SERIAL_PROTOCOL.md`](../docs/SERIAL_PROTOCOL.md) for a consolidated protocol reference.

## Current Behavior

### Normal patrol

- Status LEDs display the normal patrol state.
- OLED shows `PATROL SAFE`.
- Obstacle sensing can slow, stop, reverse, and redirect the robot.

### Fire / gas hazard

When the sensor suite detects a fire or gas condition:

- stop the robot
- flash red LEDs
- activate buzzer
- show an OLED alert
- request `PLAY:alert_fire`

### Person detected

A Jetson packet such as:

```text
STATUS:Tn:SCANNING:<LEFT|CENTER|RIGHT>
```

causes the robot to stop and enter verification mode.

The front 24 LEDs display a scanner animation and the OLED shows the scanning state.

If the detected person is substantially left or right of the camera centerline, the controller performs a short heading correction of approximately **300 ms**, stops, and then begins the face scan.

### Slow face verification

If face verification is delayed for approximately **6 seconds**, the camera servo sweeps upward from its default `100°` position toward `130°` and then returns at the same rate.

### Registered user

```text
STATUS:Tn:VERIFIED:<name>
```

The controller:

- displays green status LEDs
- shows the verified name on the OLED
- requests `PLAY:verified_<name>`
- returns to patrol after approximately 1 second

### Unknown person / timeout

```text
STATUS:Tn:UNKNOWN
FACE_TIMEOUT
```

The controller remains stopped, flashes red status LEDs, sounds the buzzer, and requests:

```text
PLAY:alert_intruder
```
