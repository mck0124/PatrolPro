# Codebase Notes

> Detailed implementation reference for PatrolPro.
>
> The original full code notes remain available in the repository history. This curated entry point intentionally points readers to the active code and documentation instead of exposing raw working notes at the repository root.

## Active runtime

- [`detect_people.py`](../../detect_people.py) — Jetson Nano person detection, tracking, face-verification orchestration, dashboard updates, and serial messaging.
- [`face_id.py`](../../face_id.py) — SCRFD/TensorRT face detection, embedding extraction, and identity matching.
- [`arduino_link.py`](../../arduino_link.py) — Jetson ↔ Arduino serial bridge.
- [`web_dashboard.py`](../../web_dashboard.py) — lightweight browser dashboard for live status, snapshots, and event review.
- [`snapshot_writer.py`](../../snapshot_writer.py) — event and snapshot persistence.

## Active embedded firmware

The current Arduino Mega implementation lives in [`PatrolPro_Phase2/`](../../PatrolPro_Phase2/).

Key modules:

- `PatrolPro_Phase2.ino` — setup and main loop.
- `PatrolController.*` — patrol and safety state machine.
- `SensorSuite.*` — fire/gas and distance sensing.
- `MotorControl.*` — four-motor motion primitives.
- `StatusOutputs.*` — LED, OLED, buzzer, and servo output handling.
- `SerialProtocol.*` — command parsing and Jetson/Arduino messages.
- `Config.h` — pins, thresholds, timings, and control constants.

## Vision pipeline

The Jetson pipeline uses TensorRT-accelerated person and face inference, persistent track IDs, face verification, and event capture. Platform-specific engine paths and Jetson setup details are documented in [Jetson Notes](JETSON_NOTES.md).

## Historical prototypes

The following directories are retained to show subsystem development and integration work, but they are not the primary runtime:

- [`IR_test/`](../../IR_test/) — IR sensor experiments.
- [`Servo_test/`](../../Servo_test/) — servo experiments.
- [`wall_bounce/`](../../wall_bounce/) — motion-control prototype.
- [`hazard_monitor/`](../../hazard_monitor/) — earlier integrated Arduino implementation.

## Related documentation

- [System Architecture](../ARCHITECTURE.md)
- [Serial Protocol](../SERIAL_PROTOCOL.md)
- [Arduino Firmware Guide](../../PatrolPro_Phase2/README_Phase2.md)
