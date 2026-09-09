# PatrolPro Documentation

This directory contains the design and implementation documentation for PatrolPro.

## Core documentation

- [System architecture](ARCHITECTURE.md) — Jetson/Arduino responsibility split, state flow, and subsystem design.
- [Serial protocol](SERIAL_PROTOCOL.md) — Jetson ↔ Arduino message contract and control commands.
- [Arduino firmware guide](../PatrolPro_Phase2/README_Phase2.md) — active Phase 2 firmware modules, states, and tuning behavior.

## Engineering references

- `reference/CODEBASE_NOTES.md` — detailed per-file implementation reference.
- `reference/JETSON_NOTES.md` — Jetson Nano environment, TensorRT setup, troubleshooting, and vision-pipeline notes.

The root [README](../README.md) is intentionally kept as the portfolio-level overview; detailed implementation and development material lives here.