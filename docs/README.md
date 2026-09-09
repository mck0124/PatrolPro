# PatrolPro Documentation

This directory contains the design and implementation documentation for PatrolPro. The root [README](../README.md) stays portfolio-focused; deeper engineering material lives here.

## Core documentation

- [System architecture](ARCHITECTURE.md) — Jetson/Arduino responsibility split, state flow, and subsystem design.
- [Serial protocol](SERIAL_PROTOCOL.md) — Jetson ↔ Arduino message contract and control commands.
- [Arduino firmware guide](../PatrolPro_Phase2/README_Phase2.md) — active Phase 2 firmware modules, state behavior, hardware outputs, and tuning.

## Engineering references

- [Codebase notes](reference/CODEBASE_NOTES.md) — active modules, responsibilities, and historical prototypes.
- [Jetson setup and runtime notes](reference/JETSON_NOTES.md) — Jetson Nano environment, TensorRT setup, troubleshooting, and vision-pipeline details.

## Reading order

For a quick technical review, start with the root README, then read the architecture document and the active firmware guide. The reference documents are intended for implementation-level details and environment troubleshooting.
