# Jetson Nano Setup and Runtime Notes

This document captures the platform-specific details needed to run PatrolPro's computer-vision stack on the Jetson Nano.

## Environment

| Component | Project environment |
| --- | --- |
| Device | Jetson Nano |
| OS | Ubuntu 18.04 / JetPack 4.x |
| Python | 3.6 |
| OpenCV | 4.1.1 |
| TensorRT | 8.0.0.1 |
| CUDA | Available through PyCUDA |

The project intentionally keeps TensorRT and PyCUDA outside a generic desktop `requirements.txt` because they are tied to the Jetson software image.

## Vision pipeline

```text
Camera
  ↓
YOLO11n TensorRT person detection
  ↓
IoU-based persistent person tracking
  ↓
SCRFD TensorRT face detection
  ↓
MobileFaceNet TensorRT face embedding
  ↓
Cosine-similarity identity matching
  ↓
Jetson ↔ Arduino status messages + event logging
```

The active runtime is [`detect_people.py`](../../detect_people.py). Face detection and recognition are implemented in [`face_id.py`](../../face_id.py).

## TensorRT engines

The Jetson runtime expects local TensorRT engines for person detection, face detection, and face embedding. Engine files are machine/runtime artifacts and are intentionally excluded from version control.

Example build pattern for dynamic-shape ONNX models:

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=model.onnx \
  --saveEngine=model.engine \
  --fp16 \
  --minShapes=input.1:1x3x640x640 \
  --optShapes=input.1:1x3x640x640 \
  --maxShapes=input.1:1x3x640x640
```

Exact tensor names depend on the exported model; inspect the engine bindings before building when necessary.

## Runtime configuration

Before running on a fresh Jetson, verify:

- TensorRT engine paths in the vision scripts.
- Arduino serial device path, typically `/dev/ttyUSB0` or `/dev/ttyACM0`.
- Face enrollment/database paths.
- CSI/USB camera availability.
- Audio output device when alerts are enabled.

Run with the browser dashboard:

```bash
python3 detect_people.py --web
```

For headless use:

```bash
python3 detect_people.py --web --no-local-display
```

## Face recognition notes

The active face pipeline uses SCRFD for detection and a TensorRT embedding model for identity matching. The project previously evaluated lighter OpenCV-based approaches, but the TensorRT path was selected to avoid CPU-side frame drops on the Jetson Nano.

Enrollment utilities:

- [`capture_enroll.py`](../../capture_enroll.py) — capture face images.
- [`enroll.py`](../../enroll.py) — build/update the local embedding database.

Enrollment images and the generated face database are runtime data and should not be committed.

## Troubleshooting

### `trtexec: command not found`

Use the Jetson TensorRT binary directly or add it to `PATH`:

```bash
export PATH="$PATH:/usr/src/tensorrt/bin"
```

### Dynamic-shape binding errors

If TensorRT reports invalid binding dimensions, ensure the engine was built with matching `--minShapes`, `--optShapes`, and `--maxShapes`, and set the binding shape before allocating buffers.

### Segmentation fault during shutdown

TensorRT/PyCUDA teardown order can matter on Jetson. The runtime explicitly closes detector/embedding contexts and uses a graceful shutdown flag so CUDA work is not interrupted mid-frame.

### Camera fails after a hard crash

Restart the Argus daemon:

```bash
sudo systemctl restart nvargus-daemon
```

### OpenCV feature availability

JetPack 4.x ships an older OpenCV build, so APIs available on newer desktop OpenCV versions may be missing. The project therefore avoids depending on newer convenience APIs in the production path.

## Generated and local-only files

The following should remain local to the Jetson/runtime environment:

```text
*.engine
*.onnx
face_db.pkl
enrollments/
snapshots/
logs/
```

See the root [README](../../README.md), [architecture documentation](../ARCHITECTURE.md), and [serial protocol](../SERIAL_PROTOCOL.md) for the broader system design.
