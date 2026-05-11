# arduino_link.py
"""
Jetson-side USB serial bridge to Arduino Mega.

Architecture:
  - Background thread continuously reads newline-terminated lines from Arduino.
  - Dispatches PLAY / MODE / SNAP commands to registered callbacks.
  - Main thread calls send() to push Jetson event strings to Arduino.
  - Thread-safe: outbound writes go through a queue.Queue.

Jetson -> Arduino events (sent via send()):
  PERSON_DETECTED          new unverified person on screen
  FACE_VERIFIED:<name>     face matched database
  FACE_UNKNOWN             face detected but no DB match (after timeout)
  FACE_TIMEOUT             person left frame without being verified
  ARRIVAL_REACHED          route endpoint QR marker was detected
  EMERGENCY_STOP           latch Arduino in motor-stop mode
  HEARTBEAT                keep-alive ping every 2 s

Arduino -> Jetson commands (received via callback):
  PLAY:<name>              play named audio clip (e.g. alert_fire, verified)
  MODE:<state>             current Arduino mode (PATROL / FIRE_ALERT / ...)
  SNAP:<category>          save a snapshot frame (person / face / verified / unknown / fire)
"""

import threading
import queue
import time
import glob
import os
import serial
from typing import Callable, List, Optional
import audio_player
import snapshot_writer

# ── config ────────────────────────────────────────────────────────────────────
SERIAL_PORT          = "/dev/ttyUSB0"   # try /dev/ttyACM0 if this fails
SERIAL_BAUD          = 115200
RECONNECT_DELAY_S    = 3.0
HEARTBEAT_INTERVAL_S = 2.0
LINE_BUF_SIZE        = 128
PORT_PATTERNS        = ("/dev/ttyUSB*", "/dev/ttyACM*")
MODE_SNAPSHOT_CATEGORIES = {
    "FIRE_ALERT": "fire_immediate",
    "SECURITY_ALERT": "unknown",
}
# ─────────────────────────────────────────────────────────────────────────────


class ArduinoLink:
    """
    Thread-safe serial bridge to Arduino Mega.

    Quick start:
        link = ArduinoLink()
        link.register_command_handler(my_handler)
        link.start()
        ...
        link.send("PERSON_DETECTED")
        link.send("FACE_VERIFIED:David")
        ...
        link.stop()

    Command handler signature:
        def my_handler(cmd: str, arg: str): ...
        # cmd in {"PLAY", "MODE", "SNAP"}
        # arg is the part after ':', e.g. "alert_fire", "PATROL", "person"
    """

    def __init__(self, port: str = SERIAL_PORT, baud: int = SERIAL_BAUD):
        self._port = port
        self._baud = baud
        self._send_queue: queue.Queue = queue.Queue()
        self._handlers: List[Callable] = []
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._connected = False

    # ── public ────────────────────────────────────────────────────────────────

    def register_command_handler(self, handler: Callable):
        """Register a callback for commands arriving from Arduino.
        Called on the reader thread — keep it fast and non-blocking."""
        self._handlers.append(handler)

    def send(self, event: str):
        """Queue a Jetson event to be sent to Arduino (non-blocking, thread-safe)."""
        self._send_queue.put(event.strip() + "\n")

    def is_connected(self) -> bool:
        return self._connected

    def start(self):
        """Start background reader/writer thread."""
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="ArduinoLink", daemon=True)
        self._thread.start()
        print(f"[ArduinoLink] started — {self._port} @ {self._baud}")

    def stop(self):
        """Signal thread to stop and wait for it (max 5 s)."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=5.0)
        print("[ArduinoLink] stopped")

    # ── private ───────────────────────────────────────────────────────────────

    def _run(self):
        while not self._stop_event.is_set():
            ser = None
            try:
                port = self._resolve_port()
                ser = serial.Serial(port, self._baud, timeout=0.1)
                # Arduino Mega resets on USB open — wait for it to boot
                time.sleep(2.0)
                self._connected = True
                self._port = port
                print(f"[ArduinoLink] connected to {self._port}")

                last_heartbeat = 0.0

                while not self._stop_event.is_set():
                    # ── heartbeat ──
                    now = time.time()
                    if now - last_heartbeat >= HEARTBEAT_INTERVAL_S:
                        self._send_queue.put("HEARTBEAT\n")
                        last_heartbeat = now

                    # ── flush outbound queue ──
                    try:
                        while True:
                            msg = self._send_queue.get_nowait()
                            ser.write(msg.encode("ascii"))
                    except queue.Empty:
                        pass

                    # ── read one line (non-blocking with timeout=0.1) ──
                    try:
                        raw = ser.readline()
                    except serial.SerialException:
                        raise
                    if raw:
                        try:
                            line = raw.decode("ascii", errors="ignore").strip()
                        except Exception:
                            continue
                        if line:
                            self._dispatch(line)

            except serial.SerialException as exc:
                self._connected = False
                print(f"[ArduinoLink] serial error: {exc}")
                print(f"[ArduinoLink] retrying in {RECONNECT_DELAY_S} s...")
            finally:
                if ser:
                    try:
                        ser.close()
                    except Exception:
                        pass
                self._connected = False

            self._stop_event.wait(RECONNECT_DELAY_S)

    def _resolve_port(self) -> str:
        """Use the configured port when present; otherwise find Arduino-like USB serial ports."""
        if self._port and os.path.exists(self._port):
            return self._port

        candidates = []
        for pattern in PORT_PATTERNS:
            candidates.extend(glob.glob(pattern))
        candidates = sorted(set(candidates))

        if candidates:
            chosen = candidates[0]
            print(f"[ArduinoLink] {self._port} not found; using {chosen}")
            return chosen

        return self._port

    def _dispatch(self, line: str):
        """Parse inbound line and call registered handlers for PLAY/MODE/SNAP."""
        if ":" in line:
            cmd, _, arg = line.partition(":")
        else:
            cmd, arg = line, ""

        cmd = cmd.strip().upper()
        arg = arg.strip()

        if cmd not in ("PLAY", "MODE", "SNAP"):
            # Pass through informational Arduino prints, suppress empty lines
            if line.strip():
                print(f"  [Arduino] {line}")
            return

        for handler in self._handlers:
            try:
                handler(cmd, arg)
            except Exception as exc:
                print(f"[ArduinoLink] handler exception: {exc}")


# ── default command handler (used by detect_people.py) ────────────────────────

def default_command_handler(cmd: str, arg: str):
    """
    Handles commands sent from Arduino to Jetson.
    PLAY  → audio_player.play(arg)
    MODE  → log current Arduino mode
    SNAP  → trigger snapshot_writer
    """
    if cmd == "PLAY":
        print(f"  [Arduino→Jetson] PLAY:{arg}")
        audio_player.play(arg)

    elif cmd == "MODE":
        print(f"\n{'─'*40}")
        print(f"  [Arduino mode] {arg}")
        print(f"{'─'*40}")
        category = MODE_SNAPSHOT_CATEGORIES.get(arg.upper())
        if category:
            snapshot_writer.save_current(category)

    elif cmd == "SNAP":
        snapshot_writer.save_current(arg)
