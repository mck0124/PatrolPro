# generate_audio.py
"""
One-shot script: generate all required WAV files for Patrol Pro.

Run on Jetson once before starting the main program:
    python3 generate_audio.py

Requires:  espeak  sox
Install:   sudo apt install espeak sox

Files created in ./audio/:
    alert_fire.wav        — fire alert announcement
    alert_intruder.wav    — intruder alert announcement
    detected_person.wav   — person detected, asking them to approach
    approach_camera.wav   — instruction to look at camera during verification
"""

import os
import shutil
import subprocess
import sys

AUDIO_DIR   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "audio")
SAMPLE_RATE = 48000   # must match your speaker/aplay default

CLIPS = {
    "alert_fire":       "Warning! Fire detected. Please evacuate immediately.",
    "alert_intruder":   "Warning! Unidentified person detected. Security alert activated.",
    "detected_person":  "Person detected. Please stand by for identity verification.",
    "2_detected":       "2 persons detected. Please stand by for identity verification.",
    "3_detected":       "3 persons detected. Please stand by for identity verification.",
    "approach_camera":  "Please look directly at the camera for face verification.",
}

# ── TTS voice settings ────────────────────────────────────────────────────────
ESPEAK_VOICE  = "en+m3"
ESPEAK_SPEED  = 130      # words per minute
ESPEAK_PITCH  = 35       # 0-99


def _check_tools():
    missing = [t for t in ("espeak", "sox") if not shutil.which(t)]
    if missing:
        print(f"[ERROR] Missing tools: {', '.join(missing)}")
        print("Install with:  sudo apt install espeak sox")
        sys.exit(1)


def _generate(name: str, text: str) -> bool:
    """Generate audio/<name>.wav at SAMPLE_RATE Hz. Returns True on success."""
    out_path = os.path.join(AUDIO_DIR, f"{name}.wav")
    tmp_path = out_path + ".tmp.wav"

    try:
        subprocess.run(
            ["espeak", "-v", ESPEAK_VOICE, "-s", str(ESPEAK_SPEED),
             "-p", str(ESPEAK_PITCH), "-w", tmp_path, text],
            check=True, timeout=15,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        subprocess.run(
            ["sox", tmp_path, "-r", str(SAMPLE_RATE), out_path],
            check=True, timeout=15,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        os.remove(tmp_path)
        print(f"  [OK]  {name}.wav")
        return True
    except subprocess.CalledProcessError as exc:
        print(f"  [FAIL] {name}.wav — {exc}")
    except Exception as exc:
        print(f"  [FAIL] {name}.wav — {exc}")
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
    return False


def main():
    _check_tools()
    os.makedirs(AUDIO_DIR, exist_ok=True)
    print(f"Generating audio clips into: {AUDIO_DIR}\n")

    ok = 0
    for name, text in CLIPS.items():
        if _generate(name, text):
            ok += 1

    print(f"\n{ok}/{len(CLIPS)} clips generated.")
    if ok < len(CLIPS):
        print("Fix errors above, then re-run.")
        sys.exit(1)

    print("\nDone. Test a clip with:")
    print(f"  aplay {AUDIO_DIR}/alert_fire.wav")
    print("\nIf you hear nothing, find your device with:  aplay -l")
    print("Then set APLAY_DEVICE in audio_player.py (e.g. 'plughw:1,0')")


if __name__ == "__main__":
    main()
