# ARGOS (Autonomous Robotic Gemstone Overhead Sorter) 🏆

Overhead-camera tracking system (Python, runs on the laptop) and ESP32 firmware for the gemstone color-sorting robot.

Main program: **`host/argos.py`** — currently set to sort **2 colors: red (`crimson`) and violet (`violet`)**.

---

## 📁 Project Layout

```
platformio.ini          ESP32 build config (points at firmware/src)
firmware/src/           ESP32 firmware: ExpoRedCrObot.ino
  secrets.example.h     copy to secrets.h and fill in the Wi-Fi name/password
host/                   Python code that runs on the laptop
  argos.py              main program: camera, setup clicks, manual + auto modes
  vision.py             arena warp, robot marker, HSV gem colors (PALETTE)
  sorter_planner.py     auto state machine: search → grip → deliver → drop
  robot_link.py         UDP link to the ESP32 (port 4217)
  gesture_control.py    MediaPipe hand gestures for manual mode
  camera_stream.py, navigation_math.py, safety.py
  assets/               robot marker image + MediaPipe hand model
tools/
  check_colors.py       test gem color detection on saved photos (no robot)
  test_roboflow.py      test the Roboflow cloud model on a dataset photo
data/
  dataset/raw_images/   arena photos used to train the Roboflow model
  captures/             frames auto-saved during Auto runs (not in git)
```

---

## 🚀 How to Run

```bash
.venv/bin/python -m pip install -r host/requirements.txt
.venv/bin/python host/argos.py --mm-per-pixel <measured value>
```

Useful options:

| Option | Meaning |
|---|---|
| `--colors crimson,violet` | Colors to sort (default). All six: `crimson,violet,gold,lime,cyan,blue` |
| `--gripper-distance-px 110` | Gripper center ahead of the marker (default 110) |
| `--detector roboflow` | Use the Roboflow cloud model (needs `ROBOFLOW_API_KEY`) instead of local HSV |
| `--robot-ip 10.218.230.31` | Robot IP if the hotspot blocks broadcast discovery |
| `--scan-cameras` / `--check-robot` | Check the camera or the robot link without moving |

### Setup Steps
1. **STEP 1:** Click the 4 arena corners.
2. **STEP 2:** Click the center of each active drop circle (right now: red and violet).
3. **READY:** Press `Spacebar` for Auto Mode, or `M` for gesture control.

### Test colors without the robot

```bash
.venv/bin/python tools/check_colors.py data/captures/raw/*.jpg
```

Writes annotated images to `data/color_check/`.

### Firmware

Copy `firmware/src/secrets.example.h` to `firmware/src/secrets.h`, fill in the Wi-Fi details, then build/upload with PlatformIO from the project root.

---

## ⌨️ Hotkeys
- `Spacebar` : Start / Stop Auto Mode
- `[` / `]` : Adjust brightness (clears drop-circle marks)
- `Z` : Re-mark drop circles · `R` : Reset corners and circles
- `H` : Save home position · `B` : Return home and restart
- `M` : Toggle Manual Mode (hand gesture control)
- `Q` : Quit
