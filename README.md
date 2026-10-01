# ARGOS (Autonomous Robotic Gemstone Overhead Sorter) 🏆

Overhead-camera tracking system (Python, runs on the laptop) and ESP32 firmware for the gemstone color-sorting robot.

Main program: **`host/argos.py`** — currently set to sort **2 colors: red (`crimson`) and violet (`violet`)**.

---

## 📁 Project Layout

```
platformio.ini          ESP32 build config (points at firmware/src)
firmware/src/           ESP32 firmware: ExpoRedCrObot.ino (motors via lib/InEngMotor)
  secrets.example.h     copy to secrets.h and fill in the Wi-Fi name/password
host/                   Python code that runs on the laptop
  argos.py              main program: camera, setup clicks, manual + auto modes
  robot_settings.json   robot size, gripper length/size, scale, speeds (edit or press G)
  setup_panel.py        the G slider panel

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

### Robot setup panel (gripper + robot size)

After the arena corners are set, press **`G`**. A slider window opens:

| Slider | What it is |
|---|---|
| Gripper length px | distance from the roof marker to the gripper center (green cross) |
| Gripper size px | radius of the green pickup circle — stone inside = close gripper |
| Robot size px | orange circle around the robot; colors inside are ignored |
| Robot rear px | how far the robot body reaches behind the marker |
| mm per px x100 | arena scale × 100 (arena width in mm ÷ 800). 0 = Auto blocked |
| Drop distance mm | how close to the drop circle before opening |
| Cruise / Creep speed | PWM for normal driving / slow final approach |

Changes apply live and save to `host/robot_settings.json` automatically. Command-line flags still override the file for that run.

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
- `G` : Robot setup panel (gripper length/size, robot size, scale, speeds)
- `M` : Toggle Manual Mode (hand gesture control)
- `Q` : Quit
