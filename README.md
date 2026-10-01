# ARGOS (Autonomous Robotic Gemstone Overhead Sorter) 🏆

Overhead-camera tracking system (Python, runs on the laptop) and ESP32 firmware for the gemstone color-sorting robot.

Main program: **`host/argos.py`** — sorts gems into the drop circles you mark (default **4 circles**; the color of each circle you click decides which gems go there).

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

  vision.py             arena warp, robot marker, HSV + Roboflow gem detection
  sorter_planner.py     auto state machine: search → grip → deliver → drop
  pilot.py              smooth continuous steering (spin / arc / reverse, creeps up to targets)
  robot_link.py         UDP link to the ESP32 (port 4217)
  gesture_control.py    MediaPipe hand gestures for manual mode
  camera_stream.py, navigation_math.py, safety.py
  assets/               robot marker image + MediaPipe hand model
tools/
  check_colors.py       test gem color detection on saved photos (no robot)
  sim_pilot.py, sim_cycle.py   offline simulations of the auto driving and full sort cycles
  smoke_auto.py         runs the real program loop on a photo with a fake robot
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
| `--sites 4` | Number of drop circles on the field (default 4) |
| `--no-gripper` | Test driving without the servo: no servo commands, each gem is visited once |
| `--colors crimson,violet` | Only look for these gem colors (default: all six) |
| `--gripper-distance-px 110` | Gripper center ahead of the marker (default 110) |
| `--detector roboflow` | Use the Roboflow cloud model (needs `ROBOFLOW_API_KEY`) instead of local HSV |
| `--robot-ip 10.218.230.31` | Robot IP if the hotspot blocks broadcast discovery |
| `--scan-cameras` / `--check-robot` | Check the camera or the robot link without moving |

### Setup Steps
1. **STEP 1:** Click the 4 arena corners.
2. **STEP 2:** Click the center of each drop circle (4 by default). The color is read from the camera.
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
| Arena width / height mm | real size of the field. Set both and restart: the top-down view then has the true proportions (better steering) and the scale is set for you |
| Auto min / max / spin power | wheel power for the auto pilot (min = slowest that still moves the rover) |
| Drop distance mm | how close to the drop circle before opening |
| Cruise / Creep speed | PWM for normal driving / slow final approach |

Changes apply live and save to `host/robot_settings.json` automatically. Command-line flags still override the file for that run.

### Auto mode (test without the servo)

```bash
.venv/bin/python host/argos.py --no-gripper --sites 4
```

1. Click the 4 arena corners, then the 4 drop circles.
2. Press **`G`** and set **Arena width / height mm** (then restart) or **mm per px**; auto stays blocked until a scale is set.
3. Place the robot, wait for the green marker arrow, press **`Space`**.

The robot steers continuously (no stop-and-go): it spins to face the gem, drives in a smooth arc, slows to a creep and stops on it, then goes to the circle with the gem's color. Drive power is the *Auto min / max / spin power* sliders. Gem detection is local HSV; for Roboflow add `--detector roboflow` and set `ROBOFLOW_API_KEY` (workflow `arena-gemstone-rover-detections-1790759099763`).

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
