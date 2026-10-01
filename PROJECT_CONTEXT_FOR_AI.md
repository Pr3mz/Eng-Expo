# ExpoRedCrObot - Ultimate AI Agent Context & Handoff Document

**ATTENTION ALL AI AGENTS:** This document contains the architectural blueprint, competition rules, and critical hardware physics required to assist the human user in developing this project. Read this thoroughly before suggesting code modifications.

---

## 1. The Competition: "Gemstone Color Sorting Robots"
Based on the official syllabus (Page 6), this project is a university engineering competition.

### 1.1 Objective & Lore
The mission is to sort a pile of unassorted "gemstones" into 6 specific color areas for creating jewelry.
*   **The 6 Colors:** Iridescent Violet, Neon Cyan, Deep Crimson, Marigold Accent, Deep Sky Blue, Lime Green.
*   **Time Limit:** 5 minutes per run.
*   **Visibility Constraint (Blind Driving):** Operators CANNOT directly see the arena or the robot. They must rely *entirely* on a video feed transmitted from an overhead arena camera.

### 1.2 Scoring & Modes
The group must execute 3 runs using two distinct modes:
1.  **Manual Mode (1X Score Base):** The robot is controlled wirelessly via a computer. However, standard keyboards/joysticks are not the primary goal. The team MUST use a **Gesture Control System** utilizing machine learning (camera) to translate human body postures into movement commands.
2.  **Autonomous Mode (5X Score Multiplier):** The robot must operate entirely without human intervention, using the overhead video stream to locate itself, the gemstones, and the drop zones, and then calculate its own path.

---

## 2. How AI & Machine Learning is Utilized in this Project
*   **For Manual Mode (`gesture_control.py`):** We use Google's **MediaPipe Hands**. The AI tracks the operator's hand landmarks to determine gestures (e.g., Pinch to Close gripper, Open Palm to Open gripper, Wrist tilt to drive).
*   **For Autonomous Mode (`vision.py` & `sorter_planner.py`):** We use **Computer Vision (OpenCV)**. 
    *   *Localization:* The robot's position and heading are tracked using an **Aruco Marker (ID: 34)** mounted on its roof.
    *   *Perception:* Homography warps the angled camera feed into a 2D top-down map. HSV color masking identifies the centroids of the gemstones and the 6 drop zones.
    *   *Planning:* A state machine calculates distances and angles (`math.atan2`) to steer the robot via skid-steering towards the targets.

---

## 3. Project File Structure & Module Responsibilities

Layout: `host/` = Python (laptop), `firmware/src/` = ESP32 (built via root `platformio.ini`), `tools/` = offline test scripts, `data/` = dataset + auto-captures. The old `Final1.py` / `Auto_Red_Green.py` were merged into one program, `host/argos.py` (both remain in git history).

### 3.1 Python Codebase (`host/`)
*   **`argos.py`**: The single main program (camera loop, setup clicks, manual + auto). Colors are chosen at launch with `--colors` (or env `ARGOS_COLORS`); `--detector hsv|roboflow` picks local HSV (default) or the Roboflow cloud model (key in env `ROBOFLOW_API_KEY`).
*   **`vision.py`**: The computer vision engine.
    *   *Task:* Robot marker detection (ID 34), Homography calculation, and HSV color thresholding.
    *   *Colors:* `ALL_COLORS` = the 6 competition colors; `PALETTE` = the colors currently sorted, default `("crimson", "violet")`. Always read it as `vision.PALETTE` (it is replaced at startup by `--colors`).
    *   *Rim check:* every gem blob is reclassified against all 6 colors in its neighbourhood, so the red-ish rim of an orange stone is not reported as crimson even when gold is not active.
*   **`sorter_planner.py`**: The Autonomous State Machine.
    *   *States:* `SEARCH` -> `HOMING` -> `APPROACH` -> `GRIP` -> `DROP` -> `HOME`.
    *   *Logic:* Emits action commands (`F`, `B`, `L`, `R`, `C`, `O`). 
*   **`robot_link.py`**: The UDP Network Layer.
    *   *Task:* Wraps Python `socket` to send commands to the ESP32 over a 2.4GHz hotspot.
    *   *Crucial Variable:* `DRIVE_SIGN = {"F": (-1, -1), ...}`. Due to how the camera/marker is mounted, "Forward" actually requires negative PWM values. NEVER change this unless the physical robot is rebuilt.
*   **`gesture_control.py`**: The ML Hand Tracking module (MediaPipe).

### 3.2 Firmware (`firmware/src/ExpoRedCrObot.ino`)
*   **Microcontroller:** ESP32 (Arduino framework via PlatformIO).
*   **Network:** Wi-Fi UDP server listening on port `4217`. (Default IP: `10.218.230.31`).
*   **Motors:** Direct LEDC PWM skid-steer (left 26/27 inverted, right 16/17, 20 kHz). `InEngMotor` is no longer used.
*   **Wi-Fi credentials:** `firmware/src/secrets.h` (not in git; copy from `secrets.example.h`).
*   **Servo:** Hardware PWM (LEDC) on **Pin 19**.
    *   `SERVO_OPEN_US = 1100`
    *   `SERVO_CLOSE_US = 1950`
*   **Safety:** 1000ms watchdog timeout stops motors if Wi-Fi drops.

---

## 4. 🚨 Critical Hardware Quirks & Known Issues (Must Read!)

If you are an AI generating code for this robot, you MUST adhere to these physical constraints:

1.  **The UDP Servo Spam Bug (Hardware Freeze):** 
    *   *Problem:* The ESP32 `ledcWrite` function resets the hardware timer. If the Python script sends `CLOSE` UDP packets at 30 FPS, the PWM wave is constantly interrupted, and the servo goes limp (loses torque).
    *   *Solution:* `robot_link.py` and `sorter_planner.py` MUST strictly rate-limit servo commands. Do not send `CLOSE` or `OPEN` more than once per second.
2.  **Skid-Steering Torque & Friction:** 
    *   *Problem:* The robot struggles to overcome static friction when turning on the arena mat.
    *   *Solution:* In `ExpoRedCrObot.ino`, `MAX_DRIVE` MUST be set to `255`. In `host/argos.py`, turning must be done in *pulses* (e.g., `time.sleep(0.18)`) rather than continuous drive, otherwise it violently overshoots the target angle.
3.  **Physical Gripper Offset (`--gripper-distance-px`):**
    *   The Aruco marker is on the roof, but the gripper jaws are extended out front.
    *   `argos.py` now defaults `--gripper-distance-px` to `110` (override with the flag or env `GRIPPER_DISTANCE_PX`) so the AI targets the visual center of the jaws, not the center of the roof.
4.  **Current Problem / Next Steps:**
    *   **Current focus:** sorting 2 colors, **red (`crimson`) and violet**, end to end on the real field. Only those two drop circles are clicked in STEP 2.
    *   Measured hues: red stones ≈ 177–179 / 0–4 (wraps), violet stones ≈ 150–165, orange stones ≈ 12–14 (their rims reach 9–10, inside the crimson range — handled by the rim check). The red arena wall also reads as crimson, so click arena corners *inside* the wall.
    *   Known gap: a clump of touching same-color stones merges into one blob larger than `max_area` and is skipped until the pile spreads.
    *   **Next milestone:** expand to all 6 colors by launching with `--colors crimson,violet,gold,lime,cyan,blue` and tuning on `tools/check_colors.py`.
