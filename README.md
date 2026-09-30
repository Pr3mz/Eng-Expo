# ExpoRedCrObot (Field-Ready Edition 🏆)

This project contains the complete, field-ready Python overhead camera tracking system and ESP32 firmware for a six-color autonomous gem-sorting robot.

Main executable: **`Field_Ready_Project/Final1.py`**

---

## ✨ Features (Hybrid AI + CV Architecture)

1. **AI Object Detection (Roboflow):** 
   - Uses an intelligent cloud-based object detection model to identify 6 colored gems, ignoring background noise, reflections, and people.
   - Features a beautiful Roboflow-style UI drawing bounding boxes directly on the live feed.
2. **Emergency CV Fallback:**
   - If the local network fails or the Cloud API times out, the robot **does not freeze**. It instantly falls back to local HSV color segmentation to keep the robot moving during the competition.
3. **Robot Exclusion Zone:**
   - Projects a dynamic 130-pixel barrier around the robot's ArUco marker to instantly delete false-positive AI gem detections on the robot's chassis.
4. **Smart Twin Resolver:**
   - Automatically handles the lighting confusion between Cyan and Blue during the drop-zone calibration step.
5. **Digital Gain Exposure Control:**
   - Software-based brightness multiplier via the `[` and `]` hotkeys for webcams that lock their hardware exposure on Mac/Windows.
6. **Mobile IP Webcam Support:**
   - Supports streaming directly from Android devices by using an HTTP URL as the camera index.

---

## 🚀 How to Run

Open a terminal and run:
```bash
cd Field_Ready_Project
python3 Final1.py
```
*(To use a mobile camera, run: `python3 Final1.py --camera-index http://192.168.x.x:8080/video`)*

### Setup Steps
1. **STEP 1:** Click the 4 corners of the arena to map the perspective (Homography).
2. **STEP 2:** Click the center of all 6 colored drop zones to calibrate the target coordinates.
3. **READY:** Place the robot in the arena, ensure the green ArUco tracker locks on, and press `Spacebar` to start autonomous sorting!

---

## ⌨️ Hotkeys

- `Spacebar` : Start / Stop Auto Mode
- `[` / `]` : Decrease / Increase Brightness
- `Q` : Quit Program
- `R` : Reset Arena Corners
- `Z` : Reset Drop Zones
- `M` : Toggle Manual Mode
  - `W`, `A`, `S`, `D` : Drive
  - `O` / `C` : Open/Close Gripper

---
*Last Updated: Sept 30, 2026 - Cleaned workspace & Integrated Roboflow AI*
