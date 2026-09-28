# Dual-Camera Autonomous & Manual Pick-and-Place Robot

A dual-mode robotic vehicle developed for the Mineral Yard engineering demonstration. This system features a hybrid architecture allowing for both manual gesture control and full dual-camera automation using overhead ArUco tracking and onboard HuskyLens terminal guidance.

All communications are streamed via zero-lag UDP to an ESP32 microcontroller controlling a dual H-bridge motor driver and twin servo grippers.

---

## 🏗 System Architecture & Modes

The project is split into three primary software pipelines:

### 1. Manual Gesture Control Mode

- **Files:** `mannual_1.0.0.py` & `src/main_1.0.0.ino`
- **Description:** Tracks dual-hand landmarks using Google MediaPipe. Translates finger counts into driving (Fists = Stop, 5 fingers = Forward, 3 = Reverse).
- **Gripping:** Showing 2 fingers on either hand sends a Smart Grip command (`'G'`) which actuates the servos fully open ($180^\circ$) or fully closed ($0^\circ$).

### 2. Autonomous Dual-Camera Mode

- **Files:** `auto_1.1.0.py` & `src/auto_1.1.0.ino`
- **Global Navigation (PC):** An overhead webcam uses Canny Edge Detection to automatically find the arena bounds and compute a Homography matrix (press `'c'` to calibrate). It then tracks the robot via **ArUco ID 30** and navigates it towards a target coordinate using UDP steering commands.
- **Terminal Handover (ESP32):** When the robot is $< 15\text{cm}$ from the target, the PC sends a Local Search command (`'X'`). The ESP32 takes over, polling the **HuskyLens** via I2C to center the gem. It drives forward, actuates the servos upon contact, and sends a `"GRABBED"` telemetry packet back to the PC to yield control.

### 3. YOLOv8 Dataset Capture Tool

- **File:** `capture_dataset.py`
- **Description:** A rapid dataset collection tool for training custom YOLO models. It uses an external webcam (Index 1) to capture images. Features a manual mode (`s`) and an Auto-Burst mode (`a`) that snaps a frame every 0.5 seconds while you move objects around the arena.

---

## 🔌 Hardware Pinout & Specifications

| Component           | ESP32 Pin  | Peripheral / Library  | Function              |
| :------------------ | :--------- | :-------------------- | :-------------------- |
| **Left Motor**      | `26`, `27` | `InEngMotor`          | Forward / Reverse     |
| **Right Motor**     | `17`, `16` | `InEngMotor`          | Forward / Reverse     |
| **Left Arm Servo**  | `19`       | Custom `ledc` Wrapper | Left Gripper Linkage  |
| **Right Arm Servo** | `32`       | Custom `ledc` Wrapper | Right Gripper Linkage |
| **HuskyLens**       | I2C Pins   | `HUSKYLENS.h`         | Onboard Local Vision  |
| **Display**         | SPI Bus    | `TFT_eSPI`            | Telemetry HUD         |

> **Note:** We utilize a custom Servo wrapper on hardware LEDC channels 4 and 5. The official `ESP32Servo` library is completely removed to prevent hardware timer freezing/conflicts with the `InEngMotor` library.

---

## 💻 Software Setup

### Prerequisites

- Python 3.11
- Arduino IDE (v2.x) with the ESP32 Board Package installed

### 1. Host Machine (PC) Setup

1. Clone this repository and create a Python virtual environment:
   ```bash
   python -m venv .venv
   source .venv/bin/activate  # On Windows: .\.venv\Scripts\Activate.ps1
   ```
2. Install the required computer vision libraries:
   ```bash
   pip install opencv-python numpy mediapipe
   ```
3. Update the `ESP32_IP` string in the Python configuration block to match the IP shown on your robot's TFT screen.

### 2. ESP32 Firmware Setup

1. Open the Arduino IDE and install required dependencies via the **Library Manager**:
   - `TFT_eSPI` by Bodmer
   - (Ensure `InEngMotor` and `HUSKYLENS` are placed in your local `libraires/` folder).
2. Configure `TFT_eSPI/User_Setup.h` to match your display driver and comment out the `TOUCH_CS` line to silence warnings.
3. Update your Wi-Fi credentials in the `.ino` sketch:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```
4. Flash `main_1.0.0.ino` for gesture control, or `auto_1.1.0.ino` for the autonomous competition mode.

---

## ⚡ Key Technical Implementations

- **Zero-Lag UDP Transmission:** Commands and Telemetry are broadcast over UDP, dropping round-trip latency below $2\text{ ms}$.
- **Automated Homography:** Eliminates the need for manual 4-point clicks by utilizing OpenCV contour/polygon approximation to warp the physical arena into a perfect 2D grid.
- **PWM Kinematic Interpolation (LERP):** The ESP32 applies continuous, non-blocking velocity interpolation ($\Delta v = 5\%$ per $10\text{ ms}$) to prevent gear shearing and wheel slip during sudden velocity changes.
