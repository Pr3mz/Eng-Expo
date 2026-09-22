# Gesture-Controlled Mineral Yard Robotic Base

A low-latency, dual-hand gesture-controlled robotic vehicle developed for the Mineral Yard engineering demonstration. The system tracks dual-hand landmarks using Google MediaPipe, maps real-time finger counts into driving and manipulator states, broadcasts UDP control packets to an ESP32 microcontroller, and streams back live AI vision telemetry from an onboard DFRobot HuskyLens sensor.

---

## System Architecture

```
                       HOST MACHINE (Python 3.11)
┌────────────────────────────────────────────────────────────────────────┐
│  Webcam Feed (640x480 @ ~30 FPS)                                       │
│       │                                                                │
│       ▼                                                                │
│  Threaded Ingestion (VideoStream)                                      │
│       │                                                                │
│       ▼                                                                │
│  MediaPipe Hands (v0.10.14 Pipeline)                                   │
│       │                                                                │
│       ▼                                                                │
│  Dual-Hand Finger Counter & Gesture Mapper                             │
│       │                                       ▲                        │
│       ▼ (UDP Port 4210: Control Commands)     │ (UDP Port 4211: Telemetry)
└───────┬───────────────────────────────────────┴────────────────────────┘
        │  Wi-Fi (802.11 b/g/n)                 │
        ▼                                       │
┌───────────────────────────────────────────────┴────────────────────────┐
│  ESP32 Microcontroller                                                 │
│       ├── UDP Command Receiver & Safety Watchdog (500ms timeout)       │
│       ├── InEngMotor Driver (LEDC Ch 0–3) ──► TB67H450 Dual H-Bridge   │
│       ├── Custom LEDC Servos (LEDC Ch 4–5) ─► Dual Claws/Arm Grippers  │
│       ├── HuskyLens Vision Sensor (I2C)   ──► Telemetry Streamer       │
│       └── Status & IP Diagnostic HUD       ──► ST7789 SPI TFT (240x240) │
└────────────────────────────────────────────────────────────────────────┘
```

---

## Hardware Pinout & Specifications

### 1. Motors & Servos

| Component               | ESP32 Pin | Peripheral / Channel           | Function / Hardware             |
| :---------------------- | :-------- | :----------------------------- | :------------------------------ |
| **Motor A (Left) IN1**  | `GPIO 26` | LEDC Channel 0 (20 kHz, 8-bit) | Left Motor Forward (TB67H450)   |
| **Motor A (Left) IN2**  | `GPIO 27` | LEDC Channel 1 (20 kHz, 8-bit) | Left Motor Reverse (TB67H450)   |
| **Motor B (Right) IN1** | `GPIO 16` | LEDC Channel 2 (20 kHz, 8-bit) | Right Motor Forward (TB67H450)  |
| **Motor B (Right) IN2** | `GPIO 17` | LEDC Channel 3 (20 kHz, 8-bit) | Right Motor Reverse (TB67H450)  |
| **Left Arm Servo**      | `GPIO 19` | LEDC Channel 4 (50 Hz, 16-bit) | Left Gripper / Sweeper Linkage  |
| **Right Arm Servo**     | `GPIO 32` | LEDC Channel 5 (50 Hz, 16-bit) | Right Gripper / Sweeper Linkage |

> **Note on Servo PWM:** To prevent conflicts with `InEngMotor` (which reserves LEDC channels 0–3), the firmware utilizes a dedicated, non-blocking LEDC servo implementation assigned to channels 4 and 5.

### 2. Sensors & Displays

| Peripheral              | ESP32 Pins                                                                    | Protocol / Bus | Notes                               |
| :---------------------- | :---------------------------------------------------------------------------- | :------------- | :---------------------------------- |
| **HuskyLens AI Camera** | `SDA: GPIO 21`, `SCL: GPIO 22`                                                | I2C (`Wire`)   | Real-time object/arrow/tag tracking |
| **ST7789 TFT Display**  | `MOSI: GPIO 23`, `SCLK: GPIO 18`<br>`CS: GPIO 5`, `DC: GPIO 2`, `RST: GPIO 4` | SPI (27 MHz)   | 240x240 Color IPS Display           |

---

## Gesture Control Mapping

The host application (`mannual_1.0.0.py`) tracks dual-hand landmarks and calculates extended finger counts using knuckle/tip geometry:

### Dual-Hand Driving Controls

| Left Hand Fingers | Right Hand Fingers | Command Code | Vehicle Action                              |
| :---------------: | :----------------: | :----------: | :------------------------------------------ |
|    `0` (Fist)     |     `0` (Fist)     |    `'S'`     | **EMERGENCY STOP** (Immediate motor cutoff) |
|  `5` (Open Palm)  |  `5` (Open Palm)   |    `'F'`     | **FORWARD** (Left & Right motors +200 PWM)  |
|        `3`        |        `3`         |    `'B'`     | **REVERSE** (Left & Right motors -200 PWM)  |

### Single-Hand Pivot & Manipulator Controls

| Left Hand Fingers | Right Hand Fingers | Command Code | Vehicle Action                                            | Debounce |
| :---------------: | :----------------: | :----------: | :-------------------------------------------------------- | :------: |
|        `1`        |        `*`         |    `'L'`     | **PIVOT LEFT** (Left -200, Right +200)                    |   None   |
|        `*`        |        `1`         |    `'R'`     | **PIVOT RIGHT** (Left +200, Right -200)                   |   None   |
|        `2`        |        `*`         |    `'Q'`     | **TOGGLE LEFT ARM** ($0^\circ \leftrightarrow 90^\circ$)  | 1.0 sec  |
|        `*`        |        `2`         |    `'E'`     | **TOGGLE RIGHT ARM** ($0^\circ \leftrightarrow 90^\circ$) | 1.0 sec  |

---

## Bidirectional UDP Telemetry Protocol

The system operates over two independent UDP ports for deterministic, zero-latency throughput:

1. **Control Stream (Host $\rightarrow$ ESP32, Port `4210`):**
   - Single-byte ASCII datagrams (`'F'`, `'B'`, `'L'`, `'R'`, `'S'`, `'Q'`, `'E'`).
   - Dispatched at ~30 Hz (`CONFIG["FPS_CAP_DELAY"] = 0.033`).
   - The ESP32 drains incoming buffers each loop cycle to eliminate packet buffering lag.
   - **Safety Watchdog:** If no packet arrives for $>500\text{ ms}$, the ESP32 automatically halts all motors.

2. **Telemetry Stream (ESP32 $\rightarrow$ Host, Port `4211`):**
   - Transmits HuskyLens detection coordinates formatted as:
     - Blocks: `HL|Block ID:<id> X:<xCenter>`
     - Arrows: `HL|Arrow ID:<id>`
     - Idle: `HL|Searching...`
   - Rendered directly onto the host machine's OpenCV HUD.

---

## Software Setup & Environment

### Prerequisites

- Python 3.11 (64-bit recommended)
- [PlatformIO](https://platformio.org/) (recommended) or Arduino IDE v2.x
- Webcam (built-in or USB)

---

### 1. Host Vision Controller Setup

1. **Clone the repository:**

   ```bash
   git clone https://github.com/Pr3mz/Eng-Expo.git
   cd Eng-Expo
   ```

2. **Create and activate a Python 3.11 virtual environment:**

   ```bash
   # macOS / Linux
   python3.11 -m venv .venv
   source .venv/bin/activate

   # Windows (PowerShell)
   py -3.11 -m venv .venv
   .\.venv\Scripts\Activate.ps1
   ```

3. **Install dependencies:**

   ```bash
   pip install -r requirements.txt
   ```

   > [!IMPORTANT]
   > **MediaPipe Version Requirement:** `requirements.txt` strictly pins `mediapipe==0.10.14`. Do not upgrade to `0.10.35+` or `1.0+`, as Google has removed the legacy `mp.solutions.hands` API required by the gesture pipeline.

4. **Configure Target Robot IP:**
   Open [`mannual_1.0.0.py`](file:///Users/prem/Documents/Eng-Expo/Eng-Expo/mannual_1.0.0.py) and ensure `ESP32_IP` matches the IP displayed on your robot's TFT screen:

   ```python
   CONFIG = {
       "ESP32_IP": "192.168.1.128",  # Set to your ESP32's assigned IP
       "UDP_PORT": 4210,
       "TELEMETRY_PORT": 4211,
       "CAMERA_INDEX": 0,
       "FPS_CAP_DELAY": 0.033,
       "ARM_DEBOUNCE_SEC": 1.0,
       "MP_CONFIDENCE": 0.7,
   }
   ```

5. **Launch the vision controller:**

   ```bash
   python mannual_1.0.0.py
   ```

   - Press **`q`** in the HUD window to safely exit.

---

### 2. ESP32 Firmware Flashing

#### Option A: Using PlatformIO (Recommended)

The project includes a ready-to-build [`platformio.ini`](file:///Users/prem/Documents/Eng-Expo/Eng-Expo/platformio.ini) preconfigured with ST7789 display flags, custom library paths, and dependencies:

1. Update Wi-Fi credentials in [`src/main_1.0.0.ino`](file:///Users/prem/Documents/Eng-Expo/Eng-Expo/src/main_1.0.0.ino):

   ```cpp
   #define WIFI_SSID "YOUR_WIFI_SSID"
   #define WIFI_PASS "YOUR_WIFI_PASSWORD"
   ```

2. Compile and upload firmware to the ESP32:

   ```bash
   pio run -t upload
   ```

3. Open the serial monitor:
   ```bash
   pio device monitor
   ```

#### Option B: Using Arduino IDE

1. Open [`src/main_1.0.0.ino`](file:///Users/prem/Documents/Eng-Expo/Eng-Expo/src/main_1.0.0.ino) in Arduino IDE.
2. Ensure the libraries located in [`libraires/`](file:///Users/prem/Documents/Eng-Expo/Eng-Expo/libraires/) are placed in your Arduino libraries folder:
   - `InEngMotor`
   - `HUSKYLENS`
   - `TFT_eSPI`
3. Configure `TFT_eSPI/User_Setup.h` to match the ST7789 SPI pin definitions (`MOSI 23`, `SCLK 18`, `CS 5`, `DC 2`, `RST 4`).
4. Select board **ESP32 Dev Module** and flash.

---

## Troubleshooting & FAQ

- **`AttributeError: module 'mediapipe' has no attribute 'solutions'`**
  - Make sure you are using `mediapipe==0.10.14`. Run `pip install mediapipe==0.10.14` in your active virtual environment.
- **Robot Not Responding to Commands**
  - Check that the host machine and the ESP32 are connected to the same Wi-Fi network (2.4 GHz band recommended).
  - Confirm the IP address on the ESP32 TFT screen matches `ESP32_IP` in `CONFIG`.
  - Check firewall settings on the host machine allowing outgoing UDP on port `4210` and incoming UDP on port `4211`.
- **ESP32 Upload Errors (`exit status 2` / Strapping Pin Issue)**
  - Ensure no external modules pull down strapping pins (`GPIO 0`, `GPIO 2`, `GPIO 12`) during the bootloader flashing sequence.
