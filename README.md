# Gesture-Controlled Mineral Yard Robotic Base

A low-latency, dual-hand gesture-controlled robotic vehicle developed for the Mineral Yard engineering demonstration. The system tracks dual-hand landmarks using Google MediaPipe, translates finger counts into driving and manipulator states, and streams low-latency telemetry over UDP to an ESP32 microcontroller controlling a Toshiba TB67H450 dual H-bridge motor driver and twin servo grippers.

---

## System Architecture

```
[ Webcam Feed (640x480 @ 30 FPS) ]
               │
               ▼
[ Threaded Frame Capture (VideoStream) ]
               │
               ▼
[ MediaPipe Hands (v0.10.14 Pipeline) ]
               │
               ▼
[ Dual-Hand Finger Counter & State Resolver ]
               │
               ▼
[ UDP Socket Broadcast (Port 4210, Sub-2ms) ]
               │  (Wi-Fi 802.11 b/g/n)
               ▼
[ ESP32 Microcontroller (WiFiUDP) ]
       ├── PWM Acceleration Interpolator ──► TB67H450 H-Bridge (DC Motors)
       ├── Non-Blocking Servo Controller ──► Dual Micro-Servos (Claws)
       └── Real-Time Status HUD           ──► ST7789 SPI TFT Display
```

---

## Hardware Pinout & Specifications

| Component | ESP32 Pin | Peripheral / Channel | Function |
| :--- | :--- | :--- | :--- |
| **Left Motor Forward** | `GPIO 26` | LEDC Channel 1 (20 kHz, 8-bit) | TB67H450 IN1 (Left Forward) |
| **Left Motor Reverse** | `GPIO 27` | LEDC Channel 0 (20 kHz, 8-bit) | TB67H450 IN2 (Left Reverse) |
| **Right Motor Forward** | `GPIO 17` | LEDC Channel 2 (20 kHz, 8-bit) | TB67H450 IN3 (Right Forward) |
| **Right Motor Reverse** | `GPIO 16` | LEDC Channel 3 (20 kHz, 8-bit) | TB67H450 IN4 (Right Reverse) |
| **Left Arm Servo** | `GPIO 12` | ESP32Servo Timer | Left Gripper / Sweeper Linkage |
| **Right Arm Servo** | `GPIO 13` | ESP32Servo Timer | Right Gripper / Sweeper Linkage |
| **Display Interface** | SPI Bus | `TFT_eSPI` Default Config | Telemetry HUD & Diagnostics |

---

## Gesture Control Mapping

The host application counts extended fingers across both hands to determine vehicle dynamics and manipulator positions:

### Dual-Hand Navigation Controls
| Left Hand Fingers | Right Hand Fingers | Operational State | Actuator Output |
| :---: | :---: | :--- | :--- |
| `0` (Fist) | `0` (Fist) | **EMERGENCY STOP** | Active regenerative braking (`target = 0`) |
| `5` (Open Palm) | `5` (Open Palm) | **FORWARD** | Smooth acceleration ramp to PWM `+255` |
| `3` | `3` | **REVERSE** | Smooth acceleration ramp to PWM `-255` |

### Single-Hand Auxiliary Controls
| Left Hand Fingers | Right Hand Fingers | Operational State | Actuator Output |
| :---: | :---: | :--- | :--- |
| `1` | `0` | **PIVOT LEFT** | Left PWM `-200`, Right PWM `+200` |
| `0` | `1` | **PIVOT RIGHT** | Left PWM `+200`, Right PWM `-200` |
| `2` | `*` | **TOGGLE LEFT ARM** | Alternate left servo between $0^\circ$ and $90^\circ$ |
| `*` | `2` | **TOGGLE RIGHT ARM** | Alternate right servo between $180^\circ$ and $90^\circ$ |

---

## Software Setup & Environment Configuration

### Prerequisites
* Windows 10/11 (64-bit) or macOS
* Python 3.11 (64-bit architecture recommended for MediaPipe wheel stability)
* Arduino IDE (v2.x) with the ESP32 Board Package installed

---

### 1. Vision Controller Setup (Host Machine)

1. Clone this repository:
   ```powershell
   git clone https://github.com/Pr3mz/Eng-Expo.git
   cd Eng-Expo
   ```

2. Create and activate an isolated Python 3.11 virtual environment:
   ```powershell
   py -3.11 -m venv .venv-py311
   .\.venv-py311\Scripts\Activate.ps1
   ```

3. Install pinned dependencies:
   ```powershell
   python -m pip install --upgrade pip
   pip install opencv-python requests
   pip install mediapipe==0.10.14
   ```

4. Open `test_vison1.py` and configure your robot's network endpoint:
   ```python
   ESP32_IP = "192.168.1.50"  # Match the IP displayed on your ESP32 screen
   UDP_PORT = 4210
   ```

5. Launch the vision controller:
   ```powershell
   python test_vison1.py
   ```

---

### 2. Firmware Compilation & Flashing (ESP32)

1. Open the Arduino IDE and install required dependencies via the **Library Manager**:
   * `ESP32Servo` by Kevin Harrington
   * `TFT_eSPI` by Bodmer

2. Configure `TFT_eSPI/User_Setup.h` to match your display driver (e.g., ST7789 or ILI9341) and pin configuration.

3. Update your Wi-Fi credentials in the `.ino` sketch:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```

4. Select your board under **Tools > Board > esp32 > ESP32 Dev Module**.

5. Connect the ESP32 via USB and flash the firmware.
   > **Note:** If upload errors (`exit status 2`) occur, ensure no external peripherals pull down strapping pins (`GPIO 0`, `GPIO 2`, `GPIO 12`) during bootloader negotiation.

6. Power cycle the robot and verify that the assigned local IP address appears on the TFT screen.

---

## Key Technical Implementations

* **Zero-Lag UDP Transmission:** Replaces blocking HTTP overhead with single-packet datagram broadcasts over UDP port `4210`, dropping command round-trip latency below $2\text{ ms}$.
* **Asynchronous Frame Ingestion:** Uses a dedicated background thread to read webcam frames continuously, isolating camera hardware timing from MediaPipe inference.
* **PWM Interpolation:** The microcontroller applies continuous acceleration smoothing ($\Delta v = 8.0 \text{ PWM units per } 10\text{ ms}$) to prevent gear shearing and wheel slip during sudden gesture transitions.
* **Network Watchdog:** Shuts down motor drives automatically if no valid packet is received within $500\text{ ms}$.
