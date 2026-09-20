# Eng-Expo
Gesture-Controlled Mineral Yard Robotic BaseA low-latency, dual-hand gesture-controlled robotic vehicle developed for the Mineral Yard engineering demonstration. The system captures dual-hand landmarks using MediaPipe, processes gesture states into telemetry packets, and streams commands over UDP to an ESP32 microcontroller driving a TB67H450 dual H-bridge motor driver and dual servo manipulators.System Architecture[ Webcam Feed ]
       │
       ▼
[ OpenCV Frame Capture ] (Threaded VideoStream)
       │
       ▼
[ MediaPipe Hands (v0.10.14) ]
       │
       ▼
[ Finger Landmark Counter & Gesture State Machine ]
       │
       ▼
[ UDP Socket Broadcast (Port 4210) ]
       │  (Wi-Fi / 802.11 b/g/n)
       ▼
[ ESP32 Receiver (WiFiUDP) ]
       ├── PWM Acceleration Interpolator ──► TB67H450 H-Bridge (DC Drive)
       ├── Direct Servo Angle Mapping    ──► Dual Micro-Servos (Claws)
       └── Non-blocking SPI HUD           ──► ST7789 TFT Display (128x160 / 240x320)
Hardware SpecificationsComponentInterface / PinFunctionMicrocontrollerESP32-WROOM-32Central controller, UDP server, PWM generationMotor DriverToshiba TB67H450Dual DC brushed motor H-bridge driverLeft Drive ForwardGPIO 26Motor PWM Channel 1 (20 kHz, 8-bit resolution)Left Drive BackwardGPIO 27Motor PWM Channel 0 (20 kHz, 8-bit resolution)Right Drive ForwardGPIO 17Motor PWM Channel 2 (20 kHz, 8-bit resolution)Right Drive BackwardGPIO 16Motor PWM Channel 3 (20 kHz, 8-bit resolution)Left Arm ServoGPIO 12Micro-servo for left gripper mechanismRight Arm ServoGPIO 13Micro-servo for right gripper mechanismDisplaySPI (TFT_eSPI)Onboard telemetry HUD (IP, connection status)Gesture Control MappingThe control engine evaluates active finger counts across both hands to determine driving trajectories and arm state toggles:Left Hand CountRight Hand CountMachine StateHardware Action00STOPActive motor braking (target = 0)55FORWARDRamped acceleration up to PWM 25533REVERSERamped reverse driving down to PWM -25510PIVOT LEFTLeft side reverse (-200), Right side forward (+200)01PIVOT RIGHTLeft side forward (+200), Right side reverse (-200)2*TOGGLE LEFT ARMAlternate left servo between $0^\circ$ and $90^\circ$*2TOGGLE RIGHT ARMAlternate right servo between $180^\circ$ and $90^\circ$Software Setup & InstallationPrerequisitesPython 3.11 (64-bit architecture)Arduino IDE (configured with ESP32 board support and required libraries)1. Host Vision Controller (Python)Clone the repository and isolate dependencies in a virtual environment:PowerShell# Clone the repository
git clone https://github.com/Pr3mz/Eng-Expo.git
cd Eng-Expo

# Create and activate Python 3.11 virtual environment
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1

# Install core dependencies (pinned MediaPipe version for Windows stability)
pip install --upgrade pip
pip install opencv-python requests
pip install mediapipe==0.10.14
Configure the ESP32 endpoint in your vision script:PythonESP32_IP = "192.168.1.50"  # Replace with the IP displayed on your ESP32 TFT
UDP_PORT = 4210
Run the controller:PowerShellpython test_vison1.py
2. Microcontroller Firmware (ESP32)Install the required Arduino libraries via the Arduino Library Manager:ESP32ServoTFT_eSPI (configured for your specific display driver and pins in User_Setup.h)Configure the Wi-Fi credentials in the sketch:C++const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
Select ESP32 Dev Module under Tools > Board.Set the correct Serial/COM port.Flash the firmware. (Note: If flashing fails, disconnect any peripheral pulling on strapping pins such as GPIO 0 or GPIO 12 during bootloader entry).Once flashed, note the IP address rendered on the TFT display screen.Technical FeaturesSub-5ms Network Latency: Transitioned from request-response HTTP protocols to non-blocking UDP packets, removing TCP handshake delays and preventing buffer lag.Asynchronous Multi-Threading: Camera acquisition runs on an independent worker thread to decouple hardware frame delivery from MediaPipe graph execution.Smooth Acceleration Interpolator: ESP32 software applies a linear interpolation step ($\Delta v = 8.0 \text{ PWM units per } 10\text{ms}$) to target velocities, preventing violent torque spikes and track slippage.Watchdog Safety Timeout: The ESP32 motor controller halts automatically if no UDP telemetry packet is received for more than 500 ms.
