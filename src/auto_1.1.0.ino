#include <WiFi.h>
#include <WiFiUDP.h>
#include <TFT_eSPI.h>
#include <InEngMotor.h>

// ==========================================
//               CONFIGURATION
// ==========================================
const char *ssid = "Pr3mz_2.4G";
const char *password = "Premzaza0967";

const int UDP_PORT = 4210;

// Hardware Pins
const int LEFT_ARM_PIN = 19;
const int RIGHT_ARM_PIN = 32;

// LEDC Channels for Servos (Avoid 0-3 used by InEngMotor)
const int LEFT_ARM_CH = 4;
const int RIGHT_ARM_CH = 5;

// Kinematics & LERP Velocity Profiling
const int MAX_SPEED = 200;
const float LERP_FACTOR = 0.05; // 5% approach per 10ms loop

// ==========================================
//               GLOBALS
// ==========================================
WiFiUDP udp;
TFT_eSPI tft = TFT_eSPI();

float currentLeftSpeed = 0.0;
float currentRightSpeed = 0.0;
int targetLeftSpeed = 0;
int targetRightSpeed = 0;

unsigned long lastMotorUpdate = 0;
unsigned long lastUdpTime = 0;
const unsigned long UDP_TIMEOUT_MS = 500;

// --- Native LEDC Servo Wrapper (Core 2.x) ---
void servoWrite(int channel, int angle)
{
  // Map Servo pulse and angle
  int pulseWidth = map(angle, 0, 180, 500, 2500);

  // Calculate Duty Cycle (20000UL = 1000000 / 50Hz)
  uint32_t duty = (pulseWidth * 65535UL) / 20000UL;

  // Set the servo angle
  ledcWrite(channel, duty);
}

// ==========================================
//               SETUP
// ==========================================
void setup()
{
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);

  // Initialize DC Motors (Reserves LEDC 0, 1, 2, 3)
  inengmotor.begin();

  // Initialize Servos (Core 2.x native LEDC)
  ledcSetup(LEFT_ARM_CH, 50, 16);
  ledcAttachPin(LEFT_ARM_PIN, LEFT_ARM_CH);

  ledcSetup(RIGHT_ARM_CH, 50, 16);
  ledcAttachPin(RIGHT_ARM_PIN, RIGHT_ARM_CH);

  // Start in OPEN state (0 degrees)
  servoWrite(LEFT_ARM_CH, 0);
  servoWrite(RIGHT_ARM_CH, 0);

  // Connect WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.println("ESP32 ONLINE");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("No HuskyLens.");
  tft.println(WiFi.localIP().toString());

  udp.begin(UDP_PORT);
}

// ==========================================
//               MAIN LOOP
// ==========================================
void loop()
{
  // 1. Drain UDP Queue (Zero-Lag)
  int packetSize = udp.parsePacket();
  char cmd = '\0';
  while (packetSize)
  {
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 255);
    if (len > 0)
    {
      cmd = incomingPacket[0];
      lastUdpTime = millis();
    }
    packetSize = udp.parsePacket();
  }

  // 2. Timeout Safety
  if (millis() - lastUdpTime > UDP_TIMEOUT_MS)
  {
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
  }
  // 3. Process Host PC Commands
  else if (cmd != '\0')
  {
    if (cmd == 'F')
    {
      targetLeftSpeed = MAX_SPEED;
      targetRightSpeed = MAX_SPEED;
    }
    else if (cmd == 'B')
    {
      targetLeftSpeed = -MAX_SPEED;
      targetRightSpeed = -MAX_SPEED;
    }
    else if (cmd == 'L')
    {
      targetLeftSpeed = -MAX_SPEED;
      targetRightSpeed = MAX_SPEED;
    }
    else if (cmd == 'R')
    {
      targetLeftSpeed = MAX_SPEED;
      targetRightSpeed = -MAX_SPEED;
    }
    else if (cmd == 'S')
    {
      targetLeftSpeed = 0;
      targetRightSpeed = 0;
    }
    else if (cmd == 'C')
    {
      // CLOSE GRIPPER
      servoWrite(LEFT_ARM_CH, 180);
      servoWrite(RIGHT_ARM_CH, 180);
    }
    else if (cmd == 'O')
    {
      // OPEN GRIPPER
      servoWrite(LEFT_ARM_CH, 0);
      servoWrite(RIGHT_ARM_CH, 0);
    }
  }

  // 4. Smooth LERP Kinematic Velocity Profiler (Non-Blocking)
  if (millis() - lastMotorUpdate > 10)
  {
    lastMotorUpdate = millis();

    // Linear Interpolation: Move current speed 5% towards target speed every 10ms
    currentLeftSpeed += (targetLeftSpeed - currentLeftSpeed) * LERP_FACTOR;
    currentRightSpeed += (targetRightSpeed - currentRightSpeed) * LERP_FACTOR;

    inengmotor.drive((int)currentLeftSpeed, (int)currentRightSpeed);
  }
}
