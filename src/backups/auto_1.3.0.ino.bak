#include <WiFi.h>
#include <WiFiUDP.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <InEngMotor.h>

// ==========================================
//               CONFIGURATION
// ==========================================
#define WIFI_SSID "Premwei"
#define WIFI_PASS "Premzaza"
#define UDP_CMD_PORT 4210

// Single Gripper Servo on Pin 19 (LEDC Channel 4; Channels 0-3 reserved by InEngMotor)
#define ARM_PIN 19
#define ARM_CH 4
#define SERVO_FREQ_HZ 50
#define SERVO_RES 16

// Motor Speeds & LERP Tuning (Proven above stall-torque deadband)
#define FORWARD_SPEED 135
#define TURN_SPEED 120
#define RETREAT_SPEED -130
#define MOTOR_ACCEL_STEP 25
#define MOTOR_UPDATE_MS 10
#define UDP_TIMEOUT_MS 500

// ==========================================
//               GLOBALS
// ==========================================
WiFiUDP udp;
TFT_eSPI tft = TFT_eSPI();

int currentLeftSpeed = 0;
int currentRightSpeed = 0;
int targetLeftSpeed = 0;
int targetRightSpeed = 0;

unsigned long lastMotorUpdate = 0;
unsigned long lastUdpTime = 0;

char incomingPacket[64];
IPAddress hostIp;
uint16_t hostPort = 0;

// Autonomous Action Sequence State
// 0 = Normal Navigation, 1 = Drop & Retreat ('D'), 2 = Grab Payload ('C')
int sequenceState = 0;
unsigned long sequenceStartTime = 0;

// --- Native LEDC Servo Wrapper (ESP32 Arduino Core 2.x) ---
void servoWrite(int channel, int angle)
{
  int pulseWidth = map(angle, 0, 180, 500, 2500);
  uint32_t duty = (pulseWidth * 65535UL) / 20000UL;
  ledcWrite(channel, duty);
}

// --- Send Single-Word UDP Confirmation Back to Host PC ---
void sendUdpReply(const char *msg)
{
  if (hostPort != 0)
  {
    udp.beginPacket(hostIp, hostPort);
    udp.print(msg);
    udp.endPacket();
  }
}

// --- Function Prototypes ---
void processUDPCommands();
void handleSafetyTimeouts();
void updateMotors();
void handleActiveSequence();

// ==========================================
//               SETUP
// ==========================================
void setup()
{
  Serial.begin(115200);

  // 1. Initialize TFT Display
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);

  // 2. Initialize DC Motors (Reserves LEDC Channels 0, 1, 2, 3)
  inengmotor.begin();

  // 3. Initialize Single Gripper Servo (LEDC Channel 4)
  ledcSetup(ARM_CH, SERVO_FREQ_HZ, SERVO_RES);
  ledcAttachPin(ARM_PIN, ARM_CH);

  // Start OPEN (0 degrees)
  servoWrite(ARM_CH, 0);

  // 4. Connect to Wi-Fi
  tft.setCursor(0, 0);
  tft.println("Connecting WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    tft.print(".");
  }

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.println("AUTO 1.3.0 READY");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.println(WiFi.localIP().toString());

  // 5. Start UDP Server
  udp.begin(UDP_CMD_PORT);
  Serial.println("UDP Server Ready on Port 4210");
}

// ==========================================
//               MAIN LOOP
// ==========================================
void loop()
{
  // If executing an autonomous Grab ('C') or Drop-and-Retreat ('D') sequence:
  if (sequenceState != 0)
  {
    handleActiveSequence();
    return;
  }

  processUDPCommands();
  handleSafetyTimeouts();
  updateMotors();
}

/**
 * @brief Executes non-blocking timed sequences for Grab ('C') and Drop & Retreat ('D').
 */
void handleActiveSequence()
{
  unsigned long elapsed = millis() - sequenceStartTime;

  if (sequenceState == 1) // 'D': RELEASE_PAYLOAD & RETREAT
  {
    if (elapsed < 300)
    {
      // Step 1 (0-300ms): Open Gripper (0 deg) & Halt Motors
      servoWrite(ARM_CH, 0);
      targetLeftSpeed = 0;
      targetRightSpeed = 0;
    }
    else if (elapsed < 1100)
    {
      // Step 2 (300-1100ms): Reverse Straight Back for 800ms via LERP
      targetLeftSpeed = RETREAT_SPEED;
      targetRightSpeed = RETREAT_SPEED;
    }
    else
    {
      // Step 3 (1100ms+): Stop Motors & Report DROPPED
      targetLeftSpeed = 0;
      targetRightSpeed = 0;
      currentLeftSpeed = 0;
      currentRightSpeed = 0;
      inengmotor.drive(0, 0);

      sendUdpReply("DROPPED");
      sequenceState = 0;
      lastUdpTime = millis();
      return;
    }
  }
  else if (sequenceState == 2) // 'C': SECURE_GRIP
  {
    if (elapsed < 300)
    {
      // Step 1 (0-300ms): Halt Motors & Close Servo (180 deg active LEDC holding power)
      servoWrite(ARM_CH, 180);
      targetLeftSpeed = 0;
      targetRightSpeed = 0;
    }
    else
    {
      // Step 2 (300ms+): Report GRABBED
      sendUdpReply("GRABBED");
      sequenceState = 0;
      lastUdpTime = millis();
      return;
    }
  }

  // Drain any incoming UDP packets while in sequence so queue stays fresh
  int packetSize = udp.parsePacket();
  while (packetSize)
  {
    udp.read(incomingPacket, 63);
    packetSize = udp.parsePacket();
  }
  lastUdpTime = millis();

  // Keep updating motors via non-blocking LERP timer during sequence
  updateMotors();
}

/**
 * @brief Drains UDP buffer and processes single-character commands ('F','B','L','R','S','C','D','O').
 */
void processUDPCommands()
{
  int packetSize = udp.parsePacket();
  char cmd = '\0';

  while (packetSize)
  {
    int len = udp.read(incomingPacket, 63);
    if (len > 0)
    {
      incomingPacket[len] = '\0';
      cmd = incomingPacket[0];
      hostIp = udp.remoteIP();
      hostPort = udp.remotePort();
      lastUdpTime = millis();
    }
    packetSize = udp.parsePacket();
  }

  if (cmd == '\0')
    return;

  switch (cmd)
  {
  case 'F':
    targetLeftSpeed = FORWARD_SPEED;
    targetRightSpeed = FORWARD_SPEED;
    break;
  case 'B':
    targetLeftSpeed = -FORWARD_SPEED;
    targetRightSpeed = -FORWARD_SPEED;
    break;
  case 'L':
    targetLeftSpeed = -TURN_SPEED;
    targetRightSpeed = TURN_SPEED;
    break;
  case 'R':
    targetLeftSpeed = TURN_SPEED;
    targetRightSpeed = -TURN_SPEED;
    break;
  case 'S':
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
    break;
  case 'C': // Close Gripper & Trigger GRABBED Sequence
    sequenceState = 2;
    sequenceStartTime = millis();
    break;
  case 'D': // Drop Payload & Trigger 800ms Retreat Sequence
  case 'O': // Also support 'O' or "RELEASE_PAYLOAD" (starts with 'R' if full word, so use 'D')
    sequenceState = 1;
    sequenceStartTime = millis();
    break;
  }
}

/**
 * @brief Stops motors if no UDP packets arrive within 500ms.
 */
void handleSafetyTimeouts()
{
  if (millis() - lastUdpTime > UDP_TIMEOUT_MS)
  {
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
  }
}

/**
 * @brief Non-blocking 100Hz LERP motor driver update (prevents InEngMotor lockups and wheel slip).
 */
void updateMotors()
{
  if (millis() - lastMotorUpdate >= MOTOR_UPDATE_MS)
  {
    lastMotorUpdate = millis();

    if (currentLeftSpeed < targetLeftSpeed)
      currentLeftSpeed = min(currentLeftSpeed + MOTOR_ACCEL_STEP, targetLeftSpeed);
    else if (currentLeftSpeed > targetLeftSpeed)
      currentLeftSpeed = max(currentLeftSpeed - MOTOR_ACCEL_STEP, targetLeftSpeed);

    if (currentRightSpeed < targetRightSpeed)
      currentRightSpeed = min(currentRightSpeed + MOTOR_ACCEL_STEP, targetRightSpeed);
    else if (currentRightSpeed > targetRightSpeed)
      currentRightSpeed = max(currentRightSpeed - MOTOR_ACCEL_STEP, targetRightSpeed);

    inengmotor.drive(currentLeftSpeed, currentRightSpeed);
  }
}
