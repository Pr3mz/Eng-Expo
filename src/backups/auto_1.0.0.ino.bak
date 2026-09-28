#include <WiFi.h>
#include <WiFiUDP.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <HUSKYLENS.h>
#include <InEngMotor.h>
#include <esp_arduino_version.h>

/*
 * User Instruction: Include ESP32Servo.h.
 * Note: To prevent Hardware PWM Timer conflicts with InEngMotor (which locks timers 0,1,2,3),
 * we use a custom fail-safe Servo wrapper explicitly tied to Channels 4 and 5.
 */
#include <ESP32Servo.h>

class CustomServo
{
public:
  void setPeriodHertz(uint16_t frequency) { periodHertz = frequency; }

  bool attach(int pin)
  {
    servoPin = pin;
    channel = nextChannel++;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttachChannel(servoPin, periodHertz, 16, channel);
    return true;
#else
    ledcSetup(channel, periodHertz, 16);
    ledcAttachPin(servoPin, channel);
    return true;
#endif
  }

  void write(int angle)
  {
    angle = constrain(angle, 0, 180);
    const uint32_t periodUs = 1000000UL / periodHertz;
    const uint32_t pulseUs = 500UL + ((uint32_t)angle * 2000UL) / 180UL;
    const uint32_t duty = (pulseUs * 65535UL) / periodUs;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(servoPin, duty);
#else
    ledcWrite(channel, duty);
#endif
  }

private:
  int servoPin = -1;
  uint16_t periodHertz = 50;
  int channel = -1;
  // Start at channel 4 to avoid InEngMotor's 0-3
  inline static int nextChannel = 4;
};

// Network Config
const char *ssid = "Pr3mz_2.4G";
const char *password = "Premzaza0967";

WiFiUDP udp;
const int UDP_PORT = 4210;
const int TELEMETRY_PORT = 4211;
char incomingPacket[255];
IPAddress lastClientIP;
bool clientConnected = false;

// Hardware
TFT_eSPI tft = TFT_eSPI();
HUSKYLENS huskylens;

CustomServo leftArm;
CustomServo rightArm;
const int LEFT_ARM_PIN = 19;
const int RIGHT_ARM_PIN = 32;

// Kinematics & LERP Velocity Profiling
const int MAX_SPEED = 200;
const float LERP_FACTOR = 0.05; // 5% approach per 10ms loop -> smooth ramping

float currentLeftSpeed = 0.0;
float currentRightSpeed = 0.0;
int targetLeftSpeed = 0;
int targetRightSpeed = 0;

unsigned long lastMotorUpdate = 0;
unsigned long lastUdpTime = 0;
const unsigned long UDP_TIMEOUT_MS = 500;

// State Machine
enum RobotState
{
  GLOBAL_NAV,
  LOCAL_SEARCH
};

RobotState currentState = GLOBAL_NAV;
unsigned long grabTime = 0;
bool grabbing = false;

void displayIdle()
{
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.println("SYS: ONLINE");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println(WiFi.localIP().toString());
}

void setup()
{
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);

  // Initialize DC Motors (Reserves LEDC 0, 1, 2, 3)
  inengmotor.begin();

  // Initialize Servos (Uses custom channels 4, 5)
  leftArm.setPeriodHertz(50);
  rightArm.setPeriodHertz(50);
  leftArm.attach(LEFT_ARM_PIN);
  rightArm.attach(RIGHT_ARM_PIN);
  leftArm.write(0); // Open position
  rightArm.write(0);

  // Initialize HuskyLens I2C
  Wire.begin();
  while (!huskylens.begin(Wire))
  {
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("HuskyLens Init Failed!");
    delay(1000);
  }

  // Connect WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }

  displayIdle();
  udp.begin(UDP_PORT);
}

void loop()
{
  // 1. Drain UDP Queue
  int packetSize = udp.parsePacket();
  char cmd = '\0';
  while (packetSize)
  {
    int len = udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = '\0';
      cmd = incomingPacket[0];
      lastClientIP = udp.remoteIP();
      clientConnected = true;
      lastUdpTime = millis();
    }
    packetSize = udp.parsePacket();
  }

  // 2. State Machine: GLOBAL_NAV vs LOCAL_SEARCH
  if (cmd == 'X')
  {
    currentState = LOCAL_SEARCH;
  }

  if (currentState == GLOBAL_NAV)
  {
    // Process Over-the-Air Global Navigation vectors
    if (millis() - lastUdpTime > UDP_TIMEOUT_MS)
    {
      targetLeftSpeed = 0;
      targetRightSpeed = 0;
    }
    else
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
    }
  }
  else if (currentState == LOCAL_SEARCH)
  {
    // Terminal Guidance Phase
    if (!grabbing)
    {
      bool targetFound = false;
      if (huskylens.request() && huskylens.isLearned() && huskylens.available())
      {
        HUSKYLENSResult result = huskylens.read();
        if (result.command == COMMAND_RETURN_BLOCK)
        {
          targetFound = true;
          int x = result.xCenter;
          int w = result.width;

          // HuskyLens Resolution is 320x240. Center is 160.
          if (x < 140)
          {
            targetLeftSpeed = -100;
            targetRightSpeed = 100; // Micro-pivot Left
          }
          else if (x > 180)
          {
            targetLeftSpeed = 100;
            targetRightSpeed = -100; // Micro-pivot Right
          }
          else
          {
            targetLeftSpeed = 100;
            targetRightSpeed = 100; // Drive Forward
          }

          // Proximity Triggered by block width
          if (w > 80)
          {
            targetLeftSpeed = 0;
            targetRightSpeed = 0;
            // Fire Servos to grab
            leftArm.write(90);
            rightArm.write(90);
            grabbing = true;
            grabTime = millis();
          }
        }
      }

      if (!targetFound)
      {
        // Spin to search for the block
        targetLeftSpeed = 100;
        targetRightSpeed = -100;
      }
    }
    else
    {
      // Waiting for Grab animation to finish
      if (millis() - grabTime > 1500)
      {
        // Ping PC to handover authority back to Global
        if (clientConnected)
        {
          udp.beginPacket(lastClientIP, TELEMETRY_PORT);
          udp.print("GRABBED");
          udp.endPacket();
        }

        // Reset state
        currentState = GLOBAL_NAV;
        grabbing = false;

        // Optionally open arms slightly or keep grabbed
        // leftArm.write(0);
        // rightArm.write(0);
      }
    }
  }

  // 3. Smooth LERP Kinematic Velocity Profiler (Non-Blocking)
  if (millis() - lastMotorUpdate > 10)
  {
    lastMotorUpdate = millis();

    // Linear Interpolation: Move current speed 5% towards target speed every 10ms
    currentLeftSpeed += (targetLeftSpeed - currentLeftSpeed) * LERP_FACTOR;
    currentRightSpeed += (targetRightSpeed - currentRightSpeed) * LERP_FACTOR;

    inengmotor.drive((int)currentLeftSpeed, (int)currentRightSpeed);
  }
}
