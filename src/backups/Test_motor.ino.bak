#include <WiFi.h>
#include <WiFiUDP.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <HUSKYLENS.h>
#include <InEngMotor.h>
#include <esp_arduino_version.h>

// Local servo wrapper: fixed to use explicit LEDC channels 4 and 5
// to prevent conflicting with InEngMotor (which uses 0, 1, 2, 3)
class Servo
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
    // In Core 3.x, if attached manually to a channel, use ledcWrite with pin
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

const char *ssid = "Pr3mz_2.4G";
const char *password = "Premzaza0967";
WiFiUDP udp;
const int UDP_PORT = 4210;
const int TELEMETRY_PORT = 4211;
char incomingPacket[255];
IPAddress lastClientIP;
bool clientConnected = false;

TFT_eSPI tft = TFT_eSPI();
HUSKYLENS huskylens;

// Servos for Camera / Arms
Servo leftArm;
Servo rightArm;
const int LEFT_ARM_PIN = 19;
const int RIGHT_ARM_PIN = 32;
bool leftArmState = false;
bool rightArmState = false;

// Smooth Motor Control
const int MAX_SPEED = 200;
const int ACCEL_STEP = 4;
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
int targetLeftSpeed = 0;
int targetRightSpeed = 0;

unsigned long lastMotorUpdate = 0;
unsigned long lastUdpTime = 0;
const unsigned long UDP_TIMEOUT_MS = 500;

String currentStatus = "IDLE";
String lastVisionData = "Searching...";

void setup()
{
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);

  // Initialize Motors using InEngMotor Library (Uses Channels 0, 1, 2, 3)
  inengmotor.begin();

  // Initialize servos (Uses Channels 4, 5)
  leftArm.setPeriodHertz(50);
  rightArm.setPeriodHertz(50);
  leftArm.attach(LEFT_ARM_PIN);
  rightArm.attach(RIGHT_ARM_PIN);
  leftArm.write(0);
  rightArm.write(0);

  // Initialize I2C and HuskyLens
  Wire.begin();
  while (!huskylens.begin(Wire))
  {
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("HuskyLens Init Failed!");
    delay(1000);
  }

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.println("Booting System...");
  tft.print("WiFi: ");
  tft.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    tft.print(".");
  }

  displayIdle();

  udp.begin(UDP_PORT);
  tft.println("\nUDP Port 4210 OK");
  delay(1000);
}

void loop()
{
  // 1. HuskyLens Non-Blocking Poll & Telemetry Send
  if (huskylens.request())
  {
    if (huskylens.isLearned() && huskylens.available())
    {
      HUSKYLENSResult result = huskylens.read();
      if (result.command == COMMAND_RETURN_BLOCK)
      {
        lastVisionData = "Block ID:" + String(result.ID) + " X:" + String(result.xCenter);
      }
      else if (result.command == COMMAND_RETURN_ARROW)
      {
        lastVisionData = "Arrow ID:" + String(result.ID);
      }
    }
    else
    {
      lastVisionData = "Searching...";
    }

    // Send Telemetry Back to Python
    if (clientConnected)
    {
      String telemetry = "HL|" + lastVisionData;
      udp.beginPacket(lastClientIP, TELEMETRY_PORT);
      udp.print(telemetry);
      udp.endPacket();
    }
  }

  // 2. Poll UDP Commands (Python Control) - Drain queue for zero lag
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

  if (cmd != '\0')
  {
    if (cmd == 'F')
    {
      targetLeftSpeed = MAX_SPEED;
      targetRightSpeed = MAX_SPEED;
      currentStatus = "UDP FWD";
    }
    else if (cmd == 'B')
    {
      targetLeftSpeed = -MAX_SPEED;
      targetRightSpeed = -MAX_SPEED;
      currentStatus = "UDP REV";
    }
    else if (cmd == 'L')
    {
      targetLeftSpeed = -MAX_SPEED;
      targetRightSpeed = MAX_SPEED;
      currentStatus = "UDP LEFT";
    }
    else if (cmd == 'R')
    {
      targetLeftSpeed = MAX_SPEED;
      targetRightSpeed = -MAX_SPEED;
      currentStatus = "UDP RIGHT";
    }
    else if (cmd == 'S')
    {
      targetLeftSpeed = 0;
      targetRightSpeed = 0;
      currentStatus = "UDP STOP";
    }
    else if (cmd == 'Q')
    {
      leftArmState = !leftArmState;
      leftArm.write(leftArmState ? 90 : 0);
      currentStatus = "L-ARM TOGGLED";
    }
    else if (cmd == 'E')
    {
      rightArmState = !rightArmState;
      rightArm.write(rightArmState ? 90 : 0);
      currentStatus = "R-ARM TOGGLED";
    }
  }

  // 3. Safety Timeouts
  if (millis() - lastUdpTime > UDP_TIMEOUT_MS)
  {
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
    if (currentStatus.startsWith("UDP"))
      currentStatus = "IDLE (TIMEOUT)";
  }

  // 4. Update Motors Instantly
  if (millis() - lastMotorUpdate > 10)
  {
    lastMotorUpdate = millis();

    currentLeftSpeed = targetLeftSpeed;
    currentRightSpeed = targetRightSpeed;

    inengmotor.drive(currentLeftSpeed, currentRightSpeed);
  }
}

void displayIdle()
{
  currentStatus = "IDLE";
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.println("STATUS: READY");
  tft.println("");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("Control IP:");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.println(WiFi.localIP().toString());
}