#include <WiFi.h>
#include <WiFiUdp.h>
#include <InEngMotor.h>
#include <TFT_eSPI.h>
#include <esp_arduino_version.h>
#include <cstring>
#include <cstdio>

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef ROBOT_WIFI_SSID
#define ROBOT_WIFI_SSID "Premwei"
#endif
#ifndef ROBOT_WIFI_PASS
#define ROBOT_WIFI_PASS "Premzaza"
#endif

// First fresh firmware for the X-ROVER. The robot joins the same phone hotspot
// as the laptop. Movement is always commanded by the computer and protected by
// a short dead-man timeout; boot and Wi-Fi reconnect never move the wheels.
namespace
{
  // Isolate this firmware from the older controller still sending M packets to 4210.
  constexpr uint16_t UDP_PORT = 4217;
  constexpr uint8_t ROBOT_MARKER_ID = 34;

  constexpr uint8_t SERVO_PIN = 19;
  constexpr uint8_t SERVO_CHANNEL = 4;
  constexpr uint32_t SERVO_HZ = 50;
  constexpr uint8_t SERVO_BITS = 16;
  constexpr uint16_t SERVO_OPEN_US = 900;   // 0 degrees (Open)
  constexpr uint16_t SERVO_CLOSE_US = 2300; // ~180 degrees (Close) - more grip force
  constexpr int MAX_DRIVE = 150;
  constexpr int TURN_SPEED = 145;
  constexpr int ACCEL_STEP = 25;
  constexpr uint32_t MOTOR_UPDATE_MS = 10;
  constexpr uint32_t COMMAND_TIMEOUT_MS = 400;
  constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 18000;
  constexpr uint32_t WIFI_RETRY_MS = 5000;

  WiFiUDP udp;
  TFT_eSPI tft;
  bool wifiConfigured = false;
  bool wifiReady = false;
  bool motorsReady = false;
  bool servoReady = false;
  bool screenReady = false;
  uint32_t lastWifiRetry = 0;
  uint32_t lastMotorUpdate = 0;
  uint32_t lastMotionCommand = 0;
  int targetLeft = 0;
  int targetRight = 0;
  int currentLeft = 0;
  int currentRight = 0;
  IPAddress replyIp;
  uint16_t replyPort = 0;
  char statusLine[24] = "";
  char packet[64];

  void driveWheels(int left, int right)
  {
    if (!motorsReady)
      return;
    inengmotor.drive(left, right);
  }

  void showStatus(const char *line)
  {
    if (!screenReady || strcmp(statusLine, line) == 0)
      return;
    snprintf(statusLine, sizeof(statusLine), "%s", line);
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft.setTextSize(2);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.println("EXPO ROBOT");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.println(statusLine);
    if (wifiReady)
    {
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.println(WiFi.localIP().toString());
    }
  }

  void stopNow()
  {
    targetLeft = targetRight = 0;
    currentLeft = currentRight = 0;
    driveWheels(0, 0);
  }

  void sendReply(const char *message)
  {
    if (!wifiReady || replyPort == 0)
      return;
    udp.beginPacket(replyIp, replyPort);
    udp.print(message);
    udp.endPacket();
  }

  void setGrip(bool close)
  {
    if (!servoReady)
    {
      sendReply("ERR SERVO");
      return;
    }
    const uint16_t pulseUs = close ? SERVO_CLOSE_US : SERVO_OPEN_US;
    const uint32_t duty = (static_cast<uint32_t>(pulseUs) * ((1UL << SERVO_BITS) - 1)) / 20000UL;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWriteChannel(SERVO_CHANNEL, duty);
#else
    ledcWrite(SERVO_CHANNEL, duty);
#endif
    showStatus(close ? "Gripper close" : "Gripper open");
    sendReply(close ? "ACK GRIP CLOSE" : "ACK GRIP OPEN");
  }

  bool connectWifi()
  {
    if (strlen(ROBOT_WIFI_SSID) == 0 || strlen(ROBOT_WIFI_PASS) < 8)
    {
      Serial.println("Wi-Fi credentials are missing in src/secrets.h; motors remain stopped.");
      showStatus("Set hotspot info");
      return false;
    }
    wifiConfigured = true;
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    Serial.println("Joining configured 2.4 GHz hotspot...");
    WiFi.begin(ROBOT_WIFI_SSID, ROBOT_WIFI_PASS);
    const uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS)
    {
      delay(250);
    }
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("Hotspot connection failed; motors remain stopped.");
      showStatus("Wi-Fi retrying");
      return false;
    }
    wifiReady = true;
    udp.begin(UDP_PORT);
    Serial.println("Connected to hotspot.");
    Serial.print("Robot IP: ");
    Serial.println(WiFi.localIP());
    Serial.printf("Safe UDP listener on %u\n", UDP_PORT);
    showStatus("Wi-Fi ready");
    return true;
  }

  void setMotion(int left, int right)
  {
    if (!motorsReady)
    {
      sendReply("ERR MOTORS");
      return;
    }
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);
    if (left == 0 && right == 0)
    {
      stopNow();
      showStatus("Stopped");
    }
    else if (left > 0 && right > 0)
    {
      targetLeft = left;
      targetRight = right;
      showStatus("Forward");
    }
    else if (left < 0 && right < 0)
    {
      targetLeft = left;
      targetRight = right;
      showStatus("Backward");
    }
    else if (left < 0 && right > 0)
    {
      targetLeft = left;
      targetRight = right;
      showStatus("Turn left");
    }
    else if (left > 0 && right < 0)
    {
      targetLeft = left;
      targetRight = right;
      showStatus("Turn right");
    }
    else
    {
      targetLeft = left;
      targetRight = right;
      showStatus("Motor command");
    }
    lastMotionCommand = millis();
    char reply[36];
    snprintf(reply, sizeof(reply), "ACK MOVE %d %d", targetLeft, targetRight);
    sendReply(reply);
  }

  void processPackets()
  {
    int size = udp.parsePacket();
    while (size > 0)
    {
      const int n = udp.read(packet, sizeof(packet) - 1);
      replyIp = udp.remoteIP();
      replyPort = udp.remotePort();
      if (n > 0)
      {
        packet[n] = '\0';
        while (n > 0 && (packet[strlen(packet) - 1] == '\r' || packet[strlen(packet) - 1] == '\n' || packet[strlen(packet) - 1] == ' '))
        {
          packet[strlen(packet) - 1] = '\0';
        }
        if (strcmp(packet, "?") == 0 || strcmp(packet, "PING") == 0)
        {
          char reply[40];
          snprintf(reply, sizeof(reply), "READY %u EXPORED", ROBOT_MARKER_ID);
          sendReply(reply);
        }
        else if (strcmp(packet, "STOP") == 0 || strcmp(packet, "S") == 0)
        {
          setMotion(0, 0);
        }
        else if (strcmp(packet, "OPEN") == 0 || strcmp(packet, "O") == 0)
        {
          setGrip(false);
        }
        else if (strcmp(packet, "CLOSE") == 0 || strcmp(packet, "C") == 0)
        {
          setGrip(true);
        }
        else if (packet[0] == 'M')
        {
          int left = 0;
          int right = 0;
          if (sscanf(packet + 1, "%d %d", &left, &right) == 2)
          {
            setMotion(left, right);
          }
          else
          {
            sendReply("ERR MOVE FORMAT");
          }
        }
        else if (strlen(packet) == 1 && strchr("FBLR", packet[0]) != nullptr)
        {
          switch (packet[0])
          {
          case 'F':
            setMotion(MAX_DRIVE, MAX_DRIVE);
            break;
          case 'B':
            setMotion(-MAX_DRIVE, -MAX_DRIVE);
            break;
          case 'L':
            setMotion(-TURN_SPEED, TURN_SPEED);
            break;
          case 'R':
            setMotion(TURN_SPEED, -TURN_SPEED);
            break;
          }
        }
        else
        {
          sendReply("ERR COMMAND");
        }
      }
      size = udp.parsePacket();
    }
  }

  void updateMotors()
  {
    if (millis() - lastMotorUpdate < MOTOR_UPDATE_MS)
      return;
    lastMotorUpdate = millis();
    
    // Use a larger step for stopping to make it fast but still slightly smooth
    int stepLeft = (targetLeft == 0) ? 100 : ACCEL_STEP;
    int stepRight = (targetRight == 0) ? 100 : ACCEL_STEP;

    if (currentLeft < targetLeft)
      currentLeft = min(currentLeft + stepLeft, targetLeft);
    else if (currentLeft > targetLeft)
      currentLeft = max(currentLeft - stepLeft, targetLeft);
      
    if (currentRight < targetRight)
      currentRight = min(currentRight + stepRight, targetRight);
    else if (currentRight > targetRight)
      currentRight = max(currentRight - stepRight, targetRight);
      
    driveWheels(currentLeft, currentRight);
  }
} // namespace

void setup()
{
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);
  screenReady = true;
  showStatus("Booting - stopped");

  inengmotor.begin();
  motorsReady = true;
  stopNow();
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  servoReady = ledcAttachChannel(SERVO_PIN, SERVO_HZ, SERVO_BITS, SERVO_CHANNEL);
#else
  ledcSetup(SERVO_CHANNEL, SERVO_HZ, SERVO_BITS);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
  servoReady = true;
#endif
  if (!servoReady) {
    Serial.println("Servo PWM unavailable; wheel controls remain available.");
  } else {
    // Reset motor to 0 angle (Open position) every time program starts
    setGrip(false); 
  }
  connectWifi();
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    if (wifiReady)
    {
      wifiReady = false;
      udp.stop();
      replyPort = 0;
      stopNow();
      showStatus("Wi-Fi lost - stop");
      Serial.println("Wi-Fi lost; motors stopped.");
    }
    stopNow();
    if (wifiConfigured && millis() - lastWifiRetry >= WIFI_RETRY_MS)
    {
      lastWifiRetry = millis();
      Serial.println("Retrying hotspot connection...");
      WiFi.reconnect();
    }
    delay(5);
    return;
  }

  if (!wifiReady)
  {
    wifiReady = true;
    udp.begin(UDP_PORT);
    lastMotionCommand = millis();
    Serial.print("Reconnected. Robot IP: ");
    Serial.println(WiFi.localIP());
    showStatus("Wi-Fi ready");
  }

  processPackets();
  if ((targetLeft != 0 || targetRight != 0) && millis() - lastMotionCommand > COMMAND_TIMEOUT_MS)
  {
    targetLeft = targetRight = 0;
    showStatus("Watchdog stop");
  }
  updateMotors();
}
