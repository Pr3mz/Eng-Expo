#include <WiFi.h>
#include <WiFiUdp.h>
#include <TFT_eSPI.h>
#include <esp_arduino_version.h>
#include <InEngMotor.h>
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
  // Wheel pins, PWM and left-wheel inversion come from the InEngMotor library
  // (GPIO 26/27 left, 16/17 right, 20 kHz, LEDC channels 0-3).
  // Gripper servo, same setup as the team's servo test sketch:
  // GPIO 19, 50 Hz, 16-bit, angle 0..180 mapped to 500..2500 us.
  constexpr uint8_t SERVO_PIN = 19;
  constexpr uint8_t SERVO_CHANNEL = 4;   // core 2.x needs an explicit channel; motors use 0-3
  constexpr uint32_t PWM_FREQ = 50;
  constexpr uint8_t PWM_RESOLUTION = 16;
  constexpr int SERVO_OPEN_ANGLE = 0;    // open hands
  constexpr int SERVO_CLOSE_ANGLE = 90;  // close hands
  constexpr uint32_t SERVO_IDLE_RELEASE_MS = 5000; // host silent this long -> servo off
  constexpr int MAX_DRIVE = 255;
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
  bool servoPowered = false;     // true while a pulse train is being sent to the gripper servo
  uint32_t lastHostPacket = 0;
  bool screenReady = false;
  uint32_t lastWifiRetry = 0;
  uint32_t lastMotorUpdate = 0;
  uint32_t lastMotionCommand = 0;
  int targetLeft = 0;
  int targetRight = 0;
  int currentLeft = 0;
  int currentRight = 0;
  // Wheel ramp. Defaults match the original behaviour (auto mode); the host
  // sends "RAMP <accel> <decel> <kick>" to make manual driving smoother.
  int accelStep = ACCEL_STEP;
  int decelStep = ACCEL_STEP;
  int kickPwm = 0;
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

  // Stop the pulse train: the servo goes limp, draws no holding current and
  // cannot cook itself against a stop.
  void releaseServo()
  {
    if (!servoReady)
      return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(SERVO_PIN, 0);
#else
    ledcWrite(SERVO_CHANNEL, 0);
#endif
    servoPowered = false;
  }

  void servoWrite(int angle)
  {
    angle = constrain(angle, 0, 180);
    // Map servo pulse and angle
    const int pulseWidth = map(angle, 0, 180, 500, 2500);
    // Calculate duty cycle
    const uint32_t duty = (pulseWidth * 65535UL) / 20000UL;
    // Set the servo angle
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(SERVO_PIN, duty);
#else
    ledcWrite(SERVO_CHANNEL, duty);
#endif
    servoPowered = true;
  }

  void setGrip(bool close)
  {
    if (!servoReady)
    {
      sendReply("ERR SERVO");
      return;
    }
    servoWrite(close ? SERVO_CLOSE_ANGLE : SERVO_OPEN_ANGLE);
    showStatus(close ? "Gripper close" : "Gripper open");
    sendReply(close ? "ACK GRIP CLOSE" : "ACK GRIP OPEN");
  }

  // "SERVO <angle>": move the gripper to an exact angle (0..180), used by the
  // host's servo tester and its saved open/close angles.
  void setGripAngle(int angle)
  {
    if (!servoReady)
    {
      sendReply("ERR SERVO");
      return;
    }
    angle = constrain(angle, 0, 180);
    servoWrite(angle);
    char reply[24];
    snprintf(reply, sizeof(reply), "ACK SERVO %d", angle);
    sendReply(reply);
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
      if (decelStep >= ACCEL_STEP && kickPwm == 0)
        stopNow(); // original hard stop
      else
        targetLeft = targetRight = 0; // soft stop: updateMotors() ramps down
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
      lastHostPacket = millis();
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
        else if (strcmp(packet, "GOFF") == 0)
        {
          releaseServo();
          showStatus("Gripper off");
          sendReply("ACK GRIP OFF");
        }
        else if (strncmp(packet, "SERVO", 5) == 0)
        {
          int angle = 0;
          if (sscanf(packet + 5, "%d", &angle) == 1)
            setGripAngle(angle);
          else
            sendReply("ERR SERVO FORMAT");
        }
        else if (strncmp(packet, "RAMP", 4) == 0)
        {
          int accel = 0, decel = 0, kick = 0;
          const int fields = sscanf(packet + 4, "%d %d %d", &accel, &decel, &kick);
          if (fields >= 2)
          {
            accelStep = constrain(accel, 1, 255);
            decelStep = constrain(decel, 1, 255);
            kickPwm = fields >= 3 ? constrain(kick, 0, 200) : 0;
            char reply[40];
            snprintf(reply, sizeof(reply), "ACK RAMP %d %d %d", accelStep, decelStep, kickPwm);
            sendReply(reply);
          }
          else
            sendReply("ERR RAMP FORMAT");
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

  // Move one wheel one step toward its target. Speeding up uses accelStep and
  // slowing down (including reversing) uses decelStep. With kickPwm > 0 a start
  // jumps straight past the motor's dead zone and a stop cuts to zero once the
  // wheel is that slow, so the ramp is spent where the wheel actually moves.
  void rampWheel(int &current, int target)
  {
    if (current == target)
      return;
    if (current == 0 && kickPwm > 0)
    {
      current = target > 0 ? min(kickPwm, target) : max(-kickPwm, target);
      return;
    }
    const bool sameSign = (current > 0 && target > 0) || (current < 0 && target < 0);
    const bool speedingUp = current == 0 || (sameSign && abs(target) > abs(current));
    if (!speedingUp && kickPwm > 0 && abs(current) <= kickPwm && (target == 0 || !sameSign))
    {
      current = 0;
      return;
    }
    const int step = speedingUp ? accelStep : decelStep;
    if (current < target)
      current = min(current + step, target);
    else
      current = max(current - step, target);
  }

  void updateMotors()
  {
    if (millis() - lastMotorUpdate < MOTOR_UPDATE_MS)
      return;
    lastMotorUpdate = millis();
    rampWheel(currentLeft, targetLeft);
    rampWheel(currentRight, targetRight);
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
  if (!motorsReady)
  {
    Serial.println("Motor PWM attach failed; wheel commands are disabled.");
    showStatus("Motor PWM error");
  }
  // Set frequency and resolution for the servo pin
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  servoReady = ledcAttach(SERVO_PIN, PWM_FREQ, PWM_RESOLUTION);
#else
  ledcSetup(SERVO_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
  servoReady = true;
#endif
  // Do not send a servo pulse at boot. This avoids an unexpected gripper move.
  if (!servoReady)
    Serial.println("Servo PWM unavailable; wheel controls remain available.");
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
  if (servoPowered && millis() - lastHostPacket > SERVO_IDLE_RELEASE_MS)
  {
    releaseServo(); // host quit or crashed: do not leave the servo holding power
    showStatus("Gripper off");
  }
  updateMotors();
}
