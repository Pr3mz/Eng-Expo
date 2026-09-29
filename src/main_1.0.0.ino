#include <WiFi.h>
#include <WiFiUDP.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <InEngMotor.h>
#include <esp_arduino_version.h>

// ==========================================
//               CONFIGURATION
// ==========================================
#define WIFI_SSID "Premwei"
#define WIFI_PASS "Premzaza"
#define UDP_CMD_PORT 4210
#define UDP_TLM_PORT 4211

// Pin Definitions
#define PIN_LEFT_ARM 19
#define PIN_RIGHT_ARM 32
// Tuning Parameters
#define MOTOR_MAX_SPEED 200
#define MOTOR_ACCEL_STEP 4
#define SERVO_FREQ_HZ 50
#define SERVO_RES 16
#define UDP_TIMEOUT_MS 500
#define MOTOR_UPDATE_MS 10
// ==========================================

// LEDC Channels for Servos (Avoid 0-3 used by InEngMotor)
#define LEFT_ARM_CH 4
#define RIGHT_ARM_CH 5

void servoWrite(int channel, int angle)
{
    // Map Servo pulse and angle
    int pulseWidth = map(angle, 0, 180, 500, 2500);

    // Calculate Duty Cycle (20000UL = 1000000 / 50Hz)
    uint32_t duty = (pulseWidth * 65535UL) / 20000UL;

    // Set the servo angle via LEDC channel
    ledcWrite(channel, duty);
}

// --- Globals ---
WiFiUDP udp;
TFT_eSPI tft = TFT_eSPI();

bool leftArmState = false;
bool rightArmState = false;

int currentLeftSpeed = 0;
int currentRightSpeed = 0;
int targetLeftSpeed = 0;
int targetRightSpeed = 0;

unsigned long lastMotorUpdate = 0;
unsigned long lastUdpTime = 0;

char incomingPacket[255];
IPAddress lastClientIP;
bool clientConnected = false;

String currentStatus = "IDLE";

// --- Function Prototypes ---
void displayIdle();
void processUDPCommands();
void handleSafetyTimeouts();
void updateMotors();

void setup()
{
    // 1. Initialize Display
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);

    // 2. Initialize Motors and Servos
    inengmotor.begin();

    // Core 2.x: ledcSetup(channel, freq, resolution) + ledcAttachPin(pin, channel)
    ledcSetup(LEFT_ARM_CH, SERVO_FREQ_HZ, SERVO_RES);
    ledcAttachPin(PIN_LEFT_ARM, LEFT_ARM_CH);

    ledcSetup(RIGHT_ARM_CH, SERVO_FREQ_HZ, SERVO_RES);
    ledcAttachPin(PIN_RIGHT_ARM, RIGHT_ARM_CH);

    // Start in CLOSED state (0 degrees)
    servoWrite(LEFT_ARM_CH, 0);
    servoWrite(RIGHT_ARM_CH, 0);

    // 4. Initialize Wi-Fi
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft.println("Booting System...");
    tft.print("WiFi: ");
    tft.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        tft.print(".");
    }

    displayIdle();

    // 5. Start UDP Server
    udp.begin(UDP_CMD_PORT);
    tft.println("\nUDP Port " + String(UDP_CMD_PORT) + " OK");
    delay(1000);
}

void loop()
{
    processUDPCommands();
    handleSafetyTimeouts();
    updateMotors();
}

/**
 * @brief Drains the UDP buffer and processes the latest single-character command.
 */
void processUDPCommands()
{
    int packetSize = udp.parsePacket();
    char cmd = '\0';

    // Drain queue for zero lag
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
        switch (cmd)
        {
        case 'F':
            targetLeftSpeed = MOTOR_MAX_SPEED;
            targetRightSpeed = MOTOR_MAX_SPEED;
            currentStatus = "UDP FWD";
            break;
        case 'B':
            targetLeftSpeed = -MOTOR_MAX_SPEED;
            targetRightSpeed = -MOTOR_MAX_SPEED;
            currentStatus = "UDP REV";
            break;
        case 'L':
            targetLeftSpeed = -MOTOR_MAX_SPEED;
            targetRightSpeed = MOTOR_MAX_SPEED;
            currentStatus = "UDP LEFT";
            break;
        case 'R':
            targetLeftSpeed = MOTOR_MAX_SPEED;
            targetRightSpeed = -MOTOR_MAX_SPEED;
            currentStatus = "UDP RIGHT";
            break;
        case 'S':
            targetLeftSpeed = 0;
            targetRightSpeed = 0;
            currentStatus = "UDP STOP";
            break;
        case 'O':
            leftArmState = false;
            servoWrite(LEFT_ARM_CH, 180); // Open
            currentStatus = "L-ARM OPEN (180)";
            break;
        case 'C':
            leftArmState = true;
            servoWrite(LEFT_ARM_CH, 0); // Close
            currentStatus = "L-ARM CLOSED (0)";
            break;
        }
    }
}

/**
 * @brief Stops motors if no UDP packets have been received recently.
 */
void handleSafetyTimeouts()
{
    if (millis() - lastUdpTime > UDP_TIMEOUT_MS)
    {
        targetLeftSpeed = 0;
        targetRightSpeed = 0;
        if (currentStatus.startsWith("UDP"))
        {
            currentStatus = "IDLE (TIMEOUT)";
        }
    }
}

/**
 * @brief Applies the target speeds to the hardware drivers at a safe update rate.
 */
void updateMotors()
{
    if (millis() - lastMotorUpdate > MOTOR_UPDATE_MS)
    {
        lastMotorUpdate = millis();
        currentLeftSpeed = targetLeftSpeed;
        currentRightSpeed = targetRightSpeed;
        inengmotor.drive(currentLeftSpeed, currentRightSpeed);
    }
}

/**
 * @brief Renders the idle / ready screen showing IP address.
 */
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
