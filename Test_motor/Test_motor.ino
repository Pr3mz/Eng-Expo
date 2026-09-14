#include <WiFi.h>
#include <WebServer.h>
#include <TFT_eSPI.h> 
#include <Wire.h>
#include <HUSKYLENS.h>

const char* ssid = "Pr3mz_2.4G";
const char* password = "Premzaza0967";
WebServer server(80);

TFT_eSPI tft = TFT_eSPI();
HUSKYLENS huskylens;

#define LEFT_F 26  
#define LEFT_B 27  
#define RIGHT_F 17 
#define RIGHT_B 16 

const float WHEEL_DIAMETER_CM = 3.5;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER_CM; 
const float TRACK_WIDTH_CM = 8.5;  
const float NO_LOAD_RPM = 160.0;   
const float RPM_DROP_PER_KG = 43.0; 
const int ACCEL_TIME_MS = 300; 

// Live tracking variables for the web dashboard
String currentStatus = "IDLE";
String lastVisionData = "No objects detected";

// --- Blue Themed HTML & Live Web Dashboard ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Precision Controller & Vision</title>
  <style>
    body { font-family: monospace; background: #001f3f; color: #39cccc; text-align: center; margin: 0; padding: 10px; }
    input { width: 60px; padding: 5px; text-align: center; font-weight: bold; background: #001122; color: #7fdbff; border: 1px solid #0074d9; }
    .grid { display: grid; grid-template-columns: repeat(3, 80px); gap: 10px; justify-content: center; margin-top: 15px; }
    button { height: 60px; font-weight: bold; background: #001122; color: #39cccc; border: 2px solid #0074d9; cursor: pointer; border-radius: 8px;}
    button:active { background: #39cccc; color: #001f3f; }
    
    .panel { background: #001122; border: 1px solid #0074d9; border-radius: 8px; padding: 10px; margin: 15px auto; width: 280px; text-align: left; }
    #log { height: 90px; background: #000; color: #7fdbff; overflow-y: scroll; padding: 6px; font-size: 11px; border: 1px solid #0074d9; }
    .status-text { color: #2ecc40; font-weight: bold; }
  </style>
  <script>
    async function sendCmd(cmd) {
      let dist = document.getElementById('dist').value;
      let deg = document.getElementById('deg').value;
      let weight = document.getElementById('weight').value;
      
      let response = await fetch(`/move?cmd=${cmd}&dist=${dist}&deg=${deg}&weight=${weight}`);
      let text = await response.text();
      
      let logBox = document.getElementById('log');
      logBox.innerHTML += "> " + text + "<br>";
      logBox.scrollTop = logBox.scrollHeight;
    }

    // Poll robot status and HuskyLens vision every 1 second
    setInterval(async function() {
      let res = await fetch('/status');
      let data = await res.json();
      document.getElementById('robot-status').innerText = data.status;
      document.getElementById('vision-data').innerText = data.vision;
    }, 1000);
  </script>
</head>
<body>
  <h2>Kinematic Web Console</h2>
  
  <!-- Live Vision & Status Panel -->
  <div class="panel">
    <b>Robot State:</b> <span id="robot-status" class="status-text">IDLE</span><br>
    <b>HuskyLens AI:</b> <span id="vision-data" style="color: #ffdc00;">Scanning...</span>
  </div>

  <div>
    Robot Load: <input type="number" id="weight" value="0.7" step="0.1"> kg<br><br>
    Distance: <input type="number" id="dist" value="10"> cm<br><br>
    Angle: <input type="number" id="deg" value="90"> &deg;
  </div>
  <div class="grid">
    <div></div>
    <button onclick="sendCmd('F')">FWD</button>
    <div></div>
    <button onclick="sendCmd('L')">LEFT</button>
    <button onclick="sendCmd('S')" style="background:#85144b;color:#ff4136;border-color:#ff4136;">STOP</button>
    <button onclick="sendCmd('R')">RIGHT</button>
    <div></div>
    <button onclick="sendCmd('B')">REV</button>
    <div></div>
  </div>

  <div class="panel">
    <div><b>Console Log:</b></div>
    <div id="log"></div>
  </div>
</body>
</html>
)rawliteral";

void setup() {
  tft.init();
  tft.setRotation(1); 
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  
  // Initialize Motors
  ledcAttachChannel(LEFT_F, 20000, 8, 1);
  ledcAttachChannel(LEFT_B, 20000, 8, 0);
  ledcAttachChannel(RIGHT_F, 20000, 8, 2);
  ledcAttachChannel(RIGHT_B, 20000, 8, 3);
  brakeMotors(); 

  // Initialize I2C and HuskyLens
  Wire.begin();
  while (!huskylens.begin(Wire)) {
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
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    tft.print("."); 
  }
  
  displayIdle();

  server.on("/", []() { server.send(200, "text/html", index_html); });
  server.on("/move", handleMove);
  server.on("/status", handleStatus); // Endpoint for website live updates
  server.begin();
}

void loop() {
  server.handleClient();
  
  // Continuously poll HuskyLens in the background
  if (huskylens.request()) {
    if (huskylens.isLearned() && huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();
      if (result.command == COMMAND_RETURN_BLOCK) {
        lastVisionData = "Block ID:" + String(result.ID) + " X:" + String(result.xCenter);
      } else if (result.command == COMMAND_RETURN_ARROW) {
        lastVisionData = "Arrow ID:" + String(result.ID);
      }
    } else {
      lastVisionData = "Searching for Target...";
    }
  }
}

void displayIdle() {
  currentStatus = "IDLE";
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.println("STATUS: IDLE");
  tft.println("");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("Control IP:");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.println(WiFi.localIP().toString());
}

void displayAction(String action, float amount, String unit, unsigned long time_ms) {
  currentStatus = action;
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.println("STATUS: ACTIVE");
  tft.println("");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println(action);
  tft.print("Target: "); tft.print(amount, 1); tft.println(unit);
  tft.print("Time:   "); tft.print(time_ms); tft.println(" ms");
}

void setMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    ledcWrite(LEFT_F, leftSpeed); ledcWrite(LEFT_B, 0);
  } else {
    ledcWrite(LEFT_F, 0); ledcWrite(LEFT_B, -leftSpeed);
  }
  if (rightSpeed >= 0) {
    ledcWrite(RIGHT_F, rightSpeed); ledcWrite(RIGHT_B, 0);
  } else {
    ledcWrite(RIGHT_F, 0); ledcWrite(RIGHT_B, -rightSpeed);
  }
}

void brakeMotors() {
  ledcWrite(LEFT_F, 255); ledcWrite(LEFT_B, 255);
  ledcWrite(RIGHT_F, 255); ledcWrite(RIGHT_B, 255);
}

// Sends live JSON data back to the webpage ticker
void handleStatus() {
  String json = "{\"status\":\"" + currentStatus + "\", \"vision\":\"" + lastVisionData + "\"}";
  server.send(200, "application/json", json);
}

void handleMove() {
  char cmd = server.arg("cmd").charAt(0);
  float dist = server.arg("dist").toFloat();
  float deg = server.arg("deg").toFloat();
  float weight = server.arg("weight").toFloat();

  if (cmd == 'S') {
    server.send(200, "text/plain", "MANUAL STOP");
    brakeMotors();
    displayIdle();
    return;
  }

  float current_rpm = NO_LOAD_RPM - (weight * RPM_DROP_PER_KG);
  if (current_rpm < 40) current_rpm = 40; 
  float velocity_cm_s = (current_rpm / 60.0) * WHEEL_CIRCUMFERENCE; 
  float ms_per_cm = 1000.0 / velocity_cm_s; 

  unsigned long total_time = 0;
  unsigned long cruise_time = 0;
  String logMsg = "";

  if (cmd == 'F' || cmd == 'B') {
    total_time = dist * ms_per_cm;
    logMsg = (cmd == 'F' ? "fd " : "bk ") + String(dist, 1) + "cm " + String(total_time) + "ms";
  } else if (cmd == 'L' || cmd == 'R') {
    float arc_length_cm = (deg / 360.0) * (PI * TRACK_WIDTH_CM);
    total_time = arc_length_cm * ms_per_cm;
    logMsg = (cmd == 'L' ? "lt " : "rt ") + String(deg, 1) + "deg " + String(total_time) + "ms";
  }

  if (total_time > (ACCEL_TIME_MS / 2)) {
    cruise_time = total_time - (ACCEL_TIME_MS / 2);
  } else {
    cruise_time = 0;
  }

  server.send(200, "text/plain", logMsg);

  if (cmd == 'F') {
    displayAction("FORWARD", dist, " cm", total_time);
    softStart('F', 255);
    delay(cruise_time);
  } else if (cmd == 'B') {
    displayAction("REVERSE", dist, " cm", total_time);
    softStart('B', 255);
    delay(cruise_time);
  } else if (cmd == 'L') {
    displayAction("PIVOT LEFT", deg, " deg", total_time);
    softStart('L', 200);
    delay(cruise_time);
  } else if (cmd == 'R') {
    displayAction("PIVOT RIGHT", deg, " deg", total_time);
    softStart('R', 200);
    delay(cruise_time);
  }

  brakeMotors();
  displayIdle();
}

void softStart(char direction, int targetSpeed) {
  int steps = 10;
  int delayPerStep = ACCEL_TIME_MS / steps;
  
  for (int i = 1; i <= steps; i++) {
    int s = (targetSpeed / steps) * i;
    if (direction == 'F') setMotors(s, s);
    if (direction == 'B') setMotors(-s, -s);
    if (direction == 'L') setMotors(-s, s);
    if (direction == 'R') setMotors(s, -s);
    delay(delayPerStep);
  }
}