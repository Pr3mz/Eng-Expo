#include <TFT_eSPI.h> // Import TFT Screen for esp32 libary

TFT_eSPI tft = TFT_eSPI(); 

int screenW, screenH; // Screen Width and Height

// Position of Left and Right Eye
int eyeLX, eyeLY;
int eyeRX, eyeRY;
int eyeR, pupilR, moveRange;

int pupilX = 0, pupilY = 0;
int targetX = 0, targetY = 0;

// Command Draw Eye
void drawEye(int cx, int cy) {
  tft.fillCircle(cx, cy, eyeR, TFT_WHITE);
  tft.drawCircle(cx, cy, eyeR, TFT_BLACK);
}

// Command Draw Pupil
void drawPupil(int cx, int cy, int offX, int offY) {
  tft.fillCircle(cx + offX, cy + offY, pupilR, TFT_BLACK);
}

// Command Clear Pupil
void clearPupil(int cx, int cy, int offX, int offY) {
  tft.fillCircle(cx + offX, cy + offY, pupilR, TFT_WHITE);
}

void setup() {
  tft.init();
  tft.setRotation(0);

  screenW = tft.width();   
  screenH = tft.height();  

  eyeR       = screenW * 0.22;
  pupilR     = eyeR * 0.4;
  moveRange  = eyeR * 0.35;

  eyeLX = screenW * 0.27;
  eyeRX = screenW * 0.73;
  eyeLY = eyeRY = screenH * 0.5;

  tft.fillScreen(TFT_GREEN);

  drawEye(eyeLX, eyeLY);
  drawEye(eyeRX, eyeRY);

  drawPupil(eyeLX, eyeLY, pupilX, pupilY);
  drawPupil(eyeRX, eyeRY, pupilX, pupilY);

  randomSeed(analogRead(0));
  pickNewTarget();
}

void pickNewTarget() {
  targetX = random(-moveRange, moveRange + 1);
  targetY = random(-moveRange, moveRange + 1);
}

void loop() {
  clearPupil(eyeLX, eyeLY, pupilX, pupilY);
  clearPupil(eyeRX, eyeRY, pupilX, pupilY);

  if (pupilX < targetX) pupilX++;
  if (pupilX > targetX) pupilX--;
  if (pupilY < targetY) pupilY++;
  if (pupilY > targetY) pupilY--;

  drawPupil(eyeLX, eyeLY, pupilX, pupilY);
  drawPupil(eyeRX, eyeRY, pupilX, pupilY);

  if (pupilX == targetX && pupilY == targetY) {
    delay(600);
    pickNewTarget();
  }

  delay(15);
}