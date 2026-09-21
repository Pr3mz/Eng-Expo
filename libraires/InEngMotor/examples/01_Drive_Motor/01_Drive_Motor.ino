#define LEFT_F 26 // Motor Left forward pin (M1_IN2)
#define LEFT_B 27 // Motor Left back pin (M1_IN1)
#define RIGHT_F 17 // Motor Right forward pin (M2_IN2)
#define RIGHT_B 16 // Motor Right back pin (M2_IN1)

void forward(int leftSpeed, int rightSpeed) {
  // Left Motor
  ledcWrite(LEFT_F, leftSpeed); // Forward Pin ON with Left Speed Value
  ledcWrite(LEFT_B, 0); // Back Pin OFF

  // Right motor
  ledcWrite(RIGHT_F, rightSpeed); // Forward Pin ON with Left Speed Value
  ledcWrite(RIGHT_B, 0); // Back Pin OFF
}

void setup() {
  // Initialize PWM for each motor pin
  // Frequency: 20000 Hz (20 kHz)
  // Resolution: 8-bit means the speed value can be anywhere from 0 to 255
  ledcAttachChannel(LEFT_F, 20000, 8, 1);
  ledcAttachChannel(LEFT_B, 20000, 8, 0);
  ledcAttachChannel(RIGHT_F, 20000, 8, 2);
  ledcAttachChannel(RIGHT_B, 20000, 8, 3);

  //      LEFT RIGHT
  forward(200, 200); //Command the borad to make motor forward with speed value
}

void loop() {
}