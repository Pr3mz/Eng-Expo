#define SERVO_PIN 19 // Set GPIO PIN 19 as Servo Pin

#define PWM_FREQ 50       // Define Frequency 50 Hz
#define PWM_RESOLUTION 16 // 16-bit

void setup() {
  ledcAttach(SERVO_PIN, PWM_FREQ, PWM_RESOLUTION); // Set Frequency and Resolution for Servo Pin
}

void servoWrite(int angle) {
  // Map Servo pulse and angle
  int pulseWidth = map(angle, 0, 180, 500, 2500);

  // Calculate Duty Cycle
  uint32_t duty = (pulseWidth * 65535UL) / 20000UL;

  // Set the servo angle
  ledcWrite(SERVO_PIN, duty); 
}

void loop() {
  // Set servo angle to 0
  servoWrite(0);
  delay(1000);

  // Set servo angle to 90
  servoWrite(90);
  delay(1000);

  // Set servo angle to 180
  servoWrite(180);
  delay(1000);
}
