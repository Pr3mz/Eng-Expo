#include <ESP32Servo.h>

Servo myServo; // Setup Servo library

#define SERVO_PIN 19 // Set GPIO PIN 19 as Servo Pin

void setup() {
  myServo.attach(SERVO_PIN); // Set Library with Servo Pin

  // Set servo angle to 0
  myServo.write(0);
  delay(1000);

  // Set servo angle to 90
  myServo.write(90);
  delay(1000);

  // Set servo angle to 180
  myServo.write(180);
  delay(1000);
}

void loop() {
}
