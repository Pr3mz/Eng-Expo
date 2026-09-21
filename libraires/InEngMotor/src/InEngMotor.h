#ifndef INENG_MOTOR_H
#define INENG_MOTOR_H

#include <Arduino.h>

class InEngMotor {
public:
  // Pin order and PWM settings follow the tested example:
  // Motor A: IN1=GPIO26, IN2=GPIO27
  // Motor B: IN1=GPIO16, IN2=GPIO17
  // PWM: 20 kHz, 8-bit, channels 0-3
  InEngMotor(uint8_t motorAIn1 = 26, uint8_t motorAIn2 = 27,
             uint8_t motorBIn1 = 16, uint8_t motorBIn2 = 17,
             uint32_t pwmFrequency = 20000, uint8_t pwmResolution = 8);

  void begin();

  // Low-level commands matching the original sketch.
  // speed: 0-255
  // forward=false follows the tested sketch and drives IN2 with PWM.
  void setMotorA(uint8_t speed, bool forward);
  void setMotorB(uint8_t speed, bool forward);
  void brakeMotorA();
  void brakeMotorB();
  void stopMotorA();
  void stopMotorB();
  void stopAllMotors();

  // Speeds are 0-255. First value is left wheel, second is right wheel.
  void forward(int leftSpeed, int rightSpeed);
  void backward(int leftSpeed, int rightSpeed);

  // Curved turns: both wheels move forward.
  // For turnLeft, normally use leftSpeed < rightSpeed.
  // For turnRight, normally use rightSpeed < leftSpeed.
  void turnLeft(int leftSpeed, int rightSpeed);
  void turnRight(int leftSpeed, int rightSpeed);

  // Spin in place: the wheels rotate in opposite directions.
  void spinLeft(int leftSpeed, int rightSpeed);
  void spinRight(int leftSpeed, int rightSpeed);

  // Signed control: positive=forward, negative=backward, zero=coast.
  void drive(int leftSpeed, int rightSpeed);

  void stop();
  void brake();

  // Motor A (GPIO 27/26) is inverted by default.
  // Use this to override the default direction settings.
  void setInvert(bool invertLeft, bool invertRight);

private:
  uint8_t _motorAIn1;
  uint8_t _motorAIn2;
  uint8_t _motorBIn1;
  uint8_t _motorBIn2;
  uint32_t _pwmFrequency;
  uint8_t _pwmResolution;
  bool _invertLeft;
  bool _invertRight;

  static int limitSpeed(int speed);
  void writeMotor(uint8_t in1, uint8_t in2, uint8_t speed, bool forward);
  void driveOneMotor(uint8_t in1, uint8_t in2, int speed, bool inverted);
};

// Ready-made object, so commands can be written as:
// inengmotor.forward(150, 150);
extern InEngMotor inengmotor;

#endif
