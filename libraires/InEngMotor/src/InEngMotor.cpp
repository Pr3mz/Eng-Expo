#include "InEngMotor.h"

InEngMotor inengmotor;

InEngMotor::InEngMotor(uint8_t motorAIn1, uint8_t motorAIn2,
                       uint8_t motorBIn1, uint8_t motorBIn2,
                       uint32_t pwmFrequency, uint8_t pwmResolution)
    : _motorAIn1(motorAIn1),
      _motorAIn2(motorAIn2),
      _motorBIn1(motorBIn1),
      _motorBIn2(motorBIn2),
      _pwmFrequency(pwmFrequency),
      _pwmResolution(pwmResolution),
      // GPIO 27/26 channel needs the opposite direction.
      _invertLeft(true),
      _invertRight(false)
{
}

#include <esp_arduino_version.h>

void InEngMotor::begin()
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // Arduino-ESP32 3.x LEDC API:
  // ledcAttachChannel(pin, frequency, resolution, channel)
  ledcAttachChannel(_motorAIn1, _pwmFrequency, _pwmResolution, 0);
  ledcAttachChannel(_motorAIn2, _pwmFrequency, _pwmResolution, 1);
  ledcAttachChannel(_motorBIn1, _pwmFrequency, _pwmResolution, 2);
  ledcAttachChannel(_motorBIn2, _pwmFrequency, _pwmResolution, 3);
#else
  // Arduino-ESP32 2.x LEDC API
  ledcSetup(0, _pwmFrequency, _pwmResolution);
  ledcAttachPin(_motorAIn1, 0);
  ledcSetup(1, _pwmFrequency, _pwmResolution);
  ledcAttachPin(_motorAIn2, 1);
  ledcSetup(2, _pwmFrequency, _pwmResolution);
  ledcAttachPin(_motorBIn1, 2);
  ledcSetup(3, _pwmFrequency, _pwmResolution);
  ledcAttachPin(_motorBIn2, 3);
#endif

  stopAllMotors();
}

void InEngMotor::setMotorA(uint8_t speed, bool forwardDirection)
{
  if (_invertLeft)
  {
    forwardDirection = !forwardDirection;
  }
  writeMotor(_motorAIn1, _motorAIn2, speed, forwardDirection);
}

void InEngMotor::setMotorB(uint8_t speed, bool forwardDirection)
{
  if (_invertRight)
  {
    forwardDirection = !forwardDirection;
  }
  writeMotor(_motorBIn1, _motorBIn2, speed, forwardDirection);
}

void InEngMotor::brakeMotorA()
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(_motorAIn1, 255);
  ledcWrite(_motorAIn2, 255);
#else
  ledcWrite(0, 255);
  ledcWrite(1, 255);
#endif
}

void InEngMotor::brakeMotorB()
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(_motorBIn1, 255);
  ledcWrite(_motorBIn2, 255);
#else
  ledcWrite(2, 255);
  ledcWrite(3, 255);
#endif
}

void InEngMotor::stopMotorA()
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(_motorAIn1, 0);
  ledcWrite(_motorAIn2, 0);
#else
  ledcWrite(0, 0);
  ledcWrite(1, 0);
#endif
}

void InEngMotor::stopMotorB()
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(_motorBIn1, 0);
  ledcWrite(_motorBIn2, 0);
#else
  ledcWrite(2, 0);
  ledcWrite(3, 0);
#endif
}

void InEngMotor::stopAllMotors()
{
  stopMotorA();
  stopMotorB();
}

void InEngMotor::forward(int leftSpeed, int rightSpeed)
{
  drive(abs(leftSpeed), abs(rightSpeed));
}

void InEngMotor::backward(int leftSpeed, int rightSpeed)
{
  drive(-abs(leftSpeed), -abs(rightSpeed));
}

void InEngMotor::turnLeft(int leftSpeed, int rightSpeed)
{
  forward(leftSpeed, rightSpeed);
}

void InEngMotor::turnRight(int leftSpeed, int rightSpeed)
{
  forward(leftSpeed, rightSpeed);
}

void InEngMotor::spinLeft(int leftSpeed, int rightSpeed)
{
  drive(-abs(leftSpeed), abs(rightSpeed));
}

void InEngMotor::spinRight(int leftSpeed, int rightSpeed)
{
  drive(abs(leftSpeed), -abs(rightSpeed));
}

void InEngMotor::drive(int leftSpeed, int rightSpeed)
{
  driveOneMotor(_motorAIn1, _motorAIn2, leftSpeed, _invertLeft);
  driveOneMotor(_motorBIn1, _motorBIn2, rightSpeed, _invertRight);
}

void InEngMotor::stop()
{
  stopAllMotors();
}

void InEngMotor::brake()
{
  brakeMotorA();
  brakeMotorB();
}

void InEngMotor::setInvert(bool invertLeft, bool invertRight)
{
  _invertLeft = invertLeft;
  _invertRight = invertRight;
}

int InEngMotor::limitSpeed(int speed)
{
  return constrain(speed, -255, 255);
}

void InEngMotor::writeMotor(uint8_t in1, uint8_t in2,
                            uint8_t speed, bool forwardDirection)
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (forwardDirection)
  {
    ledcWrite(in1, speed);
    ledcWrite(in2, 0);
  }
  else
  {
    ledcWrite(in1, 0);
    ledcWrite(in2, speed);
  }
#else
  uint8_t ch1 = (in1 == _motorAIn1) ? 0 : 2;
  uint8_t ch2 = (in1 == _motorAIn1) ? 1 : 3;
  if (forwardDirection)
  {
    ledcWrite(ch1, speed);
    ledcWrite(ch2, 0);
  }
  else
  {
    ledcWrite(ch1, 0);
    ledcWrite(ch2, speed);
  }
#endif
}

void InEngMotor::driveOneMotor(uint8_t in1, uint8_t in2,
                               int speed, bool inverted)
{
  speed = limitSpeed(speed);

  if (inverted)
  {
    speed = -speed;
  }

  if (speed > 0)
  {
    // Physical forward follows setMotorX(speed, false) from the tested sketch.
    writeMotor(in1, in2, static_cast<uint8_t>(speed), false);
  }
  else if (speed < 0)
  {
    writeMotor(in1, in2, static_cast<uint8_t>(-speed), true);
  }
  else
  {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
#else
    uint8_t ch1 = (in1 == _motorAIn1) ? 0 : 2;
    uint8_t ch2 = (in1 == _motorAIn1) ? 1 : 3;
    ledcWrite(ch1, 0);
    ledcWrite(ch2, 0);
#endif
  }
}
