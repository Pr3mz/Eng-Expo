# InEngMotor

InEngMotor is an Arduino library for controlling two DC motors on the IN-ENG
ESP32 expansion board. The board uses an onboard TB6612 dual H-bridge motor
driver and an ESP32-WROOM-32E (38-pin) module.

The library provides:

- Independent speed and direction control for two motors
- Forward, backward, curved-turn, and spin-in-place commands
- Signed left/right motor control
- Coast and active-brake commands
- Configurable motor direction inversion
- Custom motor pins, PWM frequency, and PWM resolution
- A ready-to-use global `inengmotor` object

Version 1.1.1 targets Arduino-ESP32 Core 3.x and uses its
`ledcAttachChannel()` and `ledcWrite()` APIs. The default PWM configuration is
20 kHz with 8-bit resolution.

## Requirements

- An ESP32 board supported by Arduino-ESP32 Core 3.x
- An IN-ENG ESP32 expansion board with its onboard TB6612 motor driver
- Arduino IDE or another compatible Arduino build environment
- One or two DC motors with a suitable external motor power supply

> Do not power motors directly from an ESP32 GPIO pin. Make sure the motor
> supply voltage and current are suitable for the motors and motor driver.

## Default Pin Configuration

| Wheel | Motor channel | IN1 | IN2 | Default inversion |
|---|---|---:|---:|---|
| Left | Motor A (M1)| GPIO 26 | GPIO 27 | Enabled |
| Right | Motor B (M2)| GPIO 16 | GPIO 17 | Disabled |

Motor A is inverted by default so that both wheels move in the same physical
direction on the tested IN-ENG board. If your robot moves or turns in the wrong
direction, see [Changing Motor Direction](#changing-motor-direction).

The library reserves ESP32 LEDC channels 0 through 3 for these four motor
control pins.

## Installation

### Install from a ZIP file

1. Download or create a ZIP archive of the `InEngMotor` folder.
2. Open Arduino IDE.
3. Select **Sketch > Include Library > Add .ZIP Library...**.
4. Select the ZIP archive.
5. Open **File > Examples > InEngMotor > BasicDrive**.
6. Select your ESP32 board and upload the sketch.

If an older copy is already installed, remove the old `InEngMotor` library
folder before installing the new ZIP.

The ZIP must contain one top-level `InEngMotor` folder, with
`library.properties`, `keywords.txt`, `README.md`, `src`, and `examples`
directly inside it. Do not add an extra nested `InEngMotor` folder.

### Install manually

Copy the complete project folder into your Arduino libraries directory and
restart Arduino IDE:

```text
Arduino/
`-- libraries/
    `-- InEngMotor/
        |-- examples/
        |-- src/
        |-- keywords.txt
        `-- library.properties
```

## Quick Start

```cpp
#include <InEngMotor.h>

void setup() {
  inengmotor.begin();
}

void loop() {
  inengmotor.forward(150, 150);
  delay(2000);

  inengmotor.stop();
  delay(1000);

  inengmotor.backward(150, 150);
  delay(2000);

  inengmotor.stop();
  delay(1000);
}
```

Call `begin()` once in `setup()` before sending motor commands. For movement
helpers, the first speed controls the left wheel and the second controls the
right wheel. Valid output speeds range from 0 (stopped) to 255 (full PWM duty
cycle).

## Movement Commands

```cpp
inengmotor.forward(150, 150);    // Move forward
inengmotor.backward(150, 150);   // Move backward

inengmotor.turnLeft(80, 180);    // Curve left: left wheel is slower
inengmotor.turnRight(180, 80);   // Curve right: right wheel is slower

inengmotor.spinLeft(150, 150);   // Spin left in place
inengmotor.spinRight(150, 150);  // Spin right in place

inengmotor.stop();               // Coast to a stop
inengmotor.brake();              // Actively brake both motors
```

`turnLeft()` and `turnRight()` both drive the wheels forward using the exact
speeds provided. The curve direction is produced by making the inside wheel
slower than the outside wheel.

### Signed Drive Control

Use `drive()` when each wheel needs an independent direction:

```cpp
inengmotor.drive(200, 200);    // Both wheels forward
inengmotor.drive(-200, -200);  // Both wheels backward
inengmotor.drive(200, -100);   // Left forward, right backward
inengmotor.drive(0, 180);      // Left stopped, right forward
```

Positive values move a wheel forward, negative values move it backward, and
zero lets it coast. Values are limited to the range `-255` to `255`.

## Low-Level Motor Control

Motor A is the left wheel and Motor B is the right wheel by default:

```cpp
inengmotor.setMotorA(190, false);  // Motor A forward
inengmotor.setMotorB(190, false);  // Motor B forward

inengmotor.setMotorA(190, true);   // Motor A backward
inengmotor.setMotorB(190, true);   // Motor B backward

inengmotor.stopMotorA();           // Coast Motor A
inengmotor.stopMotorB();           // Coast Motor B
inengmotor.brakeMotorA();          // Brake Motor A
inengmotor.brakeMotorB();          // Brake Motor B
inengmotor.stopAllMotors();        // Coast both motors
```

The direction argument in `setMotorA()` and `setMotorB()` follows the original
tested sketch: `false` means physical forward with the default inversion
settings, and `true` means physical backward.

## Changing Motor Direction

The default configuration is equivalent to:

```cpp
inengmotor.setInvert(true, false);
```

The first value controls Motor A/left inversion, and the second controls Motor
B/right inversion. Call `setInvert()` after `begin()` if your wiring or chassis
requires different directions:

```cpp
void setup() {
  inengmotor.begin();
  inengmotor.setInvert(false, false);
}
```

If only one wheel runs backward when calling `forward()`, toggle the inversion
value for that wheel.

## Custom Pins and PWM Settings

The global `inengmotor` object uses the board defaults. To use different pins
or PWM settings, create your own object:

```cpp
#include <InEngMotor.h>

// Motor A IN1/IN2, Motor B IN1/IN2, frequency, resolution
InEngMotor motors(25, 26, 32, 33, 20000, 8);

void setup() {
  motors.begin();
  motors.setInvert(false, false);
}

void loop() {
  motors.forward(160, 160);
}
```

Constructor signature:

```cpp
InEngMotor(
  uint8_t motorAIn1 = 26,
  uint8_t motorAIn2 = 27,
  uint8_t motorBIn1 = 16,
  uint8_t motorBIn2 = 17,
  uint32_t pwmFrequency = 20000,
  uint8_t pwmResolution = 8
);
```

## API Reference

| Method | Description |
|---|---|
| `begin()` | Attaches the four PWM outputs and stops both motors. |
| `forward(left, right)` | Drives both wheels forward. |
| `backward(left, right)` | Drives both wheels backward. |
| `turnLeft(left, right)` | Drives both wheels forward at the supplied speeds. Use a lower left speed. |
| `turnRight(left, right)` | Drives both wheels forward at the supplied speeds. Use a lower right speed. |
| `spinLeft(left, right)` | Drives the left wheel backward and the right wheel forward. |
| `spinRight(left, right)` | Drives the left wheel forward and the right wheel backward. |
| `drive(left, right)` | Uses signed speeds for independent wheel direction. |
| `stop()` | Sets both H-bridge inputs low so both motors coast. |
| `brake()` | Sets both H-bridge inputs high to actively brake both motors. |
| `setInvert(left, right)` | Configures direction inversion for each wheel. |
| `setMotorA(speed, direction)` | Controls Motor A directly with an 8-bit speed. |
| `setMotorB(speed, direction)` | Controls Motor B directly with an 8-bit speed. |
| `stopMotorA()` / `stopMotorB()` | Coasts one motor. |
| `brakeMotorA()` / `brakeMotorB()` | Actively brakes one motor. |
| `stopAllMotors()` | Coasts both motors. |

## Examples

The library includes two sketches:

- **BasicDrive** demonstrates forward, backward, curved turns, spins, and
  stopping.
- **WebControl** creates a Wi-Fi access point and serves a mobile-friendly
  browser controller. It supports separate left/right speeds, command ordering,
  heartbeats, and a 700 ms motor safety timeout.

Before uploading `WebControl`, replace the placeholder access-point
credentials:

```cpp
const char* AP_NAME = "YourRobot";
const char* AP_PASSWORD = "your-password";  // At least 8 characters
```

After uploading, open Serial Monitor at 115200 baud, connect a phone or
computer to the ESP32 access point, and open the IP address printed in Serial
Monitor.

## License

This project is released under the [MIT License](LICENSE).
