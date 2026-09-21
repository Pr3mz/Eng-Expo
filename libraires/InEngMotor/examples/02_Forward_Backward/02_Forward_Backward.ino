#include <InEngMotor.h> //Import preset libary for motor commands

void setup() {
  //Start the libary
  inengmotor.begin(); 

}

void loop() {
  //Forward: (leftspeed, rightspeed)
  inengmotor.forward(200, 200); 
  delay(2000); //Wait for 2 second
  inengmotor.stop(); // Stop Motor
  delay(800); //Wait for 800 milisecond

  //Backward: (leftspeed, rightspeed)
  inengmotor.backward(200, 200); 
  delay(2000); //Wait for 2 second
  inengmotor.stop(); //Stop Motor
  delay(800); //Wait for 800 millisecond
}

/*
  Basic Motor Commands from this libary:

  inengmotor.forward(150, 150);    // Move forward
  inengmotor.backward(150, 150);   // Move backward

  inengmotor.turnLeft(80, 180);    // Curve left: left wheel is slower
  inengmotor.turnRight(180, 80);   // Curve right: right wheel is slower

  inengmotor.spinLeft(150, 150);   // Spin left in place
  inengmotor.spinRight(150, 150);  // Spin right in place

  inengmotor.stop();               // Coast to a stop
  inengmotor.brake();              // Actively brake both motors
*/