#include <InEngMotor.h> //Import preset libary for motor commands
#define POT_PIN 34 // Set GPIO PIN 34 to Potentiometer PIN

int speedValue = 0; // Initial Speed of the motor
char command; // Initial command character

void setup() {
  // Start serial communication at 115200 baud rate for the Serial Monitor
  Serial.begin(115200);

  //Start the libary
  inengmotor.begin();
}

void loop() {

  // Read potentiometer and convert to motor speed
  int potValue = analogRead(POT_PIN);
  // Use map function to coonvert potentiometer value range to (0,255)
  speedValue = map(potValue, 0, 4095, 0, 255); 

  // Check for command from Computer
  if (Serial.available() > 0) {
    // Read the user command on the Serial Monitor
    command = Serial.read();
    // Catch the buffer input
    char buffer = Serial.read();

    // Check the user command
    switch (command) {
      
      case 'F': // Case of 'F' Command, Robot Forward
        Serial.print("Robot moves forward - Speed: ");
        Serial.println(speedValue);
        inengmotor.forward(speedValue, speedValue);
        break;

      case 'L': // Case of 'L' Command, Robot Spin Left
        
        break;

      case 'R': // Case of 'R' Command, Robot Spin Right
        
        break;
 
      case 'B': // Case of 'B' Command, Robot Backward
        
        break;

      case 'S': // Case of 'S' Command, Robot Stop
        
        break;

      default: // Case of other Command, Show "Invalid command"
        Serial.println("Invalid command");
        break;
    }
  }
}
