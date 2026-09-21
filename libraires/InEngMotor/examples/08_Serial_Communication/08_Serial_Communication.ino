// Don't forget to open serial mornitor on the top left
// and CHANGE baud rate to 115200
// The monitor will display after user type the command on the Serial Monitor

void setup() {
  // Start serial communication at 115200 baud rate for the Serial Monitor
  Serial.begin(115200);
}

void loop() {

  // Check if the Serial Monitor is available
  if (Serial.available()) {

    // Read the user command on the Serial Monitor
    char command = Serial.read();

    // Check the user command
    if (command == 'F') {
      Serial.println("Forward"); // If command is 'F', Display "Forward" on Serial Monitor
    }
    else if (command == 'B') {
      Serial.println("Backward"); // If command is 'B', Display "Backward" on Serial Monitor
    }
    else if (command == 'L') {
      Serial.println("Left"); // If command is 'L', Display "Left" on Serial Monitor
    }
    else if (command == 'R') {
      Serial.println("Right"); // If command is 'R', Display "Right" on Serial Monitor
    }
    else if (command == 'S') {
      Serial.println("Stop"); // If command is 'S', Display "Stop" on Serial Monitor
    }
  }

}