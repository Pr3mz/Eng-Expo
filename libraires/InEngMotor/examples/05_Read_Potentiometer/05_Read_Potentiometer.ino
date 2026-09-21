// Don't forget to open serial mornitor on the top Right
// and CHANGE baud rate to 115200

#define POT_PIN 34 // Set GPIO PIN 34 to Potentiometer PIN

void setup() {
  // Start serial communication at 115200 baud rate for the Serial Monitor
  Serial.begin(115200); 
}

void loop() {
  // Read the analog value of Potentiometer
  int value = analogRead(POT_PIN);
  
  // Print the value of Potentiometer to the Serial Monitor
  Serial.println(value);
  
  // Wait 100 milliseconds before taking the next reading
  delay(100);
}