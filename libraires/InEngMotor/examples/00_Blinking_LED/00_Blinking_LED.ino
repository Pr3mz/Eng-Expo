// Simple Blink Pin 23

#define LED_PIN 23 // Define GPIO 23 as LED Pin

// Code in this scope will run only once from start
void setup() { 
  pinMode(LED_PIN, OUTPUT); // Assign LED_PIN as an OUTPUT (Release Electricity)

}

// Code in this scope will repeatly after the setup finished
void loop() {

  digitalWrite(LED_PIN, HIGH); //Turn the LED ON
  delay(1000); //Wait for 1 second

  digitalWrite(LED_PIN, LOW); //Turn the LED OFF
  delay(1000); //Wait for 1 second

}
