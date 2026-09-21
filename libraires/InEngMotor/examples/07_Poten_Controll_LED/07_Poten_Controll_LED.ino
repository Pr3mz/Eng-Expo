#define POT_PIN 34 // Set GPIO PIN 34 to Potentiometer PIN

#define LED1 23 // Set GPIO PIN 23 to LED 1
#define LED2 21 // Set GPIO PIN 21 to LED 2
#define LED3 19 // Set GPIO PIN 19 to LED 3

void setup() {
  pinMode(LED1, OUTPUT); // Assign PIN 23 for Digital OUTPUT
  pinMode(LED2, OUTPUT); // Assign PIN 21 for Digital OUTPUT
  pinMode(LED3, OUTPUT); // Assign PIN 19 for Digital OUTPUT
}

void loop() {
  // Read the analog value of Potentiometer
  int potValue = analogRead(//PotenPin);

  //Check the value of Potentiometer
  if (potValue < 1365) {
    // Low Range: All LEDs ON
    
  }
  else if (potValue < 2730) {
    // Mid Range: All LEDs OFF
    
  }
  else {
    // High Range: All LEDs BLINK

    //All LEDs ON
    
    delay(500); //Wait for 500 millisecond

    //All LEDs OFF
    
    delay(500); //Wait for 500 millisecond
  }
}
