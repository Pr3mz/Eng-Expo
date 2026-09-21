#define LED1 23 // Set GPIO PIN 23 to LED 1
#define LED2 21 // Set GPIO PIN 21 to LED 2
#define LED3 19 // Set GPIO PIN 19 to LED 3

void setup() {
  // LED 1: Digital output (3.3V)
  pinMode(LED1, OUTPUT); // Assign PIN 23 for Digital OUTPUT
  digitalWrite(LED1, HIGH); // Turn ON LED 1 with digital

  // MAX of duty cycle is 255
  analogWrite(LED2, 191);  // Turn ON LED 2 with 75% of duty cycle

  analogWrite(LED3, 128);  // Turn ON LED 3 with 50% of duty cycle
}

void loop() {
}
