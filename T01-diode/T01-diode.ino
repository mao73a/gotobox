#define DIODE_PIN 6

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(DIODE_PIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(DIODE_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(DIODE_PIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
