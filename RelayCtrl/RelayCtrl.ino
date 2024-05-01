// Define the pin connected to the relay module
const int relayPin = 5; // Change this to match the pin you're using

void setup() {
  // Set the relay pin as an OUTPUT
  pinMode(relayPin, OUTPUT);
}

void loop() {
  // Turn the relay on
  digitalWrite(relayPin, HIGH);
  delay(5000); // Wait for 5 seconds
  
  // Turn the relay off
  digitalWrite(relayPin, LOW);
  delay(5000); // Wait for 5 seconds
}
