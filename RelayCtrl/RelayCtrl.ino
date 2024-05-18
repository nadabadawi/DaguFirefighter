// Define the pin connected to the relay module
const int relayPin = 5; // Change this to match the pin you're using
volatile bool triggerRelay = false; // Flag to indicate if the relay should be triggered

void setup() {
  // Set the relay pin as an OUTPUT
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);
  // Initialize serial communication
  Serial.begin(115200);

  // Attach an interrupt to trigger when serial data is available
  attachInterrupt(digitalPinToInterrupt(0), triggerInterrupt, CHANGE);
}

void loop() {
  // Check if there's serial data available
  if (Serial.available() > 0) {
    // Read serial input
    String input = Serial.readStringUntil('\n');

    // Print the received input to the serial monitor
    Serial.println("Received: " + input);

    // Check if input is "up"
    if (input == "up") {
      // Set the flag to trigger the relay
      triggerRelay = true;
    }
  }

  // Check if the flag is set to trigger the relay
  if (triggerRelay) {
    // Turn the relay on
    digitalWrite(relayPin, LOW);
    delay(2000);
    // Clear the flag
    triggerRelay = false;
    digitalWrite(relayPin, HIGH);
  }
}

void triggerInterrupt() {
  // This function is left empty because it's just a placeholder for the interrupt
}