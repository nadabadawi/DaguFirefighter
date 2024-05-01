int val = 0 ;
// lowest and highest sensor readings:
const int sensorMin = 0;     //  sensor minimum
const int sensorMax = 4096;  // sensor maximum

void setup()  {
  // initialize serial communication @ 9600 baud:
  Serial.begin(115200);
  pinMode(34,INPUT); // Flame sensor output pin connected
  pinMode(2,OUTPUT); // Buzzer  
}
void loop() {
  // read the sensor on analog A0:
	int sensorReading  = analogRead(34);
  // Serial.print("ADC Value: ");
  Serial.println(sensorReading);
  delay(10);
  // delay(1000);
  // map the sensor range (four options):
  // ex: 'long  int map(long int, long int, long int, long int, long int)'
	int range = map(sensorReading,  sensorMin, sensorMax, 0, 3);
  
  // range value:
  switch (range) {
  case 0:    // A fire closer than 1.5 feet away.
    Serial.println("** Close  Fire **");
    digitalWrite(2,HIGH); // Buzzer ON  
    break;
  case 1:    // A fire between 1-3 feet away.
    Serial.println("**  Distant Fire **");
    digitalWrite(2,HIGH); // Buzzer ON  
    break;
  case 2:    // No fire detected.
    Serial.println("No  Fire");
    digitalWrite(2,LOW); // Buzzer OFF  
    break;
  }
  delay(500);  // delay between reads
}



// // // Define the pin connected to the flame sensor
// // const int flameSensorPin = 2; // Example pin, choose according to your wiring

// // void setup() {
// //   // Initialize serial communication
// //   Serial.begin(115200);
  
// //   // Set flame sensor pin as input
// //   pinMode(flameSensorPin, INPUT);
// // }

// // void loop() {
// //   // Read the output of the flame sensor
// //   int flameDetected = digitalRead(flameSensorPin);
  
// //   // Print the output to the serial monitor
// //   if (flameDetected == HIGH) {
// //     Serial.println("Flame detected!");
// //   } else {
// //     Serial.println("No flame detected.");
// //   }
  
// //   // Delay before next reading
// //   delay(1000);
// // }
