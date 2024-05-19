#include <SoftwareSerial.h>

// Define the pins for SoftwareSerial communication
#define TX_PIN 17
#define RX_PIN 16
#define TRIGGER_PIN 12 // Pein connected to the trigger pin of the ultrasonic sensor
#define ECHO_PIN 13    // Pin connected to the echo pin of the ultrasonic sensor

// Define the motor control commands including speed value
#define MOTOR_A_FORWARD_SPEED 0xC2
#define MOTOR_B_FORWARD_SPEED 0xCA
#define MOTOR_A_BACKWARD_SPEED 0xC1
#define MOTOR_B_BACKWARD_SPEED 0xC9
#define SPEED_VALUE 0x40 // Example: 50% speed
#define SPEED_VALUE_MOTOR_A 0x60
#define STOP_VALUE 0x00

const int flameSensorPin = 5;

// Define the SoftwareSerial object
EspSoftwareSerial::UART trexSerial(-1, TX_PIN);

void setup() 
{
  Serial.begin(115200);
  trexSerial.begin(19200);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);
  
  // trexSerial.write(MOTOR_A_FORWARD_SPEED);
  // trexSerial.write((uint8_t)STOP_VALUE);
  // trexSerial.write(MOTOR_B_FORWARD_SPEED);
  // trexSerial.write((uint8_t)(STOP_VALUE));

  Serial.begin(115200);
  pinMode(flameSensorPin, INPUT);
  // attachInterrupt(flameSensorPin, handler, LOW);

}
int duration = INT_MAX;
int distance = INT_MAX;
// bool flameDetected = false;

// void IRAM_ATTR handler() {
//   flameDetected = true;
// }

void loop()
{     
  // Generate a pulse to trigger the ultrasonic sensor
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  int flameDetected = digitalRead(flameSensorPin);
  
  // trexSerial.write(MOTOR_A_FORWARD_SPEED);
  // trexSerial.write((uint8_t)STOP_VALUE);
  // trexSerial.write(MOTOR_B_FORWARD_SPEED);
  // trexSerial.write((uint8_t)(STOP_VALUE));


  // Measure the duration of the echo signal
  duration = pulseIn(ECHO_PIN, HIGH);

  distance = duration * 0.034 / 2;
    //Print the measured distance to the serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  if (flameDetected == LOW) {
    Serial.println("Flame detected!");
  } else {
    Serial.println("No flame detected.");
  }


  // If distance > 20, move forward.
  if (flameDetected == LOW)
  {
    trexSerial.write(MOTOR_A_FORWARD_SPEED);
    trexSerial.write(SPEED_VALUE_MOTOR_A);
    trexSerial.write(MOTOR_B_FORWARD_SPEED);
    trexSerial.write((SPEED_VALUE));
    // delay(2000);
    // If distance <= 20, turn right (for testing obstacle detection)
    if(distance<=50)
    {
      trexSerial.write(MOTOR_A_BACKWARD_SPEED);
      trexSerial.write(SPEED_VALUE_MOTOR_A);
      trexSerial.write(MOTOR_B_BACKWARD_SPEED);
      trexSerial.write(SPEED_VALUE);
      delay(500);
      trexSerial.write(MOTOR_A_FORWARD_SPEED);
      trexSerial.write(0x80);
      trexSerial.write(MOTOR_B_FORWARD_SPEED);
      trexSerial.write((uint8_t)0x40);
      delay(2000);
      trexSerial.enableIntTx(true);
    }
  }
  else {
    trexSerial.write(MOTOR_A_FORWARD_SPEED);
    trexSerial.write((uint8_t)0x0);
    trexSerial.write(MOTOR_B_FORWARD_SPEED);
    trexSerial.write((uint8_t)(0x0));
  }
  delay(100);
}