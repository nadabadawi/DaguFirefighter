#include <SoftwareSerial.h>
#include <limits.h>

// Define the pins for SoftwareSerial communication
#define TX_PIN 17
#define RX_PIN 16
#define TRIGGER_PIN 12 // Pin connected to the trigger pin of the ultrasonic sensor
#define ECHO_PIN 13    // Pin connected to the echo pin of the ultrasonic sensor

// Define the motor control commands including speed value
#define MOTOR_A_FORWARD 0xC2
#define MOTOR_B_FORWARD 0xCA
#define MOTOR_A_BACKWARD 0xC1
#define MOTOR_B_BACKWARD 0xC9
#define SPEED_VALUE 0x40 // Example: 50% speed
#define STOP_VALUE 0x00

const int flameSensorPin = 5;
const int Analog_flamePin = 34;
const int analogReadingMax = 4095;
const int analogReadingMin = 0;
bool flameDetected = false;
int duration = INT_MAX;
int distance = INT_MAX;

// Define the SoftwareSerial object
EspSoftwareSerial::UART trexSerial(RX_PIN, TX_PIN);

enum FireState {
  IDLE,
  START,
  STOP_CAR,
  ENABLE_PUMP,
  TURN_SERVO,
  WAIT,
  DISABLE_PUMP
};

enum DistantFireState
{
  DISTANT_IDLE,
  OBSTACLE,
  NO_OBSTACLE
};

FireState fireState = IDLE;
DistantFireState distantFireState = DISTANT_IDLE;
unsigned long lastFireStateChange = 0;
unsigned long lastDistantFireStateChange = 0;
const unsigned long pumpDuration = 100;
const unsigned long waitDuration = 10;
const unsigned long servoTurnDuration = 500;

void IRAM_ATTR flameISR() 
{
  flameDetected = (digitalRead(flameSensorPin) == LOW);
}

void moveForward()
{
  trexSerial.write(MOTOR_A_FORWARD);
  trexSerial.write(SPEED_VALUE);
  trexSerial.write(MOTOR_B_FORWARD);
  trexSerial.write(SPEED_VALUE);
}

void moveBackward()
{
  trexSerial.write(MOTOR_A_BACKWARD);
  trexSerial.write(SPEED_VALUE);
  trexSerial.write(MOTOR_B_BACKWARD);
  trexSerial.write(SPEED_VALUE);
}

void moveRight()
{
  trexSerial.write(MOTOR_A_FORWARD);
  trexSerial.write(SPEED_VALUE);
  trexSerial.write(MOTOR_B_FORWARD);
  trexSerial.write((uint8_t)STOP_VALUE);
}

void moveLeft()
{
  trexSerial.write(MOTOR_A_FORWARD);
  trexSerial.write((uint8_t)STOP_VALUE);
  trexSerial.write(MOTOR_B_FORWARD);
  trexSerial.write(SPEED_VALUE);
}

void stopCar()
{
  trexSerial.write(MOTOR_A_FORWARD);
  trexSerial.write((uint8_t)STOP_VALUE);
  trexSerial.write(MOTOR_B_FORWARD);
  trexSerial.write((uint8_t)STOP_VALUE);
}

void turnCar()
{
  moveBackward();
  delay(500);
  moveRight();
  delay(2000);
  moveLeft();
  delay(2000);
  moveForward();
  delay(1000);
  moveLeft();
  delay(2000);
  moveRight();
  delay(2000);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  trexSerial.begin(19200);
  // OUTPUTS
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(TX_PIN, OUTPUT);
  pinMode(2, OUTPUT);
  // INPUTS
  pinMode(ECHO_PIN, INPUT);
  pinMode(RX_PIN, INPUT);
  pinMode(flameSensorPin, INPUT);
  pinMode(Analog_flamePin, INPUT);
  // INTERRUPTS
  attachInterrupt(digitalPinToInterrupt(flameSensorPin), flameISR, CHANGE);
  // INITIALIZATION
  fireState = IDLE;
  distantFireState = DISTANT_IDLE;
}

bool foundObstacle()
{
  delay(50);
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // Measure the duration of the echo signal
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
  // Print the measured distance to the serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  if (distance <= 30) 
  {
    Serial.println("Less than 30!");
    return true;
    //turnCar();
  }
  return false;
  // else
  // {
  //   moveForward();
  // }
}

void updateDistantFireState()
{
  unsigned long currentTime = millis();
  switch(distantFireState)
  {
    case DISTANT_IDLE:
      Serial.println("IN IDLE");
      if(flameDetected)
      {
        if (foundObstacle())
        {
          distantFireState = OBSTACLE;
        }
        else
        {
          distantFireState = NO_OBSTACLE;
        }
        lastFireStateChange = currentTime;
      }
      break;
    case OBSTACLE:
      Serial.println("IN OBSTACLE");
      turnCar();
      stopCar();
      distantFireState = DISTANT_IDLE;
      lastFireStateChange = currentTime;
      break;
    case NO_OBSTACLE:
      Serial.println("IN NO OBSTACLE");
      // if (currentTime - lastFireStateChange < 2000)
      // {
        moveForward();
        delay(500);
      // }
      // else
      // {
        stopCar();
        distantFireState = DISTANT_IDLE;
      // }  
      break;
  }
}

void updateFireState()
{
  unsigned long currentTime = millis();

  switch (fireState)
  {
    case IDLE:
      Serial.println("IN IDLE");
      if (flameDetected)
      {
        fireState = START;
        lastFireStateChange = currentTime;
      }
      break;

    case START:
      Serial.println("IN START");
      fireState = STOP_CAR;
      lastFireStateChange = currentTime;
      break;

    case STOP_CAR:
      Serial.println("IN STOP");
      stopCar();
      fireState = ENABLE_PUMP;
      lastFireStateChange = currentTime;
      break;

    case ENABLE_PUMP:
      Serial.println("IN EN PUMP");
      digitalWrite(2, HIGH);
      fireState = TURN_SERVO;
      lastFireStateChange = currentTime;
      break;

    case TURN_SERVO:
      Serial.println("IN SERVO");
      // if (currentTime - lastFireStateChange >= pumpDuration)
      // {
        // Turn Servo (Placeholder for actual servo control)
        fireState = WAIT;
        lastFireStateChange = currentTime;
      // }
      break;

    case WAIT:
      Serial.println("IN WAIT");
      // if (currentTime - lastFireStateChange >= waitDuration)
      // {
        fireState = DISABLE_PUMP;
        lastFireStateChange = currentTime;
      // }
      break;

    case DISABLE_PUMP:
      Serial.println("IN DIS PUMP");
      digitalWrite(2, LOW);
      fireState = IDLE;
      break;
  }
}

void loop() {
  int sensorReading = analogRead(Analog_flamePin);
  int range = map(sensorReading, analogReadingMin, analogReadingMax, 0, 2);

  if(flameDetected)
  {
    Serial.println("FLAME DETECTED");
    switch(range)
    {
      case 0:
        Serial.println("** Close Fire **");
        Serial.println(sensorReading);
        updateFireState();
        break;
      case 1:
        Serial.println("** Distant Fire **");
        updateDistantFireState();
        //avoidObstacle();
        break;
      case 2:
         Serial.println("** Very Distant Fire **");
        break;
      // default:
      //   Serial.println("** No Fire Detected**");
      //   // Optionally handle these cases if needed
      //   break;
    }
  }
  else
  {
    Serial.println("NO FLAME DETECTED");
    digitalWrite(2, LOW);
    fireState = IDLE;
    distantFireState = DISTANT_IDLE;
  }
}
