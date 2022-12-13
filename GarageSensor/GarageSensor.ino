#include <VirtualWire.h>

// Settings
const int DETECT_VEHICLE_DISTANCE_MIN_CM = 0;
const int DETECT_VEHICLE_DISTANCE_MAX_CM = 100;
const int DETECT_VEHICLE_DEBOUNCE_MILLIS = 3000;
const int RF_BPS = 4800;
const char* VEHICLE_DETECTED_MESSAGE = "Detected";
const char* VEHICLE_NOT_DETECTED_MESSAGE = "Undetected";
const char* GARAGE_DOOR_OPEN_MESSAGE = "Open";
const char* GARAGE_DOOR_CLOSED_MESSAGE = "Closed";
const char MESSAGE_SEPARATOR_CHAR = '-';

// Pin assignment
const int PIN_GARAGE_DOOR_SENSOR = 10;
const int PIN_PING = 5; // Trigger Pin of Ultrasonic Sensor - https://electropeak.com/learn/interfacing-us-100-ultrasonic-distance-sensor-with-arduino/
const int PIN_PONG = 6; // Echo Pin of Ultrasonic Sensor

const int PIN_RF_TX = 4; // RF send signal pin
const int PIN_STATUS_LED = 13; // Built-in to Arduino Pro Mini

// Calculated constants
// prevent detection from taking a long time when out of range (x5 to wait for ping and echo to complete)
const unsigned long DISTANCE_ECHO_TIMEOUT = ((unsigned long) DETECT_VEHICLE_DISTANCE_MAX_CM) * 5 * 29 * 2;

// Global
bool _isVehicleDetected = false;
bool _isGarageDoorOpen = false;
unsigned long _lastDetectionChangeMillis = 0;
char _buffer[40];

void setup() {
  pinMode(PIN_GARAGE_DOOR_SENSOR, INPUT_PULLUP);

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  // Initialize ultrasonic sensor
  pinMode(PIN_PING, OUTPUT);
  pinMode(PIN_PONG, INPUT);
  digitalWrite(PIN_PING, LOW);

  // Initialize RF transmitter
  vw_set_tx_pin(PIN_RF_TX);
  vw_setup(RF_BPS);  // Bits per sec
  
  Serial.begin(9600);
}

void loop() {
  GetInput();

  //Serial.print("Is vehicle in garage: ");
  //Serial.println(_isVehicleInGarage ? "Yes" : "No");
  
  //int distanceCm = GetDistance();
  //Serial.print("Distance: ");
  //Serial.print(distanceCm);
  //Serial.println(" cm");

  sprintf(_buffer, "%s%c%s", 
    _isVehicleDetected ? VEHICLE_DETECTED_MESSAGE : VEHICLE_NOT_DETECTED_MESSAGE,
    MESSAGE_SEPARATOR_CHAR,
    _isGarageDoorOpen ? GARAGE_DOOR_OPEN_MESSAGE : GARAGE_DOOR_CLOSED_MESSAGE);

  if (_isGarageDoorOpen || _isVehicleDetected)
  {
    TransmitMessage(_buffer);
  }
  else
  {
    Serial.print("NOT transmitting: ");
    Serial.println(_buffer);
  }

  delay(100);
}

void GetInput()
{
  _isGarageDoorOpen = digitalRead(PIN_GARAGE_DOOR_SENSOR) == HIGH;
  bool isVehicleInGarage = IsVehicleInGarage();

  // Update the status LED immediately. Help with positioning that is not delayed.
  digitalWrite(PIN_STATUS_LED, isVehicleInGarage ? LOW : HIGH);

  if (_isVehicleDetected == isVehicleInGarage)
    return;

  // status changed
  if (millis() - _lastDetectionChangeMillis < DETECT_VEHICLE_DEBOUNCE_MILLIS)
    return; // This change is too soon. Try again next time

  _isVehicleDetected = isVehicleInGarage;
  _lastDetectionChangeMillis = millis();
}

void TransmitMessage(char* message)
{
  Serial.print("Transmitting: ");
  Serial.println(message);
  vw_send((uint8_t *)message, strlen(message)); // send the message
  vw_wait_tx(); // Wait until the whole message is sent
}

bool IsVehicleInGarage()
{
  long distance_cm = GetDistance();
  return distance_cm != -1 && distance_cm >= DETECT_VEHICLE_DISTANCE_MIN_CM && distance_cm <= DETECT_VEHICLE_DISTANCE_MAX_CM;
}

int GetDistance()
{
  digitalWrite(PIN_PING, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_PING, LOW);
  long duration = pulseIn(PIN_PONG, HIGH, DISTANCE_ECHO_TIMEOUT);
  long distanceCm = duration / 29 / 2;
  
  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");
  
  // Note: 0 distance = timeout (beyond max distance)
  return distanceCm == 0 ? -1 : distanceCm;
}
