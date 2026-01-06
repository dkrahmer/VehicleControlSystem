// Garage Sensor - for use with Vehicle Controller
// Author: Douglas Krahmer
// Created: 2022-10-27

#include <VirtualWire.h>

// Settings
const unsigned long DETECT_VEHICLE_DISTANCE_MIN_CM = 0UL;
const unsigned long DETECT_VEHICLE_DISTANCE_MAX_CM = 80UL;
const int DETECT_VEHICLE_CONSECUTIVE_READINGS = 5; // number of consecutive identical readings required to change state
const int DETECT_GARAGE_DOOR_CONSECUTIVE_READINGS = 5; // number of consecutive identical readings required to change state
const int RF_BPS = 4800;
const char *VEHICLE_DETECTED_MESSAGE = "Detected";
const char *VEHICLE_NOT_DETECTED_MESSAGE = "Undetected";
const char *GARAGE_DOOR_OPEN_MESSAGE = "Open";
const char *GARAGE_DOOR_CLOSED_MESSAGE = "Closed";
const char MESSAGE_SEPARATOR_CHAR = '-';

// Pin assignment
const int PIN_GARAGE_DOOR_SENSOR = 10;
const int PIN_PING = 5; // Trigger Pin of Ultrasonic Sensor - https://electropeak.com/learn/interfacing-us-100-ultrasonic-distance-sensor-with-arduino/
const int PIN_PONG = 6; // Echo Pin of Ultrasonic Sensor

const int PIN_RF_TX = 4;       // RF send signal pin
const int PIN_STATUS_LED = 13; // Built-in to Arduino Pro Mini
const int PIN_VEHICLE_DETECTED_LED = 9;
const int PIN_GARAGE_DOOR_OPEN_LED = PIN_STATUS_LED;

// Calculated constants
// prevent detection from taking a long time when out of range (x5 to wait for ping and echo to complete)
const unsigned long DISTANCE_ECHO_TIMEOUT = DETECT_VEHICLE_DISTANCE_MAX_CM * 5 * 29 * 2;

// Global
bool _isLowLevelVehicleDetected = false;
bool _isVehicleDetected = false;
bool _isGarageDoorOpen = false;
int _consecutiveVehicleReadings = 0;
bool _lastRawIsVehicleDetected = false;
int _consecutiveGarageDoorReadings = 0;
bool _lastRawIsGarageDoorOpen = false;
char _buffer[40];

void setup()
{
  pinMode(PIN_GARAGE_DOOR_SENSOR, INPUT_PULLUP);

  pinMode(PIN_VEHICLE_DETECTED_LED, OUTPUT);
  digitalWrite(PIN_VEHICLE_DETECTED_LED, LOW);

  // Initialize ultrasonic sensor
  pinMode(PIN_PING, OUTPUT);
  pinMode(PIN_PONG, INPUT);
  digitalWrite(PIN_PING, LOW);

  // Initialize RF transmitter
  vw_set_tx_pin(PIN_RF_TX);
  vw_setup(RF_BPS); // Bits per sec

  Serial.begin(9600);
}

void loop()
{
  ReadSensors();

  // Serial.print("Is vehicle in garage: ");
  // Serial.println(_isVehicleDetected ? "Yes" : "No");

  // int distanceCm = GetDistance();
  // Serial.print("Distance: ");
  // Serial.print(distanceCm);
  // Serial.println(" cm");

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

void ReadSensors()
{
  bool isGarageDoorOpen = IsGarageDoorOpen();
  bool isVehicleDetected = IsVehicleDetected();

  // Update the status LEDs immediately for in-person sensor debugging.
  digitalWrite(PIN_GARAGE_DOOR_OPEN_LED, isGarageDoorOpen ? LOW : HIGH);
  digitalWrite(PIN_VEHICLE_DETECTED_LED, isVehicleDetected ? LOW : HIGH);
}

void TransmitMessage(char *message)
{
  Serial.print("Transmitting: ");
  Serial.println(message);
  vw_send((uint8_t *)message, strlen(message)); // send the message
  vw_wait_tx();                                 // Wait until the whole message is sent
}

bool IsGarageDoorOpen()
{
  bool rawIsGarageDoorOpen = digitalRead(PIN_GARAGE_DOOR_SENSOR) == HIGH;

  // Check if this reading matches the last reading
  if (rawIsGarageDoorOpen == _lastRawIsGarageDoorOpen)
  {
    // Same as last time, increment counter
    _consecutiveGarageDoorReadings++;
  }
  else
  {
    // Different from last time, reset counter
    _consecutiveGarageDoorReadings = 1;
    _lastRawIsGarageDoorOpen = rawIsGarageDoorOpen;
  }

  // Only change state if we've seen enough consecutive identical readings
  if (_consecutiveGarageDoorReadings >= DETECT_GARAGE_DOOR_CONSECUTIVE_READINGS && rawIsGarageDoorOpen != _isGarageDoorOpen)
  {
    _isGarageDoorOpen = rawIsGarageDoorOpen;
    Serial.print("Garage door state changed to: ");
    Serial.println(_isGarageDoorOpen ? "OPEN" : "CLOSED");
  }

  return _isGarageDoorOpen;
}

bool IsVehicleDetected()
{
  long distance_cm = GetDistance();
  bool rawIsVehicleDetected = distance_cm != -1 && distance_cm >= DETECT_VEHICLE_DISTANCE_MIN_CM && distance_cm <= DETECT_VEHICLE_DISTANCE_MAX_CM;

  // Check if this reading matches the last reading
  if (rawIsVehicleDetected == _lastRawIsVehicleDetected)
  {
    // Same as last time, increment counter
    _consecutiveVehicleReadings++;
  }
  else
  {
    // Different from last time, reset counter
    _consecutiveVehicleReadings = 1;
    _lastRawIsVehicleDetected = rawIsVehicleDetected;
  }

  // Only change state if we've seen enough consecutive identical readings
  if (_consecutiveVehicleReadings >= DETECT_VEHICLE_CONSECUTIVE_READINGS && rawIsVehicleDetected != _isVehicleDetected)
  {
    _isVehicleDetected = rawIsVehicleDetected;
    Serial.print("Vehicle detection changed to: ");
    Serial.println(_isVehicleDetected ? "DETECTED" : "NOT DETECTED");
  }

  return _isVehicleDetected;
}

long GetDistance()
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
