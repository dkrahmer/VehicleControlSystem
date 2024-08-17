// Vehicle Controller
// Author: Douglas Krahmer
// Created: 2022-10-27

#define INCLUDE_RF // comment this line if not using RF input for garage sensing

#include <arduino-timer.h>
#ifdef INCLUDE_RF
#include <VirtualWire.h>
#endif

// Settings
const unsigned long RETURNING_HOME_ALLOW_DELAY_MILLIS = 5 * 60 * 1000UL;
const unsigned long DASH_CAM_MAX_ON_BATTERY_MILLIS = 2 * 24 * 60 * 60 * 1000UL;
const unsigned long DASH_CAM_MIN_ON_MILLIS = 10 * 1000UL;
const unsigned long STARTUP_MODE_DELAY_MILLIS = 1000UL;
const unsigned long MISSING_SIGNAL_MILLIS = 1 * 60 * 1000UL;
const unsigned long MIRROR_TRAVEL_FOLD_MILLIS = 3500UL;
const unsigned long MIRROR_TRAVEL_UNFOLD_MILLIS = 3500UL;
const unsigned long MIRROR_REST_MILLIS = 500UL; // time to wait before reversing direction

#ifdef INCLUDE_RF
const int RF_BPS = 4800;
#else
const int VW_MAX_MESSAGE_LEN = 20;
#endif
const char* VEHICLE_DETECTED_MESSAGE = "Detected";
const char* VEHICLE_NOT_DETECTED_MESSAGE = "Undetected";
const char* GARAGE_DOOR_OPEN_MESSAGE = "Open";
const char* GARAGE_DOOR_CLOSED_MESSAGE = "Closed";
const char MESSAGE_SEPARATOR_CHAR = '-';

// Pin assignment
const int PIN_MIRROR_OVERRIDE_FOLD_IN = 2;
const int PIN_MIRROR_OVERRIDE_FOLD_OUT = 3;

const int PIN_MIRROR_A_POWER = 4;
const int PIN_MIRROR_A_FOLD_OUT = 5;
const int PIN_MIRROR_A_FOLD_IN = 6;
const int PIN_MIRROR_B_FOLD_OUT = 7;
const int PIN_MIRROR_B_FOLD_IN = 8;
const int PIN_MIRROR_B_POWER = 9;

const int PIN_DASH_CAM_POWER = A1;

const int PIN_VEHICLE_ON_SENSOR = A0;

#ifdef INCLUDE_RF
const int PIN_RF_RX = 10; // RF receive signal pin
#else
const int PIN_VEHICLE_DETECTED_SENSOR = 11;
const int PIN_VEHICLE_NOT_DETECTED_SENSOR = 12;
const int PIN_GARAGE_DOOR_OPEN_SENSOR = A5;
#endif

// Global
Timer<> _timer = timer_create_default();

bool _isDashCamOn = false;
bool _isDashCamTimedOut = false;
bool _isMirrorAFoldPowerOn = false;
bool _isMirrorBFoldPowerOn = false;
bool _isMirrorOverrideFoldIn = false;
bool _isMirrorOverrideFoldOut = false;
bool _isMirrorAFoldResting = false;
bool _isMirrorBFoldResting = false;
bool _isVehicleOn = false;
bool _isVehicleDetected = false;
bool _isSignalTimedOut = false;
bool _isReturningHome = false;
bool _isOffWhileVehicleDetected = false;
bool _isGarageDoorOpen = false;
bool _isStartupMode = true;
bool _isFirstFullRun = false;
int _mirrorAFoldPosition = 0;
int _mirrorBFoldPosition = 0;
unsigned long _lastVehicleOnMillis = 0UL;
unsigned long _lastVehicleOffMillis = 0UL;
unsigned long _lastDashCamOnMillis = 0UL;
unsigned long _lastDashCamOnBatteryMillis = 0UL;
unsigned long _lastMessageReceivedMillis = 0UL;
unsigned long _lastVehicleDetectedMillis = 0UL;
unsigned long _lastMirrorAFoldPositionSetMillis = 0UL;
unsigned long _lastMirrorBFoldPositionSetMillis = 0UL;
#ifdef INCLUDE_RF
uint8_t _buffer[VW_MAX_MESSAGE_LEN + 1]; // +1 for the termination char
#endif

void setup()
{
  Serial.begin(9600);

  pinMode(PIN_DASH_CAM_POWER, OUTPUT);

  pinMode(PIN_MIRROR_OVERRIDE_FOLD_IN, INPUT_PULLUP);
  pinMode(PIN_MIRROR_OVERRIDE_FOLD_OUT, INPUT_PULLUP);
  pinMode(PIN_VEHICLE_ON_SENSOR, INPUT); // needs a pulldown resistor
  
  pinMode(PIN_MIRROR_A_POWER, OUTPUT);
  pinMode(PIN_MIRROR_A_FOLD_IN, OUTPUT);
  pinMode(PIN_MIRROR_A_FOLD_OUT, OUTPUT);
  pinMode(PIN_MIRROR_B_POWER, OUTPUT);
  pinMode(PIN_MIRROR_B_FOLD_IN, OUTPUT);
  pinMode(PIN_MIRROR_B_FOLD_OUT, OUTPUT);

  digitalWrite(PIN_DASH_CAM_POWER, LOW);

  digitalWrite(PIN_MIRROR_A_POWER, LOW);
  digitalWrite(PIN_MIRROR_A_FOLD_IN, LOW);
  digitalWrite(PIN_MIRROR_A_FOLD_OUT, LOW);
  digitalWrite(PIN_MIRROR_B_POWER, LOW);
  digitalWrite(PIN_MIRROR_B_FOLD_IN, LOW);
  digitalWrite(PIN_MIRROR_B_FOLD_OUT, LOW);

#ifdef INCLUDE_RF
  Serial.println("Starting RF receiver");
  
  // Initialize RF transmitter
  vw_set_rx_pin(PIN_RF_RX);
  vw_setup(RF_BPS);
  vw_rx_start();            // Start the receiver PLL running
#else
  pinMode(PIN_VEHICLE_DETECTED_SENSOR, INPUT_PULLUP);
  pinMode(PIN_VEHICLE_NOT_DETECTED_SENSOR, INPUT_PULLUP);
  pinMode(PIN_GARAGE_DOOR_OPEN_SENSOR, INPUT_PULLUP);
#endif

  Serial.print("Starting input-only sequence for ");
  Serial.print(STARTUP_MODE_DELAY_MILLIS);
  Serial.println(" ms");
}

void loop()
{
  _timer.tick<void>();
  
  HandleInputs();
  if (_isStartupMode)
  {
    if (millis() < STARTUP_MODE_DELAY_MILLIS)
      return; // only get inputs when in startup mode 

    _isStartupMode = false;
    _isFirstFullRun = true;
    Serial.println("Starting full sequence");
    return;
  }
  
  HandleDashCamPower();
  HandleMirrorPositions();
  _isFirstFullRun = false;
}

void HandleInputs()
{
  HandleRfInput();

  bool isVehicleOn = digitalRead(PIN_VEHICLE_ON_SENSOR) == HIGH;
  if (isVehicleOn != _isVehicleOn || _isFirstFullRun)
  {
    // state changed
    if (isVehicleOn)
    {
      Serial.print(_isFirstFullRun ? "Vehicle is on" : "Vehicle turned on");
      _lastVehicleOnMillis = millis();
    }
    else
    {
      Serial.print(_isFirstFullRun ? "Vehicle is off" : "Vehicle turned off");
      _lastVehicleOffMillis = millis();
      if (_isDashCamOn)
        _lastDashCamOnBatteryMillis = millis();
    }
    _isVehicleOn = isVehicleOn;
    bool isOffWhileVehicleDetected = !isVehicleOn && _isVehicleDetected;

    if (isOffWhileVehicleDetected != _isOffWhileVehicleDetected)
    {
      // only applies when the vehicle turns off while already in the garage
      // there is no affect if the vehicle detects in garage after already off
      Serial.print(" while in garage");
      _isOffWhileVehicleDetected = isOffWhileVehicleDetected;
    }
    Serial.println();
      
    SetIsReturningHome(false);
  }
    
  bool isMirrorOverrideIn = digitalRead(PIN_MIRROR_OVERRIDE_FOLD_IN) == LOW;
  if (isMirrorOverrideIn != _isMirrorOverrideFoldIn)
  {
    Serial.print("Mirror fold in override ");
    Serial.println(isMirrorOverrideIn ? "enabled" : "disabled");
    _isMirrorOverrideFoldIn = isMirrorOverrideIn;
  }

  bool isMirrorOverrideOut = digitalRead(PIN_MIRROR_OVERRIDE_FOLD_OUT) == LOW;
  if (isMirrorOverrideOut != _isMirrorOverrideFoldOut)
  {
    Serial.print("Mirror fold out override ");
    Serial.println(isMirrorOverrideOut ? "enabled" : "disabled");
    _isMirrorOverrideFoldOut = isMirrorOverrideOut;
  }
}

void HandleRfInput()
{
  uint8_t buflen = VW_MAX_MESSAGE_LEN;

  bool messageReceived = false;
  int messagesReceived = 0;
#ifdef INCLUDE_RF
  if (vw_get_message(_buffer, &buflen)) // Non-blocking
#else
  bool isVehicleNotDetected = digitalRead(PIN_VEHICLE_NOT_DETECTED_SENSOR) == LOW;
  bool isVehicleDetected = digitalRead(PIN_VEHICLE_DETECTED_SENSOR) == LOW;
  bool isGarageDoorOpen = digitalRead(PIN_GARAGE_DOOR_OPEN_SENSOR) == LOW;
  
  char _buffer[20];
  char* msg1;
  if (isVehicleDetected)
    msg1 = VEHICLE_DETECTED_MESSAGE;
  else if (isVehicleNotDetected)
    msg1 = VEHICLE_NOT_DETECTED_MESSAGE;
  else
    msg1 = "";

  if (strlen(msg1) > 0)
  {
    sprintf(_buffer, "%s%c%s", 
      msg1, 
      MESSAGE_SEPARATOR_CHAR, 
      isGarageDoorOpen ? GARAGE_DOOR_OPEN_MESSAGE : GARAGE_DOOR_CLOSED_MESSAGE);
  }
  else
  {
    _buffer[0] = 0;
  }

  buflen = strlen(_buffer);
  if (buflen > 0)
#endif
  {
    _buffer[buflen] = 0; // VirtualWire does not self terminate strings
    Serial.print("Received ");
    Serial.print(buflen);
    Serial.print(" chars: [");
    Serial.print((char*) _buffer);
    Serial.println("]");
    if (buflen > 3)
    {
      unsigned long lastVehicleDetectedMillis = _lastVehicleDetectedMillis;
      bool isVehicleDetected = _isVehicleDetected;
      if (strstr(_buffer, VEHICLE_NOT_DETECTED_MESSAGE))
      {
        messagesReceived++;
        isVehicleDetected = false;
      }
      else if (strstr(_buffer, VEHICLE_DETECTED_MESSAGE))
      {
        messagesReceived++;
        isVehicleDetected = true;
        lastVehicleDetectedMillis = millis();
        SetIsReturningHome(false);
      }

      bool isGarageDoorOpen = true;
      if (strstr(_buffer, GARAGE_DOOR_OPEN_MESSAGE))
      {
        messagesReceived++;
        isGarageDoorOpen = true;
      }
      else if (strstr(_buffer, GARAGE_DOOR_CLOSED_MESSAGE))
      {
        messagesReceived++;
        isGarageDoorOpen = false;
      }

      messageReceived = messagesReceived == 2;
      if (messageReceived)
      {
        if (_isVehicleDetected != isVehicleDetected)
        {
          _isVehicleDetected = isVehicleDetected;
          _lastVehicleDetectedMillis = lastVehicleDetectedMillis;

          Serial.print("Vehicle is ");
          Serial.print(_isVehicleDetected ? "" : "not ");
          Serial.println("detected.");
        }

        if (_isGarageDoorOpen != isGarageDoorOpen)
        {
          _isGarageDoorOpen = isGarageDoorOpen;

          Serial.print("Garage door is ");
          Serial.print(_isGarageDoorOpen ? "open" : "closed");
          Serial.println(".");
        }
      }
    }
  }

  if (messageReceived)
  {
    if (!_isReturningHome && _isVehicleOn && _isSignalTimedOut)
    {
      if (millis() - _lastVehicleOnMillis >= MISSING_SIGNAL_MILLIS // the "last vehicle on" condition is to handle driveway parking
        && millis() - _lastMessageReceivedMillis >= RETURNING_HOME_ALLOW_DELAY_MILLIS)
      {
        SetIsReturningHome(true);
      }
      else
      {
        Serial.print("Would have set 'returning home' mode but timeouts were not met. ('vehicle on' and 'no signal' for ");
        Serial.print(RETURNING_HOME_ALLOW_DELAY_MILLIS);
        Serial.println(" millis)");
      }
    }
    
    _isSignalTimedOut = false;
    _lastMessageReceivedMillis = millis();
    return;
  }
  
  if (!_isSignalTimedOut && millis() - _lastMessageReceivedMillis >= MISSING_SIGNAL_MILLIS)
  {
    Serial.print("Signal not seen for ");
    Serial.print(MISSING_SIGNAL_MILLIS);
    Serial.println(" millis.");
    _isSignalTimedOut = true;

    SetIsReturningHome(false);

    if (_isVehicleDetected)
    {
      Serial.println("Assuming not in garage.");
      _isVehicleDetected = false;
    }
  }
}

void SetIsReturningHome(bool isReturningHome)
{
  if (isReturningHome == _isReturningHome)
    return;
    
  Serial.print(isReturningHome ? "Enabling" : "Disabling");
  Serial.println(" 'returning home' mode.");
  _isReturningHome = isReturningHome;
}

void HandleMirrorPositions()
{
  bool mirrorAFoldOut;
  bool mirrorBFoldOut;
  if (_isMirrorOverrideFoldOut)
  {
    mirrorAFoldOut = true;
    mirrorBFoldOut = true;
  }
  else if (_isMirrorOverrideFoldIn)
  {
    mirrorAFoldOut = false;
    mirrorBFoldOut = false;
  }
  else
  { // automatic mode
    if (_isVehicleOn) 
    {
      mirrorAFoldOut = !(_isVehicleDetected || (_isReturningHome && _isGarageDoorOpen));
      mirrorBFoldOut = true;
    }
    else
    {
      mirrorAFoldOut = false;
      mirrorBFoldOut = false;
    }
  }
  
  SetMirrorFoldPosition('A', mirrorAFoldOut);
  SetMirrorFoldPosition('B', mirrorBFoldOut);
}

void SetMirrorFoldPosition(char mirrorId, bool mirrorOut)
{
  bool* __isMirrorFoldPowerOn;
  unsigned long* __lastMirrorFoldPositionSetMillis;
  int* __mirrorFoldPosition = &_mirrorAFoldPosition;
  bool* __isMirrorFoldResting;

  if (mirrorId == 'A')
  {
    __isMirrorFoldPowerOn = &_isMirrorAFoldPowerOn;
    __lastMirrorFoldPositionSetMillis = &_lastMirrorAFoldPositionSetMillis;
    __mirrorFoldPosition = &_mirrorAFoldPosition;
    __isMirrorFoldResting = &_isMirrorAFoldResting;
  }
  else if (mirrorId == 'B')
  {
    __isMirrorFoldPowerOn = &_isMirrorBFoldPowerOn;
    __lastMirrorFoldPositionSetMillis = &_lastMirrorBFoldPositionSetMillis;
    __mirrorFoldPosition = &_mirrorBFoldPosition;
    __isMirrorFoldResting = &_isMirrorBFoldResting;
  }

  if (*__isMirrorFoldResting)
    return; // no action while resting

  if (*__isMirrorFoldPowerOn)
  {
    long mirrorOnTime = millis() - *__lastMirrorFoldPositionSetMillis;
    if ((*__mirrorFoldPosition > 0 && mirrorOnTime >= MIRROR_TRAVEL_UNFOLD_MILLIS)
        || (*__mirrorFoldPosition < 0 && mirrorOnTime >= MIRROR_TRAVEL_FOLD_MILLIS))
      {
        SetMirrorHBridge(mirrorId, 0, "travel complete");
        RestMirrorFold(__isMirrorFoldResting);
      }
  }
  
  if (*__mirrorFoldPosition == (mirrorOut ? 1 : -1))
    return; // the mirror is already in the requested position

  if (*__isMirrorFoldPowerOn)
  {
    // If the power was already on then we need to turn off the power and wait to ensure the motor fully stops
    SetMirrorHBridge(mirrorId, 0, "stop and wait before reversing direction");
    RestMirrorFold(__isMirrorFoldResting);
  }
  else
  {
    SetMirrorHBridge(mirrorId, mirrorOut ? 1 : -1, "new position");
  }
}

void RestMirrorFold(bool* isMirrorFoldResting)
{
  *isMirrorFoldResting = true;
  _timer.in(MIRROR_REST_MILLIS, [](bool* isMirrorFoldResting) 
  {
    *isMirrorFoldResting = false; 
  }, isMirrorFoldResting);
}

void SetMirrorHBridge(char mirrorId, int mirrorFoldPosition, char* reason)
{
  bool* __isMirrorFoldPowerOn;
  int pinMirrorFoldIn;
  int pinMirrorFoldOut;
  int pinMirrorFoldPower;
  unsigned long* __lastMirrorFoldPositionSetMillis;
  int* __mirrorFoldPosition;
  
  if (mirrorId == 'A')
  {
    __isMirrorFoldPowerOn = &_isMirrorAFoldPowerOn;
    pinMirrorFoldIn = PIN_MIRROR_A_FOLD_IN;
    pinMirrorFoldOut = PIN_MIRROR_A_FOLD_OUT;
    pinMirrorFoldPower = PIN_MIRROR_A_POWER;
    __lastMirrorFoldPositionSetMillis = &_lastMirrorAFoldPositionSetMillis;
    __mirrorFoldPosition = &_mirrorAFoldPosition;
  }
  else if (mirrorId == 'B')
  {
    __isMirrorFoldPowerOn = &_isMirrorBFoldPowerOn;
    pinMirrorFoldIn = PIN_MIRROR_B_FOLD_IN;
    pinMirrorFoldOut = PIN_MIRROR_B_FOLD_OUT;
    pinMirrorFoldPower = PIN_MIRROR_B_POWER;
    __lastMirrorFoldPositionSetMillis = &_lastMirrorBFoldPositionSetMillis;
    __mirrorFoldPosition = &_mirrorBFoldPosition;
  }
  else
  {
    return; // invalid mirrorId
  }

  if (mirrorFoldPosition == 0)
  {
    Serial.print("Turning off mirror ");    
    Serial.print(mirrorId);    
    Serial.print(" folding power");    
  }
  else
  {
    Serial.print("Folding mirror ");
    Serial.print(mirrorId);    
    Serial.print(mirrorFoldPosition > 0 ? " out" : " in");
  }

  if (reason)
  {
    Serial.print(" (");
    Serial.print(reason);
    Serial.print(")");
  }
  Serial.println();
  
  digitalWrite(pinMirrorFoldIn, mirrorFoldPosition == 0 ? LOW : (mirrorFoldPosition > 0 ? LOW : HIGH));
  digitalWrite(pinMirrorFoldOut, mirrorFoldPosition == 0 ? LOW : (mirrorFoldPosition > 0 ? HIGH : LOW));
  digitalWrite(pinMirrorFoldPower, mirrorFoldPosition == 0 ? LOW : HIGH);
  *__isMirrorFoldPowerOn = mirrorFoldPosition != 0;
  
  if (mirrorFoldPosition == 0)
    return; // position 0 is only to turn off the motor. No need to update the position value.

  *__mirrorFoldPosition = mirrorFoldPosition;
  *__lastMirrorFoldPositionSetMillis = millis();
}

void HandleDashCamPower()
{
  bool dashCamOn = (_isVehicleOn || !_isVehicleDetected) && !_isOffWhileVehicleDetected;
  
  if (dashCamOn && _isDashCamTimedOut && _isVehicleOn)
  {
    // A dash cam timeout exists but the vehicle is on now so we can reset the timeout
    _isDashCamOn = false;
    _isDashCamTimedOut = false;
  }
  
  if (dashCamOn != _isDashCamOn)
  {
    if (!dashCamOn)
    {
      // make sure to not turn it off too soon (prevent damage from short cycling)
      if (millis() - _lastDashCamOnMillis < DASH_CAM_MIN_ON_MILLIS)
        return; // maybe next time
    }
    Serial.print("Turning dash cam ");
    Serial.println(dashCamOn > 0 ? "on" : "off");
    
    digitalWrite(PIN_DASH_CAM_POWER, dashCamOn ? HIGH : LOW);
    _isDashCamOn = dashCamOn;
    _isDashCamTimedOut = false;
    _lastDashCamOnMillis = millis();
    if (dashCamOn && !_isVehicleOn)
      _lastDashCamOnBatteryMillis = millis();
  }
  else
  {
    if (_isDashCamOn && !_isDashCamTimedOut && !_isVehicleOn
      && millis() - _lastDashCamOnBatteryMillis >= DASH_CAM_MAX_ON_BATTERY_MILLIS)
    {
      Serial.println("Turning dash cam off because it has exceded the max continuous on time.");
      digitalWrite(PIN_DASH_CAM_POWER, LOW);
      _isDashCamTimedOut = true;
    }
  }
}
