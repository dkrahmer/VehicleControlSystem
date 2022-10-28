#include <VirtualWire.h>

// Settings
const unsigned long RETURNING_HOME_ALLOW_DELAY_MILLIS = 20 * 1000;
const unsigned long DASH_CAM_MAX_ON_BATTERY_MILLIS = 2 * 24 * 60 * 60 * 1000;
const unsigned long DASH_CAM_MIN_ON_MILLIS = 10 * 1000;
const int STARTUP_MODE_DELAY_MILLIS = 1000;
const int MISSING_SIGNAL_MILLIS = 5 * 1000;
const int MIRROR_TRAVEL_FOLD_MILLIS = 3500;
const int MIRROR_TRAVEL_UNFOLD_MILLIS = 3500;
const int MIRROR_REST_MILLIS = 500; // time to wait before reversing direction
const int RF_BPS = 4800;
const char* VEHICLE_NOT_IN_GARAGE_MESSAGE = "Outside";
const char* VEHICLE_IN_GARAGE_MESSAGE = "In Garage";

// Pin assignment
const int PIN_MIRROR_OVERRIDE_FOLD_IN = 2;
const int PIN_MIRROR_OVERRIDE_FOLD_OUT = 3;

const int PIN_MIRROR_A_POWER = 4;
const int PIN_MIRROR_A_FOLD_OUT = 5;
const int PIN_MIRROR_A_FOLD_IN = 6;
const int PIN_MIRROR_B_FOLD_OUT = 7;
const int PIN_MIRROR_B_FOLD_IN = 8;
const int PIN_MIRROR_B_POWER = 9;

const int PIN_RF_RX = 10; // RF receive signal pin

const int PIN_DASH_CAM_POWER = A1;

const int PIN_VEHICLE_ON_SENSOR = A0;

// Global
bool _isDashCamOn = false;
bool _isDashCamTimedOut = false;
bool _isMirrorPowerOn = false;
bool _isMirrorOverrideIn = false;
bool _isMirrorOverrideOut = false;
bool _isVehicleOn = false;
bool _isVehicleInGarage = false;
bool _isSignalTimedOut = false;
bool _isReturningHome = false;
bool _isOffWhileVehicleInGarage = false;
bool _isStartupMode = true;
bool _isFirstFullRun = false;
int _mirrorPosition = 0;
unsigned long _lastVehicleOnMillis = 0;
unsigned long _lastVehicleOffMillis = 0;
unsigned long _lastDashCamOnMillis = 0;
unsigned long _lastDashCamOnBatteryMillis = 0;
unsigned long _lastMessageReceivedMillis = 0;
unsigned long _lastVehicleInGarageMillis = 0;
unsigned long _lastMirrorPositionSetMillis = 0;
uint8_t _buffer[VW_MAX_MESSAGE_LEN + 1]; // +1 for the termination char

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

  Serial.println("Starting RF receiver");
  
  // Initialize RF transmitter
  vw_set_rx_pin(PIN_RF_RX);
  vw_setup(RF_BPS);
  vw_rx_start();            // Start the receiver PLL running

  Serial.print("Starting input-only sequence for ");
  Serial.print(STARTUP_MODE_DELAY_MILLIS);
  Serial.println(" ms");
}

void loop()
{
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
  HandleRfReceive();

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
    bool isOffWhileVehicleInGarage = !isVehicleOn && _isVehicleInGarage;

    if (isOffWhileVehicleInGarage != _isOffWhileVehicleInGarage)
    {
      // only applies when the vehicle turns off while already in the garage
      // there is no affect if the vehicle detects in garage after already off
      Serial.print(" while in garage");
      _isOffWhileVehicleInGarage = isOffWhileVehicleInGarage;
    }
    Serial.println();
      
    SetIsReturningHome(false);
  }
    
  bool isMirrorOverrideIn = digitalRead(PIN_MIRROR_OVERRIDE_FOLD_IN) == LOW;
  if (isMirrorOverrideIn != _isMirrorOverrideIn)
  {
    Serial.print("Mirror fold in override ");
    Serial.println(isMirrorOverrideIn ? "enabled" : "disabled");
    _isMirrorOverrideIn = isMirrorOverrideIn;
  }

  bool isMirrorOverrideOut = digitalRead(PIN_MIRROR_OVERRIDE_FOLD_OUT) == LOW;
  if (isMirrorOverrideOut != _isMirrorOverrideOut)
  {
    Serial.print("Mirror fold out override ");
    Serial.println(isMirrorOverrideOut ? "enabled" : "disabled");
    _isMirrorOverrideOut = isMirrorOverrideOut;
  }
}

void HandleRfReceive()
{
  uint8_t buflen = VW_MAX_MESSAGE_LEN;

  bool messageReceived = false;
  if (vw_get_message(_buffer, &buflen)) // Non-blocking
  {
    _buffer[buflen] = 0; // VirtualWire does not self terminate
    Serial.print("Received ");
    Serial.print(buflen);
    Serial.print(" chars: [");
    Serial.print((char*) _buffer);
    Serial.println("]");
    if (strcmp(_buffer, VEHICLE_NOT_IN_GARAGE_MESSAGE) == 0)
    {
      messageReceived = true;
      _isVehicleInGarage = false;
    }
    else if (strcmp(_buffer, VEHICLE_IN_GARAGE_MESSAGE) == 0)
    {
      messageReceived = true;
      _isVehicleInGarage = true;
      _lastVehicleInGarageMillis = millis();
      SetIsReturningHome(false);
    }
  }

  if (messageReceived)
  {
    if (!_isReturningHome && _isSignalTimedOut && _isVehicleOn)
    {
      if (millis() - _lastVehicleOnMillis >= RETURNING_HOME_ALLOW_DELAY_MILLIS 
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
    Serial.print("Garage signal not seen for ");
    Serial.print(MISSING_SIGNAL_MILLIS);
    Serial.println(" millis.");
    _isSignalTimedOut = true;

    SetIsReturningHome(false);

    if (_isVehicleInGarage)
    {
      Serial.println("Assuming not in garage.");
      _isVehicleInGarage = false;
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
  bool mirrorsOut;
  if (_isMirrorOverrideOut)
  {
    mirrorsOut = true;
  }
  else if (_isMirrorOverrideIn)
  {
    mirrorsOut = false;
  }
  else
  { // automatic mode
    mirrorsOut = !(_isVehicleInGarage || _isReturningHome);
    if (!_isVehicleOn)
      mirrorsOut = false;
  }
  
  SetMirrorPositions(mirrorsOut);
}

void SetMirrorPositions(bool mirrorsOut)
{
  if (_isMirrorPowerOn)
  {
    long mirrorOnTime = millis() - _lastMirrorPositionSetMillis;
    if ((_mirrorPosition > 0 && mirrorOnTime >= MIRROR_TRAVEL_UNFOLD_MILLIS)
        || (_mirrorPosition < 0 && mirrorOnTime >= MIRROR_TRAVEL_FOLD_MILLIS))
      SetMirrorHBridge(0, "travel complete");
  }
  
  if (_mirrorPosition == (mirrorsOut ? 1 : -1))
    return; // the mirrors are already in the requested position

  if (_isMirrorPowerOn)
    SetMirrorHBridge(0, "wait before reversing");

  SetMirrorHBridge(mirrorsOut ? 1 : -1, "new position");
}

void SetMirrorHBridge(int mirrorPosition, char* reason)
{
  if (mirrorPosition == 0)
  {
    Serial.print("Turning off mirror folding power");    
  }
  else
  {
    Serial.print("Folding mirrors ");
    Serial.print(mirrorPosition > 0 ? "out" : "in");
  }
  if (reason)
  {
    Serial.print(" (");
    Serial.print(reason);
    Serial.print(")");
  }
  Serial.println();
  
  // turn off the mirror power (H-Bridge controller)
  digitalWrite(PIN_MIRROR_A_FOLD_IN, mirrorPosition == 0 ? LOW : (mirrorPosition > 0 ? LOW : HIGH));
  digitalWrite(PIN_MIRROR_A_FOLD_OUT, mirrorPosition == 0 ? LOW : (mirrorPosition > 0 ? HIGH : LOW));
  digitalWrite(PIN_MIRROR_B_FOLD_IN, mirrorPosition == 0 ? LOW : (mirrorPosition > 0 ? LOW : HIGH));
  digitalWrite(PIN_MIRROR_B_FOLD_OUT, mirrorPosition == 0 ? LOW : (mirrorPosition > 0 ? HIGH : LOW));
  digitalWrite(PIN_MIRROR_A_POWER, mirrorPosition == 0 ? LOW : HIGH);
  digitalWrite(PIN_MIRROR_B_POWER, mirrorPosition == 0 ? LOW : HIGH);
  
  _isMirrorPowerOn = mirrorPosition != 0;
  
  if (mirrorPosition == 0)
  {
    delay(MIRROR_REST_MILLIS);
  }
  else
  {
    _mirrorPosition = mirrorPosition;
    _lastMirrorPositionSetMillis = millis();
  }
}

void HandleDashCamPower()
{
  bool dashCamOn = (_isVehicleOn || !_isVehicleInGarage) && !_isOffWhileVehicleInGarage;
  
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
