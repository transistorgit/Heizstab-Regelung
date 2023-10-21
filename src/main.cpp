#include <Arduino.h>
#include <ArduinoModbus.h>

// device type
const auto DEVICE_TYPE{0x3286};

//comm settings
const auto BAUDRATE{9600};
const auto STATION_ID { 33 };

//pins
const auto SSR_500W { 6 };
const auto SSR_1000W { 5 };
const auto SSR_2000W { 4 };
const auto HEARTBEAT { 13 };
const auto ERROR_LED { 9 };
const auto ONE_WIRE_BUS{2};
const auto SWITCH_OVERRIDE{A5};

//modbus registers
const auto COIL_500W { 0 };
const auto COIL_1000W { 1 };
const auto COIL_2000W { 2 };
const auto INPREG_TEMPERATURE { 0 };
const auto INPREG_HEARTBEAT { 1 };  //outgoing heartbeat
const auto INPREG_POWER { 2 };     //outgoing power feedback
const auto INPREG_DEVICE_TYPE{3};  // outgoing device type, so that the client can detect the device type
const auto INPREG_MODE{4};         // outgoing switch Auto/On
const auto INPREG_DEBUGRECVEDHEARTBEAT{5};
const auto INPREG_DEBUGLASTHEARTBEAT{6};

const auto HOLDREG_HEARTBEAT{0}; // incoming heartbeat

//limits
const auto MAX_TEMP {70.0};
const auto TIMEOUT_SECONDS{300ul};

float read_temperature(){
  //TODO: read temperature from 1w thermometer
  return 42.42f;
}

void set_power(uint8_t power){
  digitalWrite(SSR_500W, power & 0x01);
  digitalWrite(SSR_1000W, power & 0x02);
  digitalWrite(SSR_2000W, power & 0x04);
}

void set_error_led(bool on){
  digitalWrite(ERROR_LED, on);
}

void setup() {

  pinMode(SSR_500W , OUTPUT);
  pinMode(SSR_1000W, OUTPUT); 
  pinMode(SSR_2000W, OUTPUT);
  pinMode(HEARTBEAT, OUTPUT);
  pinMode(ERROR_LED, OUTPUT);
  // pinMode(ONE_WIRE_BUS, INPUT);
  pinMode(SWITCH_OVERRIDE, INPUT_PULLUP);

  set_error_led(false);
  set_power(0);

  Serial.begin(BAUDRATE);
  while (!Serial)
    ;

  if(!ModbusRTUServer.begin(STATION_ID, BAUDRATE)){
    // Serial.println("Failed to start Modbus RTU Server!");
    while(1);
  }

  ModbusRTUServer.configureCoils(0, 3);
  ModbusRTUServer.configureInputRegisters(0, 7);
  ModbusRTUServer.configureHoldingRegisters(0, 1);
  ModbusRTUServer.inputRegisterWrite(INPREG_DEVICE_TYPE, DEVICE_TYPE);
}

void loop() {
  static auto heartbeat = 0;
  static int powerCmd = 0;
  static auto last_client_heartbeat = -1;
  static auto last_millis = millis();
  static auto last_update = millis();

  ModbusRTUServer.poll();
  auto temperature = read_temperature();
  ModbusRTUServer.inputRegisterWrite(INPREG_TEMPERATURE, temperature * 100);

  auto manualOverride = !digitalRead(SWITCH_OVERRIDE); // must be pulled low to activate override
  ModbusRTUServer.inputRegisterWrite(INPREG_MODE, manualOverride);

  if (manualOverride)
  {
    // manual on mode - all coils on
    set_error_led(heartbeat % 2);
    powerCmd = (1 << COIL_500W) + (1 << COIL_1000W) + (1 << COIL_2000W);
  }
  else
  { // auto mode

    set_error_led(false);
    // check if the client sends valid updates (if we got no update for a while, turn off everything)

    auto receivedHeartbeat = ModbusRTUServer.holdingRegisterRead(HOLDREG_HEARTBEAT);
    ModbusRTUServer.inputRegisterWrite(INPREG_DEBUGRECVEDHEARTBEAT, receivedHeartbeat);
    ModbusRTUServer.inputRegisterWrite(INPREG_DEBUGLASTHEARTBEAT, last_update & 0xffff);
    if (last_client_heartbeat != receivedHeartbeat)
    {
      // client sends an updated heartbeat - so everything is fine
      last_client_heartbeat = receivedHeartbeat;
      last_update = millis();

      // set power according to current client's request
      powerCmd = ModbusRTUServer.coilRead(COIL_500W) + (ModbusRTUServer.coilRead(COIL_1000W) << 1) + (ModbusRTUServer.coilRead(COIL_2000W) << 2);
    }
    else
    {
      // no update from client, shut off
      if (millis() - last_update > TIMEOUT_SECONDS * 1000ul)
      {
        powerCmd = 0;
        set_error_led(true);
      }
    }
  }
  set_power(powerCmd);

  auto power_feedback = (digitalRead(SSR_500W) * 500) + (digitalRead(SSR_1000W) * 1000) + (digitalRead(SSR_2000W) * 2000);
  ModbusRTUServer.inputRegisterWrite(INPREG_POWER, power_feedback);

  // heartbeat housekeeping
  if (millis() - last_millis > 1000ul)
  {
    last_millis = millis();

    if (heartbeat++ > 1000)
    {
      heartbeat = 0;
    }
    ModbusRTUServer.inputRegisterWrite(INPREG_HEARTBEAT, heartbeat); // testanfrage: 0x21 0x04 0x00 0x01 0x00 0x01 0x36 0xaa
    digitalWrite(HEARTBEAT, heartbeat % 2);
  }
}