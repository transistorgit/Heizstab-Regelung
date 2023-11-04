#include <Arduino.h>
#include <ArduinoModbus.h>
#include <OneWire.h>

// device type, arbitrary number
const auto DEVICE_TYPE{0xE5E1};

//comm settings
const auto BAUDRATE{9600};
const auto STATION_ID{33};

//pins
const auto SSR_500W{4};
const auto SSR_1000W{5};
const auto SSR_2000W{6};
const auto HEARTBEAT{13};
const auto ERROR_LED{9};
const auto ONE_WIRE_BUS{2};
const auto SWITCH_OVERRIDE{A5};

//modbus registers
const auto COIL_500W{0};
const auto COIL_1000W{1};
const auto COIL_2000W{2};
const auto INPREG_TEMPERATURE{0};
const auto INPREG_HEARTBEAT{1};   // outgoing heartbeat
const auto INPREG_POWER{2};       // outgoing power feedback
const auto INPREG_DEVICE_TYPE{3}; // outgoing device type, so that the client can detect the device type
const auto INPREG_MODE{4};        // outgoing switch Auto/On
const auto HOLDREG_HEARTBEAT{0};  // incoming heartbeat

OneWire ds(ONE_WIRE_BUS);
byte data[9];

//limits
const auto MAX_TEMP{50.0};
const auto TIMEOUT_SECONDS{300ul};

void set_power(uint8_t power){
  digitalWrite(SSR_500W, power & 0x01);
  digitalWrite(SSR_1000W, power & 0x02);
  digitalWrite(SSR_2000W, power & 0x04);
}

void set_error_led(bool on){
  digitalWrite(ERROR_LED, on);
}

// for testing or sensor checking
void oneWireScan()
{
  byte i;
  byte addr[8];
  
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);        // start conversion, with parasite power on at the end
}

void oneWireConvert()
{
  ds.reset();
  ds.skip();
  ds.write(0x44);
}

float oneWireRead()
{
  ds.reset();
  ds.skip();
  ds.write(0xBE);
  ds.read_bytes(data, 9);
  if (data[8] == ds.crc8(data, 8))
  {
    int16_t raw = (data[1] << 8) | data[0];
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00)
      raw = raw & ~7; // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20)
      raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40)
      raw = raw & ~1;         // 11 bit res, 375 ms
    return (float)raw / 16.0; // celsius
  }
  return 85.0;
}

void setup() {

  pinMode(SSR_500W , OUTPUT);
  pinMode(SSR_1000W, OUTPUT); 
  pinMode(SSR_2000W, OUTPUT);
  pinMode(HEARTBEAT, OUTPUT);
  pinMode(ERROR_LED, OUTPUT);
  pinMode(ONE_WIRE_BUS, INPUT);
  pinMode(SWITCH_OVERRIDE, INPUT_PULLUP);

  set_error_led(false);
  set_power(0);

  Serial.begin(BAUDRATE);
  while (!Serial)
    ;

  //oneWireScan(); //test sensors
  oneWireConvert();

  if (!ModbusRTUServer.begin(STATION_ID, BAUDRATE))
  {
    Serial.println("Failed to start Modbus RTU Server!");
    while(1);
  }

  ModbusRTUServer.configureCoils(0, 3);
  ModbusRTUServer.configureInputRegisters(0, 5);
  ModbusRTUServer.configureHoldingRegisters(0, 1);
  ModbusRTUServer.inputRegisterWrite(INPREG_DEVICE_TYPE, DEVICE_TYPE);
}

void loop()
{
  static auto heartbeat = 0;
  static auto powerCmd = 0;
  static auto last_client_heartbeat = -1;
  static auto last_millis = millis();
  static auto last_update = millis();

  ModbusRTUServer.poll();

  auto manualOverride = !digitalRead(SWITCH_OVERRIDE); // must be pulled low to activate override
  ModbusRTUServer.inputRegisterWrite(INPREG_MODE, manualOverride); // 0 AUTO, 1 FORCE ON

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
    if (last_client_heartbeat != receivedHeartbeat)
    {
      // client sends an updated heartbeat - so everything is fine
      last_client_heartbeat = receivedHeartbeat;
      last_update = millis();

      // set power according to the current client's request
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

  // heartbeat housekeeping and one-per-second-activities
  if (millis() - last_millis > 1000ul)
  {
    last_millis = millis();
    heartbeat++;

    ModbusRTUServer.inputRegisterWrite(INPREG_HEARTBEAT, heartbeat & 0xFFFF); // test request: 0x21 0x04 0x00 0x01 0x00 0x01 0x36 0xaa
    digitalWrite(HEARTBEAT, heartbeat % 2);

    auto temperature = oneWireRead();
    ModbusRTUServer.inputRegisterWrite(INPREG_TEMPERATURE, temperature * 100);
    oneWireConvert(); // sensor needs >750ms in background
  }
}