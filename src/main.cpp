#include <Arduino.h>
#include <ArduinoModbus.h>

//comm settings
const auto BAUDRATE { 19200 };
const auto STATION_ID { 33 };

//pins
const auto SSR_500W { 6 };
const auto SSR_1000W { 5 };
const auto SSR_2000W { 4 };
const auto HEARTBEAT { 13 };
const auto ERROR_LED { 9 };

//modbus registers
const auto COIL_500W { 0 };
const auto COIL_1000W { 1 };
const auto COIL_2000W { 2 };
const auto INPREG_TEMPERATURE { 0 };
const auto INPREG_HEARTBEAT { 1 };  //outgoing heartbeat
const auto INPREG_POWER { 2 };     //outgoing power feedback
const auto HOLDREG_HEARTBEAT { 0 }; //incoming heartbeat

//limits
const auto MAX_TEMP {70.0};
const auto TIMEOUT_SECONDS {300};

float read_temperature(){
  //TODO: read temperature from 1w thermometer
  return 42.0;
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
  set_error_led(false);
  set_power(0);

  Serial.begin(BAUDRATE);

  if(!ModbusRTUServer.begin(STATION_ID, BAUDRATE)){
    Serial.println("Failed to start Modbus RTU Server!");
    while(1);
  }

  ModbusRTUServer.configureCoils(0, 3);
  ModbusRTUServer.configureInputRegisters(0, 3);
  ModbusRTUServer.configureHoldingRegisters(0, 1);
}

void loop() {
  static auto heartbeat = 0;
  static auto last_client_heartbeat = -1;
  static auto last_millis = millis();

  ModbusRTUServer.poll();
  
  auto temperature = read_temperature();
  ModbusRTUServer.inputRegisterWrite(INPREG_TEMPERATURE, temperature);

  //if we got no update for a while, turn off everything
  //we sync the heartbeats and can detect a timeout by comparing the incoming heartbeat with our own
  if (last_client_heartbeat != ModbusRTUServer.holdingRegisterRead(HOLDREG_HEARTBEAT)){
    last_client_heartbeat = ModbusRTUServer.holdingRegisterRead(HOLDREG_HEARTBEAT);
    heartbeat = last_client_heartbeat;
    set_error_led(false);	

    //set power according to client request
    auto power = ModbusRTUServer.coilRead(COIL_500W) + (ModbusRTUServer.coilRead(COIL_1000W) << 1) + (ModbusRTUServer.coilRead(COIL_2000W) << 2);
    set_power(power);
  }
  else{
    //no update from client
    if(heartbeat - last_client_heartbeat > TIMEOUT_SECONDS){
      set_power(0);
      set_error_led(true);
    }
  }

  auto power_feedback = (digitalRead(SSR_500W)*500) + (digitalRead(SSR_1000W)*1000) + (digitalRead(SSR_2000W)*2000);
  ModbusRTUServer.inputRegisterWrite(INPREG_POWER, power_feedback);

  if(millis() - last_millis > 1000){
    last_millis = millis();
   
    if(heartbeat++ > 1000){ //we don't want to overflow, in case we get no update from client
      heartbeat = 0;
      last_client_heartbeat = ModbusRTUServer.holdingRegisterRead(HOLDREG_HEARTBEAT);
    }
    ModbusRTUServer.inputRegisterWrite(INPREG_HEARTBEAT, heartbeat); //testanfrage: 0x21 0x04 0x00 0x01 0x00 0x01 0x36 0xaa
    digitalWrite(HEARTBEAT, heartbeat % 2);
  }
}