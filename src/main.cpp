#include <Arduino.h>
#include <ArduinoModbus.h>

constexpr auto BAUDRATE { 19200 };
constexpr auto STATION_ID { 33 };

constexpr auto SSR_500W { 6 };
constexpr auto SSR_1000W { 5 };
constexpr auto SSR_2000W { 4 };
constexpr auto HEARTBEAT { 13 };

constexpr auto COIL_500W { 0 };
constexpr auto COIL_1000W { 1 };
constexpr auto COIL_2000W { 2 };
constexpr auto INPREG_TEMPERATURE { 0 };


void setup() {

  pinMode(SSR_500W , OUTPUT);
  pinMode(SSR_1000W, OUTPUT); 
  pinMode(SSR_2000W, OUTPUT);
  pinMode(HEARTBEAT, OUTPUT);

  Serial.begin(BAUDRATE);

  if(!ModbusRTUServer.begin(STATION_ID, BAUDRATE)){
    Serial.println("Failed to start Modbus RTU Server!");
    while(1);
  }

  ModbusRTUServer.configureCoils(0x00, 3);
  ModbusRTUServer.configureInputRegisters(0x00, 1);
}

void loop() {
  static auto heartBeat = 0;

  ModbusRTUServer.poll();
  ModbusRTUServer.inputRegisterWrite(INPREG_TEMPERATURE, heartBeat); //testanfrage: 0x21 0x04 0x00 0x00 0x00 0x01 0x36 0xaa
  delay(100);

  if(heartBeat++ > 255){
    heartBeat = 0;
  }
  digitalWrite(HEARTBEAT, heartBeat % 2);
}