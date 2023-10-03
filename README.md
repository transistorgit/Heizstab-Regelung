Modbus RTU Server for controlling a water heater by Solid State Relais

Written for Arduino Nano

If upload isn't working, check that the serial connection to the RS485 transceiver is disconnected, as these pins are shared.

For Updating the heater power, it is mandatory to also send a heartbeat counter. If no heartbeat is updated, no power will be set.
After 5min without heartbeat, the power will be shut down and the error LED will light up.