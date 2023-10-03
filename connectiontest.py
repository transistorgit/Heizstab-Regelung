import minimalmodbus
from time import sleep

instrument = None


def loop():
    heartbeat = 0
    power = False

    while True:
        instrument.write_register(0, heartbeat)  # write heartbeat counter
        # instrument.write_bit(0, power)
        # instrument.write_bit(1, power)
        # instrument.write_bit(2, power)
        instrument.write_bits(0, [0, 0, power])

        sleep(1)
        heartbeat += 1
        if heartbeat > 100:
            heartbeat = 0
        power = not power
        print(
            f"Heartbeat: {instrument.read_register(1, 0, 4)}", end=" "
        )  # read heartbeat counter
        print(f"Power: {instrument.read_register(2, 0, 4)}", end=" ")  # read power
        print(f"Temp: {instrument.read_register(0, 2, 4):.2f}", end=" ")  # read temperature
        print(f"Type: 0x{instrument.read_register(3, 0, 4):X}")


def main():
    global instrument
    while True:
        try:
            instrument = minimalmodbus.Instrument(
                "com3", 33
            )  # port name, slave address (in decimal)

            print(instrument)
            loop()

        except minimalmodbus.NoResponseError as e:
            print(e)
            print("No response from slave")
        except Exception as e:
            print(e)
        finally:
            try:
                instrument.serial.close()
            except Exception:
                pass
        sleep(1)
        print("Restart after error")


if __name__ == "__main__":
    main()
