"""
UART Service
-------------

An example showing how to write a simple program using the Nordic Semiconductor
Adapted to send programs to a MicroBlocks VM over Bluetooth
(nRF) UART service.

"""

import asyncio
import sys
import time
from itertools import count, takewhile
from typing import Iterator

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

program1 = [0xFB,1,10,30,0,4,28,0,0,0,2,4,0,0,90,1,0,0,0,0,0,0,240,0,0,0,20,0,0,0,0,0,0,0,0xFE] # Set user LED on when started
program2 = [0xFB,1,11,30,0,7,28,0,0,0,2,4,0,0,90,1,0,0,0,0,0,0,240,0,0,0,20,0,0,0,0,0,0,0,0xFE] # Start user LED on button A 
program3 = [0xFB,1,12,30,0,8,28,0,0,0,2,0,0,0,90,1,0,0,0,0,0,0,240,0,0,0,20,0,0,0,0,0,0,0,0xFE] # Turn off user LED on button B

# Blink smily face
program4 = [0xFB,1,13,122,0,4,28,0,0,0,4,13,0,0,4,15,0,0,3,0,0,0,129,2,209,1,126,3,0,0,2,233,3,0,23,1,0,0,4,6,0,0,4,12,0,0,126,2,0,0,2,233,3,0,23,1,0,0,16,243,255,255,0,0,0,0,36,0,0,0,100,105,115,112,108,97,121,0,52,0,0,0,109,98,68,105,115,112,108,97,121,0,0,0,68,0,0,0,109,98,68,105,115,112,108,97,121,79,102,102,0,0,0,0,240,0,0,0,20,0,0,0,0,0,0,0,0xFE]

# Control hatchling servo on port C from distance sensor on port A - untested
program5 = [0xFB,1,14,62,0,4,28,0,0,0,4,6,0,0,4,7,0,0,20,1,7,0,20,2,11,0,15,1,0,0,16,250,255,255,0,0,0,0,20,0,0,0,67,0,0,0,20,0,0,0,65,0,0,0,240,0,0,0,20,0,0,0,0,0,0,0,0xFE]

program6 = [0xFB,1,15,46,0,7,28,0,0,0,4,4,0,0,2,101,0,0,20,2,6,0,15,1,0,0,0,0,0,0,20,0,0,0,67,0,0,0,240,0,0,0,20,0,0,0,0,0,0,0,0xFE] # turn on rotation servo on port C when button A pressed
program7 = [0xFB,1,16,46,0,8,28,0,0,0,4,4,0,0,2,1,0,0,20,2,6,0,15,1,0,0,0,0,0,0,20,0,0,0,67,0,0,0,240,0,0,0,20,0,0,0,0,0,0,0,0xFE] # turn off rotation servo on port C when button A pressed


# Codes for starting/stopping
stopprogram = [0xFA,6,0]
startprogram = [0xFA,5,0]

# TIP: you can get this function and more from the ``more-itertools`` package.
def sliced(data: bytes, n: int) -> Iterator[bytes]:
    """
    Slices *data* into chunks of size *n*. The last slice may be smaller than
    *n*.
    """
    return takewhile(len, (data[i : i + n] for i in count(0, n)))


async def uart_terminal():
    """This is a simple "terminal" program that uses the Nordic Semiconductor
    (nRF) UART service. It reads from stdin and sends each line of data to the
    remote device. Any data received from the device is printed to stdout.
    """

    def match_nus_uuid(device: BLEDevice, adv: AdvertisementData):
        # This assumes that the device includes the UART service UUID in the
        # advertising data. This test may need to be adjusted depending on the
        # actual advertising data supplied by the device.
        if UART_SERVICE_UUID.lower() in adv.service_uuids:
            return True

        return False

    device = await BleakScanner.find_device_by_filter(match_nus_uuid)

    if device is None:
        print("no matching device found, you may need to edit match_nus_uuid().")
        sys.exit(1)

    def handle_disconnect(_: BleakClient):
        print("Device was disconnected, goodbye.")
        # cancelling all tasks effectively ends the program
        for task in asyncio.all_tasks():
            task.cancel()

    def handle_rx(_: BleakGATTCharacteristic, data: bytearray):
        print("received:", data)

    async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
        await client.start_notify(UART_TX_CHAR_UUID, handle_rx)

        print("Connected, Script menu: 1. Set user LED on, 2. Set user LED when button A pressed, 3. Turn off user LED when button B pressed, 4. Make a smily face blink, 5. Set hatchling distance sensor to control servo, stop and start will stop/start all scripts")

        loop = asyncio.get_running_loop()
        nus = client.services.get_service(UART_SERVICE_UUID)
        rx_char = nus.get_characteristic(UART_RX_CHAR_UUID)
        print("rx_char.max_write_without_response_size", rx_char.max_write_without_response_size)

        while True:
            # This waits until you type a line and press ENTER.
            # A real terminal program might put stdin in raw mode so that things
            # like CTRL+C get passed to the remote device.
            response = await loop.run_in_executor(None, sys.stdin.buffer.readline)
            print(response)
            # data will be empty on EOF (e.g. CTRL+D on *nix)
            if not response:
                break

            if(response == b'1\r\n'):
                print("Sending program 1 - user LED on")
                data = program1

            if(response == b'2\r\n'):
                print("Sending program 2 - user LED on with Button A")
                data = program2

            if(response == b'3\r\n'):
                print("Sending program 3 - user LED on with Button B")
                data = program3

            if(response == b'4\r\n'):
                print("Sending program 4 - Blink smily")
                data = program4

            if(response == b'5\r\n'):
                print("Sending program 5 - Control servo with distance sensor")
                data = program5

            if(response == b'6\r\n'):
                print("Sending program 6 - Use Button A to turn on Servo on port C")
                data = program6

            if(response == b'7\r\n'):
                print("Sending program 7 - Use Button B to turn off Servo on port C")
                data = program7

            if(response == b'stop\r\n'):
                print("Sending stop command")
                data = stopprogram

            if(response == b'start\r\n'):
                print("Sending start command")
                data = startprogram

          #  if data == b'\n':
          #      buf = "12345679012345679"
          #      await client.write_gatt_char(rx_char, b'A', response=False)
          #      await client.write_gatt_char(rx_char, b'Z', response=False)
          #      print("Sent 190")
          #      continue

            # some devices, like devices running MicroPython, expect Windows
            # line endings (uncomment line below if needed)
            # data = data.replace(b"\n", b"\r\n")

            # Writing without response requires that the data can fit in a
            # single BLE packet. We can use the max_write_without_response_size
            # property to split the data into chunks that will fit.

            # for s in sliced(data, rx_char.max_write_without_response_size):
            bufCount = 0
            for s in sliced(bytearray(data), 20):
               await client.write_gatt_char(rx_char, s, response=False)
               if (bufCount % 50) == 0:
                   time.sleep(0.001)
               # time.sleep(0.00001)

            print("sent:", data)

#                 for i in range(10):
#                     await client.write_gatt_char(rx_char, buf, response=False)


if __name__ == "__main__":
    try:
        asyncio.run(uart_terminal())
    except asyncio.CancelledError:
        # task is cancelled on disconnect, so we ignore this error
        pass
