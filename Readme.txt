Bluetooth Low Energy (BLE) for the micro:bit
John Maloney
September 2023

The folder lib/BLE_peripheral_serial contains a modified version of Sandeep Mistry's Arduino BLEPeripheral library for the micro:bit updated to work with the newer softdevices S113 and S140 for the micro:bit v2. It also works with the S110 softdevice for the micro:bit v1.

The S110 and S113 are peripheral-only softdevices for the micro:bit v1 and v2 respectively. The S140 is a central + peripheral softdevice for the micro:bit v2. However, since the S140 consumes more resources and this library does not use central mode, there is not much point in using the S140. But since the S140 uses the same API as the S113 it was easy to support, and it could be useful in the future.

Source Code and License

Sandeep's original code is here:

   https://github.com/sandeepmistry/arduino-BLEPeripheral

This variation omits files that not needed to implement BLE UART. It also omits the support for bonding and secure communications from Sandeep's library. It was also necessary to change the write buffer management system to accommodate the newer Nordic softdevice API's.

Both the original and this modified version are licensed under the MIT open source license.

Platformio Setup

I used PlatformIO to build and test this library, but I needed to make a few small changes to support the newer S113 and S140 softdevices. Those changes are documented in the Readme.txt file in the "PlatformIO Tweaks for Softdevices" folder.

Building

Once you made the changes to Platformio, you can build the test program in the "serial" folder with one of these commands:

	pio run -e microbit110 -t upload	// micro:bit v1
	pio run -e microbit113 -t upload	// micro:bit v2
	pio run -e microbit140 -t upload	// micro:bit v2

The build process will print some softdevice info that can help with debug the softdevice configuration:

*** Softdevice: s113
***   Path: /Users/johnmaloney/.platformio/packages/framework-arduinonordicnrf5/cores/nRF5/SDK/components/softdevice/s113/hex
***   Hex: s113_nrf52_7.3.0_softdevice.hex
***   MCU Family: nrf52833_xxaa.ld
***   Loader: armgcc_s113_nrf52833_xxaa.ld
***   Defs: ['PLATFORMIO', 60110, 'ARDUINO_BBC_MICROBIT_V2', 'NRF52833_XXAA', 'NRF52_SERIES', 'S113', 'ARDUINO', 10805, 'F_CPU', '16000000L', 'ARDUINO_ARCH_NRF5', 'NRF5', 'NRF52']

This info can reveal issues such as a missing softdevice.hex or loader script file.

Testing

After installing the test program, open a terminal program on the microbit (at 115200 baud) and reset the micro:bit. You should see something like this:

  Starting BLE UART (serial2)...
  Enabled softdevice S110
  *** Started! ***

The test program is serial.ino in the "serial" folder. You can edit it to run different tests. By default, it sends 2000 bytes of data as fast as it and measures the time and throughput. It repeats that test every second.

But it won't do anything until a connection is made. You can do that by running the uartTest.py program with Python3:

	python3 uartTest.py

After of few seconds of connecting, you'll start seeing lines of characters appear in bursts:

  @ABCDEFGHIJKLMNOPQRS
  @ABCDEFGHIJKLMNOPQRS
  @ABCDEFGHIJKLMNOPQRS
  ...

Meanwhile, the terminal will show the time and throughput for each burst.

The test program can also be set up to receive data and you can then type or paste text into the Python test program.

You can also use BLE test programs such Nordic's "nrf Toobox" or Adafruit's "Bluefruit Connect" for testing, I've found the Python program the most useful tool for performance testing.

	