Using Nordic Softdevices with PlatformIO and the micro:bit
September, 2023

The PlatformIO nordicnrf5 framework needs a few tweaks to use Nordic Softdevices with the micro:bit v1 and v2. These instructions are specifically for the S110 (micro:bit v1), S113, and S140 (micro:bit v2).

Prerequisites:
	- PlatformIO command line tools are installed
	- the nordicnrf5 framework is installed (installed automatically when you first compile a project for the micro:bit)

1. Update the nrf5.py script

Locate the PlatformIO frameworks folder. On MacOS and Linux it's the "~/.platformio" folder in the user's home directory. On Windows, it is "C:\Users\<username>\.platformio".

Copy the file "nrf5.py" into two locations:

	.platformio/platforms/nordicnrf51/builder/frameworks/arduino/nrf5.py
	.platformio/platforms/nordicnrf52/builder/frameworks/arduino/nrf5.py

These are the builder folders for the micro:bit v1 and v2. The same nrf5.py script is used in both places.

The updated nrf5.py script is needed because the original script in Sandeep's framework does not include the more recent softdevices. The new script also prints out useful information about the softdevice and loader file during the build process that can help you debug build problems if a file is missing or not named correctly.

2. Add desired Softdevice S113 and/or S140

The nordicnrf5 framework includes the S110 softdevice but not the newer S113 or S140 soft devices required for the micro:bit v2.

Google to find the desire softdevice download (e.g. "Nordic softdevice S113 download").
Download and unzip.

Create folders "s113" and/or "s140" in:

	.platformio/packages/framework-arduinonordicnrf5/cores/nRF5/SDK/components/softdevice

Copy files from the downloaded softdevice into the corresponding folders.

Each softdevice folder must have a structure like this (using s113 as an example):
 
s113
	doc
	headers
		<.h files for that softdevice's API>
	hex	
		s113_nrf52_7.2.0_softdevice.hex
	toolchain
		armgcc
			armgcc_s113_nrf52833_xxaa.ld

You can copy the unpopulated folder "sXXX" as a template for building a new softdevice folder.

The Nordic folder includes some additional folders and files including API documentation, release notes, and license files. I usually put all of that stuff into the doc folder but it is not required for building.

3. Copy the linker scripts from this folder into the toolchain/armgcc of the corresponding softdevice folder in:

  .platformio/packages/framework-arduinonordicnrf5/cores/nRF5/SDK/components/softdevice

4. Build!

To use a softdevice, add set the build_flags line in the platformio.ini file.

micro:bit v1:

		build_flags = -Wno-unused-function -D S110

micro:bit v2, one of:

	build_flags = -UNRF52 -D NRF52_SERIES -D S113
	build_flags = -UNRF52 -D NRF52_SERIES -D S140

The flag combination "-UNRF52 -D NRF52_SERIES" is needed to select correct processor type for micro:bit v2 (i.e. nRF52833).

The builder prints information about the softdevice and linker files it is using so you can troubleshoot if your linker file is not in the right place or has the wrong name.

That should do it!

----------------------------

NOTE 1: The softdevices.txt file is not relevant in the context of PlatformIO. No need to touch that.

NOTE 2: Softdevice RAM Requirements

The supplied linker scripts should work with the current softdevices. However, the following information may be helpful when updating to a newer softdevice version or adding a new softdevice.

There used to be a fixed amount of RAM reserved for each softdevice, and Nordic provided a linker script that set the application memory start accordingly.

However, in more recent version of the softdevice API's, the softdevice is dynamically configured and may require differing amounts of RAM. When the softdevice is started, it may give an error it doesn't have enough RAM. In case that happens, I've added code to print out a message, including instructions about how to edit the linker script (.ld file) to reserve the necessary amount of RAM.

If you open a terminal on the USB serial port (115k baud) for the board, you'll see that or any other error messages appear when the micro:bit tries to enable the softdevice.

If necessary, you can edit the linker script to increase the RAM.

NOTE 3: PlatformIO caches object files! Thus, after updating a linker script you'll need to force recompilation by deleting the build folder in the hidden .pio folder in your working directory:

	rm -rf .pio/build
