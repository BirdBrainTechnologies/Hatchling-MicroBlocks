# README #

## Hatchling and MicroBlocks ##

This repository contains the source code for the Hatchling-MicroBlocks project.

Hatchling is a product from BirdBrain Technologies <https://www.birdbraintechnologies.com> that allows kids as young as five to build and code creative robotics projects. The product uses a modified version of the MicroBlocks virtual machine to enable a special combination of *live* development and *autonomous operation*. Live development allows the user to test program changes instantly, without restarting or waiting for the program to compile and download. Autonomous operation means that programs continue to run when the board is untethered from the host computer.

Although this version of the code is specific to using the MicroBlocks virtual machine on a micro:bit connected to a Hatchling Nest Controller, the broader MicroBlocks project enables live & autonomous coding on a large mix of microcontrollers. To learn more about MicroBlocks visit the MicroBlocks website, <http://microblocks.fun>

## MicroBlocks Virtual Machine Details ##

The MicroBlock firmware (or *virtual machine*) runs on 32-bit embedded processors
with as little as 16k of RAM. It is intended to be simple,
easily ported and extended, and offer decent performance.
It includes a low-latency task scheduler that works at timescales down to ~50 microseconds
and a garbage collected memory that allows working with dynamic lists and strings – within
the limits of the available RAM, of course!

Because MicroBlocks programs run directly on the microcontroller -- in contrast to tethered systems where
the program runs on a laptop and uses the microcontroller as an I/O device -- programs can work precisely at timescales down to 10's of microseconds.

The Hatchling Project extends the MicroBlocks Virtual Machine to automatically detect components plugged into a Hatchling Nest Controller, provide status updates on what is connected to the programming apps provided with Hatchling, and communicate via SPI with the Hatchling Nest Controller's microcontroller. 

## How do I build the Hatchling-MicroBlocks firmware? ##

The Hatchling-MicroBlocks firmware, or **virtual machine**, is written in C and C++.
It is built on the Arduino platform and uses additional Arduino libraries for
features such as graphics and WiFi on boards that support those features.

### Building with PlatformIO ###

PlatformIO is the preferred build tool. PlatformIO is a cross platform build system
that is easy to install an use. Its only dependency is Python, which comes pre-installed
on MacOS and Linux and is easy to install on Windows.
You can get the PlatformIO command line interface (CLI) tools
[here](https://platformio.org/install/cli).

To compile and install the VM for a micro:bit,
plug in the board and run the following in this directory:

    pio run -e microbitV2-ble -t upload

Look at platformio.ini to see which boards are supported and what they are called.

### Building with the Arduino IDE ###

The MicroBlocks virtual machine can also be compiled and loaded onto a board using the
Arduino IDE (version 1.8 or later) with the appropriate board package and libraries
installed.

To build with the Arduino IDE, open the file *vm.ino*, select your board from the
boards manager, then click the upload button.

## Team ##

This project was developed by Tom Lauwers and Krissie Lauwers of BirdBrain Technologies, with extensive support from the MicroBlocks team (John Maloney, Bernat Romagosa, and Jens Mönig, among others).

## License ##

Hatchling-MicroBlocks is licensed under the Mozilla Public License 2.0 (MPL 2.0).
