/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

// Copyright 2018 John Maloney, Bernat Romagosa, and Jens Mönig

#include "mem.h"
#include "interp.h"
#include "persist.h"

void setup() {
	hardwareInit();
	memInit();
	primsInit();
	outputString((char *) "Welcome to Hatchling MicroBlocks!"); // Change and you'll see in the browser console
	restoreScripts();
	startAll(); // Starts all hat blocks

  // Makes sure the reset pin stays low (briefly high to allow a reset)
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delayMicroseconds(50000);
  delayMicroseconds(50000);
  delayMicroseconds(50000);
  delayMicroseconds(50000);
  digitalWrite(2, LOW);

  // May need to add 1850 ms delay for bootloader here if we put the SAMD bootloader on the board
}

void loop() {
	vmLoop(); // This is an infinite loop
}
