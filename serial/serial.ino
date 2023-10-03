// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

/*
 * Serial Port over BLE
 * Create UART service compatible with Nordic's *nRF Toolbox* and Adafruit's *Bluefruit LE* iOS/Android apps.
 *
 * BLESerial class implements same protocols as Arduino's built-in Serial class and can be used as it's wireless
 * replacement. Data transfers are routed through a BLE service with TX and RX characteristics. To make the
 * service discoverable all UUIDs are NUS (Nordic UART Service) compatible.
 *
 * Please note that TX and RX characteristics use Notify and WriteWithoutResponse, so there's no guarantee
 * that the data will make it to the other end. However, under normal circumstances and reasonable signal
 * strengths everything works well.
 */

#include <Arduino.h>
#include "BLESerial.h"

BLESerial bleSerial;

void setup() {
  // custom services and characteristics can be added as well

  Serial.begin(115200);
  Serial.println("Starting BLE UART (serial2)...");

  bleSerial.setLocalName("John's UART");
  bleSerial.begin();
  Serial.println("*** Started! ***");
}

void loop() {
//	readData();
	sendTest3();
//	loopback();
//	forward();
//	spam();
}

unsigned int startTime;
int totalBytes = 0;

void readData() {
	int doPrint = false;
	while (bleSerial.available()) {
		int ch = bleSerial.read();
		if (ch == 'A') startTime = millis();
		if (ch == 'Z') doPrint = true;
		totalBytes++;
	}
	if (doPrint) {
		int msecs = millis() - startTime;
		Serial.print("Got ");
		Serial.print(totalBytes);
		Serial.print(" bytes in ");
		Serial.print(msecs);
		Serial.print(" msecs (");
		Serial.print((1000 * totalBytes) / msecs);
		Serial.println(" bytes/sec)");
		totalBytes = 0;

		Serial.print(bleSerial.rcvCount);
		Serial.println(" buffers");
		bleSerial.rcvCount = 0;
	}
}

// forward received from Serial to BLESerial and vice versa
void forward() {
  if (bleSerial && Serial) {
    int byte;
    while ((byte = bleSerial.read()) > 0) Serial.write((char)byte);
    while ((byte = Serial.read()) > 0) bleSerial.write((char)byte);
  }
}

// echo all received data back
void loopback() {
  if (bleSerial) {
    int byte;
    while ((byte = bleSerial.read()) > 0) bleSerial.write(byte);
  }
}

// periodically sent time stamps
static uint32_t nextSendTime = 0;
static int counter = 0;

void spam() {
  uint32_t msecs = millis();
//  bleSerial.poll();
  if ((msecs >= nextSendTime) && (bleSerial.availableForWrite() > 12)) {
    bleSerial.print("counter: ");
    bleSerial.print(counter);
    counter = (counter + 1) % 100;
    nextSendTime = msecs + 1; // 7 works
    bleSerial.flush();
  }
}

void sendTest() {
  if (!bleSerial.connected()) return;

  int repeatCount = 10;
  uint32_t startMSecs = millis();
  for (int iters = 0; iters < repeatCount; iters++) {
    int sentCount = 0;
    while (sentCount < 100) {
//      if (bleSerial.availableForWrite() > 12) {
        bleSerial.print("counter: ");
        bleSerial.print(100 + sentCount);
//        bleSerial.flush();
        sentCount++;
//      }
    }
  }
  uint32_t msecs = millis() - startMSecs;
  int totalBytes = repeatCount * 100 * 12;
  Serial.print("Sent "); Serial.print(totalBytes);
  Serial.print(" in "); Serial.print(msecs); Serial.print(" msecs; ");
  Serial.print((1000.0 * totalBytes) / msecs); Serial.println(" bytes/sec");
  delay(1000);
}

void sendTest2() {
  if (!bleSerial.connected()) return;

  int iterCount = 100;
  bleSerial.flush();
  uint32_t startMSecs = millis();
  for (int iters = 0; iters < iterCount; iters++) {
    for (int ch = 65; ch < 85; ch++) {
      bleSerial.write(ch);
    }
  }

  uint32_t msecs = millis() - startMSecs;
  int totalBytes = iterCount * 20;
  Serial.print("Sent "); Serial.print(totalBytes);
  Serial.print(" in "); Serial.print(msecs); Serial.print(" msecs; ");
  Serial.print((1000.0 * totalBytes) / msecs); Serial.println(" bytes/sec");
  delay(1000);
}

void sendTest3() {
  if (!bleSerial.connected()) return;

  int iterCount = 100;
  bleSerial.flush();
  uint32_t startMSecs = millis();
  for (int iters = 0; iters < iterCount; iters++) {
    while (bleSerial.availableForWrite() < 20) /* wait for a space */;
    bleSerial.sendTest(); // send a 20 byte packet
  }

  uint32_t msecs = millis() - startMSecs;
  int totalBytes = iterCount * 20;
  Serial.print("Sent "); Serial.print(totalBytes);
  Serial.print(" in "); Serial.print(msecs); Serial.print(" msecs; ");
  Serial.print((1000.0 * totalBytes) / msecs); Serial.println(" bytes/sec");
  delay(1000);
}

