/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

// Copyright 2018 John Maloney, Bernat Romagosa, and Jens Mönig

// ideComm.cpp - Primitives to communicate with the MicroBlocks IDE.
// John Maloney, December 2023

#include <Arduino.h>

#include "mem.h"
#include "interp.h"
#include "persist.h"

int BLE_connected_to_IDE = false;
char BLE_ThreeLetterID[4];

#if defined(BLE_IDE)

// BLE Communications

#include <NimBLEDevice.h>

// BLE_SEND_MAX - maximum bytes to send in a single attribute write (max is 512)
// INTER_SEND_TIME - don't send data more often than this to avoid NimBLE error & disconnect
#define BLE_SEND_MAX 250
#define INTER_SEND_TIME 20

static BLEServer *pServer = NULL;
static BLEService *pService = NULL;
static BLECharacteristic *pTxCharacteristic;
static BLECharacteristic *pRxCharacteristic;
static char uniqueName[32];
static bool bleRunning = false;
static bool serviceOnline = false;
static uint16_t connID = -1;

static uint32 lastSendTime = 0;
static int lastRC = 0;

#define RECV_BUF_MAX 2048 // Hatchling change from 1024
static uint8_t bleRecvBuf[RECV_BUF_MAX];
static int bleBytesAvailable = 0;
static int overRuns = 0;

// MicroBlocks UUIDs:
#define SERVICE_UUID			"bb37a001-b922-4018-8e74-e14824b3a638" // MicroBlocks IDE service
#define CHARACTERISTIC_UUID_RX	"bb37a002-b922-4018-8e74-e14824b3a638"
#define CHARACTERISTIC_UUID_TX	"bb37a003-b922-4018-8e74-e14824b3a638"

/************************************************************************
Rude words to be removed
************************************************************************/
static const uint8_t first_letter[] = {
	'A',	'A',	'A',	'A',	'B',	'C',	'C',	'C',	'C',	'C',	'C',	'C',	'C',	'C',	'D',	'D',
	'D',	'D',	'D',	'D',	'D',	'D',	'D',	'D',	'F',	'F',	'F',	'F',	'F',	'F',	'F',	'F',
	'F',	'F',	'F',	'F',	'F',	'G',	'G',	'G',	'G',	'G',	'G',	'G',	'G',	'G',	'G',	'H',
	'J',	'J',	'J',	'J',	'J',	'J',	'K',	'K',	'K',	'K',	'K',	'K',	'K',	'K',	'K',	'K',
	'K',	'K',	'K',	'K',	'K',	'L',	'L',	'L',	'L',	'L',	'L',	'M',	'M',	'M',	'M',	'M',
	'M',	'M',	'M',	'M',	'N',	'N',	'N',	'N',	'N',	'N',	'N',	'O',	'O',	'P',	'P',	'P',
	'P',	'P',	'P',	'P',	'P',	'P',	'P',	'P',	'P',	'P',	'P',	'P',	'P',	'R',	'R',	'R',
	'R',	'S',	'S',	'S',	'S',	'S',	'S',	'S',	'S',	'S',	'S',	'S',	'S',	'S',	'S',	'S',
	'S',	'S',	'S',	'S',	'S',	'T',	'T',	'T',	'T',	'T',	'T',	'V',	'V',	'V',	'V',	'W',
	'W',	'W',	'W',	'W',	'W',	'X',	'X',	'A',	'A'
	};
	
	/************************************************************************/
	
	static const uint8_t second_letter[] = {
	'N',	'N',	'S',	'Z',	'C',	'A',	'A',	'A',	'L',	'N',	'O',	'O',	'O',	'U',	'C',	'I',
	'I',	'I',	'I',	'M',	'S',	'Y',	'Y',	'Y',	'A',	'A',	'C',	'C',	'G',	'K',	'O',	'Q',
	'T',	'U',	'U',	'U',	'U',	'A',	'A',	'E',	'E',	'I',	'U',	'U',	'U',	'V',	'Z',	'O',
	'A',	'E',	'I',	'O',	'Y',	'Z',	'A',	'A',	'A',	'I',	'K',	'L',	'N',	'O',	'O',	'O',
	'O',	'U',	'Y',	'Y',	'Y',	'C',	'I',	'I',	'I',	'O',	'S',	'F',	'I',	'I',	'I',	'L',
	'U',	'Y',	'Y',	'Y',	'A',	'D',	'D',	'G',	'I',	'U',	'W',	'R',	'P',	'C',	'H',	'H',
	'H',	'I',	'M',	'N',	'O',	'O',	'R',	'R',	'R',	'R',	'S',	'S',	'U',	'A',	'A',	'A',
	'C',	'A',	'A',	'A',	'C',	'E',	'F',	'H',	'J',	'L',	'N',	'O',	'O',	'T',	'U',	'U',
	'U',	'X',	'X',	'X',	'X',	'H',	'I',	'O',	'O',	'O',	'W',	'A',	'A',	'G',	'J',	'A',
	'A',	'A',	'C',	'O',	'T',	'T',	'X',	'Z',	'Z'
	};
	
	
	/************************************************************************/
	
	static const uint8_t third_letter[] = {
	'L',	'S',	'S',	'N',	'H',	'C',	'K',	'Q',	'T',	'T',	'C',	'K',	'Q',	'M',	'K',	'C',
	'K',	'Q',	'X',	'N',	'H',	'C',	'K',	'Q',	'G',	'P',	'K',	'U',	'T',	'U',	'B',	'U',
	'P',	'C',	'K',	'Q',	'X',	'I',	'Y',	'I',	'Y',	'Z',	'C',	'K',	'Q',	'R',	'Z',	'R',
	'P',	'W',	'Z',	'O',	'Z',	'Z',	'C',	'K',	'Q',	'K',	'K',	'T',	'T',	'C',	'K',	'Q',
	'X',	'M',	'C',	'K',	'Q',	'K',	'C',	'K',	'Q',	'L',	'D',	'F',	'C',	'K',	'Q',	'F',
	'F',	'C',	'K',	'Q',	'D',	'S',	'Z',	'R',	'G',	'T',	'A',	'L',	'P',	'P',	'C',	'K',
	'Q',	'S',	'S',	'S',	'O',	'T',	'C',	'K',	'N',	'Q',	'S',	'Y',	'S',	'C',	'K',	'Q',
	'K',	'C',	'K',	'Q',	'K',	'X',	'U',	'T',	'V',	'T',	'M',	'B',	'L',	'D',	'C',	'K',
	'Q',	'E',	'I',	'X',	'Y',	'C',	'T',	'C',	'K',	'Q',	'T',	'G',	'J',	'N',	'N',	'C',
	'K',	'Q',	'K',	'P',	'F',	'C',	'X',	'S',	'Z'
	};


// BLE Helper Functions
void getMACAddress(uint8 *sixBytes) {
	// Store up to six bytes of unique chip ID into the six-byte argument array.
	uint32 deviceID = NRF_FICR->DEVICEID[0];
	sixBytes[5] = deviceID & 255;
	sixBytes[4] = (deviceID >> 8) & 255;
	sixBytes[3] = (deviceID >> 16) & 255;
	sixBytes[2] = (deviceID >> 24) & 255;
	deviceID = NRF_FICR->DEVICEID[1];
	sixBytes[1] = deviceID & 255;
	sixBytes[0] = (deviceID >> 8) & 255;
}

// Checks if the name could be misconstrued
bool rude_word_check()
{
	for(unsigned int i=0;i<sizeof(first_letter);i++)
	{
		if(BLE_ThreeLetterID[0] == first_letter[i] )
		{
			if(BLE_ThreeLetterID[1] == second_letter[i])
			{
				if(BLE_ThreeLetterID[2] == third_letter[i] )
				{
					return true;
				}
			}
		}
	}
	return false;
}

void BLE_initThreeLetterID() {
	unsigned char mac[6] = {0, 0, 0, 0, 0, 0};
	getMACAddress(mac);
	int machineNum = (mac[4] << 8) | mac[5]; // 16 least signifcant bits

	BLE_ThreeLetterID[0] = 65 + (machineNum % 26);
	machineNum = machineNum / 26;
	BLE_ThreeLetterID[1] = 65 + (machineNum % 26);
	machineNum = machineNum / 26;
	BLE_ThreeLetterID[2] = 65 + (machineNum % 26);
	BLE_ThreeLetterID[3] = 0;

	// Change the middle letter to a 'B' because there are no rude words with a B in the middle
	if(rude_word_check())
		BLE_ThreeLetterID[1] = 'B';

	sprintf(uniqueName, "%s", BLE_ThreeLetterID);
}

static void displayFor(int msecs) {
	uint32 endMSecs = millisecs() + msecs;
	while (millisecs() < endMSecs) {
		captureIncomingBytes();
		updateMicrobitDisplay();
		delay(1);
	}
}

static void show_BLE_ID() {
	OBJ args[5]; // used to call primitives

	int nameLen = strlen(uniqueName);
	// Display name three times

	for(int j = 0; j < 3; j++)
	{
		for (int i = nameLen - 4; i < nameLen; i++) {
			args[0] = newStringFromBytes(&uniqueName[i], 1);
			OBJ letterShape = primMBShapeForLetter(1, args);

			args[0] = letterShape;
			args[1] = int2obj(1);
			args[2] = int2obj(1);
			primMBDrawShape(3, args);
			displayFor(700);
			primMBDisplayOff(0, args);
			displayFor(100);
			// If we've connected, return from the function
			if(isBLEConnected())
				return;
		}
		displayFor(1000);
		// If we've connected, return from the function
		if(isBLEConnected())
			return;
	}
	primMBDisplayOff(0, args);
}

static int gotSerialPing() {
	char buf[20];
	int byteCount = Serial.available();
	if (byteCount < 3) return false;
	delay(5); // wait for a few more bytes
	byteCount = Serial.available();
	if (byteCount > (int) sizeof(buf)) byteCount = sizeof(buf);
	byteCount = Serial.readBytes((char *) buf, byteCount);
	for (int i = 0; i < byteCount - 2; i++) {
		if ((buf[i] == 0xFA) && (buf[i+1] == 0x1A) && (buf[i+2] == 0)) {
			return true; // receive ping message from IDE
		}
	}
	return false;
}

static void updateConnectionMode() {
	if (BLE_connected_to_IDE) {
		if (gotSerialPing()) {
			// new serial connection; disconnect BLE
			if (connID != -1) { pServer->disconnect(connID); }
			connID = -1;
			pServer->removeService(pService);
			BLE_connected_to_IDE = false;
			serviceOnline = false;
			return;
		}
	} else {
		if (!serviceOnline && !ideConnected() && pServer) {
			// lost serial connection; restore service and advertising
			pServer->addService(pService);
			pServer->getAdvertising()->start();
			serviceOnline = true;
		}
	}
}

static void bleReceiveData(const uint8_t *data, int byteCount) {
	int available = RECV_BUF_MAX - bleBytesAvailable;
	if (byteCount > available) {
		overRuns++;
		byteCount = available;
	}

	memcpy(&bleRecvBuf[bleBytesAvailable], data, byteCount);
	bleBytesAvailable += byteCount;
}

static int bleSendData(uint8_t *data, int byteCount) {
	// do not send more often than INTER_SEND_TIME msecs
	uint32 now = millisecs();
	if (lastSendTime > now) lastSendTime = 0; // clock wrap
	if ((now - lastSendTime) < INTER_SEND_TIME) return 0;

	if (byteCount <= 0) return 0;

	// send byteCount bytes
	if (byteCount > BLE_SEND_MAX) byteCount = BLE_SEND_MAX;
	lastRC = 0; // will be set to non-zero if notify() call fails
	pTxCharacteristic->setValue(data, byteCount);
	pTxCharacteristic->indicate();
	if (lastRC != 0) {
		byteCount = 0; // write+notify failed; retry later
	}

	lastSendTime = now;
	return byteCount;
}

class MyServerCallbacks: public BLEServerCallbacks {
	void onConnect(BLEServer* pServer, ble_gap_conn_desc* desc) {
		pServer->getAdvertising()->stop(); // don't advertise while connected
		connID = desc->conn_handle;
		BLE_connected_to_IDE = true;
	}
	void onDisconnect(BLEServer* pServer) {
		pServer->getAdvertising()->start(); // restart advertising
		connID = -1;
		BLE_connected_to_IDE = false;
	}
};

class MyCallbacks: public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic, ble_gap_conn_desc* desc) {
		// Handle incoming BLE data.

		NimBLEAttValue value = pCharacteristic->getValue();
		bleReceiveData(value.data(), value.length());
	}
	void onStatus(NimBLECharacteristic* pCharacteristic, Status s, int code) {
		// Record the last return code. This is used to tell when a notify() has failed
		// (because there are no buffers) so that it can be re-tried later.

		lastRC = code;
	}
};

// Start/Stop BLE

void BLE_start() {
	if (bleRunning) return; // BLE already running

	// Create BLE Device
	BLE_initThreeLetterID();
	BLEDevice::init(uniqueName);

	// Create BLE Server
	pServer = BLEDevice::createServer();
	pServer->setCallbacks(new MyServerCallbacks());

	// Create BLE Service
	pService = pServer->createService(SERVICE_UUID);

	// Create BLE Characteristics
	pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, NIMBLE_PROPERTY::NOTIFY);
	pTxCharacteristic->setCallbacks(new MyCallbacks());
	pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, NIMBLE_PROPERTY::WRITE_NR);
	pRxCharacteristic->setCallbacks(new MyCallbacks());

	// Start the service
	pService->start();
	serviceOnline = true;
	bleRunning = true;

	BLE_resumeAdvertising();
	show_BLE_ID();
}

void BLE_stop() {
	if (!bleRunning) return; // BLE already stopped

	if (connID != -1) { pServer->disconnect(connID); }
	connID = -1;
	BLE_connected_to_IDE = false;
	serviceOnline = false;

	pServer->getAdvertising()->stop();
	pServer->removeService(pService);
	BLEDevice::deinit();

	pServer = NULL;
	pService = NULL;
	pTxCharacteristic = NULL;
	pRxCharacteristic = NULL;

	bleRunning = false;
}

// Stop and resume advertising (for use by Octo primitives)

void BLE_pauseAdvertising() {
	if (!pServer) return;
	pServer->getAdvertising()->stop();
	pServer->getAdvertising()->removeServiceUUID(NimBLEUUID(SERVICE_UUID));
}

void BLE_resumeAdvertising() {
	if (!pServer) return;

	NimBLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->reset();
	pAdvertising->addServiceUUID(SERVICE_UUID);
	pAdvertising->setName(uniqueName);
	pAdvertising->setMinInterval(100);
	pAdvertising->setMaxInterval(200);
	if (serviceOnline) pAdvertising->start();
}

// IDE receive and send

int recvBytes(uint8 *buf, int count) {
	int bytesRead;

	updateConnectionMode();

	if (!BLE_connected_to_IDE) { // no BLE connection; use Serial
		bytesRead = Serial.available();
		if (bytesRead > count) bytesRead = count; // there is only enough room for count bytes
		return Serial.readBytes((char *) buf, bytesRead);
	}

	// use BLE connection
	bytesRead = (count < bleBytesAvailable) ? count : bleBytesAvailable;
	if (bytesRead == 0) return 0;

	memcpy(buf, bleRecvBuf, bytesRead); // copy bytes to buf

	int remainingBytes = bleBytesAvailable - bytesRead;
	if (remainingBytes > 0) {
		// remove bytesRead bytes from bleRecvBuf
		memcpy(bleRecvBuf, &bleRecvBuf[bytesRead], remainingBytes);
	}
	bleBytesAvailable = remainingBytes;

	return bytesRead;
}

int sendBytes(uint8 *buf, int start, int end) {
	// Send bytes buf[start] through buf[end - 1] and return the number of bytes sent.

	if (!BLE_connected_to_IDE) { // no BLE connection; use Serial
		return Serial.write(&buf[start], end - start);
	}

	// use BLE connection
	return bleSendData(&buf[start], end - start);
}

// Hatchling add - way for VM loop to know if BLE is connected
// True if connected, false otherwise
int isBLEConnected()
{
	return BLE_connected_to_IDE;
}

#define BLE_DISABLED_FILE "/_BLE_DISABLED_"

void BLE_setEnabled(int enableFlag) {
	#if defined(ARDUINO_ARCH_ESP32) || defined(RP2040_PHILHOWER)
		// Disable BLE connections from IDE if BLE_DISABLED_FILE file exists.

		if (enableFlag) {
			deleteFile(BLE_DISABLED_FILE);
		} else {
			createFile(BLE_DISABLED_FILE);
		}
	#elif defined(NRF52)
		// xxx todo: use user settings registers or Flash page just before persistent code store
	#endif

	if (enableFlag) {
		BLE_start();
	} else {
		BLE_stop();
	}
}

int BLE_isEnabled() {
	#if defined(ARDUINO_ARCH_ESP32)
		return !fileExists(BLE_DISABLED_FILE);
	#elif defined(NRF52)
		// xxx todo: use user settings registers or Flash page just before persistent code store
		return true;
	#endif
	return false;
}

#else

// Serial Communications Only

int recvBytes(uint8 *buf, int count) {
	int bytesRead = Serial.available();
	if (bytesRead > count) bytesRead = count; // there is only enough room for count bytes
	return Serial.readBytes((char *) buf, bytesRead);
}

int sendBytes(uint8 *buf, int start, int end) {
	// Send bytes buf[start] through buf[end - 1] and return the number of bytes sent.

	return Serial.write(&buf[start], end - start);
}

// stubs for non-BLE:
void BLE_initThreeLetterID() { }
void BLE_start() { }
void BLE_stop() { }
void BLE_pauseAdvertising() { }
void BLE_resumeAdvertising() { }
void BLE_setEnabled(int enableFlag) { }
int BLE_isEnabled() {return false; }

#endif

void restartSerial() {
	// Needed to work around a micro:bit issue that Serial can lock up during Flash compaction.

	Serial.end();
	Serial.begin(115200);
}
