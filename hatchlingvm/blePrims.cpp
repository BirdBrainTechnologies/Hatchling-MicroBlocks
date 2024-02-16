/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

// Copyright 2023 John Maloney, Bernat Romagosa, and Jens Mönig

// blePrims.cpp - MicroBlocks Bluetooth Low Energy primitives
// Wenjie Wu, December 2023

#include <stdio.h>
#include <stdlib.h>
#include "mem.h"
#include "interp.h" // must be included *after* ESP8266WiFi.h

#if defined(BLE_IDE) //Octo primtives; included in standard BLE release

#include <NimBLEDevice.h>

NimBLEUUID ANDROID_OCTO_UUID	= NimBLEUUID("2540b6b0-0001-4538-bcd7-7ecfb51297c1");
NimBLEUUID iOS_OCTO_UUID		= NimBLEUUID("2540b6b0-0002-4538-bcd7-7ecfb51297c1");

static BLEScan* pOctoScanner = NULL;
static BLEAdvertising* pAdvertising = NULL;

static bool bleScannerRunning = false;
static bool hasOctoMessage = false;
static int shape_id = 0;

// record last scan payload
#define MAX_SCAN_PAYLOAD 100
static int lastScanPayloadLen = 0;
static uint8 *lastScanPayload[MAX_SCAN_PAYLOAD];
static uint8 lastScanRSSI = 0;
static uint8 lastScanAddressType = 0;
static uint8 lastScanAddress[6];

// octoIDHistory is an array of recently seen Octo ID's used for duplicate suppression.
// Its size must be a power of 2. Searching for an ID starts at searchStartIndex.
// When an ID is not found, searchStartIndex is decremented (mod the array size)
// and the new ID is added at that index. Thus, the most recently added ID
// is at searchStartIndex, the second most recent at searchStartIndex+1, etc.
// The first time an ID is seen, the entire history must be searched but after
// that, the ID will be found in one or, at most, a few steps. The common case
// is testing for an ID that has already been seen since OctoStudio often sends
// the same message 50-60 times.

typedef long long unsigned int octoMsgID;
octoMsgID allZeroMessageID;

#define OCTO_ID_HISTORY_SIZE 32
octoMsgID octoIDHistory[OCTO_ID_HISTORY_SIZE];
int searchStartIndex = 0;

static int octoIDNotYetSeen(octoMsgID id) {
int steps = 0;
	int endIndex = (searchStartIndex - 1) & (OCTO_ID_HISTORY_SIZE - 1);
	for (int i = searchStartIndex; i != endIndex; i = ((i + 1) % OCTO_ID_HISTORY_SIZE)) {
		if (octoIDHistory[i] == id) return false;
		steps++;
	}
	return true;
}

static void addIDToOctoHistory(octoMsgID id) {
	searchStartIndex = (searchStartIndex - 1) & (OCTO_ID_HISTORY_SIZE - 1);
	octoIDHistory[searchStartIndex] = id;
}

class BLEScannerCallbacks : public BLEAdvertisedDeviceCallbacks {
	void onResult(BLEAdvertisedDevice* advertisedDevice) {
		if (advertisedDevice->haveServiceUUID()) {
			// iOS
			BLEUUID uuid = advertisedDevice->getServiceUUID();
			if (iOS_OCTO_UUID.equals(uuid)) {
				std::string deviceName = advertisedDevice->getName();
				if (deviceName.length() == 16) {
					octoMsgID id;
					memcpy(&id, deviceName.c_str(), 8);
					if ((id != allZeroMessageID) && octoIDNotYetSeen(id)) {
						addIDToOctoHistory(id);
						shape_id = deviceName.back() - '0';
						if (shape_id < 0) shape_id = 255; // ensure shape_id is positive
						hasOctoMessage = true;
					}
				}
			}
		} else if (advertisedDevice->haveServiceData()) {
			// Android
			BLEUUID uuid = advertisedDevice->getServiceDataUUID();
			if (ANDROID_OCTO_UUID.equals(uuid)) {
				std::string serviceData = advertisedDevice->getServiceData();
				if (serviceData.length() == 13) {
					octoMsgID id;
					memcpy(&id, serviceData.c_str(), 8);
					if (octoIDNotYetSeen(id)) {
						addIDToOctoHistory(id);
						shape_id = serviceData[7];
						hasOctoMessage = true;
					}
				}
			}
		}

		if (lastScanPayloadLen != 0) return; // last capture has not been consumed

		// capture scan payload
		lastScanPayloadLen = advertisedDevice->getPayloadLength();
		if (lastScanPayloadLen > MAX_SCAN_PAYLOAD) lastScanPayloadLen = MAX_SCAN_PAYLOAD;
		memcpy(lastScanPayload, advertisedDevice->getPayload(), lastScanPayloadLen);

		// capture RSSI and address
		lastScanRSSI = -advertisedDevice->getRSSI(); // make it positive
		NimBLEAddress addr = advertisedDevice->getAddress();
		lastScanAddressType = addr.getType();
		memcpy(lastScanAddress, addr.getNative(), 6);
	}
};

static void scanComplete(BLEScanResults scanResults) {
	// Restarts the scanner so that we scan continuously.

	pOctoScanner->clearResults();
	pOctoScanner->start(1, scanComplete, false);
}

static void startBLEScanner() {
	if (!bleScannerRunning) {
		// initialize allZeroMessageID; ignore messages with that ID sent by iOS OctoStudio
		memcpy(&allZeroMessageID, "00000000", 8);

		pOctoScanner = BLEDevice::getScan();
		pOctoScanner->setAdvertisedDeviceCallbacks(new BLEScannerCallbacks());
		pOctoScanner->setMaxResults(0); // don't save results; use callback only
		pOctoScanner->setActiveScan(true); // required by Octo
		pOctoScanner->setDuplicateFilter(false); // good ???
		pOctoScanner->start(1, scanComplete, false);
		bleScannerRunning = true;
	}
}

static OBJ primOctoStartBeam(int argCount, OBJ *args) {
	if ((argCount < 1) || !IS_TYPE(args[0], StringType)) return falseObj;

	char *msg = obj2str(args[0]);

	// Mimic iOS beam; data is encoded in name
	BLE_pauseAdvertising();
	pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->reset();
	pAdvertising->addServiceUUID(iOS_OCTO_UUID);
	pAdvertising->setName(msg);
	pAdvertising->setMinInterval(32);
	pAdvertising->setMaxInterval(32);
	pAdvertising->start();
	return falseObj;
}

static OBJ primOctoStopBeam(int argCount, OBJ *args) {
	if (!pAdvertising) return falseObj; // not initialized thus not beaming

	BLEDevice::getAdvertising()->removeServiceUUID(iOS_OCTO_UUID);
	BLE_resumeAdvertising();
	return falseObj;
}

static OBJ primOctoReceive(int argCount, OBJ *args) {
	if (!bleScannerRunning) startBLEScanner();

	if (hasOctoMessage) {
		hasOctoMessage = false;
		return int2obj(shape_id);
	} else {
		return falseObj;
	}
}

static OBJ primScanReceive(int argCount, OBJ *args) {
	if (!bleScannerRunning) startBLEScanner();
	if (!lastScanPayloadLen) return falseObj; // no data

	int payloadLen = lastScanPayloadLen;
	int byteCount = payloadLen + 8;
	int wordCount = (byteCount + 3) / 4;
	OBJ result = newObj(ByteArrayType, wordCount, falseObj);
	if (!result) return fail(insufficientMemoryError);
	setByteCountAdjust(result, byteCount);

	// Result format is:
	//	absolute value of RSSI (one byte)
	//	address type (one byte)
	//	address (six bytes)
	//	payload (remaining bytes)
	uint8 *byteArray = (uint8 *) &FIELD(result, 0);
	byteArray[0] = lastScanRSSI;
	byteArray[1] = lastScanAddressType;
	memcpy(byteArray + 2, lastScanAddress, 6);
	memcpy(byteArray + 8, lastScanPayload, payloadLen);
	lastScanPayloadLen = 0;

	return result;
}

#endif // Octo primitives

static PrimEntry entries[] = {
	#if defined(BLE_IDE)
		{"octoStartBeam", primOctoStartBeam},
		{"octoStopBeam", primOctoStopBeam},
		{"octoReceive", primOctoReceive},
		{"scanReceive", primScanReceive},
	#endif
};

void addBLEPrims() {
	addPrimitiveSet("ble", sizeof(entries) / sizeof(PrimEntry), entries);
}
