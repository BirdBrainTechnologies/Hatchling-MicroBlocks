/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

// Copyright 2018 John Maloney, Bernat Romagosa, and Jens Mönig

// persist.c - Persistent memory for code and variables
// John Maloney, December 2017

// Porting:
// Different boards have different Flash sizes and memory layouts. In addition,
// different processors require different code to modify their Flash memory.
//
// To add a new board, add a case to the #ifdef for that board and define the constants:
//
//		START - starting address of persistent memory
//		HALF_SPACE - size (in bytes) of each half-space; must be a multiple of Flash page size
//
// and implement the platform-specific Flash functions:
//
//		void flashErase(int *startAddr, int *endAddr)
//		void flashWriteData(int *dst, int wordCount, uint8 *src)
//		void flashWriteWord(int *addr, int value)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mem.h"
#include "interp.h"
#include "persist.h"

// flash operations for supported platforms

//#if defined(NRF51) || defined(NRF52) || defined(ARDUINO_NRF52_PRIMO)
 #if false // disabled for BLE
	#include "nrf.h" // nRF51 and nRF52

	// to check available FLASH, build with -Wl,-Map,output.map and make sure __etext < START
	#if defined(NRF51)
		// BBC micro:bit and Calliope: App: 0-96k; Persistent Mem: 96k-256k
		#define START (96 * 1024)
		#define HALF_SPACE (80 * 1024)
	#elif defined(ARDUINO_NRF52_PRIMO)
		// Primo: SoftDevice: 0-112k; App: 112k-210k; Persistent Mem: 210k-488k; Boot: 488k-512k
		#define START (210 * 1024)
		#define HALF_SPACE (100 * 1024)
	#elif defined(NRF52)
		// nrf52832: SoftDevice + app: 0-316k; Persistent Mem: 316-436k; User data: 436k-464k; Boot: 464k-512k
		#define START (316 * 1024)
		#define HALF_SPACE (60 * 1024)
	#endif

	static void flashErase(int *startAddr, int *endAddr) {
		uint32 pageSize = NRF_FICR->CODEPAGESIZE / 4; // page size in words
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een; // enable Flash erase
		while (startAddr < endAddr) {
			while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
			NRF_NVMC->ERASEPAGE = (int) startAddr;
			startAddr += pageSize;
		}
		NRF_NVMC->CONFIG = 0; // disable Flash erase
	}

	void flashWriteWord(int *addr, int value) {
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen; // enable Flash write
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		*addr = value;
		NRF_NVMC->CONFIG = 0; // disable Flash write
	}

	void flashWriteData(int *dst, int wordCount, uint8 *src) {
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen; // enable Flash write
		for ( ; wordCount > 0; wordCount--) {
			int n = *src++;
			n |= *src++ << 8;
			n |= *src++ << 16;
			n |= *src++ << 24;
			while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
			*dst++ = n;
		}
		NRF_NVMC->CONFIG = 0; // disable Flash write
	}

#else
	// Simulate Flash operations using a RAM code store; allows MicroBlocks to run in RAM
	// on platforms that do not support Flash-based persistent memory. On systems with
	// a file system, the RAM code store is stored in a file to provide persistence.

	#define RAM_CODE_STORE true
	#define HALF_SPACE (10 * 1024)
	#define START (&flash[0])

	static uint8 flash[HALF_SPACE] __attribute__ ((aligned (32))); // simulated Flash memory

	static void flashErase(int *startAddr, int *endAddr) {
		int *dst = (int *) startAddr;
		while (dst < endAddr) { *dst++ = -1; }
	}

	void flashWriteWord(int *addr, int value) {
		*addr = value;
	}

	void flashWriteData(int *dst, int wordCount, uint8 *src) {
		for ( ; wordCount > 0; wordCount--) {
			int n = *src++;
			n |= *src++ << 8;
			n |= *src++ << 16;
			n |= *src++ << 24;
			*dst++ = n;
		}
	}

#endif

// variables

// persistent memory half-space ranges:
static int *start0 = (int *) START;
static int *end0 = (int *) (START + HALF_SPACE);
static int *start1 = (int *) (START + HALF_SPACE);
static int *end1 = (int *) (START + (2 * HALF_SPACE));;

static int current;		// current half-space (0 or 1)
static int *freeStart;	// first free word

#ifdef USE_CODE_FILE
	static int suspendFileUpdates = false;	// suspend slow file updates when loading a project/library
#endif

// helper functions

static void clearHalfSpace(int halfSpace) {
	int *startAddr = (0 == halfSpace) ? start0 : start1;
	int *endAddr = (0 == halfSpace) ? end0 : end1;
	flashErase(startAddr, endAddr);
}

static int cycleCount(int halfSpace) {
	// Return the cycle count for the given half-space or zero if not initialized.
	// Details: Each half-space begins with a word of the form <'S'><cycle count (24 bits)>.
	// The cycle count is incremented each time persistent memory is compacted.

	int *p = (0 == halfSpace) ? start0 : start1;
	return ('S' == ((*p >> 24) & 0xFF)) ? (*p & 0xFFFFFF) : 0;
}

static void setCycleCount(int halfSpace, int cycleCount) {
	// Store the given cycle count at the given address.

	int *p = (0 == halfSpace) ? start0 : start1;
	flashWriteWord(p, ('S' << 24) | (cycleCount & 0xFFFFFF));
}

static void initPersistentMemory() {
	// Figure out which is the current half-space and find freeStart.
	// If neither half-space has a valid cycle counter, initialize persistent memory.

	#ifdef RAM_CODE_STORE
		// Use a single persistent memory; HALF_SPACE is the total amount of RAM to use
		// Make starts and ends the same to allow the same code to work for either RAM or Flash
		start0 = start1 = (int *) START;
		end0 = end1 = (int *) (START + HALF_SPACE);
	#endif

	int c0 = cycleCount(0);
	int c1 = cycleCount(1);

	if (!c0 && !c1) { // neither half-space has a valid counter
		// Flash hasn't been used for uBlocks yet; erase it all.
		flashErase(start0, end1);
		setCycleCount(0, 1);
		current = 0;
		freeStart = start0 + 1;
		return;
	}

	int *end;
	if (c0 > c1) {
		current = 0;
		freeStart = start0 + 1;
		end = end0;
	} else {
		current = 1;
		freeStart = start1 + 1;
		end = end1;
	}

	while ((freeStart < end) && (-1 != *freeStart)) {
		int header = *freeStart;
		if ('R' != ((header >> 24) & 0xFF)) {
			outputString("Bad record found during initialization");
			clearPersistentMemory();
			return;
		}
		freeStart += *(freeStart + 1) + 2; // increment by the record length plus 2-word header
	}
	if (freeStart >= end) freeStart = end;
}

int * recordAfter(int *lastRecord) {
	// Return a pointer to the record following the given record, or NULL if there are
	// no more records. Pass NULL to get the first record.

	int *start, *end;
	if (0 == current) {
		start = start0;
		end = end0;
	} else {
		start = start1;
		end = end1;
	}
	int *p = lastRecord;
	if (NULL == lastRecord) { // return the first record
		p = (start + 1);
		return ('R' == ((*p >> 24) & 0xFF)) ? p : NULL;
	}
	if ((p >= end) || ('R' != ((*p >> 24) & 0xFF))) return NULL; // should not happen
	p += *(p + 1) + 2; // increment by the record length plus 2-word header
	if ((p >= end) || 'R' != ((*p >> 24) & 0xFF)) return NULL; // bad header; probably start of free space
	return p;
}

void outputRecordHeaders() {
	// For debugging. Output all the record headers of the current half-space.

	int recordCount = 0;
	int wordCount = 0;
	int maxID = 0;

	char s[200];
	int *p = recordAfter(NULL);
	while (p) {
		recordCount++;
		wordCount += 2 + *(p + 1);
		int id = (*p >> 8) & 0xFF;
		if (id > maxID) maxID = id;
		sprintf(s, "%d %d %d (%d words)",
			(*p >> 16) & 0xFF, (*p >> 8) & 0xFF, *p & 0xFF, *(p + 1));
		outputString(s);

// xxx debug: dump contents
// int chunkWords = *(p + 1);
// sprintf(s, "\t%x (%d words)", *p, chunkWords);
// outputString(s);
// int *w = p + 2;
// for (int i = 0; i < chunkWords; i++) {
// 	int word = *w;
// 	sprintf(s, "\t\t%d: %d %d %d %d ", i,
// 	(word & 255), ((word >> 8) & 255), ((word >> 16) & 255), ((word >> 24) & 255));
// 	outputString(s);
// 	w++;
// }

		p = recordAfter(p);
	}
	sprintf(s, "%d records, %d words, maxID %d, compaction cycles %d",
		recordCount, wordCount, maxID, cycleCount(current));
	outputString(s);

	int bytesUsed = 4 * (freeStart - ((0 == current) ? start0 : start1));
	sprintf(s, "%d bytes used (%d%%) of %d",
		bytesUsed, (100 * bytesUsed) / HALF_SPACE, HALF_SPACE);
	outputString(s);
}

void eraseCheck() {
	int badCount = 0;
	int *start = (0 == current) ? start1 : start0; // inactive half space
	int *end = (0 == current) ? end1 : end0;
	for (int *p = start; p < end; p++) {
		if (*p != 0xFFFFFFFF) badCount++;
		if (*p != 0xFFFFFFFF) {
			char s[200];
			sprintf(s, "bad %d: %x", (p - start), *p);
			outputString(s);
		}
	}
	reportNum("Non-erased words:", badCount);
}

void dumpHex() {
	int *start = (0 == current) ? start0 : start1;
	int *end = freeStart + 5000;
	for (int *p = start; p <= end; ) {
		char s[200];
		sprintf(s, "%d: %x %x %x %x %x %x %x %x %x %x",
			(p - start), p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9]);
		outputString(s);
		p += 10;
	}
}

static int * compactionStartRecord() {
	// Return a pointer to the first record at which to start compaction or script restoration
	// at startup.

	int *ptr = recordAfter(NULL);
	int *result = ptr; // first record in half-space; default if no 'deleteAll' records found
	while (ptr) {
		int type = (*ptr >> 16) & 0xFF;
		ptr = recordAfter(ptr);
		if (deleteAll == type) result = ptr;
	}
	return result;
}

static int * copyChunk(int *dst, int *src) {
	// Copy the chunk record at src to dst and return the new value of dst.

	int wordCount = *(src + 1) + 2;
	flashWriteData(dst, wordCount, (uint8 *) src);
	return dst + wordCount;
}

static void updateChunkTable() {
	memset(chunks, 0, sizeof(chunks)); // clear chunk table

	int *p = compactionStartRecord();
	while (p) {
		int recType = (*p >> 16) & 0xFF;
		if (chunkCode == recType) {
			int chunkIndex = (*p >> 8) & 0xFF;
			if (chunkIndex < MAX_CHUNKS) {
				chunks[chunkIndex].chunkType = *p & 0xFF;
				chunks[chunkIndex].code = p;
			}
		}
		if (chunkDeleted == recType) {
			int chunkIndex = (*p >> 8) & 0xFF;
			if (chunkIndex < MAX_CHUNKS) {
				chunks[chunkIndex].chunkType = unusedChunk;
				chunks[chunkIndex].code = NULL;
			}
		}
		p = recordAfter(p);
	}

	// update code pointers for tasks
	for (int i = 0; i < MAX_TASKS; i++) {
		if (tasks[i].status) { // task entry is in use
			tasks[i].code = chunks[tasks[i].taskChunkIndex].code;
		}
	}
}

// Flash Compaction

#ifndef RAM_CODE_STORE

struct {
	int *chunkCodeRec;
	int *attributeRecs[CHUNK_ATTRIBUTE_COUNT];
} chunkData;

static char chunkProcessed[256];
static char varProcessed[256];

static int * copyChunkInfo(int id, int *src, int *dst) {
	// Copy the most recent data about the chunk with the given ID to dst and return
	// the new dst pointer. If the given chunk id has been processed do nothing.

	if (chunkProcessed[id]) return dst;

	// clear chunkData
	memset(&chunkData, 0, sizeof(chunkData));

	// scan rest of the records to get the most recent info about this chunk
	while (src) {
		int attributeID;
		if (id == ((*src >> 8) & 0xFF)) { // id field matches
			int type = (*src >> 16) & 0xFF;
			switch (type) {
			case chunkCode:
				chunkData.chunkCodeRec = src;
				break;
			case chunkAttribute:
				attributeID = *src & 0xFF;
				if (attributeID < CHUNK_ATTRIBUTE_COUNT) {
					chunkData.attributeRecs[attributeID] = src;
				}
				break;
			case chunkDeleted:
				memset(&chunkData, 0, sizeof(chunkData)); // clear chunkData
				break;
			}
		}
		src = recordAfter(src);
	}
	if (chunkData.chunkCodeRec) {
		dst = copyChunk(dst, chunkData.chunkCodeRec);
		for (int i = 0; i < CHUNK_ATTRIBUTE_COUNT; i++) {
			if (chunkData.attributeRecs[i]) dst = copyChunk(dst, chunkData.attributeRecs[i]);
		}
	}
	chunkProcessed[id] = true;
	return dst;
}

static int * copyVarInfo(int id, int *src, int *dst) {
	if (varProcessed[id]) return dst;

	// record info from first reference to this variable
	int type = (*src >> 16) & 0xFF;
	int *nameRec = (varName == type) ? src : NULL;

	// scan rest of the records to get the most recent info about this variable
	while (src) {
		int type = (*src >> 16) & 0xFF;
		switch (type) {
		case varName:
			if (id == ((*src >> 8) & 0xFF)) nameRec = src;
			break;
		case varsClearAll:
			nameRec = NULL;
			break;
		}
		src = recordAfter(src);
	}
	if (nameRec) dst = copyChunk(dst, nameRec);
	varProcessed[id] = true;
	return dst;
}

static void compactFlash() {
	// Copy only the most recent chunk and variable records to the other half-space.
	// Details:
	//	1. erase the other half-space
	//	2. clear the chunk and variable processed flags
	//	3. find the start point for the scan (half space start or after latest 'deleteAll' record)
	//	4. for each chunk and variable record in the current half-space
	//		a. if the chunk or variable has been processed, skip it
	//		b. gather the most recent information about that chunk or variable
	//		c. copy that information into the other half-space
	//		d. mark the chunk or variable as processed
	//	5. switch to the other half-space
	//	6. remember the free pointer for the new half-space

	// clear the processed flags
	memset(chunkProcessed, 0, sizeof(chunkProcessed));
	memset(varProcessed, 0, sizeof(varProcessed));

	// clear the destination half-space and init dst pointer
	clearHalfSpace(!current);
	int *dst = ((0 == !current) ? start0 : start1) + 1;

	int *src = compactionStartRecord(NULL);
	while (src) {
		int header = *src;
		int type = (header >> 16) & 0xFF;
		int id = (header >> 8) & 0xFF;
		if ((chunkCode <= type) && (type <= chunkDeleted)) {
			dst = copyChunkInfo(id, src, dst);
		} else if ((varName <= type) && (type <= varsClearAll)) {
			dst = copyVarInfo(id, src, dst);
		}
		src = recordAfter(src);
	}

	// increment the cycle counter and switch to the other half-space
	setCycleCount(!current, cycleCount(current) + 1); // this commits the compaction
	current = !current;
	freeStart = dst;

	updateChunkTable();

	#if defined(NRF51) || defined(ARDUINO_BBC_MICROBIT_V2)
		// Compaction messes up the serial port on the micro:bit v1 and v2 and Calliope
		restartSerial();
	#endif

	char s[100];
	int bytesUsed = 4 * (freeStart - ((0 == current) ? start0 : start1));
	sprintf(s, "Compacted Flash code store\n%d bytes used (%d%%) of %d",
		bytesUsed, (100 * bytesUsed) / HALF_SPACE, HALF_SPACE);
	outputString(s);
}

#endif // compactFlash

// RAM compaction

#ifdef RAM_CODE_STORE

static int keepCodeChunk(int id, int header, int *start) {
	// Return true if this code chunk should be kept when compacting RAM.

	if (unusedChunk == chunks[id].chunkType) return false; // code chunk was deleted

	int *rec = start;
	while (rec) {
		// superceded only if all fields of header match (i.e. type, id, attributeType)
		if (*rec == header) return false; // superceded
		rec = recordAfter(rec);
	}
	return true;
}

static void compactRAM(int printStats) {
	// Compact a RAM-based code store in place. In-place compaction is possible in RAM since,
	// unlike Flash memory, RAM can be re-written without first erasing it. This approach
	// allows twice as much space for code as the half-space design.
	//
	// In-place compaction differs from half-space compaction because the destination of the
	// copying operations cannot overlap with the unprocessed portion of the code store. This
	// constraint is ensured by maintaining record order and "sliding down" all the surviving
	// records to so that all unused space is left at the end of the code store. where it is
	// available for storing new records.
	//
	// Details:
	//	1. find the start point for the scan (half space start or after latest 'deleteAll' record)
	//	2. find the most recent "varsClearAll" record
	//	3. for each chunk and variable record in the current half-space
	//		a. deterimine if the record should be kept or skipped
	//		b. if kept, copy the record down to the destination pointer
	//	4. update the free pointer
	//	5. clear the rest of the code store
	//	6. update the compaction count
	//	7. re-write the code file

	int *dst = ((0 == !current) ? start0 : start1) + 1;
	int *src = compactionStartRecord();

	if (!src) return; // nothing to compact

	// find the most recent varsClearAll record
	int *varsStart = src;
	int *rec = src;
	while (rec) {
		if (varsClearAll == ((*rec >> 16) & 0xFF)) varsStart = rec;
		rec = recordAfter(rec);
	}

	while (src) {
		int *next = recordAfter(src);
		int header = *src;
		int type = (header >> 16) & 0xFF;
		int id = (header >> 8) & 0xFF;
		if ((chunkCode <= type) && (type <= chunkAttribute) && keepCodeChunk(id, header, next)) {
			dst = copyChunk(dst, src);
		} else if ((varName == type) && (src >= varsStart)) {
			dst = copyChunk(dst, src);
		} else {
		}
		src = next;
	}

	freeStart = dst;
	memset(freeStart, 0, (4 * (end0 - freeStart))); // clear everything following freeStart

	updateChunkTable();

	// re-write the code file
	#if USE_CODE_FILE
		setCycleCount(current, cycleCount(current) + 1);
		clearCodeFile(cycleCount(current));
		int *codeStart = ((0 == current) ? start0 : start1) + 1; // skip half-space header
		writeCodeFile((uint8 *) codeStart, 4 * (freeStart - codeStart));
	#endif

	if (printStats) {
		char s[100];
		int bytesUsed = 4 * (freeStart - ((0 == current) ? start0 : start1));
		sprintf(s, "Compacted RAM code store\n%d bytes used (%d%%) of %d",
			bytesUsed, (100 * bytesUsed) / HALF_SPACE, HALF_SPACE);
		outputString(s);
	}
}
#endif

// entry points

void clearPersistentMemory() {
	int c0 = cycleCount(0);
	int c1 = cycleCount(1);
	int count = (c0 > c1) ? c0 : c1;
	current = !current;
	clearHalfSpace(current);
	freeStart = (0 == current) ? start0 + 1 : start1 + 1;
	setCycleCount(current, count + 1);
}

int * appendPersistentRecord(int recordType, int id, int extra, int byteCount, uint8 *data) {
	// Append the given record at the end of the current half-space and return it's address.
	// Header word: <tag = 'R'><record type><id of chunk/variable/comment><extra> (8-bits each)
	// Perform a compaction if necessary.
	int wordCount = (byteCount + 3) / 4;
	int *end = (0 == current) ? end0 : end1;
	if ((freeStart + 2 + wordCount) > end) {
		compactCodeStore();
		end = (0 == current) ? end0 : end1;
		if ((freeStart + 2 + wordCount) > end) {
			outputString("Not enough room even after compaction");
			return NULL;
		}
	}
	// write the record
	int header = ('R' << 24) | ((recordType & 0xFF) << 16) | ((id & 0xFF) << 8) | (extra & 0xFF);

// xxx debug: dump contents
// char s[500];
// sprintf(s, "Appending type %d id %d (%d words)", recordType, id, wordCount);
// outputString(s);
// char *p = data;
// for (int i = 0; i < wordCount; i++) {
// 	sprintf(s, "\t%d: %d %d %d %d ", i, *p, *(p + 1), *(p + 2), *(p + 3));
// 	outputString(s);
// 	p += 4;
// }

	#if USE_CODE_FILE
		if (!suspendFileUpdates) {
			writeCodeFileWord(header);
			writeCodeFileWord(wordCount);
			writeCodeFile(data, 4 * wordCount);
		}
	#endif

	int *result = freeStart;
	flashWriteWord(freeStart++, header);
	flashWriteWord(freeStart++, wordCount);
	if (wordCount) flashWriteData(freeStart, wordCount, data);
	freeStart += wordCount;
	return result;
}

void compactCodeStore() {
	#ifdef RAM_CODE_STORE
		compactRAM(true);
	#else
		compactFlash();
	#endif
}

void restoreScripts() {
	initPersistentMemory();

	#if USE_CODE_FILE
		int codeFileBytes = initCodeFile(flash, HALF_SPACE);
		int *start = current ? start1 : start0;
		freeStart = start + (codeFileBytes / 4);
	#endif

	updateChunkTable();

	// Give feedback:
	int chunkCount = 0;
	for (int i = 0; i < MAX_CHUNKS; i++) {
		if (chunks[i].code) chunkCount++;
	}
	char s[100];
	sprintf(s, "Restored %d scripts", chunkCount);
	outputString(s);
	outputString("Started");
}

int *scanStart() {
	// Return a pointer to the first record at which to start scanning the current code.

	int *ptr = recordAfter(NULL);
	int *result = ptr; // default if no 'deleteAll' records found
	while (ptr) {
		int type = (*ptr >> 16) & 0xFF;
		ptr = recordAfter(ptr);
		if (deleteAll == type) result = ptr;
	}
	return result;
}

void suspendCodeFileUpdates() {
	#ifdef USE_CODE_FILE
		suspendFileUpdates = true;
	#endif
}

void resumeCodeFileUpdates() {
	#ifdef USE_CODE_FILE
		if (suspendFileUpdates) {
			compactRAM(false); // also updates code file
		}
		suspendFileUpdates = false;
	#endif
}
