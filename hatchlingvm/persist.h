/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

// Copyright 2018 John Maloney, Bernat Romagosa, and Jens Mönig

// persist.h - Persistent memory
// John Maloney, December 2017

#ifdef __cplusplus
extern "C" {
#endif

// Persistent Memory Records

// Records in persistent memory start with two header words. They have the form:
//	<'R'><record type><id of chunk/variable/comment><extra> (8-bits for each field)
//	word count (32-bits)
//	... word count data words ...
//
// Not all record types use the <extra> header field.

#define PERSISTENT_HEADER_WORDS 2

typedef enum {
	chunkCode = 10,
	chunkDeleted = 19,
	varName = 21,
	varsClearAll = 29,
	deleteAll = 218, // 218 in hex is 0xDA, short for "delete all"
} RecordType_t;

// Persistent Memory Operations

int * appendPersistentRecord(int recordType, int id, int extra, int byteCount, uint8 *data);
void clearPersistentMemory();
int * recordAfter(int *lastRecord);
void restoreScripts();
int *scanStart();
void compactCodeStore();

#ifdef __cplusplus
}
#endif
