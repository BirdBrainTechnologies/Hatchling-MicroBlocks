/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

// Copyright 2018 John Maloney, Bernat Romagosa, and Jens Mönig

// interp.c - Simple interpreter based on 32-bit opcodes
// John Maloney, April 2017 + modifications by Tom Lauwers, November 2024

#define _DEFAULT_SOURCE // enable usleep() declaration from unistd.h
#include <Arduino.h> // only for debugging
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "mem.h"
#include "interp.h"
#include "persist.h"

// RECENT is a threshold for waking up tasks waiting on timers
// The timer can be up to this many usecs past the wakeup time.

#define RECENT 10000000

// Hatchling component types
#define SERVO 1
#define MOTOR 2
#define FAIRY 3
#define NEOPXL 4
#define NEOPXL_STRIP 5

// Interpreter State

CodeChunkRecord chunks[MAX_CHUNKS];

Task tasks[MAX_TASKS];
int taskCount = 0;

OBJ vars[MAX_VARS];
OBJ lvars[MAX_LISTS];

// For Hatchling - set a fancy name array to flash in the vmLoop when no BLE connection has occurred
//char fancyName[8] = {'E',' ','R',' ','R',' ',' ',' '};

// Error Reporting

// When a primitive encounters an error, it calls fail() with an error code.
// The VM stops the task and records the error code and IP where the error occurred.

static uint8 errorCode = noError;
static int taskSleepMSecs = 0;

OBJ fail(uint8 errCode) {
	errorCode = errCode;
	return falseObj;
}

int failure() {
	return errorCode != noError;
}

void taskSleep(int msecs) {
	// Make the current task sleep for the given number of milliseconds to free up cycles.
	taskSleepMSecs = msecs;
	errorCode = sleepSignal;
}

// Printing

#define PRINT_BUF_SIZE 800
static char printBuffer[PRINT_BUF_SIZE];
static int printBufferByteCount = 0;

int extraByteDelay = 1000; // default of 1000 usecs assumes serial throughput of ~1000 bytes/sec

static void printObj(OBJ obj) {
	// Append a printed representation of the given object to printBuffer.

	char *dst = &printBuffer[printBufferByteCount];
	int n = PRINT_BUF_SIZE - printBufferByteCount;

	if (isInt(obj)) snprintf(dst, n, "%d", obj2int(obj));
	else if (obj == falseObj) snprintf(dst, n, "false");
	else if (obj == trueObj) snprintf(dst, n, "true");
	else if (objType(obj) == StringType) {
		snprintf(dst, n, "%s", obj2str(obj));
	} else if (objType(obj) == ListType) {
		snprintf(dst, n, "[%d item list]", obj2int(FIELD(obj, 0)));
	} else if (objType(obj) == ByteArrayType) {
		snprintf(dst, n, "(%d bytes)", BYTES(obj));
	} else {
		snprintf(dst, n, "(object type: %d)", objType(obj));
	}
	printBufferByteCount = strlen(printBuffer);
}

static void printArgs(int argCount, OBJ *args, int forSay, int insertSpaces) {
	// Print all args into printBuffer and return the size of the resulting string.

	if (forSay) {
		printBuffer[0] = 2; // type is string (printBuffer is used as outputValue message body)
		printBufferByteCount = 1;
	} else {
		printBufferByteCount = 0;
	}
	printBuffer[printBufferByteCount] = 0; // null terminate

	for (int i = 0; i < argCount; i++) {
		printObj(args[i]);
		if (insertSpaces && (i < (argCount - 1)) && (printBufferByteCount < PRINT_BUF_SIZE)) {
			printBuffer[printBufferByteCount++] = ' '; // add a space
			printBuffer[printBufferByteCount] = 0; // null terminate
		}
	}
}

static int bytesForObject(OBJ value) {
	// Return the number of bytes needed to transmit the given value.

	int headerBytes = 6; // message header (5 bytes) + type byte
	if (isInt(value)) { // 32-bit integer
		return headerBytes + 4;
	} else if (IS_TYPE(value, StringType)) { // string
		int len = strlen(obj2str(value));
		if (len > 800) len = 800;
		return headerBytes + len;
	} else if ((value == trueObj) || (value == falseObj)) { // boolean
		return headerBytes + 1;
	}
	return 512; // maximum that might be needed, based on size of buffer in sendValueMessage
}

// Broadcast

OBJ lastBroadcast = zeroObj; // Note: This variable must be processed by the garbage collector!

static void primSendBroadcast(int argCount, OBJ *args) {
	// Variadic broadcast; all args are concatenated into printBuffer.
	printArgs(argCount, args, false, false);
	// save the last broadcasted message
	lastBroadcast = newStringFromBytes(printBuffer, printBufferByteCount);
	startReceiversOfBroadcast(printBuffer, printBufferByteCount);
	sendBroadcastToIDE(printBuffer, printBufferByteCount);
}

OBJ primBroadcastToIDEOnly(int argCount, OBJ *args) {
	// Broadcast a string to the IDE only, not locally.

	printArgs(argCount, args, false, false);
	sendBroadcastToIDE(printBuffer, printBufferByteCount);
	return falseObj;
}

// Timer

static uint32 timerStart = 0;

static void resetTimer() { timerStart = millisecs(); }

static int timer() {
	// Return the number of milliseconds since the timer was last reset.
	// Note: The millisecond clock is the 32-bit microsecond clock divided by 1000,
	// so it wraps around to zero when the microsecond clock wraps, which occurs
	// about every 72 minutes and 35 seconds. That's the maximum duration that can
	// be measured with this simple timer implementation.

	const uint32 msecWrap = 4294967; // 2^32 / 1000, value at which the millisecond clock wraps

	uint32 now = millisecs();
	if (now < timerStart) { // clock wrapped
		return (msecWrap - timerStart) + now; // time to wrap + time since wrap
	}
	return now - timerStart;
}

// String Access

static inline char * nextUTF8(char *s) {
	// Return a pointer to the start of the UTF8 character following the given one.
	// If s points to a null byte (i.e. end of the string) return it unchanged.

	if (!*s) return s; // end of string
	if ((uint8) *s < 128) return s + 1; // single-byte character
	if (0xC0 == (*s & 0xC0)) s++; // start of multi-byte character
	while (0x80 == (*s & 0xC0)) s++; // skip continuation bytes
	return s;
}

static int countUTF8(char *s) {
	int count = 0;
	while (*s) {
		s = nextUTF8(s);
		count++;
	}
	return count;
}

static OBJ charAt(OBJ stringObj, int i) {
	char *start = obj2str(stringObj);
	while (i-- > 1) { // find start of the ith Unicode character
		if (!*start) return fail(indexOutOfRangeError); // end of string
		start = nextUTF8(start);
	}
	int byteCount = nextUTF8(start) - start;
	OBJ result = newString(byteCount);
	if (result) {
		memcpy(obj2str(result), start, byteCount);
	}
	return result;
}

// Board Type

#define BOARD_TYPE_SIZE 32

// statically allocated object for the boardType primitive result
static struct {
	uint32 header;
	char body[BOARD_TYPE_SIZE];
} boardTypeObj;

OBJ primBoardType() {
	strncpy(boardTypeObj.body, boardType(), BOARD_TYPE_SIZE - 1);
	int wordCount = (strlen(boardTypeObj.body) + 4) / 4;
	boardTypeObj.header = HEADER(StringType, wordCount);
	return (OBJ) &boardTypeObj;
}

// Hatchling Inline Primitives

// Note support
static int hatchlingNoteIsPlaying = false;
static uint32 hatchlingNoteEndTime = 0;  // microseconds

// Scroll text support
static int textDisplaying = false;
static int charCounter = 0;
static uint32 nextDisplayTime = 0; // microseconds
static int textDisplayPos = 5;
char* textToDisplay;
static int textLength = 0;

#define DISPLAYTIME 100000  // Time between micro:bit display updates while scrolling (current 100 ms, so each char will take 500 ms to scroll on and another 500 ms to scroll off)

// Port support
static int portActive[6] = {false, false, false, false, false, false};
static int portType[6] = {0, 0, 0, 0, 0, 0};
static uint32 portEndTime[6] = {0, 0, 0, 0, 0, 0}; // microseconds

// baseline frequencies in milliHertz for the base octave
static uint32 toneFrequencies[12] = {261626,277183,293665,311127,329628,349228,369994,391995,415305,440000,466164,493883};

static OBJ primHatchlingPlayNote(int argCount, OBJ *args) {
	// Args: midi note, beats
	// Use built-in speaker pin for output.
	// Sets hatchlingNoteEndTime and expects interpreter loop to turn it off.

	int midiNote = evalInt(args[0]);

	if ((midiNote < 24 && midiNote != 0) || (midiNote > 120)) return falseObj;

	int octave = midiNote/12-5; // sets the possible octaves from -4 to 6, with 0 being the baseline octave

	uint32 frequency = toneFrequencies[midiNote%12]; // set the base frequency, which needs to be adjusted by the octave

	if(octave > 0)
	{
		for(int count = 0; count < octave; count++)
		{
			frequency = frequency*2; //double by however many octaves we have
		}
	}
	else if(octave < 0)
	{
		for(int count = 0; count > octave; count--)
		{
			frequency = frequency/2; //halve by however many octaves we have below 0
		}
	}
	// Reduce by 1000 to get the actual frequency in hertz
	frequency = frequency/1000;

	int noteLength = evalInt(args[1]); // Sent by the interface as 1 (1 = full note), 2, 4, 8, 16 (16=sixteenth note) and 3, 6, 12, 24 are the dotted notes (50% longer) 
	int currTempo = getTempo();

	int durationMSecs; 
	
	// Dotted notes are divisible by 3
	if(noteLength%3 == 0)
	{
		durationMSecs=9000*1/noteLength*60/currTempo;  // Converting the duration to be 50% longer and adjusting for the fact that noteLength is also 50% more 
	}
	else
	{
		durationMSecs=4000*1/noteLength*60/currTempo;
	}
	if (durationMSecs < 10) return falseObj; // too short

	// start playing tone, unless this is a rest
	if(midiNote != 0)
	{
		OBJ toneArgs[] = { int2obj(-1), int2obj(frequency) };
		primPlayTone(2, toneArgs);

		hatchlingNoteIsPlaying = true;
		hatchlingNoteEndTime = microsecs() + (1000 * durationMSecs);
	}
	taskSleep(durationMSecs);
	
	return trueObj;
}

static OBJ primHatchlingPlayTone(int argCount, OBJ *args) {
	// Args: frequency, duration in milliseconds
	// Use built-in speaker pin for output.
	// Sets hatchlingNoteEndTime and expects interpreter loop to turn it off.

	int freq = evalInt(args[0]);
	if ((freq < 20) || (freq > 20000)) return falseObj;

	int durationMSecs = evalInt(args[1]);
	if (durationMSecs < 10) return falseObj; // too short

	// start playing tone
	OBJ toneArgs[] = { int2obj(-1), int2obj(freq) };
	primPlayTone(2, toneArgs);

	hatchlingNoteIsPlaying = true;
	hatchlingNoteEndTime = microsecs() + (1000 * durationMSecs);
	taskSleep(durationMSecs);

	return trueObj;
}

static OBJ primHatchlingServoWithDelay(int argCount, OBJ *args) {
	// Args: port, position, duration in milliseconds
	// Sets endTime

    if (!IS_TYPE(args[0], StringType)) return fail(needsStringError);
    int ch = obj2str(args[0])[0];
    int pinNum = -1; // default value: invalid port
    if (('A' <= ch) && (ch <= 'F')) pinNum = ch - 'A';
    if (('a' <= ch) && (ch <= 'f')) pinNum = ch - 'a';

	if(pinNum == -1) return falseObj;

	int durationMSecs = evalInt(args[2]);
	if (durationMSecs > 10000) return falseObj; // too long (10 seconds+)

	// start servo
	OBJ servoArgs[] = { args[0], args[1] };
	primPositionServos(2, servoArgs);

	// If the duration is very short, then just set the port and do not turn it back off later
	if(durationMSecs < 10)
	{
		portActive[pinNum] = false; // In case it is already true from being set previously. Port Active just means that it needs to be turned off later
		portType[pinNum] = SERVO; // Set the type regardless
	}
	else {
		portActive[pinNum] = true;
		portEndTime[pinNum] = microsecs() + (1000 * durationMSecs);
		portType[pinNum] = SERVO; // Set the type regardless
		taskSleep(durationMSecs);
	}
	return trueObj;
}

static OBJ primHatchlingMotorWithDelay(int argCount, OBJ *args) {
	// Args: port, speed, duration in milliseconds
	// Sets endTime and expects vmLoop to turn it off

    if (!IS_TYPE(args[0], StringType)) return fail(needsStringError);
    int ch = obj2str(args[0])[0];
    int pinNum = -1; // default value: invalid port
    if (('A' <= ch) && (ch <= 'F')) pinNum = ch - 'A';
    if (('a' <= ch) && (ch <= 'f')) pinNum = ch - 'a';

	if(pinNum == -1) return falseObj;

	int durationMSecs = evalInt(args[2]);
	if (durationMSecs > 10000) return falseObj; // too long (10 seconds+)

	// start motor
	OBJ motorArgs[] = { args[0], args[1] };
	primRotationServos(2, motorArgs);

	// If the duration is very short, then just set the port and do not turn it back off later
	if(durationMSecs < 10)
	{
		portActive[pinNum] = false; // In case it is already true from being set previously. Port Active just means that it needs to be turned off later
		portType[pinNum] = MOTOR; // Set the type regardless
	}
	else {
		portActive[pinNum] = true;
		portEndTime[pinNum] = microsecs() + (1000 * durationMSecs);
		portType[pinNum] = MOTOR; // Set the type regardless
		taskSleep(durationMSecs);
	}
	return trueObj;
}

static OBJ primHatchlingFairyLightWithDelay(int argCount, OBJ *args) {
	// Args: port, intensity, duration in milliseconds
	// Sets endTime and expects vmLoop to turn it off

    if (!IS_TYPE(args[0], StringType)) return fail(needsStringError);
    int ch = obj2str(args[0])[0];
    int pinNum = -1; // default value: invalid port
    if (('A' <= ch) && (ch <= 'F')) pinNum = ch - 'A';
    if (('a' <= ch) && (ch <= 'f')) pinNum = ch - 'a';

	if(pinNum == -1) return falseObj;

	int durationMSecs = evalInt(args[2]);
	if (durationMSecs > 10000) return falseObj; // too long (10 seconds+)

	// start fairy lights
	OBJ fairyArgs[] = { args[0], args[1] };
	primFairyLights(2, fairyArgs);

	// If the duration is very short, then just set the port and do not turn it back off later
	if(durationMSecs < 10)
	{
		portActive[pinNum] = false; // In case it is already true from being set previously. Port Active just means that it needs to be turned off later
		portType[pinNum] = FAIRY; // Set the type regardless
	}
	else
	{
		portActive[pinNum] = true;
		portEndTime[pinNum] = microsecs() + (1000 * durationMSecs);
		portType[pinNum] = FAIRY; // Set the type regardless
		taskSleep(durationMSecs);
	}
	return trueObj;
}

static OBJ primNeopixelWithDelay(int argCount, OBJ *args) {
	// Args: port, intensity (RGB), duration in milliseconds
	// Sets endTime and expects vmLoop to turn it off

    if (!IS_TYPE(args[0], StringType)) return fail(needsStringError);
    int ch = obj2str(args[0])[0];
    int pinNum = -1; // default value: invalid port
    if (('A' <= ch) && (ch <= 'F')) pinNum = ch - 'A';
    if (('a' <= ch) && (ch <= 'f')) pinNum = ch - 'a';

	if(pinNum == -1) return falseObj;

	int durationMSecs = evalInt(args[4]);
	if (durationMSecs > 10000) return falseObj; // too long (10 seconds+)

	// start neopixel light
	OBJ NeopixelArgs[] = {args[0], args[1], args[2], args[3]};
	primNeoPixel(4, NeopixelArgs);
	// If the duration is very short, then just set the port and do not turn it back off later
	if(durationMSecs < 10)
	{
		portActive[pinNum] = false; // In case it is already true from being set previously. Port Active just means that it needs to be turned off later
		portType[pinNum] = NEOPXL; // Set the type regardless
	}
	else
	{
		portActive[pinNum] = true;
		portEndTime[pinNum] = microsecs() + (1000 * durationMSecs);
		portType[pinNum] = NEOPXL;
		taskSleep(durationMSecs);
	}
	return trueObj;
}

static OBJ primNeopixelStripBuiltIn(int argCount, OBJ *args) {
	// Args: port, which LED setting (all, or 1, 2, 3, 4), intensity (RGB)

    if (!IS_TYPE(args[0], StringType)) return fail(needsStringError);
    int ch = obj2str(args[0])[0];
    int pinNum = -1; // default value: invalid port
    if (('A' <= ch) && (ch <= 'F')) pinNum = ch - 'A';
    if (('a' <= ch) && (ch <= 'f')) pinNum = ch - 'a';

	if(pinNum == -1) return falseObj;

	// start strip lights
	OBJ NeopixelStripArgs[] = {args[0], args[1], args[2], args[3], args[4]};
	primNeoPixelStrip(5, NeopixelStripArgs);
	portActive[pinNum] = false; // In case it is already true from being set previously. Port Active being true  just means that it needs to be turned off later
	portType[pinNum] = NEOPXL_STRIP; // Set the type regardless
	
	return trueObj;
}

static OBJ primHLDisplayText(int argCount, OBJ *args) {
	
	charCounter = 0;
	textDisplayPos = 5;

	textToDisplay = (char *)malloc(100); // Strings up to 100 bytes are okay

	if(textToDisplay == NULL)
	{
		outputString("Scroll text memory allocation failed");
		return falseObj;
	}

	if (isInt(args[0]))
	{
		long int numberToDisplay = obj2int(args[0]); // Numbers from -2^31 to 2^31-1
		sprintf(textToDisplay, "%ld", numberToDisplay); // Turn the number into a string
	}
	else
	{
		textToDisplay = obj2str(args[0]);
	}

	textLength = strlen(textToDisplay);

	OBJ displayArgs[3];

	displayArgs[0] = newStringFromBytes(&textToDisplay[0], 1);
	OBJ letterShape = primMBShapeForLetter(1, displayArgs);

	displayArgs[0] = letterShape;
	displayArgs[1] = int2obj(textDisplayPos); // Displays the letter in x = 5, y = 1 position to start scrolling in. 
	displayArgs[2] = int2obj(1);
	primMBDrawShape(3,displayArgs);
	textDisplaying = true;
	nextDisplayTime = microsecs() + DISPLAYTIME; // Will update every 100ms
	// Total time in milliseconds that the text will display
    int timeToSleep = 400 + 500*textLength; // First char takes 400 ms to fully appear, and then each char takes another 500 ms to traverse the screen
	taskSleep(timeToSleep);

	return trueObj;
}
// Misc primitives

static OBJ primModulo(int argCount, OBJ *args) {
	int n = evalInt(args[0]);
	int modulus = evalInt(args[1]);
	if (0 == modulus) return fail(zeroDivide);
	if (modulus < 0) modulus = -modulus;
	int result = n % modulus;
	if (result < 0) result += modulus;
	return int2obj(result);
}

static OBJ primRandom(int argCount, OBJ *args) {
	int first = 1, last = 100; // defaults for zero arguments
	if (argCount == 1) { // use range [1..arg]
		first = 1;
		last = evalInt(args[0]);
		if (last < 0) first = -1;
	} else if (argCount == 2) { // use range [first..last]
		first = evalInt(args[0]);
		last = evalInt(args[1]);
	}
	if (first > last) { // ensure first <= last
		int tmp = first;
		first = last;
		last = tmp;
	}
	int range = (last + 1) - first; // if first == last range is 1 and first is returned
	return int2obj(first + (rand() % range)); // result range is [first..last], inclusive
}

static OBJ primMinimum(int argCount, OBJ *args) {
	if (argCount < 1) return fail(notEnoughArguments);
	int result = obj2int(args[0]);
	for (int i = 0; i < argCount; i++) {
		OBJ arg = args[i];
		if (!isInt(arg)) return fail(needsIntegerError);
		int n = obj2int(arg);
		if (n < result) result = n;
	}
	return int2obj(result);
}

static OBJ primMaximum(int argCount, OBJ *args) {
	if (argCount < 1) return fail(notEnoughArguments);
	int result = obj2int(args[0]);
	for (int i = 0; i < argCount; i++) {
		OBJ arg = args[i];
		if (!isInt(arg)) return fail(needsIntegerError);
		int n = obj2int(arg);
		if (n > result) result = n;
	}
	return int2obj(result);
}

static inline int compareObjects(OBJ obj1, OBJ obj2) {
	// Compare two objects with the given operator and return one of:
	//	-1 (<), 0 (==), 1 (>)
	// For mixed string-int comparison, try to convert the string to an integer.
	// Set nonComparableError flag if the objects are not comparable.

	int n1 = 0, n2 = 0;
	if (IS_TYPE(obj1, StringType) && IS_TYPE(obj2, StringType)) {
		return strcmp(obj2str(obj1), obj2str(obj2));
	} else if (IS_TYPE(obj1, StringType) && isInt(obj2)) {
		n1 = strtol(obj2str(obj1), NULL, 10);
		n2 = obj2int(obj2);
	} else if (isInt(obj1) && IS_TYPE(obj2, StringType)) {
		n1 = obj2int(obj1);
		n2 = strtol(obj2str(obj2), NULL, 10);
	} else if (isInt(obj1) && isInt(obj2)) {
		// Note: For efficiency, caller should handle this special case
		n1 = obj2int(obj1);
		n2 = obj2int(obj2);
	} else {
		fail(nonComparableError);
	}
	if (n1 < n2) return -1;
	if (n1 > n2) return 1;
	return 0;
}

static OBJ primCompare(int op, OBJ obj1, OBJ obj2) {
	// Compare objects with the given operator:
	//	-2 (<), -1 (<=), 0 (==), 1 (>=), 2 (>)
	// Return a boolean. Set nonComparableError error if objects are not comparable.

	int result = compareObjects(obj1, obj2);
	if (result < 0) return (op < 0) ? trueObj : falseObj;
	if (result > 0) return (op > 0) ? trueObj : falseObj;
	return ((-1 <= op) && (op <= 1)) ? trueObj : falseObj;
}

static int stringsEqual(OBJ obj1, OBJ obj2) {
	// Return true if the given strings have the same length and contents.
	// Assume s1 and s2 are of Strings.

	int byteCount = 4 * objWords(obj1);
	if (byteCount != (4 * objWords(obj2))) return false; // different lengths
	char *s1 = (char *) &FIELD(obj1, 0);
	char *s2 = (char *) &FIELD(obj2, 0);
	char *end = s1 + byteCount;
	while (s1 < end) {
		if (!*s1 && !*s2) return true; // null terminator in both strings
		if (*s1++ != *s2++) return false; // not equal
	}
	return true;
}

static OBJ argOrDefault(OBJ *fp, int argNum, OBJ defaultValue) {
	// Useful for working with optional arguments.
	// Return the given argument or defaultValue if the argument was not supplied by the caller.

	if (argNum < 1) return defaultValue; // argNum index is 1-based
	int actualArgCount = obj2int(*(fp - 3));
	if (argNum > actualArgCount) return defaultValue; // argument not supplied, return default
	return *(fp - (4 + actualArgCount) + argNum); // return the desired argument
}

static int functionNameMatches(int chunkIndex, char *functionName) {
	// Return true if given chunk is the function with the given function name.
	// by checking the function name in the function's metadata.

	const uint32 META_FLAG = 240;
	uint32 wordCount = ((uint32 *) chunks[chunkIndex].code)[1];
	uint32 *code = (uint32 *) chunks[chunkIndex].code + PERSISTENT_HEADER_WORDS;
	int metaStart = -1;
	for (int i = 0; i < wordCount; i++) {
		if (META_FLAG == code[i]) {
			metaStart = i;
			break;
		}
	}
	if (metaStart < 0) return false; // no metadata

	OBJ meta = (OBJ) &code[metaStart + 1];
	if (!IS_TYPE(meta, StringType)) return false; // bad metadata; should not happen
	meta += HEADER_WORDS + WORDS(meta); // skip var names string
	if (!IS_TYPE(meta, StringType)) return false; // bad metadata; should not happen

	// s is a tab-delimited string with meta information about the function:
	//	libraryName libraryCategory blockType funcName specString argTypes
	char *s = obj2str(meta);

	// skip the first three tab-delimited fields (libraryName libraryCategory blockType)
	for (int i = 0; i < 3; i++) {
		s = strchr(s, '\t'); // find next tab
		if (!s) return false; // tab not found; should not happen
		s += 1;
	}

	// return true if s begins with the given function name
	return (strstr(s, functionName) == s);
}

static int chunkIndexForFunction(char *functionName) {
	// Return the chunk index for the function with the given name or -1 if not found.

	int nameLength = strlen(functionName);
	for (int i = 0; i < MAX_CHUNKS; i++) {
		int chunkType = chunks[i].chunkType;
		if (functionHat == chunkType) {
			if (broadcastMatches(i, functionName, nameLength)) return i;
			if (functionNameMatches(i, functionName)) return i;
		}
	}
	return -1;
}

PrimitiveFunction findPrimitive(char *namedPrimitive);

static int findCallee(char *functionOrPrimitiveName) {
	// Look for a primitive match first since that is fast
	PrimitiveFunction f = findPrimitive(functionOrPrimitiveName);
	if (f) return (int) f;

	// Look for a user-defined function match (slow if no match found!)
	int result = chunkIndexForFunction(functionOrPrimitiveName);
	if (result >= 0) return (0xFFFFFF00 | result); // set top 24 bits to show callee is a chunk
	// assume: result < 256 (MAX_CHUNKS) so it fits in low 8 bits

	fail(primitiveNotImplemented);
	return -1;
}

// Interpreter

// Macros to pop arguments for commands and reporters (pops args, leaves result on stack)
#define POP_ARGS_COMMAND() { sp -= arg; }
#define POP_ARGS_REPORTER() { sp -= arg - 1; }

// Macro to check for stack overflow
#define STACK_CHECK(n) { \
	if (((sp + (n)) - task->stack) > STACK_LIMIT) { \
		errorCode = stackOverflow; \
		goto error; \
	} \
}

// Macros to support function calls
#define IN_CALL() (fp > task->stack)

// Macro to inline dispatch in the end of each opcode (avoiding a jump back to the top)
#define DISPATCH() { \
	if (errorCode) goto error; \
	op = *ip++; \
	arg = ARG(op); \
	task->sp = sp - task->stack; /* record stack pointer for garbage collector */ \
/*	printf("ip: %d cmd: %d arg: %d sp: %d\n", (ip - task->code), CMD(op), arg, (sp - task->stack)); */ \
	goto *jumpTable[CMD(op)]; \
}

// Macro for debugging stack errors
#define SHOW_SP(s) { \
	outputString(s); \
	reportNum("sp", sp - task->stack); \
	reportNum("fp", fp - task->stack); \
}

static void runTask(Task *task) {
	register int op;
	register int *ip;
	register OBJ *sp;
	register OBJ *fp;
	int arg, tmp;
	OBJ tmpObj;

	// initialize jump table
	static void *jumpTable[] = {
		&&halt_op,
		&&noop_op,
		&&pushImmediate_op,
		&&pushBigImmediate_op,
		&&pushLiteral_op,
		&&pushVar_op,
		&&storeVar_op,
		&&incrementVar_op,
		&&pushArgCount_op,
		&&pushArg_op,
		&&storeArg_op,
		&&incrementArg_op,
		&&pushLocal_op,
		&&storeLocal_op,
		&&incrementLocal_op,
		&&pop_op,
		&&jmp_op,
		&&jmpTrue_op,
		&&jmpFalse_op,
		&&decrementAndJmp_op,
		&&callFunction_op,
		&&returnResult_op,
		&&waitMicros_op,
		&&waitMillis_op,
		&&sendBroadcast_op,
		&&recvBroadcast_op,
		&&stopAllButThis_op,
		&&forLoop_op,
		&&initLocals_op,
		&&getArg_op,
		&&getLastBroadcast_op,
		&&jmpOr_op,
		&&jmpAnd_op,
		&&minimum_op,
		&&maximum_op,
		&&lessThan_op,
		&&lessOrEq_op,
		&&equal_op,
		&&notEqual_op,
		&&greaterOrEq_op,
		&&greaterThan_op,
		&&not_op,
		&&add_op,
		&&subtract_op,
		&&multiply_op,
		&&divide_op,
		&&modulo_op,
		&&absoluteValue_op,
		&&random_op,
		&&hexToInt_op,
		&&bitAnd_op,
		&&bitOr_op,
		&&bitXor_op,
		&&bitInvert_op,
		&&bitShiftLeft_op,
		&&bitShiftRight_op,
		&&longMultiply_op,
		&&isType_op,
		&&jmpFalse_op, // this is the waitUntil opcode, an alias for jmpFalse_op
		&&pop_op, // this is the ignoreArgs opcode, an alias for pop_op
		&&newList_op,
		&&RESERVED_op,
		&&fillList_op,
		&&at_op,
		&&atPut_op,
		&&length_op,
		&&RESERVED_op,
		&&RESERVED_op,
		&&RESERVED_op,
		&&RESERVED_op,
		&&millis_op,
		&&micros_op,
		&&timer_op,
		&&resetTimer_op,
		&&sayIt_op,
		&&logData_op,
		&&boardType_op,
		&&comment_op,
		&&argOrDefault_op,
		&&RESERVED_op,
		&&analogPins_op,
		&&digitalPins_op,
		&&analogRead_op,
		&&analogWrite_op,
		&&digitalRead_op,
		&&digitalWrite_op,
		&&digitalSet_op,
		&&digitalClear_op,
		&&buttonA_op,
		&&buttonB_op,
		&&setUserLED_op,
		&&i2cSet_op,
		&&i2cGet_op,
		&&spiSend_op,
		&&spiRecv_op,
		&&RESERVED_op,
		&&RESERVED_op,
		&&RESERVED_op,
		&&RESERVED_op,
		&&RESERVED_op,
		&&mbDisplay_op,
		&&mbDisplayOff_op,
		&&mbPlot_op,
		&&mbUnplot_op,
		&&mbTiltX_op,
		&&mbTiltY_op,
		&&mbTiltZ_op,
		&&mbTemp_op,
		&&neoPixelSend_op,
		&&drawShape_op,
		&&shapeForLetter_op,
		&&neoPixelSetPin_op,
		&&hatchlingPlayNote_op, // Adding the Hatchling delay built-in primitives
		&&hatchlingServoWithDelay_op,
		&&hatchlingMotorWithDelay_op,
		&&hatchlingFairyLightWithDelay_op,
		&&hatchlingNeopixelWithDelay_op,
		&&hatchlingNeopixelStripBuiltIn_op,
		&&hatchlingPlayTone_op,
		&&hatchlingDisplayText_op,
		&&RESERVED_op,
		&&RESERVED_op,
		&&RESERVED_op,
		&&RESERVED_op,
		&&callCustomCommand_op,
		&&callCustomReporter_op,
		&&callCommandPrimitive_op,
		&&callReporterPrimitive_op,
	};

	// Restore task state
	ip = task->code + task->ip;
	sp = task->stack + task->sp;
	fp = task->stack + task->fp;

	DISPATCH();

	error:
		// sleepSignal is not a actual error; it just suspends the current task
		if (sleepSignal == errorCode) {
			errorCode = noError; // clear the error
			if (taskSleepMSecs > 0) {
				task->status = waiting_micros;
				task->wakeTime = microsecs() + (taskSleepMSecs * 1000);
			}
			goto suspend;
		}
		// tmp encodes the error location: <22 bit ip><8 bit chunkIndex>
		tmp = ((ip - task->code) << 8) | (task->currentChunkIndex & 0xFF);
		sendTaskError(task->taskChunkIndex, errorCode, tmp);
		task->status = unusedTask;
		errorCode = noError; // clear the error
		goto suspend;
	suspend:
		// save task state
		task->ip = ip - task->code;
		task->sp = sp - task->stack;
		task->fp = fp - task->stack;
		return;
	RESERVED_op:
	halt_op:
		sendTaskDone(task->taskChunkIndex);
		task->status = unusedTask;
		goto suspend;
	noop_op:
		DISPATCH();
	pushImmediate_op:
		STACK_CHECK(1);
		*sp++ = (OBJ) arg;
		DISPATCH();
	pushBigImmediate_op:
		STACK_CHECK(1);
		*sp++ = (OBJ) *ip++;
		DISPATCH();
	pushLiteral_op:
		STACK_CHECK(1);
		*sp++ = (OBJ) (ip + arg); // arg is offset from the current ip to the literal object
		DISPATCH();
	pushVar_op:
		STACK_CHECK(1);
		if (arg >= MAX_VARS) {
			*sp++ = lvars[arg - MAX_VARS];
		} else {
			*sp++ = vars[arg];
		}
		DISPATCH();
	storeVar_op:
		vars[arg] = *--sp;
		DISPATCH();
	incrementVar_op:
		tmp = evalInt(vars[arg]);
		if (!errorCode) {
			vars[arg] = int2obj(tmp + evalInt(*--sp));
		}
		DISPATCH();
	pushArgCount_op:
		STACK_CHECK(1);
		*sp++ = IN_CALL() ? *(fp - 3) : zeroObj;
		DISPATCH();
	pushArg_op:
		STACK_CHECK(1);
		if (IN_CALL()) {
			*sp++ = *(fp - obj2int(*(fp - 3)) - 3 + arg);
		} else {
			*sp++ = fail(notInFunction);
		}
		DISPATCH();
	storeArg_op:
		if (IN_CALL()) {
			*(fp - obj2int(*(fp - 3)) - 3 + arg) = *--sp;
		} else {
			fail(notInFunction);
		}
		DISPATCH();
	incrementArg_op:
		if (IN_CALL()) {
			tmp = evalInt(*(fp - obj2int(*(fp - 3)) - 3 + arg)) + evalInt(*--sp);
			*(fp - obj2int(*(fp - 3)) - 3 + arg) = int2obj(tmp);
		} else {
			fail(notInFunction);
		}
		DISPATCH();
	pushLocal_op:
		STACK_CHECK(1);
		*sp++ = *(fp + arg);
		DISPATCH();
	storeLocal_op:
		*(fp + arg) = *--sp;
		DISPATCH();
	incrementLocal_op:
		*(fp + arg) = int2obj(obj2int(*(fp + arg)) + evalInt(*--sp));
		DISPATCH();
	pop_op:
		sp -= arg;
		if (sp >= task->stack) {
			DISPATCH();
		} else {
			vmPanic("Stack underflow");
		}
		DISPATCH();
	jmp_op:
		ip += arg;
		if (arg < 0) goto suspend;
		DISPATCH();
	jmpTrue_op:
		if (trueObj == (*--sp)) ip += arg;
		if ((arg < 0) && (trueObj == *sp)) goto suspend;
		DISPATCH();
	jmpFalse_op:
		if (trueObj != (*--sp)) ip += arg; // treat any value but true as false
		if ((arg < 0) && (trueObj != *sp)) goto suspend;
		DISPATCH();
	 decrementAndJmp_op:
		if (isInt(*(sp - 1))) {
			tmp = obj2int(*(sp - 1)) - 1; // decrement loop counter (normal case)
		} else {
			tmp = evalInt(*(sp - 1)) - 1; // decrement loop counter (first time: convert string to int if needed)
		}
		if (tmp >= 0) {
			ip += arg; // loop counter >= 0, so branch
			*(sp - 1) = int2obj(tmp); // update loop counter
			goto suspend;
		} else {
			sp--; // loop done, pop loop counter
		}
		DISPATCH();
	callFunction_op:
		// function call stack layout for N function arguments and M local variables:
		// local M-1
		// ...
		// local 0 <- fp points here during call, so the value of local m is *(fp + m)
		// *(fp - 1), the old fp
		// *(fp - 2), return address, <22 bit ip><8 bit chunkIndex> encoded as an integer object
		// *(fp - 3), # of function arguments
		// arg N-1
		// ...
		// arg 0
		tmp = (arg >> 8) & 0xFF; // callee's chunk index (middle byte of arg)
		if (chunks[tmp].chunkType != functionHat) {
			fail(badChunkIndexError);
			goto error;
		}
		STACK_CHECK(3);
		*sp++ = int2obj(arg & 0xFF); // # of arguments (low byte of arg)
		*sp++ = int2obj(((ip - task->code) << 8) | (task->currentChunkIndex & 0xFF)); // return address
		*sp++ = int2obj(fp - task->stack); // old fp
		fp = sp;
		task->currentChunkIndex = tmp; // callee's chunk index (middle byte of arg)
		task->code = chunks[task->currentChunkIndex].code;
		ip = task->code + PERSISTENT_HEADER_WORDS; // first instruction in callee
		DISPATCH();
	returnResult_op:
		tmpObj = *(sp - 1); // return value
		if (fp == task->stack) { // not in a function call
			if (!hasOutputSpace(bytesForObject(tmpObj) + 100)) { // leave room for other messages
				ip--; // retry when task is resumed
				goto suspend;
			}
			sendTaskReturnValue(task->taskChunkIndex, tmpObj);
			task->status = unusedTask;
			goto suspend;
		}
		sp = fp - obj2int(*(fp - 3)) - 3; // restore stack pointer; *(fp - 3) is the arg count
		*sp++ = tmpObj; // push return value (no need for a stack check; just recovered at least 3 words from the old call frame)
		tmp = obj2int(*(fp - 2)); // return address
		task->currentChunkIndex = tmp & 0xFF;
		task->code = chunks[task->currentChunkIndex].code;
		ip = task->code + ((tmp >> 8) & 0x3FFFFF); // restore old ip
		fp = task->stack + obj2int(*(fp - 1)); // restore the old fp
		DISPATCH();
	waitMicros_op:
	 	tmp = evalInt(*(sp - 1)); // wait time in usecs
	 	POP_ARGS_COMMAND();
	 	if (tmp <= 30) {
	 		if (tmp <= 0) { DISPATCH(); } // don't wait at all
			// busy-wait for wait times up to 30 usecs to avoid a context switch
			tmp = microsecs() + tmp - 3; // wake time
			while ((microsecs() - tmp) >= RECENT) { } // busy wait
			DISPATCH();
		}
		task->status = waiting_micros;
		task->wakeTime = (microsecs() + tmp) - 10; // adjusted for approximate scheduler overhead
		goto suspend;
	waitMillis_op:
	 	tmp = evalInt(*(sp - 1)); // wait time in usecs
	 	POP_ARGS_COMMAND();
	 	if (tmp <= 0) { DISPATCH(); } // don't wait at all
	 	if (tmp > 3600000) {
	 		fail(waitTooLong);
	 		goto error;
	 	}
		task->status = waiting_micros;
		task->wakeTime = microsecs() + ((1000 * tmp) - 10);
		goto suspend;
	sendBroadcast_op:
		primSendBroadcast(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	recvBroadcast_op:
		POP_ARGS_COMMAND(); // pop the broadcast name (a literal string)
		DISPATCH();
	stopAllButThis_op:
		stopAllTasksButThis(task); // clears all tasks except the current one
		DISPATCH();
	forLoop_op:
		// stack layout:
		// *(sp - 1) the loop counter (decreases from N to 1); falseObj the very first time
		// *(sp - 2) N, the total loop count or item count of a list, string or byte array
		// *(sp - 3) the object being iterated over: an integer, list, string, or byte array

		tmpObj = *(sp - 1); // loop counter, or falseObj the very first time
		if (falseObj == tmpObj) { // first time: compute N, the total iterations (in tmp)
			tmpObj = *(sp - 3);
			if (isInt(tmpObj)) {
				tmp = obj2int(tmpObj);
			} else if (IS_TYPE(tmpObj, ListType)) {
				tmp = obj2int(FIELD(tmpObj, 0));
			} else if (IS_TYPE(tmpObj, StringType)) {
				tmp = countUTF8(obj2str(tmpObj));
			} else if (IS_TYPE(tmpObj, ByteArrayType)) {
				tmp = BYTES(tmpObj);
			} else {
				fail(badForLoopArg);
				goto error;
			}
			*(sp - 2) = int2obj(tmp); // save N, the total iterations; tmp is initial loop counter
		} else { // not the first time
			tmp = obj2int(tmpObj) - 1; // decrement the loop counter (in tmp)
		}
		if (tmp > 0) { // loop counter > 0
			*(sp - 1) = int2obj(tmp); // store the loop counter
			tmp = obj2int(*(sp - 2)) - tmp; // set tmp to the loop index (increasing from 0 to N-1)
			tmpObj = *(sp - 3); // set tmpObj to thing being iterated over
			if (isInt(tmpObj)) {
				// set the index variable to the loop index
				*(fp + arg) = int2obj(tmp + 1); // add 1 to get range 1..N
			} else if (IS_TYPE(tmpObj, ListType)) {
				// set the index variable to the next list item
				*(fp + arg) = FIELD(tmpObj, tmp + 1); // skip count field
			} else if (IS_TYPE(tmpObj, StringType)) {
				// set the index variable to the next character of a string
				*(fp + arg) = charAt(tmpObj, tmp + 1);
			} else if (IS_TYPE(tmpObj, ByteArrayType)) {
				// set the index variable to the next byte of a byte array
				*(fp + arg) = int2obj(((uint8 *) &FIELD(tmpObj, 0))[tmp]);
			} else {
				fail(badForLoopArg);
				goto error;
			}
		} else { // loop counter <= 0
			ip++; // skip the following jmp instruction thus ending the loop
		}
		DISPATCH();
	initLocals_op:
		// Reserve stack space for 'arg' locals initialized to zero
		STACK_CHECK(arg);
		while (arg-- > 0) *sp++ = zeroObj;
		DISPATCH();
	getArg_op:
		// For variadic functions. Unlike pushVar, the argument index is passed on the stack.
		STACK_CHECK(1);
		if (IN_CALL()) {
			tmp = evalInt(*(sp - 1));
			if ((1 <= tmp) && (tmp <= obj2int(*(fp - 3)))) { // if arg index in range:
				*(sp - arg) = *(fp - obj2int(*(fp - 3)) - 4 + tmp);
			} else {
				fail(argIndexOutOfRange);
			}
		} else {
			fail(notInFunction);
		}
		POP_ARGS_REPORTER();
		DISPATCH();
	getLastBroadcast_op:
		*(sp - arg) = lastBroadcast;
		POP_ARGS_REPORTER();
		DISPATCH();
	jmpOr_op:
		// if true, jump leaving true (result of "or" expression) on stack, otherwise pop
		if (trueObj == *(sp - 1)) { ip += arg; } else { sp--; }
		DISPATCH();
	jmpAnd_op:
		// if not true, push false (result of "and" expression) on stack and jump
		if (trueObj != (*--sp)) { // treat any value but true as false
			*sp++ = falseObj;
			ip += arg;
		}
		DISPATCH();

	// For the primitive ops below, arg is the number of arguments (any primitive can be variadic).
	// Commands pop all their arguments.
	// Reporters pop all their arguments and leave a result on the top of the stack.
	minimum_op:
		*(sp - arg) = primMinimum(arg, sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	maximum_op:
		*(sp - arg) = primMaximum(arg, sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	lessThan_op:
		tmpObj = *(sp - 2);
		if (isInt(tmpObj) && isInt(*(sp - 1))) { // special case for integers:
			*(sp - arg) = (obj2int(tmpObj) < obj2int(*(sp - 1))) ? trueObj : falseObj;
		} else {
			*(sp - arg) = primCompare(-2, tmpObj, *(sp - 1));
		}
		POP_ARGS_REPORTER();
		DISPATCH();
	lessOrEq_op:
		tmpObj = *(sp - 2);
		if (isInt(tmpObj) && isInt(*(sp - 1))) { // special case for integers:
			*(sp - arg) = (obj2int(tmpObj) <= obj2int(*(sp - 1))) ? trueObj : falseObj;
		} else {
			*(sp - arg) = primCompare(-1, tmpObj, *(sp - 1));
		}
		POP_ARGS_REPORTER();
		DISPATCH();
	equal_op:
		tmpObj = *(sp - 2);
		if (tmpObj == *(sp - 1)) { // identical objects
			*(sp - arg) = trueObj;
		} else if (tmpObj <= trueObj) {
			*(sp - arg) = falseObj; // boolean, not equal
		} else if (isInt(tmpObj) && isInt(*(sp - 1))) {
			*(sp - arg) = falseObj; // integer, not equal
		} else if (IS_TYPE(tmpObj, StringType) && IS_TYPE(*(sp - 1), StringType)) {
			*(sp - arg) = (stringsEqual(tmpObj, *(sp - 1)) ? trueObj : falseObj);
		} else {
			*(sp - arg) = falseObj; // not comparable, so not equal
		}
		POP_ARGS_REPORTER();
		DISPATCH();
	notEqual_op:
		tmpObj = *(sp - 2);
		if (tmpObj == *(sp - 1)) { // identical objects
			*(sp - arg) = falseObj;
		} else if (tmpObj <= trueObj) {
			*(sp - arg) = trueObj; // boolean, not equal
		} else if (isInt(tmpObj) && isInt(*(sp - 1))) {
			*(sp - arg) = trueObj; // integer, not equal
		} else if (IS_TYPE(tmpObj, StringType) && IS_TYPE(*(sp - 1), StringType)) {
			*(sp - arg) = (stringsEqual(tmpObj, *(sp - 1)) ? falseObj : trueObj);
		} else {
			*(sp - arg) = trueObj; // not comparable, so not equal
		}
		POP_ARGS_REPORTER();
		DISPATCH();
	greaterOrEq_op:
		tmpObj = *(sp - 2);
		if (isInt(tmpObj) && isInt(*(sp - 1))) { // special case for integers:
			*(sp - arg) = (obj2int(tmpObj) >= obj2int(*(sp - 1))) ? trueObj : falseObj;
		} else {
			*(sp - arg) = primCompare(1, tmpObj, *(sp - 1));
		}
		POP_ARGS_REPORTER();
		DISPATCH();
	greaterThan_op:
		tmpObj = *(sp - 2);
		if (isInt(tmpObj) && isInt(*(sp - 1))) { // special case for integers:
			*(sp - arg) = (obj2int(tmpObj) > obj2int(*(sp - 1))) ? trueObj : falseObj;
		} else {
			*(sp - arg) = primCompare(2, tmpObj, *(sp - 1));
		}
		POP_ARGS_REPORTER();
		DISPATCH();
	not_op:
		*(sp - arg) = (trueObj == *(sp - 1)) ? falseObj : trueObj;
		POP_ARGS_REPORTER();
		DISPATCH();
	add_op:
		*(sp - arg) = int2obj(evalInt(*(sp - 2)) + evalInt(*(sp - 1)));
		POP_ARGS_REPORTER();
		DISPATCH();
	subtract_op:
		*(sp - arg) = int2obj(evalInt(*(sp - 2)) - evalInt(*(sp - 1)));
		POP_ARGS_REPORTER();
		DISPATCH();
	multiply_op:
		*(sp - arg) = int2obj(evalInt(*(sp - 2)) * evalInt(*(sp - 1)));
		POP_ARGS_REPORTER();
		DISPATCH();
	divide_op:
		tmp = evalInt(*(sp - 1));
		*(sp - arg) = ((0 == tmp) ? fail(zeroDivide) : int2obj(evalInt(*(sp - 2)) / tmp));
		POP_ARGS_REPORTER();
		DISPATCH();
	modulo_op:
		*(sp - arg) = primModulo(arg, sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	absoluteValue_op:
		*(sp - arg) = int2obj(abs(evalInt(*(sp - 1))));
		POP_ARGS_REPORTER();
		DISPATCH();
	random_op:
		*(sp - arg) = primRandom(arg, sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	hexToInt_op:
		*(sp - arg) = primHexToInt(arg, sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();

	// bit operations:
	bitAnd_op:
		*(sp - arg) = int2obj(evalInt(*(sp - 2)) & evalInt(*(sp - 1)));
		POP_ARGS_REPORTER();
		DISPATCH();
	bitOr_op:
		*(sp - arg) = int2obj(evalInt(*(sp - 2)) | evalInt(*(sp - 1)));
		POP_ARGS_REPORTER();
		DISPATCH();
	bitXor_op:
		*(sp - arg) = int2obj(evalInt(*(sp - 2)) ^ evalInt(*(sp - 1)));
		POP_ARGS_REPORTER();
		DISPATCH();
	bitInvert_op:
		*(sp - arg) = int2obj(~evalInt(*(sp - 1)));;
		POP_ARGS_REPORTER();
		DISPATCH();
	bitShiftLeft_op:
		*(sp - arg) = int2obj(evalInt(*(sp - 2)) << evalInt(*(sp - 1)));
		POP_ARGS_REPORTER();
		DISPATCH();
	bitShiftRight_op:
		*(sp - arg) = int2obj(evalInt(*(sp - 2)) >> evalInt(*(sp - 1)));
		POP_ARGS_REPORTER();
		DISPATCH();
	longMultiply_op:
		{
			long long product = (long long) (evalInt(*(sp - 3))) * (long long) (evalInt(*(sp - 2)));
			tmp = (int) ((product >> (evalInt(*(sp - 1)))) & 0xFFFFFFFF);
			*(sp - arg) = int2obj(tmp);
		}
		POP_ARGS_REPORTER();
		DISPATCH();
	// list operations:
	newList_op:
		*(sp - arg) = primNewList(arg, sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	fillList_op:
		primFillList(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	at_op:
		*(sp - arg) = primAt(arg, sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	atPut_op:
		primAtPut(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	length_op:
		*(sp - arg) = primLength(arg, sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	// miscellaneous operations:
	isType_op:
		{
			char *type = obj2str(*(sp - 1));
			switch (objType(*(sp - 2))) {
				case BooleanType:
					*(sp - arg) = strcmp(type, "boolean") == 0 ? trueObj : falseObj;
					break;
				case IntegerType:
					*(sp - arg) = strcmp(type, "number") == 0 ? trueObj : falseObj;
					break;
				case StringType:
					*(sp - arg) = strcmp(type, "string") == 0 ? trueObj : falseObj;
					break;
				case ListType:
					*(sp - arg) = strcmp(type, "list") == 0 ? trueObj : falseObj;
					break;
				case ByteArrayType:
					*(sp - arg) = strcmp(type, "byte array") == 0 ? trueObj : falseObj;
					break;
			}
		}
		POP_ARGS_REPORTER();
		DISPATCH();
	millis_op:
		STACK_CHECK(1);
		*sp++ = int2obj(millisecs());
		DISPATCH();
	micros_op:
		STACK_CHECK(1);
		*sp++ = int2obj(microsecs() & 0x3FFFFFFF); // low 30-bits so result is positive
		DISPATCH();
	timer_op:
		STACK_CHECK(1);
		*sp++ = int2obj(timer());
		DISPATCH();
	resetTimer_op:
		resetTimer();
		POP_ARGS_COMMAND();
		DISPATCH();
	sayIt_op:
		if (!ideConnected()) {
			POP_ARGS_COMMAND(); // serial port not open; do nothing
			DISPATCH();
		}
		printArgs(arg, sp - arg, true, true);
		if (!hasOutputSpace(printBufferByteCount + 100)) { // leave room for other messages
			ip--; // retry when task is resumed
			goto suspend;
		}
		sendSayForChunk(printBuffer, printBufferByteCount, task->taskChunkIndex);
		POP_ARGS_COMMAND();
		// wait for data to be sent; prevents use in tight loop from clogging serial line
		task->status = waiting_micros;
		task->wakeTime = microsecs() + (extraByteDelay * (printBufferByteCount + 6));
		goto suspend;
	logData_op:
		if (!ideConnected()) {
			POP_ARGS_COMMAND(); // serial port not open; do nothing
			DISPATCH();
		}
		printArgs(arg, sp - arg, false, true);
		if (!hasOutputSpace(printBufferByteCount + 100)) { // leave room for other messages
			ip--; // retry when task is resumed
			goto suspend;
		}
		logData(printBuffer);
		POP_ARGS_COMMAND();
		// wait for data to be sent; prevents use in tight loop from clogging serial line
		task->status = waiting_micros;
		task->wakeTime = microsecs() + (extraByteDelay * (printBufferByteCount + 6));
		goto suspend;
	boardType_op:
		*(sp - arg) = primBoardType();
		POP_ARGS_REPORTER();
		DISPATCH();
	comment_op:
		POP_ARGS_COMMAND();
		DISPATCH();
	argOrDefault_op:
		if (arg < 2) {
			*(sp - arg) = fail(notEnoughArguments); // not enough arguments to primitive
		} else if (fp <= task->stack) {
			*(sp - arg) = *(sp - 1); // not in a function call; return default value
		} else {
			*(sp - arg) = argOrDefault(fp, obj2int(*(sp - 2)), *(sp - 1));
		}
		POP_ARGS_REPORTER();
		DISPATCH();

	// I/O operations:
	analogPins_op:
		*(sp - arg) = primAnalogPins(sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	digitalPins_op:
		*(sp - arg) = primDigitalPins(sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	analogRead_op:
		*(sp - arg) = primAnalogRead(arg, sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	analogWrite_op:
		primAnalogWrite(sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	digitalRead_op:
		*(sp - arg) = primDigitalRead(arg, sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	digitalWrite_op:
		primDigitalWrite(sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	digitalSet_op:
		// no args to pop; pin number is encoded in arg field of instruction
		primDigitalSet(arg, true);
		DISPATCH();
	digitalClear_op:
		// no args to pop; pin number is encoded in arg field of instruction
		primDigitalSet(arg, false);
		DISPATCH();
	buttonA_op:
		*(sp - arg) = primButtonA(sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	buttonB_op:
		*(sp - arg) = primButtonB(sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	setUserLED_op:
		primSetUserLED(sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	i2cSet_op:
		primI2cSet(sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	i2cGet_op:
		*(sp - arg) = primI2cGet(sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	spiSend_op:
		primSPISend(sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	spiRecv_op:
		*(sp - arg) = primSPIRecv(sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();

	// micro:bit operations:
	mbDisplay_op:
		primMBDisplay(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	mbDisplayOff_op:
		primMBDisplayOff(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	mbPlot_op:
		primMBPlot(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	mbUnplot_op:
		primMBUnplot(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	mbTiltX_op:
		*(sp - arg) = primMBTiltX(arg, sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	mbTiltY_op:
		*(sp - arg) = primMBTiltY(arg, sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	mbTiltZ_op:
		*(sp - arg) = primMBTiltZ(arg, sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	mbTemp_op:
		*(sp - arg) = primMBTemp(arg, sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	neoPixelSend_op:
		primNeoPixelSend(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	drawShape_op:
		primMBDrawShape(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	shapeForLetter_op:
		*(sp - arg) = primMBShapeForLetter(arg, sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
	neoPixelSetPin_op:
		primNeoPixelSetPin(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	hatchlingPlayNote_op: // Adding the Hatchling delay-based built-in primitives
		*(sp - arg) = primHatchlingPlayNote(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	hatchlingPlayTone_op: 
		*(sp - arg) = primHatchlingPlayTone(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	hatchlingServoWithDelay_op:
		*(sp - arg) = primHatchlingServoWithDelay(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	hatchlingMotorWithDelay_op: 
		*(sp - arg) = primHatchlingMotorWithDelay(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	hatchlingFairyLightWithDelay_op: 
		*(sp - arg) = primHatchlingFairyLightWithDelay(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	hatchlingNeopixelWithDelay_op: 
		*(sp - arg) = primNeopixelWithDelay(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	hatchlingNeopixelStripBuiltIn_op: 
		*(sp - arg) = primNeopixelStripBuiltIn(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();		
	hatchlingDisplayText_op: 
		*(sp - arg) = primHLDisplayText(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();		

	// call a function using the function name and parameter list:
	callCustomCommand_op:
	callCustomReporter_op:
		if (arg > 0) {
			taskSleep(-1); // do background VM tasks sooner
			uint32 callee = -1;
			OBJ params = *(sp - 1); // save the parameters array, if any
			// look up the function or primitive name
			if ((arg == 1) && (IS_TYPE(*(sp - 1), StringType))) {
				callee = findCallee(obj2str(*(sp - 1)));
			} else if ((arg == 2) && (IS_TYPE(*(sp - 2), StringType))) {
				callee = findCallee(obj2str(*(sp - 2)));
			}
			POP_ARGS_COMMAND();
			if (callee != -1) { // found a callee
				int paramCount = 0;
				if (arg == 2) { // has an optional parameters list (the second argument)
					if (IS_TYPE(params, ListType)) { // push the parameters onto the stack
						paramCount = (obj2int(FIELD(params, 0)) & 0xFF);
						for (int i = 1; i <= paramCount; i++) {
							*sp++ = FIELD(params, i);
						}
					} else { // fail: parameters must be a list
						*sp++ = fail(needsListError);
						DISPATCH();
					}
				}

				// invoke the callee
				task->sp = sp - task->stack; // record the stack pointer in case callee does a GC
				if ((callee & 0xFFFFFF00) == 0xFFFFFF00) { // callee is a MicroBlocks function (i.e. a chunk index)
					arg = ((callee & 0xFF) << 8) | paramCount;
					goto callFunction_op;
				} else { // callee is a named primitive (i.e. a pointer to a C function)
					tmpObj = ((PrimitiveFunction) callee)(paramCount, sp - paramCount); // call the primitive
					tempGCRoot = NULL; // clear tempGCRoot in case it was used
					sp -= paramCount;
					*sp++ = tmpObj; // push primitive return value
					DISPATCH();
				}
			}
		}
		// failed: bad arguments
		*sp++ = falseObj; // push a dummy return value
		DISPATCH();

	// named primitives:
	callCommandPrimitive_op:
		callPrimitive(arg, sp - arg);
		POP_ARGS_COMMAND();
		DISPATCH();
	callReporterPrimitive_op:
		*(sp - arg) = callPrimitive(arg, sp - arg);
		POP_ARGS_REPORTER();
		DISPATCH();
}

// Task Scheduler

static int currentTaskIndex = -1;

// Sets the fancy name to use in vmLoop
/*void setFancyName(const char *nameFromMac)
{
	fancyName[0] = nameFromMac[0];
	fancyName[2] = nameFromMac[1];
	fancyName[4] = nameFromMac[2];
}*/
void vmLoop() {
	// Run the next runnable task. Wake up any waiting tasks whose wakeup time has arrived.

	int count = 0;
	int wrapCounter = 0; // Counts the number of clock wraps
//	int i = 0;
//	int initialCount = 0;
	uint8 hlData[8]; // Array to hold Hatchling sensor data and port states
    uint32 timeToSPI = microsecs();
//	uint32 timeToChange = millis();
	uint32 timeToTransmit = millis();
	//uint32 stopTime = timeToTransmit + 30000;
//	uint32 startTime = millis();
	int soundTime = 100;
//	int timeOut = 15000; // The Hatchling will stop displaying its initials and color code after 15 seconds (or earlier if BLE gets connected)
	bool isBLEConnect = false;
//	bool prevBLEConnect = false;
//	bool advertisingTimeOver = false; 
	OBJ tone_args[2];
	tone_args[0] = int2obj(-1);
	// Arguments for stopping a port if it has been activated using Level 1 blocks
	/*OBJ motor_args[2];
	OBJ fairy_args[2];
	OBJ neopixel_args[4];
	OBJ neopixel_strip_args[5];*/

	uint8_t noteState = 255;
	
	// No need to change this every time - we always used the internal buzzer, which is pin -1, and we always want to turn off the ports, so these should be 0
	/*tone_args[0] = int2obj(-1);
	motor_args[1] = int2obj(0);
	fairy_args[1] = int2obj(0);
	neopixel_args[1] = int2obj(0);
	neopixel_args[2] = int2obj(0);
	neopixel_args[3] = int2obj(0);
	neopixel_strip_args[1] = newStringFromBytes((const char *) "all", 3);
	neopixel_strip_args[2] = int2obj(0);
	neopixel_strip_args[3] = int2obj(0);
	neopixel_strip_args[4] = int2obj(0);*/



	while (true) {
		if (count-- < 0) {
			// do background VM tasks once every N VM loop cycles
		// With a single loop that turns on/off the user LED and the LED display every 333 us, I see:
		// Total loop time (from one high edge to the next high edge) varies between 60 and 150 us
		// With an empty loop it is 22 us
		// Uptime with an empty loop is 5.2 us
		// Uptime with a program varies from 5 us to 25 us


			// Send our sensor data/port states every 500 ms if we're connected over BLE
			if(millis()-timeToTransmit > 500 && isBLEConnected())
			{
				//queueByte(252);
				getHatchlingData(hlData);
			/*	for(i = 0; i < 13; i++)
				{
					queueByte(hlData[i]);
				}*/
				// Send port states and other HL sensors if connected over Bluetooth
				sendBroadcastToIDE(hlData, 8); 
				timeToTransmit = millis();
			}
			// In this case, we have just connected, so set up the notes to play the connection sound and reset the clap and button counters
			if(isBLEConnected() && isBLEConnect == false)
			{
				isBLEConnect=true;
				soundTime = millis(); 
				timeToTransmit = millis()-500; // Start sending sensor data upon connection
				noteState = 0;			
				// This effectively resets the clap and button presses when we connect
				getClaps();
				getButtonPresses();
			}
			// In this case, set up the notes to play the disconnection sound
			else if(!isBLEConnected() && isBLEConnect == true)
			{
				isBLEConnect = false;
				soundTime = millis(); 
				noteState = 10;
				// Play the first note of the disconnection sound
				tone_args[1] = int2obj(587);
				primPlayTone(2, tone_args);
			}

			// Play the connection sound
			if(noteState == 0 && millis()-soundTime > 200)
			{
				// Play the next note
				soundTime = millis();
				tone_args[1] = int2obj(329);
				primPlayTone(2, tone_args);
				noteState++;
			}
			else if(noteState == 1 && millis()-soundTime > 100)
			{
				// Play the next note
				soundTime = millis();
				tone_args[1] = int2obj(523);
				primPlayTone(2, tone_args);
				noteState++;
			}
			else if(noteState == 2 && millis()-soundTime > 100)
			{
				//Play the third note
				soundTime = millis();
				tone_args[1] = int2obj(587);
				primPlayTone(2, tone_args);
				noteState++;
			}
			else if(noteState == 3 && millis()-soundTime > 100)
			{
				// Play the next note
				soundTime = millis();
				tone_args[1] = int2obj(740);
				primPlayTone(2, tone_args);
				noteState++;
			}
			else if(noteState == 4 && millis()-soundTime > 100)
			{
				//Turn off the buzzer
				tone_args[1] = int2obj(0);
				primPlayTone(2, tone_args);
				noteState=255;
			}

			// Play the remainder of the disconnection sound
			if(noteState == 10 && millis()-soundTime > 100)
			{
				// Play the next note
				soundTime = millis();
				tone_args[1] = int2obj(494);
				primPlayTone(2, tone_args);
				noteState++;
			}
			else if(noteState == 11 && millis()-soundTime > 100)
			{
				//Play the third note
				soundTime = millis();
				tone_args[1] = int2obj(392);
				primPlayTone(2, tone_args);
				noteState++;
			}
			else if(noteState == 12 && millis()-soundTime > 100)
			{
				//Play the third note
				soundTime = millis();
				tone_args[1] = int2obj(262);
				primPlayTone(2, tone_args);
				noteState++;
			}
			else if(noteState == 13 && millis()-soundTime > 100)
			{
				//Turn off the buzzer
				tone_args[1] = int2obj(0);
				primPlayTone(2, tone_args);
				noteState=255;
			}

			// turn off currently playing note, if any
			if (hatchlingNoteIsPlaying && (microsecs() > hatchlingNoteEndTime)) {
				tone_args[1] = int2obj(0);
				primPlayTone(2, tone_args);
				hatchlingNoteIsPlaying = false;
			}
			
			// Need to increment the display to the next character if we are displaying text

			// Turn off any ports that need to be turned off
			for(int pinNum = 0; pinNum < 6; pinNum++)
			{
				if(portActive[pinNum] && (microsecs() > portEndTime[pinNum])) {
					switch(portType[pinNum]) {
						// We don't depower the servo
						case SERVO:
							portActive[pinNum] = false;
							break;				
						case MOTOR:
						case FAIRY:
						case NEOPXL:
							stopHLPort(pinNum);
							portActive[pinNum] = false;
							break;
						case NEOPXL_STRIP:
						// Nothing needs to happen since we don't use time delay here
							break;
						default:
							break;
					}
				}
			}

			// If we're scrolling text, display the next character
			if(textDisplaying == true && (microsecs() > nextDisplayTime))
			{
				OBJ displayArgs[3];
				// If we've reached the end of the string, stop displaying
				if(charCounter >= textLength)
				{
					free(textToDisplay); // Free the memory used by the scrolling text
					textDisplaying = false;
					primMBDisplayOff(0,displayArgs);
				}
				else
				{
					textDisplayPos--;
					
					if(textDisplayPos == -5)
					{
						textDisplayPos = 0;
						charCounter++;
					}
					// Draw the current character exiting the screen
					displayArgs[0] = newStringFromBytes(&textToDisplay[charCounter], 1);
					OBJ letterShape = primMBShapeForLetter(1, displayArgs);

					displayArgs[0] = letterShape;
					displayArgs[1] = int2obj(textDisplayPos); // Displays the letter in x = 0 (or less), y = 1 position to start scrolling out. 
					displayArgs[2] = int2obj(1);
					primMBDrawShape(3,displayArgs);
					// Draw the next character entering the screen - multiple primMBDrawShapes can be called on the display
					// Only do this if there's a character to display
					if(charCounter < textLength-1)
					{
						displayArgs[0] = newStringFromBytes(&textToDisplay[charCounter+1], 1);
						letterShape = primMBShapeForLetter(1, displayArgs);

						displayArgs[0] = letterShape;
						displayArgs[1] = int2obj(textDisplayPos+5); // Displays the letter in x = 5 (or less), y = 1 position to start scrolling it. 
						displayArgs[2] = int2obj(1);
						primMBDrawShape(3,displayArgs);
					}
					nextDisplayTime = microsecs() + DISPLAYTIME; // Updates every 100ms
				}
			}

			// Checking for clock wrap around - basically if timeToSPI is greater than microsecs, then we need to reset all of our timing due to a clock wrap
			if(timeToSPI > microsecs())
			{
				wrapCounter++;
				// If we've wrapped four times, then we've been on for 72 minutes * 4 - time to power down Hatchling
				if(wrapCounter > 3)
					turnOffHatchling();

				timeToSPI = microsecs();   // Might cause up to a 5000 us delay every 72 minutes, but reading with a 10 ms delay instead of 5 is immaterial
				timeToTransmit = millis(); // Might cause up to a 1/2 second delay in reporting sensors every 72 minutes, immaterial
				if(textDisplaying)
					nextDisplayTime = microsecs(); // This will trigger an update for the letter if currently printing, might be visible as a tiny glitch
				
				if(noteState != 255)
					soundTime = millis(); // This may cause one note to play for up to 200 ms instead of 100

				if(hatchlingNoteIsPlaying)
					hatchlingNoteEndTime = microsecs(); // If a note is playing, this will turn it off in the next loop. This may result in an abbreviated note every 72 minutes

				// Reset time for any ports that are - may result in an abbreviated movement every 72 minutes
				for(int pinNum = 0; pinNum < 6; pinNum++)
				{
					if(portActive[pinNum])
						portEndTime[pinNum] = microsecs();
				}

			}

		// Check if it is time to get a sensor reading from Hatchling to check port states, attached sensor values
		// Do this every 5 ms
			if((microsecs() - timeToSPI) > 5000) //&& advertisingTimeOver)
			{
				//pinMode(1, OUTPUT);
				//digitalWrite(1, HIGH);
				readHatchlingSensors(); // This function currently takes 260 us
				timeToSPI = microsecs(); // Update time
				//digitalWrite(1, LOW);
			}


			updateMicrobitDisplay();
			checkButtons(); // Checks if a button is pressed every 10 ms, starts when button hat blocks if appropriate
			
			// Only do this if you're not making sound right now, or haven't made sound within the last 100 ms
			if(microsecs() > hatchlingNoteEndTime+100000)
			{
				checkClaps(); // Checks if there has been a clap every 2.5 ms, starts hat blocks if appropriate, this function takes around 25 us. 
			}
		// Could add check accelerometer for shake, checking for claps, etc here - TOM NOTE
			processMessage();

			count = 25; // must be under 30 when building on mbed to avoid serial errors
		}

		// Execute all of the tasks in the VM
		//if(advertisingTimeOver)
		//{
		int runCount = 0;
		uint32 usecs = 0; // compute times only the first time they are needed
		for (int t = 0; t < taskCount; t++) {
			currentTaskIndex++;
			if (currentTaskIndex >= taskCount) currentTaskIndex = 0;
			Task *task = &tasks[currentTaskIndex];
			if (unusedTask == task->status) {
				continue;
			} else if (running == task->status) {
				runTask(task);
				runCount++;
				break;
			} else if (waiting_micros == task->status) {
				if (!usecs) usecs = microsecs(); // get usecs
				if ((usecs - task->wakeTime) < RECENT) task->status = running;
			}
			if (running == task->status) {
				runTask(task);
				runCount++;
				break;
			}
		}
		//}
	}
}
