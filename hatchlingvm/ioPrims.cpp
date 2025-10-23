/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

// Copyright 2018 John Maloney, Bernat Romagosa, and Jens Mönig

// ioPrims.cpp - Microblocks IO primitives and hardware dependent functions
// John Maloney, April 2017

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>

#include "mem.h"
#include "interp.h"

static void initPins(void); // forward reference
static void initRandomSeed(void); // forward reference

// Timing Functions and Hardware Initialization

#if defined(NRF51) || defined(NRF52)

#define USE_NRF5x_CLOCK true

// Both NIMBLE and the Nordic Softdevice BLE systems use Timer0.
// MicroBlocks only supports BLE on the nRF52, we use TIMER1 on nRF52.
// Use TIMER0 on the nRF51, since that is the only 32-bit timer on the nRF51.
#if defined(NRF52)
  #define MB_TIMER NRF_TIMER1
  #define MB_TIMER_IRQn TIMER1_IRQn
  #define MB_TIMER_IRQHandler TIMER1_IRQHandler
#else
  #define MB_TIMER NRF_TIMER0
  #define MB_TIMER_IRQn TIMER0_IRQn
  #define MB_TIMER_IRQHandler TIMER0_IRQHandler
#endif

static void initClock_NRF5x() {
	MB_TIMER->TASKS_SHUTDOWN = true;
	MB_TIMER->MODE = 0; // timer (not counter) mode
	MB_TIMER->BITMODE = 3; // 32-bit
	MB_TIMER->PRESCALER = 4; // 1 MHz (16 MHz / 2^4)
	MB_TIMER->TASKS_START = true;
}

uint32 microsecs() {
	MB_TIMER->TASKS_CAPTURE[0] = true;
	return MB_TIMER->CC[0];
}

uint32 millisecs() {
	// Note: The divide operation makes this slower than microsecs(), so use microsecs()
	// when high-performance is needed. The millisecond clock is effectively only 22 bits
	// so, like the microseconds clock, it wraps around every 72 minutes.

	return microsecs() / 1000;
}

#else // not NRF5x

uint32 microsecs() { return (uint32) micros(); }
uint32 millisecs() { return (uint32) millis(); }

#endif

// Hardware Initialization

void hardwareInit() {
	Serial.begin(115200);
	#ifdef USE_NRF5x_CLOCK
		initClock_NRF5x();
	#endif
	#if defined(ARDUINO_BBC_MICROBIT_V2) || defined(CALLIOPE_V3)
		// Use synthesized LF clock to free up pin the speaker pin (P0.00)
		NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Synth;
		NRF_CLOCK->TASKS_LFCLKSTART = 1;

		// Disable NFC by writing NRF_UICR->NFCPINS to free up pin 8 (P0.10)
		// (this change does not take effect until the next hard reset)
		if (NRF_UICR->NFCPINS & 1) {
			NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen; // enable Flash write
			while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
			NRF_UICR->NFCPINS = 0xFFFFFFFE;
			NRF_NVMC->CONFIG = 0; // disable Flash write
		}
	#endif
	initPins();
	initRandomSeed();
}

// General Purpose I/O Pins

#define BUTTON_PRESSED LOW

#if defined(ARDUINO_BBC_MICROBIT)

	#define BOARD_TYPE "micro:bit"
	#define DIGITAL_PINS 29
	#define ANALOG_PINS 6
	#define TOTAL_PINS DIGITAL_PINS
	static const int analogPin[] = {A0, A1, A2, A3, A4, A5};

	// See variant.cpp in variants/BBCMicrobit folder for a detailed pin map.
	// Pins 0-20 are for micro:bit pads and edge connector
	//	(but pin numbers 17-18 correspond to 3.3 volt pads, not actual I/O pins)
	// Pins 21-22: RX, TX (for USB Serial?)
	// Pins 23-28: COL4, COL5, COL6, ROW1, ROW2, ROW3
	// Button A: pin 5
	// Button B: pin 11
	// Analog pins: The micro:bit does not have dedicated analog input pins;
	// the analog pins are aliases for digital pins 0-4 and 10.

#elif defined(ARDUINO_BBC_MICROBIT_V2)

	#define BOARD_TYPE "micro:bit v2"
	#define DIGITAL_PINS 29
	#define ANALOG_PINS 7
	#define TOTAL_PINS DIGITAL_PINS
	static const int analogPin[] = {A0, A1, A2, A3, A4, A5, A6};
	#define DEFAULT_TONE_PIN 27

#endif

// Board Type

const char * boardType() { return BOARD_TYPE; }

// Pin Modes

// The current pin input/output mode is recorded in the currentMode[] array to
// avoid calling pinMode() unless mode has actually changed. (This speeds up pin I/O.)

#define MODE_NOT_SET (-1)
static char currentMode[TOTAL_PINS];
static char pwmRunning[TOTAL_PINS];

#define SET_MODE(pin, newMode) { \
	if ((newMode) != currentMode[pin]) { \
		pinMode((pin), newMode); \
		currentMode[pin] = newMode; \
	} \
}

int pinCount() { return TOTAL_PINS; }

void setPinMode(int pin, int newMode) {
	// Function to set pin modes from other modules. (The SET_MODE macro is local to this file.)

	SET_MODE(pin, newMode);
}

static void initPins(void) {
	// Initialize currentMode to MODE_NOT_SET (neither INPUT nor OUTPUT)
	// to force the pin's mode to be set on first use.

	#if defined(NRF52)
		// Use 8-bit PWM resolution on NRF52 to improve quaility of audio output.
		// With 8-bit samples, the PWM sampling rate is 16 MHz / 256 = 62500 samples/sec,
		// allowing a 0.1 to 0.33 uF capacitor to be used as a simple low-pass filter.
		// The analog write primitve takes a 10-bit value, as it does on all MicroBlocks boards,
		// but on NRF52 only the 8 most signifcant bits are used.
		analogWriteResolution(8);
	#endif

	for (int i = 0; i < TOTAL_PINS; i++) {
		currentMode[i] = MODE_NOT_SET;
	}
}

void turnOffPins() {
	for (int pin = 0; pin < TOTAL_PINS; pin++) {
		if (OUTPUT == currentMode[pin]) {
			digitalWrite(pin, LOW);
			pinMode(pin, INPUT);
			currentMode[pin] = INPUT;
		}
	}
}

#if defined(ARDUINO_BBC_MICROBIT_V2) || defined(CALLIOPE_V3)

static void setHighDrive(int pin) {
	if ((pin < 0) || (pin >= PINS_COUNT)) return;
	pin = g_ADigitalPinMap[pin];
	NRF_GPIO_Type* port = (NRF_GPIO_Type*) ((pin < 32) ? 0x50000000 : 0x50000300);
	port->PIN_CNF[pin & 0x1F] |= (3 << 8); // high drive 1 and 0
}

#endif

// Pin IO Primitives

OBJ primAnalogPins(OBJ *args) { return int2obj(ANALOG_PINS); }

OBJ primDigitalPins(OBJ *args) { return int2obj(DIGITAL_PINS); }

OBJ primAnalogRead(int argCount, OBJ *args) {
	if (!isInt(args[0])) { fail(needsIntegerError); return int2obj(0); }
	int pinNum = obj2int(args[0]);

	#if defined(ARDUINO_BBC_MICROBIT)
		if (10 == pinNum) pinNum = 5; // map pin 10 to A5
	#elif defined(ARDUINO_BBC_MICROBIT_V2)
		if (6 == pinNum) return int2obj(readAnalogMicrophone()); // A6
		if (10 == pinNum) pinNum = 5; // map pin 10 to A5
		if (29 == pinNum) return int2obj(readAnalogMicrophone());
	#elif defined(ARDUINO_CALLIOPE_MINI)
		if (0 == pinNum) return int2obj(readAnalogMicrophone());
	#elif defined(CALLIOPE_V3)
		if (10 == pinNum) pinNum = 5; // map pin 10 to A5
		if (16 == pinNum) pinNum = 6; // map pin 16 to A6
		if (18 == pinNum) pinNum = 3; // map pin 18 to A3
		if (29 == pinNum) return int2obj(readAnalogMicrophone());
	#endif

	if ((pinNum < 0) || (pinNum >= ANALOG_PINS)) return int2obj(0);
	int pin = analogPin[pinNum];
	#if !defined(ESP8266)
		SET_MODE(pin, INPUT);
		if ((argCount > 1) && (trueObj == args[1])) { pinMode(pin, INPUT_PULLUP); }
	#endif
	return int2obj(analogRead(pin));
}

void primAnalogWrite(OBJ *args) {
	if (!isInt(args[0]) || !isInt(args[1])) { fail(needsIntegerError); return; }
	int pinNum = obj2int(args[0]);
	int value = obj2int(args[1]);
	if (value < 0) value = 0;
	if (value > 1023) value = 1023;
	if ((pinNum < 0) || (pinNum >= TOTAL_PINS)) return;

	int modeChanged = (OUTPUT != currentMode[pinNum]);
	(void)(modeChanged); // reference var to suppress compiler warning

	SET_MODE(pinNum, OUTPUT);
	#if defined(ARDUINO_BBC_MICROBIT_V2)
		if ((27 == pinNum) && modeChanged) setHighDrive(pinNum); // use high drive for speaker
	#endif

	#if defined(NRF52)
		// On NRF52, wait until last PWM cycle is finished before writing a new value.
		// Prevents a tight loop writing audio samples from exceeding the PWM sample rate.
		NRF_PWM_Type* pwm = NRF_PWM0;
		if (pwm->EVENTS_SEQSTARTED[0]) {
			while (!pwm->EVENTS_PWMPERIODEND) /* wait */;
			pwm->EVENTS_PWMPERIODEND = 0;
		}
		value = (value >> 2); // On NRF52, use only the top 8-bits of the 10-bit value
	#endif

	analogWrite(pinNum, value); // sets the PWM duty cycle on a digital pin

	if (OUTPUT == currentMode[pinNum]) { // using PWM, not DAC
		pwmRunning[pinNum] = true;
	}
}

OBJ primDigitalRead(int argCount, OBJ *args) {
	if (!isInt(args[0])) { fail(needsIntegerError); return int2obj(0); }
	int pinNum = obj2int(args[0]);
	if ((pinNum < 0) || (pinNum >= TOTAL_PINS)) return falseObj;
	int mode = INPUT;
	if ((argCount > 1) && (trueObj == args[1])) mode = INPUT_PULLUP;
	SET_MODE(pinNum, mode);
	return (HIGH == digitalRead(pinNum)) ? trueObj : falseObj;
}

void primDigitalWrite(OBJ *args) {
	if (!isInt(args[0])) { fail(needsIntegerError); return; }
	int pinNum = obj2int(args[0]);
	int flag = (trueObj == args[1]);
	primDigitalSet(pinNum, flag);
}

void primDigitalSet(int pinNum, int flag) {
	// This supports a compiler optimization. If the arguments of a digitalWrite
	// are compile-time constants, the compiler can generate a digitalSet or digitalClear
	// instruction, thus saving the cost of pushing the pin number and boolean.
	// (This can make a difference in time-sensitives applications like sound generation.)
	if ((pinNum < 0) || (pinNum >= TOTAL_PINS)) return;

	SET_MODE(pinNum, OUTPUT);
	#if defined(ARDUINO_BBC_MICROBIT_V2)
		if (28 == pinNum) {
			stopPWM();
			setHighDrive(pinNum); // use high drive for microphone
		}
	#endif

	if (pwmRunning[pinNum]) {
		analogWrite(pinNum, (flag ? 1023 : 0));
	} else {
		digitalWrite(pinNum, (flag ? HIGH : LOW));
	}
}

// User LED

void primSetUserLED(OBJ *args) {
	#if defined(HAS_LED_MATRIX)
		// Special case: Plot or unplot one LED in the LED matrix.
		OBJ coords[2] = { int2obj(3), int2obj(1) };
		if (trueObj == args[0]) {
			primMBPlot(2, coords);
		} else {
			primMBUnplot(2, coords);
		}
	#else
		if (PIN_LED < TOTAL_PINS) {
			SET_MODE(PIN_LED, OUTPUT);
		} else {
			pinMode(PIN_LED, OUTPUT);
		}
		int output = (trueObj == args[0]) ? HIGH : LOW;
		#ifdef INVERT_USER_LED
			output = !output;
		#endif
		digitalWrite(PIN_LED, output);
	#endif
}

// User Buttons

OBJ primButtonA(OBJ *args) {
	#ifdef PIN_BUTTON_A
		SET_MODE(PIN_BUTTON_A, INPUT);
		return (BUTTON_PRESSED == digitalRead(PIN_BUTTON_A)) ? trueObj : falseObj;
	#else
		return falseObj;
	#endif
}

OBJ primButtonB(OBJ *args) {
	#ifdef PIN_BUTTON_B
		SET_MODE(PIN_BUTTON_B, INPUT);
		return (BUTTON_PRESSED == digitalRead(PIN_BUTTON_B)) ? trueObj : falseObj;
	#else
		return falseObj;
	#endif
}

// Random number generator seed

static void initRandomSeed() {
	// Initialize the random number generator with a random seed when started (if possible).

	#if defined(NRF51) || defined(NRF52)
		#define RNG_BASE 0x4000D000
		#define RNG_START (RNG_BASE)
		#define RNG_STOP (RNG_BASE + 4)
		#define RNG_VALRDY (RNG_BASE + 0x100)
		#define RNG_VALUE (RNG_BASE + 0x508)
		uint32 seed = 0;
		*((int *) RNG_START) = true; // start random number generation
		for (int i = 0; i < 4; i++) {
			while (!*((volatile int *) RNG_VALRDY)) /* wait */;
			seed = (seed << 8) | *((volatile int *) RNG_VALUE);
			*((volatile int *) RNG_VALRDY) = 0;
		}
		*((int *) RNG_STOP) = true; // end random number generation
		randomSeed(seed);
	#else
		uint32 seed = 0;
		for (int i = 0; i < ANALOG_PINS; i++) {
			int p = analogPin[i];
			pinMode(p, INPUT);
			seed = (seed << 1) ^ analogRead(p);
		}
		randomSeed(seed);
	#endif
}

// Stop PWM

void stopPWM() {
	// Stop hardware PWM. Only implemented on nRF52 boards for now.
	// Unfortunately, the Arduino API doesn't include a way to stop PWM on a pin once it has
	// been started so we'd need to modify the Arduino library code or implement our own
	// version of analogWrite() to reset their internal PWM data structures. Instead, this
	// function simply turns off the PWM hardare on nRF52 boards.

	#if defined(NRF52)
		NRF_PWM0->TASKS_STOP = 1;
		NRF_PWM1->TASKS_STOP = 1;
		NRF_PWM2->TASKS_STOP = 1;
		NRF_PWM0->ENABLE = 0;
		NRF_PWM1->ENABLE = 0;
		NRF_PWM2->ENABLE = 0;
	#endif
}

// Servo and Tone

#if defined(NRF51) || defined(NRF52)

// NRF5 Servo State

#define MAX_SERVOS 8
#define UNUSED 255

static int servoIndex = 0;
static char servoPinHigh = false;
static char servoPin[MAX_SERVOS] = {UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED};
static unsigned short servoPulseWidth[MAX_SERVOS] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// NRF5 Tone State

static int tonePin = -1;
static unsigned short toneHalfPeriod = 1000;
static char tonePinState = 0;
static char servoToneTimerStarted = 0;

static void startServoToneTimer() {
	// enable timer interrupts
	NVIC_EnableIRQ(MB_TIMER_IRQn);

	// get current timer value
 	MB_TIMER->TASKS_CAPTURE[0] = true;
 	uint32_t wakeTime = MB_TIMER->CC[0];

	// set initial wake times a few (at least 2) usecs in the future to kick things off
	MB_TIMER->CC[2] = wakeTime + 5;
	MB_TIMER->CC[3] = wakeTime + 5;

	// enable interrrupts on CC[2] and CC[3]
	MB_TIMER->INTENSET = TIMER_INTENSET_COMPARE2_Msk | TIMER_INTENSET_COMPARE3_Msk;

	servoToneTimerStarted = true;
}

extern "C" void MB_TIMER_IRQHandler() {
	if (MB_TIMER->EVENTS_COMPARE[2]) { // tone waveform generator (CC[2])
		uint32_t wakeTime = MB_TIMER->CC[2];
		MB_TIMER->EVENTS_COMPARE[2] = 0; // clear interrupt
		if (tonePin >= 0) {
			tonePinState = !tonePinState;
			digitalWrite(tonePin, tonePinState);
			MB_TIMER->CC[2] = (wakeTime + toneHalfPeriod); // next wake time
		}
	}

	if (MB_TIMER->EVENTS_COMPARE[3]) { // servo waveform generator (CC[3])
		uint32_t wakeTime = MB_TIMER->CC[3] + 12;
		MB_TIMER->EVENTS_COMPARE[3] = 0; // clear interrupt

		if (servoPinHigh && (0 <= servoIndex) && (servoIndex < MAX_SERVOS)) {
			digitalWrite(servoPin[servoIndex], LOW); // end the current servo pulse
		}

		// find the next active servo
		servoIndex = (servoIndex + 1) % MAX_SERVOS;
		while ((servoIndex < MAX_SERVOS) && (UNUSED == servoPin[servoIndex])) {
			servoIndex++;
		}

		if (servoIndex < MAX_SERVOS) { // start servo pulse for servoIndex
			digitalWrite(servoPin[servoIndex], HIGH);
			servoPinHigh = true;
			MB_TIMER->CC[3] = (wakeTime + servoPulseWidth[servoIndex]);
		} else { // idle until next set of pulses
			servoIndex = -1;
			servoPinHigh = false;
			MB_TIMER->CC[3] = (wakeTime + 18000);
		}
	}
}

void stopServos() {
	for (int i = 0; i < MAX_SERVOS; i++) {
		servoPin[i] = UNUSED;
		servoPulseWidth[i] = 1500;
	}
	servoPinHigh = false;
	servoIndex = 0;
}

static void nrfDetachServo(int pin) {
	for (int i = 0; i < MAX_SERVOS; i++) {
		if (pin == servoPin[i]) {
			servoPulseWidth[i] = 1500;
			servoPin[i] = UNUSED;
		}
	}
}

void setServo(int pin, int usecs) {
	if (!servoToneTimerStarted) startServoToneTimer();

	if (usecs <= 0) { // turn off servo
		nrfDetachServo(pin);
		return;
	}

	for (int i = 0; i < MAX_SERVOS; i++) {
		if (pin == servoPin[i]) { // update the pulse width for the given pin
			servoPulseWidth[i] = usecs;
			return;
		}
	}

	for (int i = 0; i < MAX_SERVOS; i++) {
		if (UNUSED == servoPin[i]) { // found unused servo entry
			servoPin[i] = pin;
			servoPulseWidth[i] = usecs;
			return;
		}
	}
}

#else // use Arduino Servo library

#include <Servo.h>
Servo servo[TOTAL_PINS];

static void setServo(int pin, int usecs) {
	if (usecs <= 0) {
		if (servo[pin].attached()) servo[pin].detach();
	} else {
		if (!servo[pin].attached()) {
			// allow a wide range of pulse widths; MicroBlocks library imposes its own limits
			servo[pin].attach(pin, 200, 3000);
		}
		servo[pin].writeMicroseconds(usecs);
	}
}

void stopServos() {
	for (int pin = 0; pin < TOTAL_PINS; pin++) {
		if (servo[pin].attached()) servo[pin].detach();
	}
}

#endif // servos

// Tone Generation

#if defined(NRF51) || defined(NRF52)

static void setTone(int pin, int frequency) {
	tonePin = pin;
	toneHalfPeriod = 500000 / frequency;
	if (!servoToneTimerStarted) {
		startServoToneTimer();
	}
	MB_TIMER->TASKS_CAPTURE[2] = true;
	MB_TIMER->CC[2] = (MB_TIMER->CC[2] + toneHalfPeriod); // set next wakeup time
}

void stopTone() { tonePin = -1; }

#else // use Arduino Tone functions

int tonePin = -1;

static void setTone(int pin, int frequency) {
	if (pin != tonePin) stopTone();
	tonePin = pin;
	tone(tonePin, frequency);
}

void stopTone() {
	if (tonePin >= 0) {
		noTone(tonePin);
		SET_MODE(tonePin, INPUT);
	}
	tonePin = -1;
}

#endif // tone

// Tone Primitives

#ifndef DEFAULT_TONE_PIN
	#define DEFAULT_TONE_PIN 0
#endif

OBJ primHasTone(int argCount, OBJ *args) {
	#if defined(ARDUINO_SAM_DUE)
		return falseObj;
	#else
		return trueObj;
	#endif
}

OBJ primPlayTone(int argCount, OBJ *args) {
	// playTone <pin> <freq>
	// If freq > 0, generate a 50% duty cycle square wave of the given frequency
	// on the given pin. If freq <= 0 stop generating the square wave.
	// Return true on success, false if primitive is not supported.

	OBJ pinArg = args[0];
	OBJ freqArg = args[1];
	if (!isInt(pinArg) || !isInt(freqArg)) return falseObj;
	int pin = obj2int(pinArg);
	if ((pin < 0) || (pin >= DIGITAL_PINS)) pin = DEFAULT_TONE_PIN;

	SET_MODE(pin, OUTPUT);
	int frequency = obj2int(freqArg);
	if ((frequency < 16) || (frequency > 100000)) {
		stopTone();
		digitalWrite(pin, LOW);
	} else {
		setTone(pin, frequency);
	}
	return trueObj;
}

OBJ primHasServo(int argCount, OBJ *args) { return trueObj; }

OBJ primSetServo(int argCount, OBJ *args) {
	// setServo <pin> <usecs>
	// If usecs > 0, generate a servo control signal with the given pulse width
	// on the given pin. If usecs <= 0 stop generating the servo signal.
	// Return true on success, false if primitive is not supported.

	OBJ pinArg = args[0];
	OBJ usecsArg = args[1];
	if (!isInt(pinArg) || !isInt(usecsArg)) return falseObj;
	int pin = obj2int(pinArg);
	if ((pin < 0) || (pin >= DIGITAL_PINS)) return falseObj;
	int usecs = obj2int(usecsArg);
	if (usecs > 5000) { usecs = 5000; } // maximum pulse width is 5000 usecs
	SET_MODE(pin, OUTPUT);
	setServo(pin, usecs);
	return trueObj;
}

// Software serial (output only)

#if !defined(__not_in_flash_func)
  #define __not_in_flash_func(funcName) funcName
#endif

static OBJ __not_in_flash_func(primSoftwareSerialWriteByte)(int argCount, OBJ *args) {
	// Write a byte to the given pin at the given baudrate using software serial.

	if (argCount < 3) return fail(notEnoughArguments);
	int byte = evalInt(args[0]);
	int pinNum = evalInt(args[1]);
	int baud = evalInt(args[2]);
	int bitTime = 1000000 / baud;

	// adjust the bitTime for slower cpu's
	#if defined(ARDUINO_ARCH_SAMD)
		bitTime -= 3;
	#elif defined(NRF51) || defined(ESP8266) || defined(ARDUINO_SAM_DUE) || defined(RP2040_PHILHOWER)
		bitTime -= 2;
	#else
		bitTime -= 1;
	#endif

	if ((pinNum < 0) || (pinNum >= TOTAL_PINS)) return falseObj;
	SET_MODE(pinNum, OUTPUT);

	// start bit
	digitalWrite(pinNum, LOW);
	delayMicroseconds(bitTime);

	// eight data bits, LSB first
	digitalWrite(pinNum, (byte & 1) ? HIGH : LOW);
	delayMicroseconds(bitTime);
	digitalWrite(pinNum, (byte & 2) ? HIGH : LOW);
	delayMicroseconds(bitTime);
	digitalWrite(pinNum, (byte & 4) ? HIGH : LOW);
	delayMicroseconds(bitTime);
	digitalWrite(pinNum, (byte & 8) ? HIGH : LOW);
	delayMicroseconds(bitTime);
	digitalWrite(pinNum, (byte & 16) ? HIGH : LOW);
	delayMicroseconds(bitTime);
	digitalWrite(pinNum, (byte & 32) ? HIGH : LOW);
	delayMicroseconds(bitTime);
	digitalWrite(pinNum, (byte & 64) ? HIGH : LOW);
	delayMicroseconds(bitTime);
	digitalWrite(pinNum, (byte & 128) ? HIGH : LOW);
	delayMicroseconds(bitTime);

	// stop bit
	digitalWrite(pinNum, HIGH);
	delayMicroseconds(bitTime);

	return falseObj;
}

// forward to primitives that don't take argCount

static OBJ primSetUserLED2(int argCount, OBJ *args) { primSetUserLED(args); return falseObj; }
static OBJ primAnalogWrite2(int argCount, OBJ *args) { primAnalogWrite(args); return falseObj; }
static OBJ primDigitalWrite2(int argCount, OBJ *args) { primDigitalWrite(args); return falseObj; }

static PrimEntry entries[] = {
	{"hasTone", primHasTone},
	{"playTone", primPlayTone},
	{"hasServo", primHasServo},
	{"setServo", primSetServo},
	{"softWriteByte", primSoftwareSerialWriteByte},
	{"setUserLED", primSetUserLED2},
	{"analogRead", primAnalogRead},
	{"analogWrite", primAnalogWrite2},
	{"digitalRead", primDigitalRead},
	{"digitalWrite", primDigitalWrite2},
};

void addIOPrims() {
	addPrimitiveSet("io", sizeof(entries) / sizeof(PrimEntry), entries);
}
