/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

// Copyright 2019 John Maloney, Bernat Romagosa, and Jens Mönig

// miscPrims.c - Miscellaneous primitives
// John Maloney, May 2019

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "mem.h"
#include "interp.h"

OBJ primHexToInt(int argCount, OBJ *args) {
	if (!IS_TYPE(args[0], StringType)) return fail(needsStringError);

	char *s = obj2str(args[0]);
	if ('#' == *s) s++; // skip leading # if there is one
	long result = strtol(s, NULL, 16);
	if ((result < -536870912) || (result > 536870911)) return fail(hexRangeError);
	return int2obj(result);
}

OBJ primRescale(int argCount, OBJ *args) {
	if (argCount < 5) return fail(notEnoughArguments);
	int inVal = evalInt(args[0]);
	int inMin = evalInt(args[1]);
	int inMax = evalInt(args[2]);
	int outMin = evalInt(args[3]);
	int outMax = evalInt(args[4]);

	if (inMax == inMin) return fail(zeroDivide);

	int result = outMin + (((inVal - inMin) * (outMax - outMin)) / (inMax - inMin));
	return int2obj(result);
}

static OBJ primSine(int argCount, OBJ *args) {
	// Returns the sine of the given angle * 2^14 (i.e. a fixed point integer with 13 bits of
	// fraction). The input is the angle in hundreths of a degree (e.g. 4500 means 45 degrees).

	const float hundrethsToRadians = 6.2831853071795864769 / 36000.0;
	return int2obj((int) round(16384.0 * sin(evalInt(args[0]) * hundrethsToRadians)));
}

static OBJ primSqrt(int argCount, OBJ *args) {
	// Returns the integer part of a square root of a given number multiplied by
	// 1000 (e.g. sqrt(2) = 1414).

	return int2obj((int) round(1000 * sqrt(evalInt(args[0]))));
}

static OBJ primArctan(int argCount, OBJ *args) {
	// Returns angle (in hundredths of a degree) of vector dx, dy.

	if (argCount < 2) return fail(notEnoughArguments);
	if (!isInt(args[0]) || !isInt(args[1])) return fail(needsIntegerError);

	double x = obj2int(args[0]);
	double y = obj2int(args[1]);
	double degreeHundredths = (18000.0 * atan2(y, x)) / 3.141592653589793238463;

	return int2obj((int) round(degreeHundredths));
}

static OBJ primPressureToAltitude(int argCount, OBJ *args) {
	// Computes the altitude difference (in millimeters) for a given pressure difference.
	//  dH = 44330 * [ 1 - ( p / p0 ) ^ ( 1 / 5.255) ]

	if (argCount < 2) return fail(notEnoughArguments);
	int p0 = obj2int(args[0]);
	int p = obj2int(args[1]);
	double result = 44330.0 * (1.0 - pow((double) p / p0, (1.0 / 5.255))); // meters
	return int2obj((int) (1000.0 * result)); // return result in millimeters
}

static OBJ primConnectedToIDE(int argCount, OBJ *args) {
	return ideConnected() ? trueObj : falseObj;
}

// Primitives

static PrimEntry entries[] = {
	{"hexToInt", primHexToInt},
	{"rescale", primRescale},
	{"sin", primSine},
	{"sqrt", primSqrt},
	{"atan2", primArctan},
	{"pressureToAltitude", primPressureToAltitude},
	{"connectedToIDE", primConnectedToIDE},
	{"broadcastToIDE", primBroadcastToIDEOnly},
};

void addMiscPrims() {
	addPrimitiveSet("misc", sizeof(entries) / sizeof(PrimEntry), entries);
}
