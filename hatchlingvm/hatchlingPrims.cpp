/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

// Copyright 2019 John Maloney, Bernat Romagosa, and Jens Mönig

// hatchlingPrims.c - Miscellaneous primitives
// Tom Lauwers, May 2023

#include <Arduino.h> // Need to include when using many Arduino functions
#include <SPI.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "mem.h"
#include "interp.h"

static int spiSpeed = 1000000;
static int spiMode = SPI_MODE0;

#define GP_PORT_TOTAL 6    // Number of ports on Hatchling
#define TOTAL_POSSIBLE_ACCESSORIES 26 // Number of possible different Hatchling component types

#define SPICMDLENGTH 21 // Length of SPI command to get all sensor values and we need to pad other commands to this size
#define PORTCMDLENGTH 7   // Length of SPI command for setting port states
#define SETPORTCMDLENGTH 19 // Length of SPI cmd for setting port values
#define SETPORTNEOPXLSTRIP 14 // Length of SPI cmd for setting neopixel strip on one port

// Command codes for various SPI settings
#define HATCHLING_SET_ONBOARD_LEDS                0xE0
#define HATCHLING_SET_PORT_STATES                 0xE1
#define HATCHLING_SET_GP_PORTS                    0xE2
#define HATCHLING_SET_EXTERNAL_PXL                0xE3
#define HATCHLING_READ_SENSORS                    0xDE
#define HATCHLING_POWEROFF_SAMD                   0xD6  // This will power off the device - used for battery save mode
#define HATCHLING_STOPALL                         0xDF

#define MAX_BRIGHTNESS 0x40 // Defines the maximum brightness of onboard LEDs
#define MIN_BRIGHTNESS 0x0A // Defines a dim LED

//GP Port States
#define ROTATION_SERVO					 1             //Rotation servo signals
#define POSITION_SERVO					 6             //Position servo signals
#define NEOPXL_SINGLE                    2             //Single Neopixel
#define NEOPXL_STRIP                     3             //Strip of Neopixel
#define ANALOG_SENSOR                    4             //Analog input
#define DIGITAL_OUT                      5             //Software PWM Enabled Digital Output
#define PORTOFF                          0             //Configured as digital input, start up state

#define FAIRY_LIGHTS_ID         8

// Hatchling battery thresholds
#define HL_FULL_BATT                                 228                           //All leds green
#define HL_BATT_THRESH1                              211							//Three yellow leds below this
#define HL_BATT_THRESH2                              194	

#define SPI_DELAY                                    500                           // Determines how long to wait after an SPI command for the Hatchling to execute on the command. 
                                                                                   // Potential improvement - only do this if two SPI commands are sent immediately one after the other 

// Port ID values - basically, what raw analog sensor reading corresponds to what specific component?
static const uint8_t id_values[TOTAL_POSSIBLE_ACCESSORIES] = {255, 10, 21, 32, 44, 56, 66, 77, 87, 97, 107, 118, 128, 138, 147, 156, 165, 174, 183, 192, 201, 210, 219, 228, 236, 245};


// Arrays for some pre-set colors (in order: red, orange, yellow, green, teal, blue, purple, white) - used for making the LED color code
static const uint8_t ledcolors[8][3] = {{MAX_BRIGHTNESS, 0, 0}, {MAX_BRIGHTNESS, MAX_BRIGHTNESS/3, 0}, {MAX_BRIGHTNESS, MAX_BRIGHTNESS, 0}, {0, MAX_BRIGHTNESS, 0},
                                        {0, MAX_BRIGHTNESS, MAX_BRIGHTNESS}, {0, 0, MAX_BRIGHTNESS}, {MAX_BRIGHTNESS, 0, MAX_BRIGHTNESS}, {MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS}};

uint8 hatchlingSPISensors[SPICMDLENGTH];

uint8 GP_ID_vals[GP_PORT_TOTAL] = {31, 31, 31, 31, 31, 31};
uint8 GP_ID_vals_compare[GP_PORT_TOTAL] = {31, 31, 31, 31, 31, 31};
uint16_t Filtered_ID_Vals[GP_PORT_TOTAL] = {0, 0, 0, 0, 0, 0};
//uint8 Port_lock[6] = {0, 0, 0, 0, 0, 0}; // If we've identified that a component is attached, lock that port so it only changes if it first goes through an "unplugged" situation
bool Port_lock[6] = {false, false, false, false, false, false}; // If we've identified that a component is attached, lock that port so it only changes if it first goes through an "unplugged" situation


// Since any time you set one port you have to set all ports, create a port values command array that holds the current setting of all ports
uint8 PortValuesCommand[SPICMDLENGTH] = {HATCHLING_SET_GP_PORTS, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// Need a different command to control a Neopixel strip, and we need 6 of these to store up to six potential strip states
uint8 PixelStripCommand[6][SPICMDLENGTH];// = {HATCHLING_SET_EXTERNAL_PXL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// Holds the port states that we have passed on to the hatchling
uint8 setPortCommands[SPICMDLENGTH] = {HATCHLING_SET_PORT_STATES, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool sendUpdateCommand = false; // Used to determine if we need to send an port state update via SPI
bool checkPortSetting = false; // We use this to indicate if we need to check if the ports are set correctly
uint8 stabilizeCounter = 0;

uint8 GPSensorValues[GP_PORT_TOTAL] = {0, 0, 0, 0, 0, 0};

static void initSPI() {
	setPinMode(PIN_SPI_MISO, INPUT);
	setPinMode(PIN_SPI_SCK, OUTPUT);
	setPinMode(PIN_SPI_MOSI, OUTPUT);
	SPI.begin();
	SPI.beginTransaction(SPISettings(spiSpeed, MSBFIRST, spiMode));
}

void getHatchlingData(uint8 *hlData)
{
    int i = 0;
    hlData[0] = 252; // All other returns start with 250 or 251

    // Copy the port states into the array
    for(i= 1; i < GP_PORT_TOTAL+1; i++)
    {
        hlData[i] = GP_ID_vals[i-1];

        // Temporary hack to control neopixel strips from Hatchling app
        if(hlData[i] == 10)
            hlData[i] = 9;
    }  
    hlData[7] = hatchlingSPISensors[20];
}

void setPortsViaSPI()
{
    pinMode(16, OUTPUT);
    digitalWrite(16, LOW);

    initSPI(); // Does begin transaction in there
    // Transfer the command that sets the values of all of the ports
    SPI.transfer(HATCHLING_SET_GP_PORTS);
    for (int i = 1; i < SPICMDLENGTH; i++) {
        SPI.transfer(PortValuesCommand[i]);
    }
    SPI.endTransaction();
    
    digitalWrite(16, HIGH);    
    delayMicroseconds(SPI_DELAY); //Give Hatchling time to execute an SPI command 
}

// Helper function used with turning off the port on a time delay
void stopHLPort(int pinNum)
{
    PortValuesCommand[pinNum*3 + 1] = 0;
    PortValuesCommand[pinNum*3 + 2] = 0;
    PortValuesCommand[pinNum*3 + 3] = 0;
    setPortsViaSPI();
}

// Primitives

// Function to control attached Fairy Lights
OBJ primFairyLights(int argCount, OBJ *args) {
	if (!isInt(args[1])) { fail(needsIntegerError); return args[1]; }
    if (!IS_TYPE(args[0], StringType)) return fail(needsStringError);
    int ch = obj2str(args[0])[0];
    int pinNum = -1; // default value: invalid port
    if (('A' <= ch) && (ch <= 'F')) pinNum = ch - 'A';
    if (('a' <= ch) && (ch <= 'f')) pinNum = ch - 'a';

	if (pinNum == -1)
    {
        return args[0];
    }
	int value = obj2int(args[1]);
	if (value < 0) value = 0;
	if (value > 255) value = 255;

    // Only do the rest if you have fairy lights attached on that port
    if(GP_ID_vals[pinNum] == FAIRY_LIGHTS_ID) 
    {
        // Set the appropriate part of the command 
        PortValuesCommand[pinNum*3 + 1] = 0;
        PortValuesCommand[pinNum*3 + 2] = 0;
        PortValuesCommand[pinNum*3 + 3] = value;

        setPortsViaSPI();
	    return trueObj;
    }
    return falseObj;
}

// Function to set the tempo
OBJ primSetTempo(int argCount, OBJ *args) {
	if (!isInt(args[0])) { fail(needsIntegerError); return args[0]; }
 
	int value = obj2int(args[0]);
    // This is the tempo in beats per minute
	if (value < 20) value = 20;
	if (value > 2000) value = 2000;

    setTempo(value);
    return trueObj;
 }

 // Function to get the tempo
OBJ primGetTempo(int argCount, OBJ *args) {
	int result = getTempo();
	return int2obj(result);
 }


// Function to control attached position servo
OBJ primPositionServos(int argCount, OBJ *args) {
	if (!isInt(args[1])) { fail(needsIntegerError); return args[1]; }

    if (!IS_TYPE(args[0], StringType)) return fail(needsStringError);
    int ch = obj2str(args[0])[0];
    int pinNum = -1; // default value: invalid port
    if (('A' <= ch) && (ch <= 'F')) pinNum = ch - 'A';
    if (('a' <= ch) && (ch <= 'f')) pinNum = ch - 'a';

	if (pinNum == -1)
    {
        return args[0];
    }
	int value = obj2int(args[1])+2; // Adjust it because to send a 0 or 1 = turning it off

	if (value > 255) value = 255;

    // Only do the rest if you have a servo attached on that port
    if(GP_ID_vals[pinNum] > 0 && GP_ID_vals[pinNum] < 7) 
    {
        // Set the appropriate part of the command 
        PortValuesCommand[pinNum*3 + 1] = 0;
        PortValuesCommand[pinNum*3 + 2] = 0;
        PortValuesCommand[pinNum*3 + 3] = value;

        setPortsViaSPI();
	    return trueObj;
    }
    return falseObj;
}


// Function to control attached rotation servo
OBJ primRotationServos(int argCount, OBJ *args) {
	if (!isInt(args[1])) { fail(needsIntegerError); return args[1]; }

    if (!IS_TYPE(args[0], StringType)) return fail(needsStringError);
    int ch = obj2str(args[0])[0];
    int pinNum = -1; // default value: invalid port
    if (('A' <= ch) && (ch <= 'F')) pinNum = ch - 'A';
    if (('a' <= ch) && (ch <= 'f')) pinNum = ch - 'a';

	if (pinNum == -1)
    {
        return args[0];
    }
	int value = obj2int(args[1]);
	if (value < -100) value = -100;
	if (value > 100) value = 100;


    // Only do the rest if you have a servo attached on that port
    if(GP_ID_vals[pinNum] > 0 && GP_ID_vals[pinNum] < 7) 
    {
        // Convert value to something useful for motors. 116 and 64 are where the motor starts moving
        if(value > 0)
        {
            value = 116 + value*59/100; //Basically set the range to 5 to 175 to avoid sending a power off command (0, 1, or 2)

        }
        else if(value < 0)
        {
            value = 64 + value*59/100; //Basically set the range to 5 to 175 to avoid sending a power off command (0, 1, or 2)    
        }        

        // Convert value to something useful for rotation servos
        /*if(value < 5 && value > -5) // create a dead zone
        { 
            value = 0;
        }
        else
        {
            value = value/3 + 91; // 91 is the midpoint of our servo command - need to double check this - this is the code for rotation servos
        }*/
        // Set the appropriate part of the command 
        PortValuesCommand[pinNum*3 + 1] = 0;
        PortValuesCommand[pinNum*3 + 2] = 0;
        PortValuesCommand[pinNum*3 + 3] = value;

        setPortsViaSPI();
	    return trueObj;
    }
    return falseObj;
}


// Function to control attached neopixel LED
OBJ primNeoPixel(int argCount, OBJ *args) {
	if (!isInt(args[1]) || !isInt(args[2]) || !isInt(args[3])) { fail(needsIntegerError); return falseObj; }
    if (!IS_TYPE(args[0], StringType)) return fail(needsStringError);
    int ch = obj2str(args[0])[0];
    int pinNum = -1; // default value: invalid port
    if (('A' <= ch) && (ch <= 'F')) pinNum = ch - 'A';
    if (('a' <= ch) && (ch <= 'f')) pinNum = ch - 'a';

	if (pinNum == -1)
    {
        return args[0];
    }
    // Red and Green are switched on later LEDs, so red = arg 2, green = arg 1. I have not done this for the Neopixel strip
    int greenVal = obj2int(args[1])*255/100;
	if (greenVal < 0) greenVal = 0;
	if (greenVal > 255) greenVal = 255;	

	int redVal = obj2int(args[2])*255/100;
	if (redVal < 0) redVal = 0;
	if (redVal > 255) redVal = 255;
   
    int blueVal = obj2int(args[3])*255/100;
	if (blueVal < 0) blueVal = 0;
	if (blueVal > 255) blueVal = 255;
    // Only do the rest if you have a neopixel attached on that port
    if(GP_ID_vals[pinNum] == 7 || GP_ID_vals[pinNum] == 9) 
    {
        // Set the appropriate part of the command 
        PortValuesCommand[pinNum*3 + 1] = redVal;
        PortValuesCommand[pinNum*3 + 2] = greenVal;
        PortValuesCommand[pinNum*3 + 3] = blueVal;

        setPortsViaSPI();
	    return trueObj;
    }

    
    // Only do the rest if you have a neopixel strip attached on that port
    if(GP_ID_vals[pinNum] == 10) 
    {
        // Set the first two bytes of the array
        PixelStripCommand[pinNum][0] = HATCHLING_SET_EXTERNAL_PXL;
        PixelStripCommand[pinNum][1] = pinNum;       

        // Set all LEDs for now    
        for(uint8 i = 0; i < 4; i++)
        {
            PixelStripCommand[pinNum][i*3+2] = redVal;
            PixelStripCommand[pinNum][i*3+3] = greenVal;
            PixelStripCommand[pinNum][i*3+4] = blueVal;
        }
        
        // Else we set the LED at the position only
        /*else
        {
            int LEDnum = obj2int(args[1])-1; // Converting from 1-4 to 0-3
            if(LEDnum >= 0 && LEDnum <= 3)
            {
                PixelStripCommand[pinNum][LEDnum*3+2] = redLedValue;
                PixelStripCommand[pinNum][LEDnum*3+3] = greenLedValue;
                PixelStripCommand[pinNum][LEDnum*3+4] = blueLedValue;
            }
            // If it's not between 1 and 4 we return the value as an error
            else
            {
                return args[1];
            }
        }*/
        // Set the neopixel strip
        pinMode(16, OUTPUT);
        digitalWrite(16, LOW);

        initSPI(); // Does begin transaction in there
        for (int i = 0; i < SPICMDLENGTH; i++) {
            SPI.transfer(PixelStripCommand[pinNum][i]);
        }
        SPI.endTransaction();
            
        digitalWrite(16, HIGH);
        delayMicroseconds(SPI_DELAY); //Give Hatchling time to execute an SPI command 
	    return trueObj;
    }

    return falseObj;
}

// Function to control attached neopixel LED Strip
OBJ primNeoPixelStrip(int argCount, OBJ *args) {
	for(uint8 i = 2; i < argCount; i++)
    {
        if (!isInt(args[i])) { fail(needsIntegerError); return args[i]; }
    }
    if (!IS_TYPE(args[0], StringType)) return fail(needsStringError);
    int ch = obj2str(args[0])[0];
    int pinNum = -1; // default value: invalid port
    if (('A' <= ch) && (ch <= 'F')) pinNum = ch - 'A';
    if (('a' <= ch) && (ch <= 'f')) pinNum = ch - 'a';

	if (pinNum == -1)
    {
        return args[0];
    }

    // Only do the rest if you have a neopixel strip attached on that port
    if(GP_ID_vals[pinNum] == 10) 
    {
        // Set the first two bytes of the array
        PixelStripCommand[pinNum][0] = HATCHLING_SET_EXTERNAL_PXL;
        PixelStripCommand[pinNum][1] = pinNum;       

        int redLedValue = obj2int(args[2])*255/100;
        if (redLedValue < 0) redLedValue = 0;
        if (redLedValue > 255) redLedValue = 255;
        int greenLedValue = obj2int(args[3])*255/100;
        if (greenLedValue < 0) greenLedValue = 0;
        if (greenLedValue > 255) greenLedValue = 255;
        int blueLedValue = obj2int(args[4])*255/100;
        if (blueLedValue < 0) blueLedValue = 0;
        if (blueLedValue > 255) blueLedValue = 255;
        // If the position argument is a string, then we set all LEDs
        if (IS_TYPE(args[1], StringType))
        {            
            for(uint8 i = 0; i < 4; i++)
            {
                PixelStripCommand[pinNum][i*3+2] = redLedValue;
                PixelStripCommand[pinNum][i*3+3] = greenLedValue;
                PixelStripCommand[pinNum][i*3+4] = blueLedValue;
            }
        }
        // Else we set the LED at the position only
        else
        {
            int LEDnum = obj2int(args[1])-1; // Converting from 1-4 to 0-3
            if(LEDnum >= 0 && LEDnum <= 3)
            {
                PixelStripCommand[pinNum][LEDnum*3+2] = redLedValue;
                PixelStripCommand[pinNum][LEDnum*3+3] = greenLedValue;
                PixelStripCommand[pinNum][LEDnum*3+4] = blueLedValue;
            }
            // If it's not between 1 and 4 we return the value as an error
            else
            {
                return args[1];
            }
        }
        // Set the neopixel strip
        pinMode(16, OUTPUT);
        digitalWrite(16, LOW);

        initSPI(); // Does begin transaction in there
        for (int i = 0; i < SPICMDLENGTH; i++) {
            SPI.transfer(PixelStripCommand[pinNum][i]);
        }
        SPI.endTransaction();
            
        digitalWrite(16, HIGH);
        delayMicroseconds(SPI_DELAY); //Give Hatchling time to execute an SPI command 
	    return trueObj;
    }
    return falseObj;
}

OBJ primHatchlingRead(int argCount, OBJ *args) {

	int wordCount = (SPICMDLENGTH + 3) / 4; // Needs to round to an even number (multiple of 4)
	// Allocates a byte array of 6 words
	OBJ result = newObj(ByteArrayType, wordCount, falseObj);
	
	// Checks if memory allocation was successful
	if (!result) return fail(insufficientMemoryError);

	// Sets the byte length to 21 instead of 24 (4*6)
	setByteCountAdjust(result, SPICMDLENGTH);
	uint8 *data = (uint8*) &FIELD(result, 0); // Creates a pointer to the first payload byte

    // Places the (already read via SPI) hatchling sensor report packet into something that can be reported in MicroBlocks
    for(uint8 i = 0; i < SPICMDLENGTH; i++)
    {
        data[i] = hatchlingSPISensors[i];
    }   
	return result;
}

OBJ primDistanceSensor(int argCount, OBJ *args) {

    if (!IS_TYPE(args[0], StringType)) return fail(needsStringError);
    int ch = obj2str(args[0])[0];
    int pinNum = -1; // default value: invalid port
    if (('A' <= ch) && (ch <= 'F')) pinNum = ch - 'A';
    if (('a' <= ch) && (ch <= 'f')) pinNum = ch - 'a';

	if (pinNum == -1)
    {
        return args[0];
    }
    // Return a value only if we have a distance sensor attached
    if(GP_ID_vals[pinNum] == 14) 
    {	
        return int2obj(GPSensorValues[pinNum]);
    }
    return int2obj(0);//falseObj;
}

OBJ primLightSensor(int argCount, OBJ *args) {

    if (!IS_TYPE(args[0], StringType)) return fail(needsStringError);
    int ch = obj2str(args[0])[0];
    int pinNum = -1; // default value: invalid port
    if (('A' <= ch) && (ch <= 'F')) pinNum = ch - 'A';
    if (('a' <= ch) && (ch <= 'f')) pinNum = ch - 'a';

	if (pinNum == -1)
    {
        return args[0];
    }
    // Return a value only if we have a distance sensor attached
    if(GP_ID_vals[pinNum] == 20) 
    {	
        return int2obj(GPSensorValues[pinNum]);
    }
    return int2obj(0);//falseObj;
}

OBJ primBigButton(int argCount, OBJ *args) {

    if (!IS_TYPE(args[0], StringType)) return fail(needsStringError);
    int ch = obj2str(args[0])[0];
    int pinNum = -1; // default value: invalid port
    if (('A' <= ch) && (ch <= 'F')) pinNum = ch - 'A';
    if (('a' <= ch) && (ch <= 'f')) pinNum = ch - 'a';

	if (pinNum == -1)
    {
        return args[0];
    }
    // Return a value only if we have a distance sensor attached
    if(GP_ID_vals[pinNum] == 17) 
    {	
        return int2obj(GPSensorValues[pinNum]);
    }
    return int2obj(0);//falseObj;
}

// This is just for debugging for now
OBJ primPortState(int argCount, OBJ *args) {

    if (!IS_TYPE(args[0], StringType)) return fail(needsStringError);
    int ch = obj2str(args[0])[0];
    int pinNum = -1; // default value: invalid port
    if (('A' <= ch) && (ch <= 'F')) pinNum = ch - 'A';
    if (('a' <= ch) && (ch <= 'f')) pinNum = ch - 'a';

	if (pinNum == -1)
    {
        return args[0];
    }
    // Return a value only if we have a distance sensor attached
    return int2obj(Filtered_ID_Vals[pinNum]);
}



OBJ primLoudness(int argCount, OBJ *args) {
	// Read a measure of ambient loudness of the environment from the microphone.

	int result = getLoudness();
	return int2obj(result);
}

OBJ primClaps(int argCount, OBJ *args) {
	// Read the number of claps since the last time we read the number of claps

	int result = getClaps();
	return int2obj(result);
}

OBJ primButtons(int argCount, OBJ *args) {
	// Read the number of button presses since the last time we read the number of button presses

	int result = getButtonPresses();
	return int2obj(result);
}


// Gets called from vmLoop approximately once every 5 ms. Initiates a read transaction and then organizes the data in all of the appropriate spots
void readHatchlingSensors() {

    uint8_t GP_ID_vals_new[6] = {31, 31, 31, 31, 31, 31}; // Initialize each value to 31

    pinMode(16, OUTPUT);
    digitalWrite(16, LOW);

	initSPI(); // Does begin transaction in there
    hatchlingSPISensors[0] = SPI.transfer(0xDE);
	for (int i = 1; i < SPICMDLENGTH; i++) {
		hatchlingSPISensors[i] = SPI.transfer(0xFF);
    }
	SPI.endTransaction();
    
    digitalWrite(16, HIGH);
    // Place the sensor values from attached components into one array
    for(int i = 0; i < GP_PORT_TOTAL; i++)
    {
        GPSensorValues[i] = hatchlingSPISensors[i*2+3];
    }

    // Now go through the ID values and filter them, then assign them a component type
    for(int i=14;i<20;i++)
    {   
        Filtered_ID_Vals[i-14] = (Filtered_ID_Vals[i-14]*2+hatchlingSPISensors[i])/3; // Infinite time filter to reduce noise
        // If nothing is plugged in, the value should be 255
        if(Filtered_ID_Vals[i-14] > 250)
        {
            GP_ID_vals_new[i-14] = 0;
        } 
        else {
            for(int j = 1; j < TOTAL_POSSIBLE_ACCESSORIES; j++)
            {
                if((Filtered_ID_Vals[i-14] < (id_values[j] + 7)) && (Filtered_ID_Vals[i-14] > (id_values[j] -7)))
                {
                    GP_ID_vals_new[i-14] = j;
                    break; // break out of inner loop only
                }
            }
        }
    }

    // If any of the ports have changed, we'll need to tell the SAMD microcontroller to update their state
    for(int i = 0; i < GP_PORT_TOTAL; i++)
    {
        if(GP_ID_vals_new[i]!=GP_ID_vals_compare[i])
        {
            stabilizeCounter = 0;
        }
        GP_ID_vals_compare[i] = GP_ID_vals_new[i];
    }

    // Basically wait 20 iterations (about 100 ms) for values to stabilize. Might need to adjust this 
    if(stabilizeCounter < 20)
    {
        stabilizeCounter++;
    }
    // If you've waited twenty cycles for values to stabilize, time to update the Hatchling port states
    else if(stabilizeCounter == 20)
    {   
        stabilizeCounter++; // Prevents this from being used again                

        for(int i=0; i < GP_PORT_TOTAL; i++)
        {
            // Only update the state of the port if it has changed
            if(GP_ID_vals[i] != GP_ID_vals_compare[i])
            {
                // Only update the state if the port is not locked
                // To unlock a port the value needs to go to 0 (nothing plugged in) before turning back into something. 
                if(GP_ID_vals_compare[i] == 0 || GP_ID_vals_compare[i] == 31)
                {
                    Port_lock[i] = false; // Unlock the port if nothing is plugged in
                    GP_ID_vals[i] = GP_ID_vals_compare[i]; // Update the value sent to the tablet/device
                    sendUpdateCommand = true; // Make sure to update the Hatchling daughter controller so it knows to update the port to a "port off" state
                }
                // If the port is unlocked, you can update it to the new component that is plugged into it
                else if(Port_lock[i] == false)
                {
                    GP_ID_vals[i] = GP_ID_vals_compare[i];
                    sendUpdateCommand = true;
                    Port_lock[i] = true; // Lock the port since we are setting it
                }
            }
        }
        // If we're updating any of the ports, then we need to set all of the ports
        if(sendUpdateCommand)
        {
            memset(setPortCommands, 0, SPICMDLENGTH);
            setPortCommands[0] = HATCHLING_SET_PORT_STATES;
            for(int i=0; i < GP_PORT_TOTAL; i++)
            {            
                // Update the state of the port
                if((GP_ID_vals[i] ==1) || (GP_ID_vals[i] == 2))
                {
                    setPortCommands[i+1] = ROTATION_SERVO;
                }
                else if((GP_ID_vals[i] > 2) && (GP_ID_vals[i] < 7))
                {
                    setPortCommands[i+1] = POSITION_SERVO;
                }
                else if((GP_ID_vals[i] == 7) || (GP_ID_vals[i] == 9))
                {
                    setPortCommands[i+1] = NEOPXL_SINGLE;
                }
                else if((GP_ID_vals[i] == 10) || (GP_ID_vals[i] == 11))
                {
                    setPortCommands[i+1] = NEOPXL_STRIP;
                }
                else if((GP_ID_vals[i] > 11) && (GP_ID_vals[i] < 22))
                {
                    setPortCommands[i+1] = ANALOG_SENSOR;
                }
                else if((GP_ID_vals[i] == 8) || ((GP_ID_vals[i] > 21) && (GP_ID_vals[i] < 26)))
                {
                    setPortCommands[i+1] = DIGITAL_OUT;
                }
                else
                {
                    setPortCommands[i+1] = PORTOFF;
                }
            }
        }
    }
    
    // Do the following to check if the port setting went through correctly - does not seem necessary
    if(checkPortSetting)
    {
        for(int i = 0; i < GP_PORT_TOTAL; i++)
        {
            // If at least one of the commands doesn't agree, we need to set the port states again
            if(setPortCommands[i+1] != hatchlingSPISensors[2*i+2])
            {
                sendUpdateCommand = true; // Basically, send the update port types command again
                outputString("Writing Ports Again"); //Debugging only
            }
        }
        checkPortSetting = false; //If sendUpdateCommand isn't true, this will stay false. Otherwise, it'll be true again in the next cycle. 
    }
            // Only update the settings if at least one port required an update
    if(sendUpdateCommand)
    {
        // Send the update command
        pinMode(16, OUTPUT);
        digitalWrite(16, LOW);

        initSPI(); // Does begin transaction in there
        for (int i = 0; i < SPICMDLENGTH; i++) {
            SPI.transfer(setPortCommands[i]);
        }
        SPI.endTransaction();
        
        digitalWrite(16, HIGH);
        delayMicroseconds(SPI_DELAY); //Give Hatchling time to execute an SPI command    
        checkPortSetting = true;
        sendUpdateCommand = false;
        /*outputString("Updating Ports"); // For debugging
        char portCmds[6];
        for(int i = 0; i < 6; i++)
        {
            portCmds[i] = (char)hatchlingSPISensors[i+14];//*2+2];//setPortCommands[i+1];
        }
        sendBroadcastToIDE(portCmds,6); // For debugging, to see what we just sent to the Hatchling
        */

    }
}

// Sends the stop command to the Hatchling - turns off LEDs and servos
void stopHatchling()
{
    uint8_t stopCommand[SPICMDLENGTH];
    
    memset(stopCommand, 0xFF, SPICMDLENGTH);
    stopCommand[0] = HATCHLING_STOPALL;    pinMode(16, OUTPUT);
    digitalWrite(16, LOW);

    initSPI(); // Does begin transaction in there
    for (int i = 0; i < SPICMDLENGTH; i++) {
        SPI.transfer(stopCommand[i]);
        PortValuesCommand[i] = 0; // Reset the port values command
    }
    SPI.endTransaction();
        
    digitalWrite(16, HIGH);
    memset(GP_ID_vals_compare, 31, GP_PORT_TOTAL); // Resetting the port identifiers to force an onboard LED update when we reconnect
}
static PrimEntry entries[] = {
	{"rd", primHatchlingRead},
    {"fl", primFairyLights},
    {"st", primSetTempo},
    {"gt", primGetTempo},
    {"psv", primPositionServos},
    {"rsv", primRotationServos},
    {"np", primNeoPixel},
    {"nps", primNeoPixelStrip},
    {"ds", primDistanceSensor},
    {"ls", primLightSensor},
    {"bb", primBigButton},
    {"ps", primPortState},
	{"ld", primLoudness},
	{"cl", primClaps},
	{"bt", primButtons},
};

void addHatchlingPrims() {
	addPrimitiveSet("h", sizeof(entries) / sizeof(PrimEntry), entries);
}

