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
//#include "ioPrims.cpp"

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


uint8 stabilizeCounter = 0;

uint8 GPSensorValues[GP_PORT_TOTAL] = {0, 0, 0, 0, 0, 0};

static void initSPI() {
	setPinMode(PIN_SPI_MISO, INPUT);
	setPinMode(PIN_SPI_SCK, OUTPUT);
	setPinMode(PIN_SPI_MOSI, OUTPUT);
	SPI.begin();
	SPI.beginTransaction(SPISettings(spiSpeed, MSBFIRST, spiMode));
}

void setPortsViaSPI()
{
    pinMode(16, OUTPUT);
    digitalWrite(16, LOW);

    initSPI(); // Does begin transaction in there
    // Transfer the command that sets the values of all of the ports
    for (int i = 0; i < SPICMDLENGTH; i++) {
        SPI.transfer(PortValuesCommand[i]);
    }
    SPI.endTransaction();
    
    digitalWrite(16, HIGH);    
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
	int value = obj2int(args[1]);
	if (value < 0) value = 0;
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
        // Convert value to something useful for rotation servos
        value = value/3 + 90;

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
	int redVal = obj2int(args[1])*255/100;
	if (redVal < 0) redVal = 0;
	if (redVal > 255) redVal = 255;
	
    int greenVal = obj2int(args[2])*255/100;
	if (greenVal < 0) greenVal = 0;
	if (greenVal > 255) greenVal = 255;	
    
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
        pinMode(16, OUTPUT);
        digitalWrite(16, LOW);

        initSPI(); // Does begin transaction in there
        for (int i = 0; i < SPICMDLENGTH; i++) {
            SPI.transfer(PixelStripCommand[pinNum][i]);
        }
        SPI.endTransaction();
            
        digitalWrite(16, HIGH);
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
                if((Filtered_ID_Vals[i-14] < (id_values[j] + 5)) && (Filtered_ID_Vals[i-14] > (id_values[j] -5)))
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
        stabilizeCounter++;
        
        uint8 setPortCommands[SPICMDLENGTH];
        
        memset(setPortCommands, 0, SPICMDLENGTH);
        setPortCommands[0] = HATCHLING_SET_PORT_STATES;
        bool sendUpdateCommand = false;
        for(int i=0; i < GP_PORT_TOTAL; i++)
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
            else if(GP_ID_vals[i] == 10)
            {
                setPortCommands[i+1] = NEOPXL_STRIP;
            }
            else if((GP_ID_vals[i] > 10) && (GP_ID_vals[i] < 22))
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


        // Only update the settings if at least one port required an update
        if(sendUpdateCommand)
        {
            //delayMicroseconds(100); // Give the previous SPI transaction time to wrapup, may be unneeded
            pinMode(16, OUTPUT);
            digitalWrite(16, LOW);

            initSPI(); // Does begin transaction in there
            for (int i = 0; i < SPICMDLENGTH; i++) {
                SPI.transfer(setPortCommands[i]);
            }
            SPI.endTransaction();
                
            digitalWrite(16, HIGH);

            // The following code checks if things actually got sent properly - might need to implement something similar
            /*bool settingGood = false;
            while(!settingGood)
            {
                // Send the update command
                setHatchlingPortStates(setPortCommands, HATCHLING_SPI_LENGTH);
                fiber_sleep(5); // It takes 5 ms for the sensor packet to update
                uint8_t spi_readback[HATCHLING_SPI_SENSOR_LENGTH];    
                
                // Check that the Hatchling has successfully received the port settings by reading the sensors
                spiReadHatchling(spi_readback);
                settingGood = true;
                for(i = 0; i < GP_PORT_TOTAL; i++)
                {
                    // If at least one of the commands doesn't agree, we need to set the port states again, then rerun this loop
                    if(setPortCommands[i+1] != spi_readback[2*i+2])
                    {
                        settingGood = false;
                        //uBit.serial.sendChar(0x43); // For debugging
                    }
                }
            }*/
        }
    }
}

static PrimEntry entries[] = {
	{"rd", primHatchlingRead},
    {"fl", primFairyLights},
    {"psv", primPositionServos},
    {"rsv", primRotationServos},
    {"np", primNeoPixel},
    {"nps", primNeoPixelStrip},
    {"ds", primDistanceSensor},
};

void addHatchlingPrims() {
	addPrimitiveSet("h", sizeof(entries) / sizeof(PrimEntry), entries);
}

