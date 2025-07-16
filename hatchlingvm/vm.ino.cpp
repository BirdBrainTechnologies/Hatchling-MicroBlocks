# 1 "C:\\Users\\TOMLAU~1\\AppData\\Local\\Temp\\tmptoan17y7"
#include <Arduino.h>
# 1 "C:/Users/TomLauwers/OneDrive - RW Thrive/Documents/Hatchling-MicroBlocks/hatchlingvm/vm.ino"






#include "mem.h"
#include "interp.h"
#include "persist.h"
void setup();
void loop();
#line 11 "C:/Users/TomLauwers/OneDrive - RW Thrive/Documents/Hatchling-MicroBlocks/hatchlingvm/vm.ino"
void setup() {
 hardwareInit();
 memInit();
 primsInit();


  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delayMicroseconds(50000);
  delayMicroseconds(50000);
  delayMicroseconds(50000);
  delayMicroseconds(50000);
  digitalWrite(2, LOW);

 outputString((char *) "Welcome to Hatchling MicroBlocks!");
 restoreScripts();
 if (BLE_isEnabled()) BLE_start();

  if(!isBLEConnected())
   startAll();


  pinMode(28, OUTPUT);
  digitalWrite(28, HIGH);


}

void loop() {
 vmLoop();
}