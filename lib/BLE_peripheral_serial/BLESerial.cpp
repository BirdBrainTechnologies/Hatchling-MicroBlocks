#include "BLESerial.h"

// #define BLE_SERIAL_DEBUG

// FLUSH_MSECS determines how long to wait before sending a partly full transmit buffer.
// Waiting a bit increases the odds of fitting more data in each packet for efficiency.
#define FLUSH_MSECS 100

BLESerial* BLESerial::_instance = NULL;

BLESerial::BLESerial() :
  BLEPeripheral()
{
  BLESerial::_instance = this;
  this->_rxHead = this->_rxTail = 0;
  this->_txCount = 0;
  this->_lastFlushTime = 0;

  addAttribute(this->_uartService);
  addAttribute(this->_uartNameDescriptor);
  setAdvertisedServiceUuid(this->_uartService.uuid());
  addAttribute(this->_rxCharacteristic);
  addAttribute(this->_rxNameDescriptor);
  this->_rxCharacteristic.setEventHandler(BLEWritten, BLESerial::_received);
  addAttribute(this->_txCharacteristic);
  addAttribute(this->_txNameDescriptor);
}

void BLESerial::begin(...) {
  BLEPeripheral::begin();
  this->_rxHead = this->_rxTail = 0;
  this->_txCount = 0;
  this->_lastFlushTime = 0;
  #ifdef BLE_SERIAL_DEBUG
    Serial.println(F("BLESerial::begin()"));
  #endif
}

void BLESerial::poll() {
  if (millis() < this->_lastFlushTime + FLUSH_MSECS) {
    BLEPeripheral::poll();
  } else {
    this->flush();
  }
}

void BLESerial::end() {
  this->flush();
  this->_rxCharacteristic.setEventHandler(BLEWritten, NULL);
  this->_rxHead = this->_rxTail = 0;
  BLEPeripheral::disconnect();
}

int BLESerial::available(void) {
  BLEPeripheral::poll();
  int retval = (this->_rxHead - this->_rxTail + sizeof(this->_rxBuffer)) % sizeof(this->_rxBuffer);
  #ifdef BLE_SERIAL_DEBUG
    Serial.print(F("BLESerial::available() = "));
    Serial.println(retval);
  #endif
  return retval;
}

int BLESerial::peek(void) {
  BLEPeripheral::poll();
  if (this->_rxTail == this->_rxHead) return -1;
  uint8_t byte = this->_rxBuffer[ (this->_rxTail + 1) % sizeof(this->_rxBuffer)];
  #ifdef BLE_SERIAL_DEBUG
    Serial.print(F("BLESerial::peek() = "));
    Serial.print((char) byte);
    Serial.print(F(" 0x"));
    Serial.println(byte, HEX);
  #endif
  return byte;
}

int BLESerial::read(void) {
  BLEPeripheral::poll();
  if (this->_rxTail == this->_rxHead) return -1;
  this->_rxTail = (this->_rxTail + 1) % sizeof(this->_rxBuffer);
  uint8_t byte = this->_rxBuffer[this->_rxTail];
  #ifdef BLE_SERIAL_DEBUG
    Serial.print(F("BLESerial::read() = "));
    Serial.print((char) byte);
    Serial.print(F(" 0x"));
    Serial.println(byte, HEX);
  #endif
  return byte;
}

void BLESerial::flush(void) {
  if (this->_txCount == 0) return;
  bool dataSent = this->_txCharacteristic.setValue(this->_txBuffer, this->_txCount);
  if (dataSent) {
    this->_lastFlushTime = millis();
    this->_txCount = 0;
    BLEPeripheral::poll();
  } else {
    this->_lastFlushTime = millis() - (FLUSH_MSECS - 5); // try again in a few milliseconds
  }
  #ifdef BLE_SERIAL_DEBUG
    Serial.println(F("BLESerial::flush()"));
  #endif
}

size_t BLESerial:: availableForWrite() {
  this->poll();
  return sizeof(this->_txBuffer) - this->_txCount;
}

size_t BLESerial::write(uint8_t byte) {
  BLEPeripheral::poll();
  if (this->_txCharacteristic.subscribed() == false) return 0;
  this->_txBuffer[this->_txCount++] = byte;
  if (this->_txCount == sizeof(this->_txBuffer)) flush();
  if (this->_txCount != 0) {
    delayMicroseconds(500);
  	return 0; // no data sent
  }
  #ifdef BLE_SERIAL_DEBUG
    Serial.print(F("BLESerial::write("));
    Serial.print((char) byte);
    Serial.print(F(" 0x"));
    Serial.print(byte, HEX);
    Serial.println(F(") = 1"));
  #endif
  return 1;
}

BLESerial::operator bool() {
  bool retval = BLEPeripheral::connected();
  #ifdef BLE_SERIAL_DEBUG
    Serial.print(F("BLESerial::operator bool() = "));
    Serial.println(retval);
  #endif
  return retval;
}

void BLESerial::_received(const uint8_t* data, size_t size) {
  this->rcvCount++;
  for (size_t i = 0; i < size; i++) {
    this->_rxHead = (this->_rxHead + 1) % sizeof(this->_rxBuffer);
    this->_rxBuffer[this->_rxHead] = data[i];
  }
  #ifdef BLE_SERIAL_DEBUG
    Serial.print(F("BLESerial::received("));
    for (int i = 0; i < size; i++) Serial.print((char) data[i]);
    Serial.println(F(")"));
  #endif
}

void BLESerial::_received(BLECentral& /*central*/, BLECharacteristic& rxCharacteristic) {
  BLESerial::_instance->_received(rxCharacteristic.value(), rxCharacteristic.valueLength());
}

void BLESerial::sendTest() {
  // For testing. Fill transmit buffer and send it.

  for (int i = 0; i < sizeof(_txBuffer); i++) {
    this->_txBuffer[i] = 64 + i;
  }
  this->_txCount = sizeof(this->_txBuffer);
  this->flush();
}
