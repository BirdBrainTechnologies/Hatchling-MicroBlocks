#ifndef _BLE_SERIAL_H_
#define _BLE_SERIAL_H_

#include <Arduino.h>
#include <BLEPeripheral.h>

#define RX_BUFFER_SIZE 2000

class BLESerial : public BLEPeripheral, public Stream
{
  public:
    BLESerial();

    void begin(...);
    const char* getLocalName();
    void poll();
    void end();

    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    virtual void flush(void);
    virtual size_t availableForWrite();
    virtual size_t write(uint8_t byte);
    using Print::write;
    virtual operator bool();
    int rcvCount = 0;

	virtual void sendTest();

  private:
    static BLESerial* _instance;

    size_t _rxHead;
    size_t _rxTail;
    size_t _rxCount() const;
    uint8_t _rxBuffer[RX_BUFFER_SIZE];
    size_t _txCount;
    uint8_t _txBuffer[BLE_ATTRIBUTE_MAX_VALUE_LENGTH]; // Currently set to 20 bytes/packet
    unsigned long _lastFlushTime;

    // Changing the UUIDs to be random and not the standard ones from Nordic
    BLEService _uartService = BLEService("BC4C31B0-647F-11EE-8C99-0242AC120002"); //BLEService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    BLEDescriptor _uartNameDescriptor = BLEDescriptor("2901", "UART");
    BLECharacteristic _rxCharacteristic = BLECharacteristic("BC4C31B1-647F-11EE-8C99-0242AC120002", BLEWriteWithoutResponse, BLE_ATTRIBUTE_MAX_VALUE_LENGTH); //BLECharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWriteWithoutResponse, BLE_ATTRIBUTE_MAX_VALUE_LENGTH);
    BLEDescriptor _rxNameDescriptor = BLEDescriptor("2901", "RX - Receive Data (Write)");
    BLECharacteristic _txCharacteristic = BLECharacteristic("BC4C31B2-647F-11EE-8C99-0242AC120002", BLENotify, BLE_ATTRIBUTE_MAX_VALUE_LENGTH); //BLECharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, BLE_ATTRIBUTE_MAX_VALUE_LENGTH);
    BLEDescriptor _txNameDescriptor = BLEDescriptor("2901", "TX - Transfer Data (Notify)");

    void _received(const uint8_t* data, size_t size);
    static void _received(BLECentral& /*central*/, BLECharacteristic& rxCharacteristic);
};

#endif
