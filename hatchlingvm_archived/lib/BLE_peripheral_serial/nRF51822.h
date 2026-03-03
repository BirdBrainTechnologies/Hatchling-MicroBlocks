// Copyright (c) Sandeep Mistry. All rights reserved.
// Extensively modified by John Maloney in 2023 to add support for S113 and S140.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef _NRF_51822_H_
#define _NRF_51822_H_

#include <Arduino.h>
#include "BLEDevice.h"

#if defined(S110)
  #include <SDK/components/softdevice/s110/headers/ble.h>
  #include <SDK/components/softdevice/s110/headers/ble_gatts.h>
  #include <SDK/components/softdevice/s110/headers/ble_gattc.h>
  #include <SDK/components/softdevice/s110/headers/nrf_sdm.h>
  #include <SDK/components/softdevice/s110/headers/ble_hci.h>
#elif defined(S113)
  #include <SDK/components/softdevice/s113/headers/ble.h>
  #include <SDK/components/softdevice/s113/headers/ble_gatts.h>
  #include <SDK/components/softdevice/s113/headers/ble_gattc.h>
  #include <SDK/components/softdevice/s113/headers/nrf_sdm.h>
  #include <SDK/components/softdevice/s113/headers/ble_hci.h>
#elif defined(S140)
  #include <SDK/components/softdevice/s140/headers/ble.h>
  #include <SDK/components/softdevice/s140/headers/ble_gatts.h>
  #include <SDK/components/softdevice/s140/headers/ble_gattc.h>
  #include <SDK/components/softdevice/s140/headers/nrf_sdm.h>
  #include <SDK/components/softdevice/s140/headers/ble_hci.h>
#endif

class nRF51822 : public BLEDevice {
  friend class BLEPeripheral;

  protected:
    struct localCharacteristicInfo {
      BLECharacteristic* characteristic;
      BLEService* service;

      ble_gatts_char_handles_t handles;
      bool notifySubscribed;
      bool indicateSubscribed;
    };

    struct remoteServiceInfo {
      BLERemoteService* service;

      ble_uuid_t uuid;
      ble_gattc_handle_range_t handlesRange;
    };

    struct remoteCharacteristicInfo {
      BLERemoteCharacteristic* characteristic;
      BLERemoteService* service;

      ble_uuid_t uuid;
      ble_gatt_char_props_t properties;
      uint16_t valueHandle;
    };

    nRF51822();

    virtual ~nRF51822();

    virtual void enableBLE(); // Added for Hatchling - splitting initialization in two so I can get the Mac address into the name

    virtual void begin(unsigned char advertisementDataSize,
                BLEEirData *advertisementData,
                unsigned char scanDataSize,
                BLEEirData *scanData,
                BLELocalAttribute** localAttributes,
                unsigned char numLocalAttributes,
                BLERemoteAttribute** remoteAttributes,
                unsigned char numRemoteAttributes);

    virtual void poll();
    virtual void end();

    virtual bool setTxPower(int txPower);
    virtual void startAdvertising();
    virtual void updateAdvertising();
    virtual void disconnect();

    virtual bool updateCharacteristicValue(BLECharacteristic& characteristic);
    virtual bool broadcastCharacteristic(BLECharacteristic& characteristic);

    virtual bool canReadRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    virtual bool readRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    virtual bool canWriteRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    virtual bool writeRemoteCharacteristic(BLERemoteCharacteristic& characteristic, const unsigned char value[], unsigned char length);
    virtual bool canSubscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    virtual bool subscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    virtual bool canUnsubscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    virtual bool unsubcribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic);

    virtual void requestAddress();

  private:

    unsigned char                     _advData[31];
    unsigned char                     _advDataLen;
    unsigned char                     _advHandle;
    bool                              _hasScanData;
    unsigned char                     _srData[31];
    unsigned char                     _srDataLen;

    BLECharacteristic*                _broadcastCharacteristic;

    uint16_t                          _connectionHandle;

    unsigned char                     _numLocalCharacteristics;
    struct localCharacteristicInfo*   _localCharacteristicInfo;

    unsigned char                     _numRemoteServices;
    struct remoteServiceInfo*         _remoteServiceInfo;
    unsigned char                     _remoteServiceDiscoveryIndex;
    unsigned char                     _numRemoteCharacteristics;
    struct remoteCharacteristicInfo*  _remoteCharacteristicInfo;
    bool                              _remoteRequestInProgress;
};

#endif
