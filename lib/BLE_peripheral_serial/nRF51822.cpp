// Copyright (c) Sandeep Mistry. All rights reserved.
// Extensively modified by John Maloney in 2023 to add support for S113 and S140.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#if defined(NRF51) || defined(NRF52) || defined(NRF52_SERIES)

#include "Arduino.h"

#include "BLEAttribute.h"
#include "BLEService.h"
#include "BLECharacteristic.h"
#include "BLEDescriptor.h"
#include "BLEUtil.h"
#include "BLEUuid.h"
#include "nRF51822.h"

extern uint32_t __data_start__;

// #define NRF_51822_DEBUG

#define BLE_CONN_CFG_TAG_SERIAL 1 // our custom BLE configuration
#define TX_QUEUE_SIZE 50 // was 10          // queue up to this many WRITE_CMDs or HVNs

// Definitions for compatability between SoftDevice APIs

#ifdef S110
  #define BLE_EVT_PTR_ALIGNMENT BLE_EVTS_PTR_ALIGNMENT
  #define BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE BLE_EVT_TX_COMPLETE
  #define BLE_GATT_ATT_MTU_DEFAULT GATT_MTU_SIZE_DEFAULT
  #define BLE_GAP_ADV_SET_HANDLE_NOT_SET -1
#endif

#define BLE_STACK_EVT_MSG_BUF_SIZE (sizeof(ble_evt_t) + (BLE_GATT_ATT_MTU_DEFAULT))

// SoftDevice error handling

#if defined(S110)
	static void sdErrorHandler(uint32_t pc, uint16_t line_number, const uint8_t * p_file_name) {
		Serial.print("Softdevice error ");
		Serial.print(" pc: "); Serial.print(pc);
		Serial.print(" line: "); Serial.print(line_number);
		Serial.print(" file: "); Serial.print((char *) p_file_name);
		Serial.println();
	}
#else
	static void sdErrorHandler(uint32_t id, uint32_t pc, uint32_t info) {
		Serial.print("Softdevice error: "); Serial.print(id);
		Serial.print(" pc: "); Serial.print(pc);
		Serial.print(" info: "); Serial.print(info);
		Serial.println();
	}
#endif

static void checkReturnCode(const char *msg, uint32_t rc) {
	if (rc == NRF_SUCCESS) return;
	#ifdef S110
	  if (rc == 0x3004) return; // returned when no buffer available; not an error
	#else
	  if (rc == NRF_ERROR_RESOURCES) return; // returned when no buffer available; not an error
	#endif
	Serial.print(" error: ");
	Serial.println(rc, HEX);
}

static void printEvent(const char *msg, ble_evt_t* bleEvt) {
	Serial.print(msg);
	Serial.print(F(" [evt: 0x"));
	Serial.print(bleEvt->header.evt_id, HEX);
	Serial.print(F(" len: "));
	Serial.print(bleEvt->header.evt_len);
	Serial.println(F("]"));
}

// Helper functions

static void enableSoftDevice() {
	int rc = -1;

    #ifdef S110
		rc = sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_RC_250_PPM_250MS_CALIBRATION, sdErrorHandler);
		checkReturnCode("sd_softdevice_enable (a)", rc);
	#else
		// Note: S113 only works with RC clock source, not XTAL
		nrf_clock_lf_cfg_t cfg = {
			.source        = NRF_CLOCK_LF_SRC_RC,
			.rc_ctiv       = 8, //16
			.rc_temp_ctiv  = 2,
			.accuracy      = NRF_CLOCK_LF_ACCURACY_250_PPM
		};
		// Note: error handler cannot be NULL.
		rc = sd_softdevice_enable(&cfg, sdErrorHandler);
		checkReturnCode("sd_softdevice_enable (b)", rc);
	#endif

	const char *softDeviceName = "<unknown>";
	#ifdef S110
		softDeviceName = "S110";
	#elif defined(S113)
		softDeviceName = "S113";
	#elif defined(S132)
		softDeviceName = "S132";
	#elif defined(S140)
		softDeviceName = "S140";
	#endif

	if (rc) {
		Serial.print("Error enabling softdevice ");
		Serial.print(softDeviceName);
		Serial.print("; code: ");
		Serial.println(rc);
		return; // failed
	} else {
		Serial.print("Enabled softdevice ");
		Serial.println(softDeviceName);
	}
}

static void configureBLE() {
	// Configure BLE queues Must be done before enabling BLE.

  #ifndef S110
    uint32_t appDataStart = (uint32_t) &__data_start__;
	ble_cfg_t cfg;
	int rc;

	/* set connection event length */
	memset(&cfg, 0, sizeof(ble_cfg_t));
	cfg.conn_cfg.conn_cfg_tag = BLE_CONN_CFG_TAG_SERIAL;
	cfg.conn_cfg.params.gap_conn_cfg.event_length = 6; // 6 * 1.25 ms = 7.5 ms
	// Note: This corresponds to the old BLE_CONN_BW_HIGH for default ATT_MTU
	cfg.conn_cfg.params.gap_conn_cfg.conn_count = 1; // one link with this configuration
	rc = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &cfg, appDataStart);
	checkReturnCode("sd_ble_cfg_set (a)", rc);

	/* set HVN queue size */
	memset(&cfg, 0, sizeof(ble_cfg_t));
	cfg.conn_cfg.conn_cfg_tag = BLE_CONN_CFG_TAG_SERIAL;
	cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = TX_QUEUE_SIZE;
	rc = sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &cfg, appDataStart);
	checkReturnCode("sd_ble_cfg_set (b)", rc);

	/* set WRITE_CMD queue size */
	memset(&cfg, 0, sizeof(ble_cfg_t));
	cfg.conn_cfg.conn_cfg_tag = BLE_CONN_CFG_TAG_SERIAL;
	cfg.conn_cfg.params.gattc_conn_cfg.write_cmd_tx_queue_size = TX_QUEUE_SIZE;
	rc = sd_ble_cfg_set(BLE_CONN_CFG_GATTC, &cfg, appDataStart);
	checkReturnCode("sd_ble_cfg_set (c)", rc);
  #endif
}

uint32_t sd_ble_gatts_value_set(uint16_t handle, uint16_t offset, uint16_t* const p_len, uint8_t const * const p_value) {
  ble_gatts_value_t val;

  val.len = *p_len;
  val.offset = offset;
  val.p_value = (uint8_t*)p_value;
  return sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID, handle, &val);
}

// class definition

nRF51822::nRF51822() :
  BLEDevice(),

  _advDataLen(0),
  _advHandle(BLE_GAP_ADV_SET_HANDLE_NOT_SET),
  _hasScanData(false),
  _srDataLen(0),

  _broadcastCharacteristic(NULL),

  _connectionHandle(BLE_CONN_HANDLE_INVALID),

  _numLocalCharacteristics(0),
  _localCharacteristicInfo(NULL),

  _numRemoteServices(0),
  _remoteServiceInfo(NULL),
  _remoteServiceDiscoveryIndex(0),
  _numRemoteCharacteristics(0),
  _remoteCharacteristicInfo(NULL),
  _remoteRequestInProgress(false)
{
}

nRF51822::~nRF51822() {
  this->end();
}

// Splitting begin function into two pieces so that we can get the mac address into the advertising data
void nRF51822::enableBLE()
{
  int rc; 
  enableSoftDevice();
  configureBLE();

  #ifdef S110
    ble_enable_params_t enableParams = {
        .gatts_enable_params = {
            .service_changed = true,
            .attr_tab_size = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT
        }
    };
    rc = sd_ble_enable(&enableParams);
	checkReturnCode("sd_ble_enable (a)", rc);
  #else

    uint32_t appDataStart = (uint32_t) &__data_start__;
    rc = sd_ble_enable(&appDataStart);
	checkReturnCode("sd_ble_enable (b)", rc);
	if (rc == NRF_ERROR_NO_MEM) {
		uint32_t kbytesNeed = ((appDataStart - 0x20000000) + 1023) / 1024;
		Serial.println();
		Serial.println("************************************************************");
		Serial.print("   *** This softdevice requires at least ");
		Serial.print(kbytesNeed);
		Serial.println("k of RAM. ***");
		Serial.print("          Current RAM start 0x"); Serial.println((uint32_t) &__data_start__, HEX);
		Serial.print("          Minimum RAM start 0x"); Serial.println(appDataStart, HEX);
		Serial.println("Change the RAM start in the loader script (.ld file) in ");
		Serial.print("<softdevice>/toolchain/armgcc folder to: 0x");
		Serial.println(0x20000000 + (kbytesNeed * 1024), HEX);
		Serial.println();
		Serial.println("Note: Either delete your .pio/build folder or edit a source");
		Serial.println("file to force a recompile after editing the loader script.");
		Serial.println("************************************************************");
		Serial.println();
	}
  #endif

}

void nRF51822::begin(unsigned char advertisementDataSize,
                      BLEEirData *advertisementData,
                      unsigned char scanDataSize,
                      BLEEirData *scanData,
                      BLELocalAttribute** localAttributes,
                      unsigned char numLocalAttributes,
                      BLERemoteAttribute** remoteAttributes,
                      unsigned char numRemoteAttributes)
{
  int rc;

  #ifdef NRF_51822_DEBUG
	ble_version_t version;
	rc = sd_ble_version_get(&version);
	checkReturnCode("sd_ble_version_get", rc);

	Serial.print(F("version = "));
	Serial.print(version.version_number);
	Serial.print(F(" "));
	Serial.print(version.company_id);
	Serial.print(F(" "));
	Serial.print(version.subversion_number);
	Serial.println();
  #endif

  ble_gap_conn_params_t gap_conn_params;

  gap_conn_params.min_conn_interval = 40;  // in 1.25ms units
  gap_conn_params.max_conn_interval = 80;  // in 1.25ms unit
  gap_conn_params.slave_latency     = 0;
  gap_conn_params.conn_sup_timeout  = 4000 / 10; // in 10ms unit

  rc = sd_ble_gap_ppcp_set(&gap_conn_params);
  checkReturnCode("sd_ble_gap_ppcp_set", rc);

  this->_advDataLen = 0;

  // flags
  this->_advData[this->_advDataLen + 0] = 2;
  this->_advData[this->_advDataLen + 1] = 0x01;
  this->_advData[this->_advDataLen + 2] = 0x06;

  this->_advDataLen += 3;

  if (advertisementDataSize && advertisementData) {

    for (int i = 0; i < advertisementDataSize; i++) {
      this->_advData[this->_advDataLen + 0] = advertisementData[i].length + 1;
      this->_advData[this->_advDataLen + 1] = advertisementData[i].type;
      this->_advDataLen += 2;

      memcpy(&this->_advData[this->_advDataLen], advertisementData[i].data, advertisementData[i].length);

      this->_advDataLen += advertisementData[i].length;
    }
  }

  if (scanDataSize && scanData) {
    for (int i = 0; i < scanDataSize; i++) {
      this->_srData[this->_srDataLen + 0] = scanData[i].length + 1;
      this->_srData[this->_srDataLen + 1] = scanData[i].type;
      this->_srDataLen += 2;

      memcpy(&this->_srData[this->_srDataLen], scanData[i].data, scanData[i].length);

      this->_srDataLen += scanData[i].length;
      _hasScanData = true;
    }
  }

  rc = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
  checkReturnCode("sd_ble_gap_appearance_set (a)", rc);

  for (int i = 0; i < numLocalAttributes; i++) {
    BLELocalAttribute *localAttribute = localAttributes[i];

    if (localAttribute->type() == BLETypeCharacteristic) {
      this->_numLocalCharacteristics++;
    }
  }

  this->_numLocalCharacteristics -= 3; // 0x2a00, 0x2a01, 0x2a05

  this->_localCharacteristicInfo = (struct localCharacteristicInfo*)malloc(sizeof(struct localCharacteristicInfo) * this->_numLocalCharacteristics);

  unsigned char localCharacteristicIndex = 0;

  uint16_t handle = 0;
  BLEService *lastService = NULL;

  for (int i = 0; i < numLocalAttributes; i++) {
    BLELocalAttribute *localAttribute = localAttributes[i];
    BLEUuid uuid = BLEUuid(localAttribute->uuid());
    const unsigned char* uuidData = uuid.data();
    unsigned char value[255];

    ble_uuid_t nordicUUID;

    if (uuid.length() == 2) {
      nordicUUID.uuid = (uuidData[1] << 8) | uuidData[0];
      nordicUUID.type = BLE_UUID_TYPE_BLE;
    } else {
      unsigned char uuidDataTemp[16];

      memcpy(&uuidDataTemp, uuidData, sizeof(uuidDataTemp));

      nordicUUID.uuid = (uuidData[13] << 8) | uuidData[12];

      uuidDataTemp[13] = 0;
      uuidDataTemp[12] = 0;

      rc = sd_ble_uuid_vs_add((ble_uuid128_t*)&uuidDataTemp, &nordicUUID.type);
      checkReturnCode("sd_ble_uuid_vs_add (a)", rc);
    }

    if (localAttribute->type() == BLETypeService) {
      BLEService *service = (BLEService *)localAttribute;

      if (strcmp(service->uuid(), "1800") == 0 || strcmp(service->uuid(), "1801") == 0) {
        continue; // skip
      }

      rc = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &nordicUUID, &handle);
      checkReturnCode("sd_ble_gatts_service_add", rc);

      lastService = service;
    } else if (localAttribute->type() == BLETypeCharacteristic) {
      BLECharacteristic *characteristic = (BLECharacteristic *)localAttribute;

      if (strcmp(characteristic->uuid(), "2a00") == 0) {
        ble_gap_conn_sec_mode_t secMode;
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&secMode); // no security is needed

        rc = sd_ble_gap_device_name_set(&secMode, characteristic->value(), characteristic->valueLength());
        checkReturnCode("sd_ble_gap_device_name_set", rc);
     } else if (strcmp(characteristic->uuid(), "2a01") == 0) {
        const uint16_t *appearance = (const uint16_t*)characteristic->value();

        rc = sd_ble_gap_appearance_set(*appearance);
        checkReturnCode("sd_ble_gap_appearance_set (b)", rc);
      } else if (strcmp(characteristic->uuid(), "2a05") == 0) {
        // do nothing
      } else {
        uint8_t properties = characteristic->properties() & 0xfe;
        uint16_t valueLength = characteristic->valueLength();

        this->_localCharacteristicInfo[localCharacteristicIndex].characteristic = characteristic;
        this->_localCharacteristicInfo[localCharacteristicIndex].notifySubscribed = false;
        this->_localCharacteristicInfo[localCharacteristicIndex].indicateSubscribed = false;
        this->_localCharacteristicInfo[localCharacteristicIndex].service = lastService;

        ble_gatts_char_md_t characteristicMetaData;
        ble_gatts_attr_md_t clientCharacteristicConfigurationMetaData;
        ble_gatts_attr_t    characteristicValueAttribute;
        ble_gatts_attr_md_t characteristicValueAttributeMetaData;

        memset(&characteristicMetaData, 0, sizeof(characteristicMetaData));

        memcpy(&characteristicMetaData.char_props, &properties, 1);

        characteristicMetaData.p_char_user_desc  = NULL;
        characteristicMetaData.p_char_pf         = NULL;
        characteristicMetaData.p_user_desc_md    = NULL;
        characteristicMetaData.p_cccd_md         = NULL;
        characteristicMetaData.p_sccd_md         = NULL;

        if (properties & (BLENotify | BLEIndicate)) {
          memset(&clientCharacteristicConfigurationMetaData, 0, sizeof(clientCharacteristicConfigurationMetaData));

          BLE_GAP_CONN_SEC_MODE_SET_OPEN(&clientCharacteristicConfigurationMetaData.read_perm);
          BLE_GAP_CONN_SEC_MODE_SET_OPEN(&clientCharacteristicConfigurationMetaData.write_perm);

          clientCharacteristicConfigurationMetaData.vloc = BLE_GATTS_VLOC_STACK;

          characteristicMetaData.p_cccd_md = &clientCharacteristicConfigurationMetaData;
        }

        memset(&characteristicValueAttributeMetaData, 0, sizeof(characteristicValueAttributeMetaData));

        if (properties & (BLERead | BLENotify | BLEIndicate)) {
          BLE_GAP_CONN_SEC_MODE_SET_OPEN(&characteristicValueAttributeMetaData.read_perm);
        }

        if (properties & (BLEWriteWithoutResponse | BLEWrite)) {
          BLE_GAP_CONN_SEC_MODE_SET_OPEN(&characteristicValueAttributeMetaData.write_perm);
        }

        characteristicValueAttributeMetaData.vloc       = BLE_GATTS_VLOC_STACK;
        characteristicValueAttributeMetaData.rd_auth    = 0;
        characteristicValueAttributeMetaData.wr_auth    = 0;
        characteristicValueAttributeMetaData.vlen       = !characteristic->fixedLength();

        for (int j = (i + 1); j < numLocalAttributes; j++) {
          localAttribute = localAttributes[j];

          if (localAttribute->type() != BLETypeDescriptor) {
            break;
          }

          BLEDescriptor *descriptor = (BLEDescriptor *)localAttribute;

          if (strcmp(descriptor->uuid(), "2901") == 0) {
            characteristicMetaData.p_char_user_desc        = (uint8_t*)descriptor->value();
            characteristicMetaData.char_user_desc_max_size = descriptor->valueLength();
            characteristicMetaData.char_user_desc_size     = descriptor->valueLength();
          } else if (strcmp(descriptor->uuid(), "2904") == 0) {
            characteristicMetaData.p_char_pf = (ble_gatts_char_pf_t *)descriptor->value();
          }
        }

        memset(&characteristicValueAttribute, 0, sizeof(characteristicValueAttribute));

        characteristicValueAttribute.p_uuid       = &nordicUUID;
        characteristicValueAttribute.p_attr_md    = &characteristicValueAttributeMetaData;
        characteristicValueAttribute.init_len     = valueLength;
        characteristicValueAttribute.init_offs    = 0;
        characteristicValueAttribute.max_len      = characteristic->valueSize();
        characteristicValueAttribute.p_value      = NULL;

        rc = sd_ble_gatts_characteristic_add(BLE_GATT_HANDLE_INVALID, &characteristicMetaData, &characteristicValueAttribute, &this->_localCharacteristicInfo[localCharacteristicIndex].handles);
        checkReturnCode("sd_ble_gatts_characteristic_add", rc);

        if (valueLength) {
          for (int j = 0; j < valueLength; j++) {
            value[j] = (*characteristic)[j];
          }

          rc = sd_ble_gatts_value_set(this->_localCharacteristicInfo[localCharacteristicIndex].handles.value_handle, 0, &valueLength, value);
          checkReturnCode("sd_ble_gatts_value_set (a)", rc);
        }

        localCharacteristicIndex++;
      }
    } else if (localAttribute->type() == BLETypeDescriptor) {
      BLEDescriptor *descriptor = (BLEDescriptor *)localAttribute;

      if (strcmp(descriptor->uuid(), "2901") == 0 ||
          strcmp(descriptor->uuid(), "2902") == 0 ||
          strcmp(descriptor->uuid(), "2903") == 0 ||
          strcmp(descriptor->uuid(), "2904") == 0) {
        continue; // skip
      }

      uint16_t valueLength = descriptor->valueLength();

      ble_gatts_attr_t descriptorAttribute;
      ble_gatts_attr_md_t descriptorMetaData;

      memset(&descriptorAttribute, 0, sizeof(descriptorAttribute));
      memset(&descriptorMetaData, 0, sizeof(descriptorMetaData));

      descriptorMetaData.vloc = BLE_GATTS_VLOC_STACK;
      descriptorMetaData.vlen = (valueLength == descriptor->valueLength()) ? 0 : 1;
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&descriptorMetaData.read_perm);

      descriptorAttribute.p_uuid    = &nordicUUID;
      descriptorAttribute.p_attr_md = &descriptorMetaData;
      descriptorAttribute.init_len  = valueLength;
      descriptorAttribute.max_len   = descriptor->valueLength();
      descriptorAttribute.p_value   = NULL;

      rc = sd_ble_gatts_descriptor_add(BLE_GATT_HANDLE_INVALID, &descriptorAttribute, &handle);
      checkReturnCode("sd_ble_gatts_descriptor_add", rc);

      if (valueLength) {
        for (int j = 0; j < valueLength; j++) {
          value[j] = (*descriptor)[j];
        }

        rc = sd_ble_gatts_value_set(handle, 0, &valueLength, value);
        checkReturnCode("sd_ble_gatts_value_set (b)", rc);
      }
    }
  }

  if ( numRemoteAttributes > 0) {
    numRemoteAttributes -= 2; // 0x1801, 0x2a05
  }

  for (int i = 0; i < numRemoteAttributes; i++) {
    BLERemoteAttribute *remoteAttribute = remoteAttributes[i];

    if (remoteAttribute->type() == BLETypeService) {
      this->_numRemoteServices++;
    } else if (remoteAttribute->type() == BLETypeCharacteristic) {
      this->_numRemoteCharacteristics++;
    }
  }

  this->_remoteServiceInfo = (struct remoteServiceInfo*)malloc(sizeof(struct remoteServiceInfo) * this->_numRemoteServices);
  this->_remoteCharacteristicInfo = (struct remoteCharacteristicInfo*)malloc(sizeof(struct remoteCharacteristicInfo) * this->_numRemoteCharacteristics);

  BLERemoteService *lastRemoteService = NULL;
  unsigned char remoteServiceIndex = 0;
  unsigned char remoteCharacteristicIndex = 0;

  for (int i = 0; i < numRemoteAttributes; i++) {
    BLERemoteAttribute *remoteAttribute = remoteAttributes[i];
    BLEUuid uuid = BLEUuid(remoteAttribute->uuid());
    const unsigned char* uuidData = uuid.data();

    ble_uuid_t nordicUUID;

    if (uuid.length() == 2) {
      nordicUUID.uuid = (uuidData[1] << 8) | uuidData[0];
      nordicUUID.type = BLE_UUID_TYPE_BLE;
    } else {
      unsigned char uuidDataTemp[16];

      memcpy(&uuidDataTemp, uuidData, sizeof(uuidDataTemp));

      nordicUUID.uuid = (uuidData[13] << 8) | uuidData[12];

      uuidDataTemp[13] = 0;
      uuidDataTemp[12] = 0;

      rc = sd_ble_uuid_vs_add((ble_uuid128_t*)&uuidDataTemp, &nordicUUID.type);
      checkReturnCode("sd_ble_uuid_vs_add (b)", rc);
    }

    if (remoteAttribute->type() == BLETypeService) {
      this->_remoteServiceInfo[remoteServiceIndex].service = lastRemoteService = (BLERemoteService *)remoteAttribute;
      this->_remoteServiceInfo[remoteServiceIndex].uuid = nordicUUID;

      memset(&this->_remoteServiceInfo[remoteServiceIndex].handlesRange, 0, sizeof(this->_remoteServiceInfo[remoteServiceIndex].handlesRange));

      remoteServiceIndex++;
    } else if (remoteAttribute->type() == BLETypeCharacteristic) {
      this->_remoteCharacteristicInfo[remoteCharacteristicIndex].characteristic = (BLERemoteCharacteristic *)remoteAttribute;
      this->_remoteCharacteristicInfo[remoteCharacteristicIndex].service = lastRemoteService;
      this->_remoteCharacteristicInfo[remoteCharacteristicIndex].uuid = nordicUUID;

      memset(&this->_remoteCharacteristicInfo[remoteCharacteristicIndex].properties, 0, sizeof(this->_remoteCharacteristicInfo[remoteCharacteristicIndex].properties));
      this->_remoteCharacteristicInfo[remoteCharacteristicIndex].valueHandle = 0;

      remoteCharacteristicIndex++;
    }
  }

  this->startAdvertising();
}

void nRF51822::poll() {
  uint32_t   evtBuf[BLE_STACK_EVT_MSG_BUF_SIZE] __attribute__ ((__aligned__(BLE_EVT_PTR_ALIGNMENT)));
  uint16_t   evtLen = sizeof(evtBuf);
  ble_evt_t* bleEvt = (ble_evt_t*) evtBuf;
  int        rc = -1;

  rc = sd_ble_evt_get((uint8_t*) evtBuf, &evtLen);
  if (rc == NRF_ERROR_NOT_FOUND) return; // no event; do nothing
  if (rc != NRF_SUCCESS) {
    checkReturnCode("sd_ble_evt_get", rc);
    return;
  }
  if (rc == NRF_SUCCESS) {
    switch (bleEvt->header.evt_id) {
      case BLE_GAP_EVT_CONNECTED:
      #ifdef NRF_51822_DEBUG
        printEvent("BLE_GAP_EVT_CONNECTED", bleEvt);
        char address[18];
        BLEUtil::addressToString(bleEvt->evt.gap_evt.params.connected.peer_addr.addr, address);
        Serial.print(F("Connected to: "));
        Serial.println(address);
      #endif

        this->_connectionHandle = bleEvt->evt.gap_evt.conn_handle;
        if (this->_eventListener) {
          this->_eventListener->BLEDeviceConnected(*this, bleEvt->evt.gap_evt.params.connected.peer_addr.addr);
        }

        if (this->_minimumConnectionInterval >= BLE_GAP_CP_MIN_CONN_INTVL_MIN &&
            this->_maximumConnectionInterval <= BLE_GAP_CP_MAX_CONN_INTVL_MAX) {
          ble_gap_conn_params_t gap_conn_params;

          gap_conn_params.min_conn_interval = this->_minimumConnectionInterval;  // in 1.25ms units
          gap_conn_params.max_conn_interval = this->_maximumConnectionInterval;  // in 1.25ms unit
          gap_conn_params.slave_latency     = 0;
          gap_conn_params.conn_sup_timeout  = 4000 / 10; // in 10ms unit

          rc = sd_ble_gap_conn_param_update(this->_connectionHandle, &gap_conn_params);
          checkReturnCode("sd_ble_gap_conn_param_update", rc);
        }

        if (this->_numRemoteServices > 0) {
          rc = sd_ble_gattc_primary_services_discover(this->_connectionHandle, 1, NULL);
          checkReturnCode("sd_ble_gattc_primary_services_discover (a)", rc);
        }
        break;

      case BLE_GAP_EVT_DISCONNECTED:
      #ifdef NRF_51822_DEBUG
        printEvent("BLE_GAP_EVT_DISCONNECTED", bleEvt);
      #endif
        this->_connectionHandle = BLE_CONN_HANDLE_INVALID;

        for (int i = 0; i < this->_numLocalCharacteristics; i++) {
          struct localCharacteristicInfo* localCharacteristicInfo = &this->_localCharacteristicInfo[i];

          localCharacteristicInfo->notifySubscribed = false;
          localCharacteristicInfo->indicateSubscribed = false;

          if (localCharacteristicInfo->characteristic->subscribed()) {
            if (this->_eventListener) {
              this->_eventListener->BLEDeviceCharacteristicSubscribedChanged(*this, *localCharacteristicInfo->characteristic, false);
            }
          }
        }

        if (this->_eventListener) {
          this->_eventListener->BLEDeviceDisconnected(*this);
        }

        // clear remote handle info
        for (int i = 0; i < this->_numRemoteServices; i++) {
          memset(&this->_remoteServiceInfo[i].handlesRange, 0, sizeof(this->_remoteServiceInfo[i].handlesRange));
        }

        for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
          memset(&this->_remoteCharacteristicInfo[i].properties, 0, sizeof(this->_remoteCharacteristicInfo[i].properties));
          this->_remoteCharacteristicInfo[i].valueHandle = 0;
        }

        this->_remoteRequestInProgress = false;

        this->startAdvertising();
        break;

  #ifndef S110 // non-S101 events: the following events are not defined in the S110 API

      case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
      #ifdef NRF_51822_DEBUG
        printEvent("BLE_GAP_EVT_PHY_UPDATE_REQUEST", bleEvt);
      #endif
        ble_gap_phys_t phys;
        phys.rx_phys = BLE_GAP_PHY_2MBPS;
        phys.tx_phys = BLE_GAP_PHY_2MBPS;
        rc = sd_ble_gap_phy_update(bleEvt->evt.gap_evt.conn_handle, &phys);
        checkReturnCode("sd_ble_gap_phy_update", rc);
        break;

      case BLE_GAP_EVT_PHY_UPDATE:
      #ifdef NRF_51822_DEBUG
        printEvent("BLE_GAP_EVT_PHY_UPDATE", bleEvt);
		Serial.print("   status "); Serial.println(bleEvt->evt.gap_evt.params.phy_update.status);
		Serial.print("   tx_phy "); Serial.println(bleEvt->evt.gap_evt.params.phy_update.tx_phy);
		Serial.print("   rx_phy "); Serial.println(bleEvt->evt.gap_evt.params.phy_update.rx_phy);
      #endif
        break;

      case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
      #ifdef NRF_51822_DEBUG
        printEvent("BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST", bleEvt);
      #endif
        rc = sd_ble_gap_data_length_update(bleEvt->evt.gap_evt.conn_handle, NULL, NULL); // use automatic values
        checkReturnCode("sd_ble_gap_data_length_update", rc);
        break;

      case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
      #ifdef NRF_51822_DEBUG
        printEvent("BLE_GAP_EVT_DATA_LENGTH_UPDATE", bleEvt);
		Serial.print("   max_tx_octets "); Serial.println(bleEvt->evt.gap_evt.params.data_length_update.effective_params.max_tx_octets);
		Serial.print("   max_rx_octets "); Serial.println(bleEvt->evt.gap_evt.params.data_length_update.effective_params.max_rx_octets);
		Serial.print("   max_tx_time_us "); Serial.println(bleEvt->evt.gap_evt.params.data_length_update.effective_params.max_tx_time_us);
		Serial.print("   max_rx_time_us "); Serial.println(bleEvt->evt.gap_evt.params.data_length_update.effective_params.max_rx_time_us);
      #endif
      break;

      case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
      #ifdef NRF_51822_DEBUG
        printEvent("BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST", bleEvt);
      #endif
        rc = sd_ble_gatts_exchange_mtu_reply(bleEvt->evt.gap_evt.conn_handle, BLE_GATT_ATT_MTU_DEFAULT);
        checkReturnCode("sd_ble_gatts_exchange_mtu_reply", rc);
        break;

      case BLE_GATTS_EVT_HVN_TX_COMPLETE:
      #ifdef NRF_51822_DEBUG
        printEvent("BLE_GATTS_EVT_HVN_TX_COMPLETE", bleEvt);
        Serial.print(F(" count: "));
        Serial.println(bleEvt->evt.gatts_evt.params.hvn_tx_complete.count);
      #endif
        break;

  #endif // end non-S101 events

      case BLE_GATTS_EVT_WRITE: {
      #ifdef NRF_51822_DEBUG
        printEvent("BLE_GATTS_EVT_WRITE", bleEvt);
        Serial.print(F(" handle = "));
        Serial.println(bleEvt->evt.gatts_evt.params.write.handle, DEC);
        BLEUtil::printBuffer(bleEvt->evt.gatts_evt.params.write.data, bleEvt->evt.gatts_evt.params.write.len);
      #endif
        uint16_t handle = bleEvt->evt.gatts_evt.params.write.handle;

        for (int i = 0; i < this->_numLocalCharacteristics; i++) {
          struct localCharacteristicInfo* localCharacteristicInfo = &this->_localCharacteristicInfo[i];

          if (localCharacteristicInfo->handles.value_handle == handle) {
            if (this->_eventListener) {
              this->_eventListener->BLEDeviceCharacteristicValueChanged(*this, *localCharacteristicInfo->characteristic, bleEvt->evt.gatts_evt.params.write.data, bleEvt->evt.gatts_evt.params.write.len);
            }
            break;
          } else if (localCharacteristicInfo->handles.cccd_handle == handle) {
            uint8_t* data  = &bleEvt->evt.gatts_evt.params.write.data[0];
            uint16_t value = data[0] | (data[1] << 8);

            localCharacteristicInfo->notifySubscribed = (value & 0x0001);
            localCharacteristicInfo->indicateSubscribed = (value & 0x0002);

            bool subscribed = (localCharacteristicInfo->notifySubscribed || localCharacteristicInfo->indicateSubscribed);

            if (subscribed != localCharacteristicInfo->characteristic->subscribed()) {
              if (this->_eventListener) {
                this->_eventListener->BLEDeviceCharacteristicSubscribedChanged(*this, *localCharacteristicInfo->characteristic, subscribed);
              }
              break;
            }
          }
        }
        break;
      }

      case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
      #ifdef NRF_51822_DEBUG
        printEvent("BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE", bleEvt);
        Serial.println(bleEvt->evt.gattc_evt.params.write_cmd_tx_complete.count);
      #endif
        break;

      case BLE_GATTS_EVT_SYS_ATTR_MISSING:
      #ifdef NRF_51822_DEBUG
        printEvent("BLE_GATTS_EVT_SYS_ATTR_MISSING", bleEvt);
        Serial.println(bleEvt->evt.gatts_evt.params.sys_attr_missing.hint);
      #endif
        rc = sd_ble_gatts_sys_attr_set(this->_connectionHandle, NULL, 0, 0);
        checkReturnCode("sd_ble_gatts_sys_attr_set (a)", rc);
        break;

      case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
      #ifdef NRF_51822_DEBUG
        printEvent("BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP", bleEvt);
        Serial.println(bleEvt->evt.gattc_evt.gatt_status, HEX);
      #endif
        if (bleEvt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_SUCCESS) {
          uint16_t count = bleEvt->evt.gattc_evt.params.prim_srvc_disc_rsp.count;
          for (int i = 0; i < count; i++) {
            for (int j = 0; j < this->_numRemoteServices; j++) {
              if ((bleEvt->evt.gattc_evt.params.prim_srvc_disc_rsp.services[i].uuid.type == this->_remoteServiceInfo[j].uuid.type) &&
                  (bleEvt->evt.gattc_evt.params.prim_srvc_disc_rsp.services[i].uuid.uuid == this->_remoteServiceInfo[j].uuid.uuid)) {
                this->_remoteServiceInfo[j].handlesRange = bleEvt->evt.gattc_evt.params.prim_srvc_disc_rsp.services[i].handle_range;
                break;
              }
            }
          }

          uint16_t startHandle = bleEvt->evt.gattc_evt.params.prim_srvc_disc_rsp.services[count - 1].handle_range.end_handle + 1;

          rc = sd_ble_gattc_primary_services_discover(this->_connectionHandle, startHandle, NULL);
          checkReturnCode("sd_ble_gattc_primary_services_discover (b)", rc);
        } else {
          // done discovering services
          for (int i = 0; i < this->_numRemoteServices; i++) {
            if (this->_remoteServiceInfo[i].handlesRange.start_handle != 0 && this->_remoteServiceInfo[i].handlesRange.end_handle != 0) {
              this->_remoteServiceDiscoveryIndex = i;

              rc = sd_ble_gattc_characteristics_discover(this->_connectionHandle, &this->_remoteServiceInfo[i].handlesRange);
              checkReturnCode("sd_ble_gattc_characteristics_discover (a)", rc);
              break;
            }
          }
        }
        break;

      case BLE_GATTC_EVT_CHAR_DISC_RSP:
      #ifdef NRF_51822_DEBUG
        printEvent("BLE_GATTC_EVT_CHAR_DISC_RSP", bleEvt);
        Serial.println(bleEvt->evt.gattc_evt.gatt_status, HEX);
      #endif
        if (bleEvt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_SUCCESS) {
          ble_gattc_handle_range_t serviceHandlesRange = this->_remoteServiceInfo[this->_remoteServiceDiscoveryIndex].handlesRange;

          uint16_t count = bleEvt->evt.gattc_evt.params.char_disc_rsp.count;

          for (int i = 0; i < count; i++) {
            for (int j = 0; j < this->_numRemoteCharacteristics; j++) {
              if ((this->_remoteServiceInfo[this->_remoteServiceDiscoveryIndex].service == this->_remoteCharacteristicInfo[j].service) &&
                  (bleEvt->evt.gattc_evt.params.char_disc_rsp.chars[i].uuid.type == this->_remoteCharacteristicInfo[j].uuid.type) &&
                  (bleEvt->evt.gattc_evt.params.char_disc_rsp.chars[i].uuid.uuid == this->_remoteCharacteristicInfo[j].uuid.uuid)) {
                this->_remoteCharacteristicInfo[j].properties = bleEvt->evt.gattc_evt.params.char_disc_rsp.chars[i].char_props;
                this->_remoteCharacteristicInfo[j].valueHandle = bleEvt->evt.gattc_evt.params.char_disc_rsp.chars[i].handle_value;
              }
            }

            serviceHandlesRange.start_handle = bleEvt->evt.gattc_evt.params.char_disc_rsp.chars[i].handle_value;
          }

          rc = sd_ble_gattc_characteristics_discover(this->_connectionHandle, &serviceHandlesRange);
          checkReturnCode("sd_ble_gattc_characteristics_discover (b)", rc);
        } else {
          bool discoverCharacteristics = false;

          for (int i = this->_remoteServiceDiscoveryIndex + 1; i < this->_numRemoteServices; i++) {
            if (this->_remoteServiceInfo[i].handlesRange.start_handle != 0 && this->_remoteServiceInfo[i].handlesRange.end_handle != 0) {
              this->_remoteServiceDiscoveryIndex = i;
              rc = sd_ble_gattc_characteristics_discover(this->_connectionHandle, &this->_remoteServiceInfo[i].handlesRange);
              checkReturnCode("sd_ble_gattc_characteristics_discover (c)", rc);
              discoverCharacteristics = true;
              break;
            }
          }

          if (!discoverCharacteristics) {
            if (this->_eventListener) {
              this->_eventListener->BLEDeviceRemoteServicesDiscovered(*this);
            }
          }
        }
        break;

      case BLE_GATTC_EVT_READ_RSP: {
      #ifdef NRF_51822_DEBUG
        printEvent("BLE_GATTC_EVT_READ_RSP", bleEvt);
        Serial.println(bleEvt->evt.gattc_evt.gatt_status, HEX);
        Serial.println(bleEvt->evt.gattc_evt.params.read_rsp.handle, DEC);
        BLEUtil::printBuffer(bleEvt->evt.gattc_evt.params.read_rsp.data, bleEvt->evt.gattc_evt.params.read_rsp.len);
      #endif
        this->_remoteRequestInProgress = false;

        uint16_t handle = bleEvt->evt.gattc_evt.params.read_rsp.handle;
        for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
          if (this->_remoteCharacteristicInfo[i].valueHandle == handle) {
            if (this->_eventListener) {
              this->_eventListener->BLEDeviceRemoteCharacteristicValueChanged(*this, *this->_remoteCharacteristicInfo[i].characteristic, bleEvt->evt.gattc_evt.params.read_rsp.data, bleEvt->evt.gattc_evt.params.read_rsp. len);
            }
            break;
          }
        }
        break;
      }

      case BLE_GATTC_EVT_WRITE_RSP:
      #ifdef NRF_51822_DEBUG
        printEvent("BLE_GATTC_EVT_WRITE_RSP", bleEvt);
        Serial.println(bleEvt->evt.gattc_evt.gatt_status, HEX);
        Serial.println(bleEvt->evt.gattc_evt.params.write_rsp.handle, DEC);
      #endif
        this->_remoteRequestInProgress = false;
        break;

      case BLE_GATTC_EVT_HVX: {
      #ifdef NRF_51822_DEBUG
        printEvent("BLE_GATTC_EVT_HVX", bleEvt);
        Serial.println(bleEvt->evt.gattc_evt.gatt_status, HEX);
        Serial.println(bleEvt->evt.gattc_evt.params.hvx.handle, DEC);
      #endif
        uint16_t handle = bleEvt->evt.gattc_evt.params.hvx.handle;

        if (bleEvt->evt.gattc_evt.params.hvx.type == BLE_GATT_HVX_INDICATION) {
          rc = sd_ble_gattc_hv_confirm(this->_connectionHandle, handle);
          checkReturnCode("sd_ble_gattc_hv_confirm", rc);
        }

        for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
          if (this->_remoteCharacteristicInfo[i].valueHandle == handle) {
            if (this->_eventListener) {
              this->_eventListener->BLEDeviceRemoteCharacteristicValueChanged(*this, *this->_remoteCharacteristicInfo[i].characteristic, bleEvt->evt.gattc_evt.params.read_rsp.data, bleEvt->evt.gattc_evt.params.read_rsp. len);
            }
            break;
          }
        }
        break;
      }

      default:
        printEvent("UNHANDLED EVENT", bleEvt);
        break;
    }
  }
}

void nRF51822::end() {
  int rc = sd_softdevice_disable();
  checkReturnCode("sd_softdevice_disable", rc);

  if (this->_remoteCharacteristicInfo) {
    free(this->_remoteCharacteristicInfo);
  }

  if (this->_remoteServiceInfo) {
    free(this->_remoteServiceInfo);
  }

  if (this->_localCharacteristicInfo) {
    free(this->_localCharacteristicInfo);
  }

  this->_numLocalCharacteristics = 0;
  this->_numRemoteServices = 0;
  this->_numRemoteCharacteristics = 0;
}

bool nRF51822::updateCharacteristicValue(BLECharacteristic& characteristic) {
  bool success = true;
  int rc;

  for (int i = 0; i < this->_numLocalCharacteristics; i++) {
    struct localCharacteristicInfo* localCharacteristicInfo = &this->_localCharacteristicInfo[i];

    if (localCharacteristicInfo->characteristic == &characteristic) {
      if (&characteristic == this->_broadcastCharacteristic) {
        this->broadcastCharacteristic(characteristic);
      }

      uint16_t valueLength = characteristic.valueLength();
      rc = sd_ble_gatts_value_set(localCharacteristicInfo->handles.value_handle, 0, &valueLength, characteristic.value());
      checkReturnCode("sd_ble_gatts_value_set (c)", rc);

      ble_gatts_hvx_params_t hvxParams;
      memset(&hvxParams, 0, sizeof(hvxParams));
      hvxParams.handle = localCharacteristicInfo->handles.value_handle;
      hvxParams.offset = 0;
      hvxParams.p_data = NULL;
      hvxParams.p_len  = &valueLength;

      if (localCharacteristicInfo->notifySubscribed) {
        hvxParams.type = BLE_GATT_HVX_NOTIFICATION;
        rc = sd_ble_gatts_hvx(this->_connectionHandle, &hvxParams);
        checkReturnCode("sd_ble_gatts_hvx (a)", rc);
        success = (rc == NRF_SUCCESS);
      }

      if (localCharacteristicInfo->indicateSubscribed) {
        hvxParams.type = BLE_GATT_HVX_INDICATION;
        rc = sd_ble_gatts_hvx(this->_connectionHandle, &hvxParams);
        checkReturnCode("sd_ble_gatts_hvx (b)", rc);
        success = (rc == NRF_SUCCESS);
      }
    }
  }
  return success;
}

bool nRF51822::broadcastCharacteristic(BLECharacteristic& characteristic) {
  bool success = false;

  for (int i = 0; i < this->_numLocalCharacteristics; i++) {
    struct localCharacteristicInfo* localCharacteristicInfo = &this->_localCharacteristicInfo[i];

    if (localCharacteristicInfo->characteristic == &characteristic) {
      if (characteristic.properties() & BLEBroadcast && localCharacteristicInfo->service) {
        unsigned char advData[31];
        unsigned char advDataLen = this->_advDataLen;

        // copy the existing advertisement data
        memcpy(advData, this->_advData, advDataLen);

        advDataLen += (4 + characteristic.valueLength());

        if (advDataLen <= 31) {
          BLEUuid uuid = BLEUuid(localCharacteristicInfo->service->uuid());

          advData[this->_advDataLen + 0] = 3 + characteristic.valueLength();
          advData[this->_advDataLen + 1] = 0x16;

          memcpy(&advData[this->_advDataLen + 2], uuid.data(), 2);
          memcpy(&advData[this->_advDataLen + 4], characteristic.value(), characteristic.valueLength());

          this->updateAdvertising();
          success = true;

          this->_broadcastCharacteristic = &characteristic;
        }
      }
      break;
    }
  }

  return success;
}

bool nRF51822::canReadRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  bool success = false;

  for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
    if (this->_remoteCharacteristicInfo[i].characteristic == &characteristic) {
      success = (this->_remoteCharacteristicInfo[i].valueHandle &&
                  this->_remoteCharacteristicInfo[i].properties.read &&
                  !this->_remoteRequestInProgress);
      break;
    }
  }

  return success;
}

bool nRF51822::readRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  bool success = false;

  for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
    if (this->_remoteCharacteristicInfo[i].characteristic == &characteristic) {
      if (this->_remoteCharacteristicInfo[i].valueHandle && this->_remoteCharacteristicInfo[i].properties.read) {
        this->_remoteRequestInProgress = true;
        success = (sd_ble_gattc_read(this->_connectionHandle, this->_remoteCharacteristicInfo[i].valueHandle, 0) == NRF_SUCCESS);
      }
      break;
    }
  }

  return success;
}

bool nRF51822::canWriteRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  bool success = false;

  for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
    if (this->_remoteCharacteristicInfo[i].characteristic == &characteristic) {
      if (this->_remoteCharacteristicInfo[i].valueHandle) {
        if (this->_remoteCharacteristicInfo[i].properties.write) {
          success = !this->_remoteRequestInProgress;
        } else if (this->_remoteCharacteristicInfo[i].properties.write_wo_resp) {
          success = true;
        }
      }
      break;
    }
  }

  return success;
}

bool nRF51822::writeRemoteCharacteristic(BLERemoteCharacteristic& characteristic, const unsigned char value[], unsigned char length) {
  bool success = false;

  for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
    if (this->_remoteCharacteristicInfo[i].characteristic == &characteristic) {
      if ((this->_remoteCharacteristicInfo[i].valueHandle) &&
            (this->_remoteCharacteristicInfo[i].properties.write_wo_resp ||
            (this->_remoteCharacteristicInfo[i].properties.write))) {

        ble_gattc_write_params_t writeParams;
        writeParams.write_op = (this->_remoteCharacteristicInfo[i].properties.write) ? BLE_GATT_OP_WRITE_REQ : BLE_GATT_OP_WRITE_CMD;
        writeParams.handle = this->_remoteCharacteristicInfo[i].valueHandle;
        writeParams.offset = 0;
        writeParams.len = length;
        writeParams.p_value = (uint8_t*)value;

        this->_remoteRequestInProgress = true;
        int rc = sd_ble_gattc_write(this->_connectionHandle, &writeParams);
        checkReturnCode("sd_ble_gattc_write (a)", rc);
        success = (rc == NRF_SUCCESS);
      }
      break;
    }
  }
  return success;
}

bool nRF51822::canSubscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  bool success = false;

  for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
    if (this->_remoteCharacteristicInfo[i].characteristic == &characteristic) {
      success = (this->_remoteCharacteristicInfo[i].valueHandle &&
                (this->_remoteCharacteristicInfo[i].properties.notify || this->_remoteCharacteristicInfo[i].properties.indicate));
      break;
    }
  }
  return success;
}

bool nRF51822::subscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  bool success = false;

  for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
    if (this->_remoteCharacteristicInfo[i].characteristic == &characteristic) {
      if (this->_remoteCharacteristicInfo[i].valueHandle &&
                  (this->_remoteCharacteristicInfo[i].properties.notify || this->_remoteCharacteristicInfo[i].properties.indicate)) {

        uint16_t value = (this->_remoteCharacteristicInfo[i].properties.notify ? 0x0001 : 0x002);

        ble_gattc_write_params_t writeParams;
        writeParams.write_op = BLE_GATT_OP_WRITE_REQ;
        writeParams.handle = (this->_remoteCharacteristicInfo[i].valueHandle + 1); // don't discover descriptors for now
        writeParams.offset = 0;
        writeParams.len = sizeof(value);
        writeParams.p_value = (uint8_t*)&value;

        this->_remoteRequestInProgress = true;

        int rc = sd_ble_gattc_write(this->_connectionHandle, &writeParams);
        checkReturnCode("sd_ble_gattc_write (b)", rc);
        success = (rc == NRF_SUCCESS);
      }
      break;
    }
  }
  return success;
}

bool nRF51822::canUnsubscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  return this->canSubscribeRemoteCharacteristic(characteristic);
}

bool nRF51822::unsubcribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  bool success = false;

  for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
    if (this->_remoteCharacteristicInfo[i].characteristic == &characteristic) {
      if (this->_remoteCharacteristicInfo[i].valueHandle &&
                  (this->_remoteCharacteristicInfo[i].properties.notify || this->_remoteCharacteristicInfo[i].properties.indicate)) {

        uint16_t value = 0x0000;

        ble_gattc_write_params_t writeParams;
        writeParams.write_op = BLE_GATT_OP_WRITE_REQ;
        writeParams.handle = (this->_remoteCharacteristicInfo[i].valueHandle + 1); // don't discover descriptors for now
        writeParams.offset = 0;
        writeParams.len = sizeof(value);
        writeParams.p_value = (uint8_t*)&value;

        this->_remoteRequestInProgress = true;

        int rc = sd_ble_gattc_write(this->_connectionHandle, &writeParams);
        checkReturnCode("sd_ble_gattc_write (c)", rc);
        success = (rc == NRF_SUCCESS);
      }
      break;
    }
  }
  return success;
}

bool nRF51822::setTxPower(int txPower) {
  if (txPower <= -40) {
    txPower = -40;
  } else if (txPower <= -30) {
    txPower = -30;
  } else if (txPower <= -20) {
    txPower = -20;
  } else if (txPower <= -16) {
    txPower = -16;
  } else if (txPower <= -12) {
    txPower = -12;
  } else if (txPower <= -8) {
    txPower = -8;
  } else if (txPower <= -4) {
    txPower = -4;
  } else if (txPower <= 0) {
    txPower = 0;
  } else {
    txPower = 4;
  }

  int rc;
  #ifdef S110
    rc = sd_ble_gap_tx_power_set(txPower);
  #else
    rc = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, _connectionHandle, txPower);
  #endif
  checkReturnCode("sd_ble_gap_tx_power_set (a)", rc);
  return (rc == NRF_SUCCESS);
}

void nRF51822::startAdvertising() {
  int rc;

  #ifdef S110 // S110 advertising setup

    ble_gap_adv_params_t advertisingParameters;
    memset(&advertisingParameters, 0x00, sizeof(advertisingParameters));

	advertisingParameters.type        = this->_connectable ? BLE_GAP_ADV_TYPE_ADV_IND : ( this->_hasScanData ? BLE_GAP_ADV_TYPE_ADV_SCAN_IND : BLE_GAP_ADV_TYPE_ADV_NONCONN_IND );
	advertisingParameters.p_peer_addr = NULL;
	advertisingParameters.fp          = BLE_GAP_ADV_FP_ANY;
	advertisingParameters.p_whitelist = NULL;
	advertisingParameters.interval    = (this->_advertisingInterval * 16) / 10; // advertising interval (in units of 0.625 ms)
	advertisingParameters.timeout     = 0;
    rc = sd_ble_gap_adv_data_set(this->_advData, this->_advDataLen, _srData, _srDataLen);
    checkReturnCode("sd_ble_gap_adv_data_set (a)", rc);

	// set advertising power (S110)
    rc = sd_ble_gap_tx_power_set(0);
    checkReturnCode("sd_ble_gap_tx_power_set (b)", rc);

	// start advertisting (S110)
	rc = sd_ble_gap_adv_start(&advertisingParameters);
    checkReturnCode("sd_ble_gap_adv_start (a)", rc);

  #else // non-S110 advertising setup

    ble_gap_adv_data_t adv_data;
    memset(&adv_data, 0, sizeof(adv_data));
    adv_data.adv_data.p_data = _advData;
    adv_data.adv_data.len = _advDataLen;
    adv_data.scan_rsp_data.p_data = _srData;
    adv_data.scan_rsp_data.len = _srDataLen;

    int advType = 1;
    if (this->_connectable) {
      advType = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    } else {
      if (this->_hasScanData) {
        advType = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
      } else {
        advType = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
      }
    }

    ble_gap_adv_params_t adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.properties.type = advType;
    adv_params.p_peer_addr   = NULL;
    adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
    adv_params.interval      = (this->_advertisingInterval * 16) / 10; // advertising interval (in units of 0.625 ms)
    adv_params.duration      = 0;
    adv_params.primary_phy   = BLE_GAP_PHY_1MBPS; // Must be changed to connect in long range. (BLE_GAP_PHY_CODED)
    adv_params.secondary_phy = BLE_GAP_PHY_1MBPS;

    rc = sd_ble_gap_adv_set_configure(&this->_advHandle, &adv_data, &adv_params);
    checkReturnCode("sd_ble_gap_adv_set_configure (a)", rc);

	// set advertising power (non S110)
    rc = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, this->_advHandle, 0);
    checkReturnCode("sd_ble_gap_tx_power_set (c)", rc);

	// start advertising (non S110)
    rc = sd_ble_gap_adv_start(this->_advHandle, BLE_CONN_CFG_TAG_SERIAL);
    checkReturnCode("sd_ble_gap_adv_start (c)", rc);

  #endif // non-S110 advertising setup
}

void nRF51822::updateAdvertising() {
  int rc;

  #ifdef S110
    rc = sd_ble_gap_adv_data_set(this->_advData, this->_advDataLen, NULL, 0); // untested
    checkReturnCode("sd_ble_gap_adv_data_set (S110)", rc);
  #else
    ble_gap_adv_data_t adv_data;
    memset(&adv_data, 0, sizeof(adv_data));
    adv_data.adv_data.p_data = _advData;
    adv_data.adv_data.len = _advDataLen;
    adv_data.scan_rsp_data.p_data = NULL;
    adv_data.scan_rsp_data.len = 0;

    // must clear current adverstising data before changing
    rc = sd_ble_gap_adv_set_configure(&this->_advHandle, NULL, NULL);
    checkReturnCode("sd_ble_gap_adv_set_configure (b)", rc);

    rc = sd_ble_gap_adv_set_configure(&this->_advHandle, &adv_data, NULL);
    checkReturnCode("sd_ble_gap_adv_set_configure (c)", rc);
  #endif
}

void nRF51822::disconnect() {
  sd_ble_gap_disconnect(this->_connectionHandle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}

void nRF51822::requestAddress() {
  ble_gap_addr_t gapAddress;

  #ifdef S110
    sd_ble_gap_address_get(&gapAddress);
  #else
    sd_ble_gap_addr_get(&gapAddress);
  #endif

  if (this->_eventListener) {
    this->_eventListener->BLEDeviceAddressReceived(*this, gapAddress.addr);
  }
}

#endif
