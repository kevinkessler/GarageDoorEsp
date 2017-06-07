/*
 * bt_config.h
 *
 *  Created on: May 7, 2017
 *      Author: kevin
 */

#ifndef COMPONENTS_BLUETOOTH_INCLUDE_BT_CONFIG_H_
#define COMPONENTS_BLUETOOTH_INCLUDE_BT_CONFIG_H_



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_gatts_api.h"


#define STORAGE_NAMESPACE "URL_BUTTON"
#define NVS_SSID "ssid"
#define NVS_PASSWORD "password"
#define NVS_OTA "ota"
#define NVS_MQTT "mqtt"

void initBluetooth(void);

// URL Button Service Table Index
enum
{
	URLB_SVC,

	URLB_SSID_CHAR,
	URLB_SSID_VAL,

	URLB_PASSWORD_CHAR,
	URLB_PASSWORD_VAL,

	URLB_OTA_CHAR,
	URLB_OTA_VAL,

	URLB_MQTT_CHAR,
	URLB_MQTT_VAL,

	URLB_SBMT_CHAR,
	URLB_SBMT_VAL,

	URLB_NB
};

typedef struct {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;

}gatts_profile_inst;



#endif /* COMPONENTS_BLUETOOTH_INCLUDE_BT_CONFIG_H_ */
