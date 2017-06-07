/*
 * bt_config.c
 *
 *  Created on: May 7, 2017
 *      Author: kevin
 */

#include "esp_system.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_log.h"
#include "bt_config.h"

#define GATTS_TABLE_TAG  "GARAGE_BT_TABLE"

#define GARAGE_BT_SERVICE_UUID 0x0100
#define GARAGE_BT_SSID_UUID 0x0200
#define GARAGE_BT_PASSWORD_UUID 0x0201
#define GARAGE_BT_OTA_UUID 0x0202
#define GARAGE_BT_MQTT_UUID 0x0203
#define GARAGE_BT_SBMT_UUID 0x204
#define NUM_PROFILES 1
#define GARAGE_BT_PROFILE_NUM 0
#define GARAGE_BT_SVC_INST_ID 0
#define GARAGE_BT_APP_ID 0x01
#define UUID_SIZE 16
#define SSID_MAX_LENGTH 32
#define PASSWORD_MAX_LENGTH 64
#define URL_MAX_LENGTH 128


// 00000000-401e-11e7-a919-92ebcb67fe33
static uint8_t garage_bt_base_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0x33, 0xfe, 0x67, 0xcb, 0xeb, 0x92, 0x19, 0xa9, 0xe7, 0x11, 0x1e, 0x40, 0x00, 0x00, 0x00, 0x00,
};


#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
//static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
//static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE|ESP_GATT_CHAR_PROP_BIT_READ;

// Pointers to memory that must be freed
static uint8_t *service_uuid;
static uint8_t *ssid_uuid;
static uint8_t *password_uuid;
static uint8_t *http_uuid;
static uint8_t *mqtt_uuid;
static uint8_t *sbmt_uuid;
static uint8_t *ssid_str;
static uint8_t *password_str;
static uint8_t *ota_str;
static uint8_t *mqtt_str;
static esp_ble_adv_data_t* scan_resp_data;
static esp_gatts_attr_db_t* garage_bt_gatt_db;
uint16_t *handle_table;
static gatts_profile_inst *garage_bt_profile_tab;


static uint8_t submit_val=0;
static uint16_t ssid_len;
static uint16_t password_len;
static uint16_t ota_len;
static uint16_t mqtt_len;

static const esp_ble_adv_params_t url_button_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_NONCONN_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static const esp_ble_adv_data_t garage_bt_scan_resp_config = {
    .set_scan_rsp = true,
    .include_name = false,
    .include_txpower = false,
    .manufacturer_len =0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = NULL,
    .p_service_uuid = NULL,
};

static const esp_ble_adv_data_t garage_bt_adv_config = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};


static const esp_gatts_attr_db_t garage_bt_const_gatt_db[URLB_NB] =
{
    // URL Button Service Declaration

	[URLB_SVC]                            =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
	  sizeof(garage_bt_base_uuid), sizeof(garage_bt_base_uuid), (uint8_t *)&garage_bt_base_uuid}},

	// URL Button SSID Characteristic Declaration
	[URLB_SSID_CHAR]            =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	  CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

	// URL Button SSID Value
	[URLB_SSID_VAL]             	=
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&garage_bt_base_uuid, ESP_GATT_PERM_WRITE|ESP_GATT_PERM_READ,
	  SSID_MAX_LENGTH,0, NULL}},

	// URL Button Password Characteristic Declaration
	[URLB_PASSWORD_CHAR]           =
	{{ESP_GATT_AUTO_RSP},{ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	  CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

	// URL Button Password Value
	[URLB_PASSWORD_VAL]             	=
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&garage_bt_base_uuid, ESP_GATT_PERM_WRITE,
	  PASSWORD_MAX_LENGTH,0, NULL}},

	// URL Button HTTP Characteristic Declaration
	[URLB_OTA_CHAR]            =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	  CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

	// URL Button HTTP Value
	[URLB_OTA_VAL]             	=
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&garage_bt_base_uuid, ESP_GATT_PERM_WRITE|ESP_GATT_PERM_READ,
	  URL_MAX_LENGTH,0, NULL}},

	// URL Button MQTT Characteristic Declaration
	[URLB_MQTT_CHAR]            =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	  CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

	// URL Button MQTT Value
	[URLB_MQTT_VAL]             	=
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&garage_bt_base_uuid, ESP_GATT_PERM_WRITE|ESP_GATT_PERM_READ,
	  URL_MAX_LENGTH,0, NULL}},
	// URL Button MQTT Characteristic Declaration
	[URLB_SBMT_CHAR]            =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	  CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

	// URL Button MQTT Value
	[URLB_SBMT_VAL]             	=
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&garage_bt_base_uuid, ESP_GATT_PERM_WRITE|ESP_GATT_PERM_READ,
	  1,0, NULL}},
};
static void url_button_profile_event_handler(esp_gatts_cb_event_t event,esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    	ESP_LOGI(GATTS_TABLE_TAG,"Start Advertising");
        esp_ble_gap_start_advertising((esp_ble_adv_params_t *)&url_button_adv_params);
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
    	ESP_LOGI(GATTS_TABLE_TAG,"Scan Rsp Data Set");
    	break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed\n");
        }
        break;
    default:
        break;
    }
}

#ifdef NOTDEF
static void log_uuid(uint8_t *uuid)
{
	ESP_LOGI("UUID","%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x\n",uuid[15],uuid[14],uuid[13],uuid[12],uuid[11],uuid[10],uuid[9],uuid[8],
			uuid[7],uuid[6],uuid[5],uuid[4],uuid[3],uuid[2],uuid[1],uuid[0]);
}
#endif
static void make_uuid(uint8_t *base_uuid, uint8_t *target_uuid, uint16_t uuid_16)
{
	memcpy(target_uuid,base_uuid,16);

	target_uuid[12] = uuid_16 & 0xff;
	target_uuid[13] = (uuid_16 >> 8) & 0xff;

}

size_t get_nvs_value(nvs_handle n_handle, char *value_name, uint8_t** value, uint16_t max_length)
{
    size_t len;
    esp_err_t err;

    *value=(uint8_t *)malloc(max_length);
    if(n_handle != 0)
    	err = nvs_get_str(n_handle,value_name,NULL,&len);
    else
    	err = ESP_ERR_NVS_NOT_FOUND;

    if (err == ESP_OK)
    {
    	nvs_get_str(n_handle,value_name,(char *)*value,&len);
    	ESP_LOGI("NVS","NVS Value for %s is %s",value_name,*value);
    }
    else if (err == ESP_ERR_NVS_NOT_FOUND)
    {
    	*value[0]=1;
    	len=0;
    	ESP_LOGI("NVS","No value for %s %p  ",value_name, *value);
    }
    else
    	ESP_ERROR_CHECK(err);

    return len;

}

static void allocate_bt_memory()
{
	esp_err_t err = nvs_flash_init();
	ESP_ERROR_CHECK( err );

	garage_bt_profile_tab = (gatts_profile_inst *)malloc(sizeof(esp_gatts_attr_db_t[NUM_PROFILES]));
	garage_bt_profile_tab[GARAGE_BT_PROFILE_NUM].gatts_cb = url_button_profile_event_handler;
	garage_bt_profile_tab[GARAGE_BT_PROFILE_NUM].gatts_if = ESP_GATT_IF_NONE;
	garage_bt_profile_tab[GARAGE_BT_PROFILE_NUM].app_id = GARAGE_BT_APP_ID;

	garage_bt_gatt_db=(esp_gatts_attr_db_t *)malloc(sizeof(esp_gatts_attr_db_t[URLB_NB]));
	memcpy(garage_bt_gatt_db,&garage_bt_const_gatt_db,sizeof(esp_gatts_attr_db_t[URLB_NB]));


	service_uuid = (uint8_t *)malloc(UUID_SIZE);
	ssid_uuid = (uint8_t *)malloc(UUID_SIZE);
	password_uuid=(uint8_t *)malloc(UUID_SIZE);
	http_uuid=(uint8_t *)malloc(UUID_SIZE);
	mqtt_uuid=(uint8_t *)malloc(UUID_SIZE);
	sbmt_uuid=(uint8_t *)malloc(UUID_SIZE);

	make_uuid(garage_bt_base_uuid, service_uuid, GARAGE_BT_SERVICE_UUID);
	make_uuid(garage_bt_base_uuid,ssid_uuid, GARAGE_BT_SSID_UUID);
	make_uuid(garage_bt_base_uuid,password_uuid,GARAGE_BT_PASSWORD_UUID);
	make_uuid(garage_bt_base_uuid,http_uuid,GARAGE_BT_OTA_UUID);
	make_uuid(garage_bt_base_uuid,mqtt_uuid,GARAGE_BT_MQTT_UUID);
	make_uuid(garage_bt_base_uuid,sbmt_uuid,GARAGE_BT_SBMT_UUID);

	garage_bt_gatt_db[URLB_SVC].att_desc.value=service_uuid;

    nvs_handle nvs_handle;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &nvs_handle);
    if(err == ESP_ERR_NVS_NOT_FOUND)
    	nvs_handle = 0l;
    else
    	ESP_ERROR_CHECK(err);

    size_t len=get_nvs_value(nvs_handle,NVS_SSID,&ssid_str,garage_bt_gatt_db[URLB_SSID_VAL].att_desc.max_length);
    ssid_len=len;
    garage_bt_gatt_db[URLB_SSID_VAL].att_desc.uuid_p=ssid_uuid;
    garage_bt_gatt_db[URLB_SSID_VAL].att_desc.length=ssid_len;
    garage_bt_gatt_db[URLB_SSID_VAL].att_desc.value=ssid_str;

	len=get_nvs_value(nvs_handle,NVS_PASSWORD,&password_str,garage_bt_gatt_db[URLB_PASSWORD_VAL].att_desc.max_length);
	password_len=len;
	garage_bt_gatt_db[URLB_PASSWORD_VAL].att_desc.uuid_p=password_uuid;
	garage_bt_gatt_db[URLB_PASSWORD_VAL].att_desc.length=password_len;
	garage_bt_gatt_db[URLB_PASSWORD_VAL].att_desc.value=password_str;

	len=get_nvs_value(nvs_handle,NVS_OTA,&ota_str, garage_bt_gatt_db[URLB_OTA_VAL].att_desc.max_length);
	ota_len=len;
	garage_bt_gatt_db[URLB_OTA_VAL].att_desc.uuid_p=http_uuid;
	garage_bt_gatt_db[URLB_OTA_VAL].att_desc.length=ota_len;
	garage_bt_gatt_db[URLB_OTA_VAL].att_desc.value=ota_str;

	len=get_nvs_value(nvs_handle,NVS_MQTT,&mqtt_str, garage_bt_gatt_db[URLB_MQTT_VAL].att_desc.max_length);
	mqtt_len=len;
	garage_bt_gatt_db[URLB_MQTT_VAL].att_desc.uuid_p=mqtt_uuid;
	garage_bt_gatt_db[URLB_MQTT_VAL].att_desc.length=mqtt_len;
	garage_bt_gatt_db[URLB_MQTT_VAL].att_desc.value=mqtt_str;

	garage_bt_gatt_db[URLB_SBMT_VAL].att_desc.uuid_p=sbmt_uuid;
	garage_bt_gatt_db[URLB_SBMT_VAL].att_desc.length=1;
	garage_bt_gatt_db[URLB_SBMT_VAL].att_desc.value=&submit_val;

	handle_table=(uint16_t *)malloc(sizeof(uint16_t *) * URLB_NB);

	scan_resp_data = (esp_ble_adv_data_t *)malloc(sizeof(esp_ble_adv_data_t));
	memcpy(scan_resp_data,&garage_bt_scan_resp_config,sizeof(esp_ble_adv_data_t));
	scan_resp_data->p_service_uuid=service_uuid;
	nvs_close(nvs_handle);
}

static void set_advertising()
{
	esp_ble_gap_set_device_name("GarageDoor");
	esp_ble_gap_config_adv_data((esp_ble_adv_data_t *)&garage_bt_adv_config);
	esp_ble_gap_config_adv_data(scan_resp_data);

}

static void free_bt_memory()
{

	free(service_uuid);
	free(ssid_uuid);
	free(password_uuid);
	free(http_uuid);
	free(mqtt_uuid);
	free(ssid_str);
	free(garage_bt_gatt_db);
	free(scan_resp_data);
	free(handle_table);
}

static void write_nvs_key(uint8_t *value,uint16_t length,nvs_handle nvs_handle,char *nvs_key)
{

	value[length]='\0';
	if(length==0)
		nvs_erase_key(nvs_handle,nvs_key);
	else
		nvs_set_str(nvs_handle,nvs_key,(char *)value);

	ESP_LOGI("NVS_WRITE","Key %s, Value %s, Length %d", nvs_key,value,length);


}
static void submit_config(uint8_t *submit,esp_gatt_if_t gatts_if)
{
	nvs_handle nvs_handle;

	esp_ble_gatts_close(gatts_if,garage_bt_profile_tab[GARAGE_BT_PROFILE_NUM].conn_id);

	if(*submit)
	{
		esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
		ESP_ERROR_CHECK(err);

		write_nvs_key(ssid_str,ssid_len,nvs_handle,NVS_SSID);
		write_nvs_key(password_str,password_len,nvs_handle,NVS_PASSWORD);
		write_nvs_key(ota_str,ota_len,nvs_handle,NVS_OTA);
		write_nvs_key(mqtt_str,mqtt_len,nvs_handle,NVS_MQTT);

		nvs_commit(nvs_handle);
		nvs_close(nvs_handle);
	}

	esp_ble_gatts_stop_service(handle_table[URLB_SVC]);
	esp_restart();

}

static void store_string(uint8_t *value, uint16_t *current_len, uint8_t *segment, uint16_t offset,uint16_t len, uint16_t max_len)
{
	if(*current_len + offset > max_len)
		return;

	memcpy(value + offset,segment,len);
	*current_len = offset + len;
}

static void url_button_profile_event_handler(esp_gatts_cb_event_t event,
										   esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI("PROFILE_HAN", "event = %d\n",event);
    switch (event) {
    	case ESP_GATTS_REG_EVT:
		    ESP_LOGI("PROFILE_HAN", "App ID %x\n", param->reg.app_id);

			set_advertising();

        	ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        	esp_ble_gatts_create_attr_tab(garage_bt_gatt_db, gatts_if,
        								URLB_NB, GARAGE_BT_SVC_INST_ID);

       	break;
    	case ESP_GATTS_READ_EVT:
    		ESP_LOGI("PROFILE_HAN","Read Conn %x, Trans %x, Handle %x, Offset %d Long %x Resp %x\n",
    				param->read.conn_id,param->read.trans_id,param->read.handle,param->read.offset,param->read.is_long,
					param->read.need_rsp);
       	 break;
    	case ESP_GATTS_WRITE_EVT:
    		ESP_LOGI("PROFILE_HAN","Write Conn %x, Trans %x, Handle %x, Offset %d Long %x Resp %x,Len %d, Value %s \n",
    				param->write.conn_id,param->write.trans_id,param->write.handle,param->write.offset,
					param->write.is_prep,param->write.need_rsp,param->write.len, param->write.value);

    		if(param->write.handle == handle_table[URLB_SSID_VAL])
    			store_string(ssid_str, &ssid_len, param->write.value, param->write.offset, param->write.len, SSID_MAX_LENGTH);
    		if(param->write.handle == handle_table[URLB_PASSWORD_VAL])
    			store_string(password_str, &password_len, param->write.value, param->write.offset, param->write.len, PASSWORD_MAX_LENGTH);
    		if(param->write.handle == handle_table[URLB_OTA_VAL])
    			store_string(ota_str, &ota_len, param->write.value, param->write.offset, param->write.len, URL_MAX_LENGTH);
    		if(param->write.handle == handle_table[URLB_MQTT_VAL])
    			store_string(mqtt_str, &mqtt_len, param->write.value, param->write.offset, param->write.len, URL_MAX_LENGTH);

    		if(param->write.handle == handle_table[URLB_SBMT_VAL])
    			submit_config(param->write.value,gatts_if);
      	 	break;
    	case ESP_GATTS_EXEC_WRITE_EVT:
    		ESP_LOGI("PROFILE_HAN","Write Exec Conn %x, Trans %x, Exec Flag %x \n",
				param->exec_write.conn_id,param->exec_write.trans_id,param->exec_write.exec_write_flag);

    		break;
    	case ESP_GATTS_MTU_EVT:
    		break;
   	    case ESP_GATTS_CONF_EVT:
   	    	break;
    	case ESP_GATTS_UNREG_EVT:
    		break;
    	case ESP_GATTS_CREATE_EVT:
        	break;
    	case ESP_GATTS_ADD_INCL_SRVC_EVT:
    		break;
    	case ESP_GATTS_ADD_CHAR_EVT:
    		ESP_LOGI("PROFILE_HAN","Add Char Attr Han %x Char UUID %x Service Handle %x Status %x \n",
    				param->add_char.attr_handle,param->add_char.char_uuid.uuid.uuid128[12],param->add_char.service_handle,param->add_char.status);
    		break;
    	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
    		break;
    	case ESP_GATTS_DELETE_EVT:
    		ESP_LOGI("PROFILE_HAN","Delete Event");
    		esp_bluedroid_disable();
    		esp_bluedroid_deinit();
    		free_bt_memory();
        	break;
    	case ESP_GATTS_START_EVT:
        	break;
    	case ESP_GATTS_STOP_EVT:
    		ESP_LOGI("PROFILE_HAN","Stop Event");
    		esp_ble_gatts_delete_service(handle_table[URLB_SVC]);
        	break;
    	case ESP_GATTS_CONNECT_EVT:
    		ESP_LOGI("PROFILE_HAN","Connect Event");
    		esp_ble_gap_stop_advertising();
    		garage_bt_profile_tab[GARAGE_BT_PROFILE_NUM].conn_id=param->connect.conn_id;
        	break;
    	case ESP_GATTS_DISCONNECT_EVT:
    		break;
    	case ESP_GATTS_OPEN_EVT:
    		break;
    	case ESP_GATTS_CANCEL_OPEN_EVT:
    		break;
    	case ESP_GATTS_CLOSE_EVT:
    		break;
    	case ESP_GATTS_LISTEN_EVT:
    		break;
    	case ESP_GATTS_CONGEST_EVT:
    		break;
    	case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
    		ESP_LOGI("PROFILE_HAN", "The number handle =%x\n",
    				param->add_attr_tab.num_handle);
    		if(param->add_attr_tab.num_handle == URLB_NB){
    			memcpy(handle_table, param->add_attr_tab.handles,
					sizeof(uint16_t*) * URLB_NB);
    			esp_ble_gatts_start_service(handle_table[URLB_SVC]);
    			for(int i=0;i<URLB_NB;i++)
    			{
    				ESP_LOGI("PROFILE_HAN","%d = %x\n",i,handle_table[i]);
    			}
    		}

    		break;
    	case ESP_GATTS_SET_ATTR_VAL_EVT:
    		break;
	}

    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
		esp_ble_gatts_cb_param_t *param)
{
	ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d\n", event, gatts_if);

	/* If event is register event, store the gatts_if for each profile */
	if (event == ESP_GATTS_REG_EVT) {
		if (param->reg.status == ESP_GATT_OK) {
			garage_bt_profile_tab[GARAGE_BT_PROFILE_NUM].gatts_if = gatts_if;
		} else {
			ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n",
					param->reg.app_id,
					param->reg.status);
			return;
		}
	}

	do {
		int idx;
		for (idx = 0; idx < NUM_PROFILES; idx++) {
			if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
					gatts_if == garage_bt_profile_tab[idx].gatts_if) {
				if (garage_bt_profile_tab[idx].gatts_cb) {
					garage_bt_profile_tab[idx].gatts_cb(event, gatts_if, param);
				}
			}
		}
	} while (0);
}

void initBluetooth(void)
{
    esp_err_t ret;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed\n", __func__);
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed\n", __func__);
        return;
    }

    allocate_bt_memory();
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(GARAGE_BT_APP_ID);
    return;
}


