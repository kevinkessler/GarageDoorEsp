#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "mqtt.h"
#include "rgbled.h"
#include "GarageDoor.h"
#include "bt_config.h"

static char* ssid;
static char* password;
static char* ota;
static char* mqtt;

EventGroupHandle_t wifi_event_group;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
    	ESP_LOGI("MQTT","EVENT Start");
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    	ESP_LOGI("MAIN", "got ip:%s\n",ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
    	init_mqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
    	stop_mqtt();
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static uint16_t get_nvs_value(nvs_handle handle,char *value_name, char **value)
{
    size_t len;
    esp_err_t err;

    if(handle != 0)
    	err = nvs_get_str(handle,value_name,NULL,&len);
    else
    	err = ESP_ERR_NVS_NOT_FOUND;

    if (err == ESP_OK)
    {
    	*value=(char *)malloc(len);
    	nvs_get_str(handle,value_name,(char *)*value,&len);
    }
    else if (err == ESP_ERR_NVS_NOT_FOUND)
    {
    	len=0;
    }
    else
    	ESP_ERROR_CHECK(err);

    return len;

}

static bool initWifi()
{

	nvs_handle nvs_handle;
	esp_err_t err;
	wifi_config_t wifi_config = {
	        .sta = {
	            .ssid = "",
	            .password = "",
	        },
	    };;

	err=nvs_flash_init();
	ESP_ERROR_CHECK(err);

    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &nvs_handle);
    if(err == ESP_ERR_NVS_NOT_FOUND)
    	return 0;
    else
    	ESP_ERROR_CHECK(err);

    size_t len;
    ESP_LOGI("MAIN","SSID %p",&ssid);
    len=get_nvs_value(nvs_handle, NVS_SSID, &ssid);
    if(len == 0)
    	return 0;
    ESP_LOGI("MAIN","Got SSID %s",ssid);
    memcpy(wifi_config.sta.ssid,ssid,len);
    wifi_config.sta.ssid[len]='\0';

    len=get_nvs_value(nvs_handle, NVS_PASSWORD, &password);
    if(len == 0)
    	return 0;
    ESP_LOGI("MAIN","Got Password %s",password);
    memcpy(wifi_config.sta.password,password,len);
    wifi_config.sta.password[len]='\0';

    size_t ota_len=get_nvs_value(nvs_handle, NVS_OTA,&ota);
    size_t mqtt_len=get_nvs_value(nvs_handle, NVS_MQTT, &mqtt);

    if(ota_len > 0)
    	ESP_LOGI("MAIN","Got HTTP Url %s %d",ota,ota_len);

    if(mqtt_len > 0)
    	ESP_LOGI("MAIN","GOT MQTT Url %s %d",mqtt, mqtt_len);

    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

    ESP_LOGI("MAIN", "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
	return 1;
}

void app_main(void)
{
/*	initRGBLed();
	ESP_LOGI("MAIN","initBack");
	if(!initWifi())
	{
		setRGBColor(RGB_BLINK_SLOW,RGB_BLUE);
		initBluetooth();
	}
	else
	{
		setRGBColor(RGB_BLINK_SLOW,RGB_GREEN);
		startHttpsTask();
	}*/

	ArduCam_init();
}

