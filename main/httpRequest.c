/*
 * httpRequest.c
 *
 *  Created on: Jun 6, 2017
 *      Author: kevin
 */

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_request.h"
#include "esp_log.h"
#include "esp_system.h"
#include "GarageDoor.h"

#define HTTPS_TAG "HTTPS"

extern EventGroupHandle_t wifi_event_group;

void https_get_task(void *pvParameters)
{
    request_t *req;
    int status;

	while(1)
	{
		xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
		                            pdTRUE, pdTRUE, portMAX_DELAY);
		ESP_LOGI(HTTPS_TAG,"HTTPS Task awakes");

	    req = req_new("https://google.com");
	    status = req_perform(req);
	    req_clean(req);
	    ESP_LOGI(HTTPS_TAG, "Finish request, status=%d, freemem=%d", status, esp_get_free_heap_size());
	    vTaskDelete(NULL);

	}
}

void startHttpsTask()
{
	xTaskCreate(&https_get_task, "https_get_task", 8192, NULL, 5, NULL);
}

