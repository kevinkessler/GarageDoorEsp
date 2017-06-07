/*
 * mqqt.c
 *
 *  Created on: May 23, 2017
 *      Author: kevin
 */

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "mqtt.h"
#include "GarageDoor.h"


void connected_cb(mqtt_client *client, mqtt_event_data_t *event_data)
{
    mqtt_subscribe(client, "/test", 0);
    mqtt_publish(client, "/test", "howdy!", 6, 0, 0);
}

void disconnected_cb(mqtt_client *client, mqtt_event_data_t *event_data)
{

}
void reconnect_cb(mqtt_client *client, mqtt_event_data_t *event_data)
{

}
void subscribe_cb(mqtt_client *client, mqtt_event_data_t *event_data)
{
    ESP_LOGI("MQTT","[APP] Subscribe ok, test publish msg\n");

    mqtt_publish(client, "/test", "abcde", 5, 0, 0);
}

void publish_cb(mqtt_client *client, mqtt_event_data_t *event_data)
{

}
void data_cb(mqtt_client *client, mqtt_event_data_t *event_data)
{

    if (event_data->data_offset == 0) {

        char *topic = malloc(event_data->topic_length + 1);
        memcpy(topic, event_data->topic, event_data->topic_length);
        topic[event_data->topic_length] = 0;
        ESP_LOGI("MQTT","[APP] Publish topic: %s\n", topic);
        free(topic);
    }

    // char *data = malloc(event_data->data_length + 1);
    // memcpy(data, event_data->data, event_data->data_length);
    // data[event_data->data_length] = 0;
    ESP_LOGI("MQTT","[APP] Publish data[%d/%d bytes]\n",
         event_data->data_length + event_data->data_offset,
         event_data->data_total_length);
         // data);

    // free(data);

}

mqtt_settings settings = {
    .host = "pi-hub.home",
    .port = 1883, // unencrypted
    .client_id = "mqtt_client_id",
    .username = "user",
    .password = "pass",
    .clean_session = 0,
    .keepalive = 120,
    .lwt_topic = "/lwt",
    .lwt_msg = "offline",
    .lwt_qos = 0,
    .lwt_retain = 0,
    .connected_cb = connected_cb,
    .disconnected_cb = disconnected_cb,
    .reconnect_cb = reconnect_cb,
    .subscribe_cb = subscribe_cb,
    .publish_cb = publish_cb,
    .data_cb = data_cb
};

void init_mqtt(void)
{
	mqtt_start(&settings);
}

void stop_mqtt()
{
	mqtt_stop();
}

