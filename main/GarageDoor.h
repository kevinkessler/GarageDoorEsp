/*
 * GarageDoor.h
 *
 *  Created on: May 23, 2017
 *      Author: kevin
 */

#ifndef MAIN_GARAGEDOOR_H_
#define MAIN_GARAGEDOOR_H_
#include "freertos/event_groups.h"

#define WIFI_CONNECTED_BIT (1<<0)

void init_mqtt(void);
void stop_mqtt(void);
void startHttpsTask(void);
void setRGBColor(uint8_t blink,uint8_t color);
void ArduCam_init(void);
void ArduCam_take_picture(void);
void ArduCam_cleanup(void);
uint8_t ArduCam_read_buffer(uint8_t **buffer, uint16_t *length, uint8_t *is_more);
void initCamera();
void takePicture();


void ArduCam_set_Exposure_level(uint8_t level);

// Make Eclipse stop complaining
#define true 1
#define false 0


#endif /* MAIN_GARAGEDOOR_H_ */
