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

#endif /* MAIN_GARAGEDOOR_H_ */
