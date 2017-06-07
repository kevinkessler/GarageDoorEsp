/*
 * rgcled.h
 *
 *  Created on: May 16, 2017
 *      Author: kevin
 */

#ifndef COMPONENTS_RGBLED_INCLUDE_RGBLED_H_
#define COMPONENTS_RGBLED_INCLUDE_RGBLED_H_


#define RGBLED_TASK_PRIORITY 2

#define RGBLED_LOG "RGBLED"

#define RGB_BLINK_NONE 0x00
#define RGB_BLINK_SLOW 0x01
#define RGB_BLINK_FAST 0x02

// Since the LED is common anode, the ground is switched, so a duty cycle of 1024 is never ground, and is off
#define DUTY_CYCLE(x) (1024 - 1024*((x)/100))

enum {
	RGB_NONE,
	RGB_RED,
	RGB_GREEN,
	RGB_BLUE,
	RGB_YELLOW,
	RGB_WHITE,
	RGB_CYAN,
	RGB_MAGENTA,

	RGB_MAX_COLOR
};



void initRGBLed(void);

void testRGBLed();


#endif /* COMPONENTS_RGBLED_INCLUDE_RGBLED_H_ */
