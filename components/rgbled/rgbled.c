/*
 * rgbled.c
 *
 *  Created on: May 16, 2017
 *      Author: kevin
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "rgbled.h"
#include "GarageDoor.h"

#define RED_LED 25
#define GREEN_LED 26
#define BLUE_LED 27

typedef struct {
	uint16_t red_duty;
	uint16_t green_duty;
	uint16_t blue_duty;
} rgb_color_t;


static const rgb_color_t rgbled_colors[RGB_MAX_COLOR] =
{
		[RGB_NONE] = {DUTY_CYCLE(0),DUTY_CYCLE(0),DUTY_CYCLE(0)},
		[RGB_RED] = {DUTY_CYCLE(100),DUTY_CYCLE(0),DUTY_CYCLE(0)},
		[RGB_GREEN] = {DUTY_CYCLE(0),DUTY_CYCLE(100),DUTY_CYCLE(0)},
		[RGB_BLUE] = {DUTY_CYCLE(0),DUTY_CYCLE(0),DUTY_CYCLE(100)},
		[RGB_YELLOW] = {DUTY_CYCLE(100),DUTY_CYCLE(100),DUTY_CYCLE(0)},
		[RGB_WHITE] = {DUTY_CYCLE(100),DUTY_CYCLE(100),DUTY_CYCLE(100)},
		[RGB_CYAN] = {DUTY_CYCLE(0),DUTY_CYCLE(100),DUTY_CYCLE(100)},
		[RGB_MAGENTA] = {DUTY_CYCLE(100),DUTY_CYCLE(0),DUTY_CYCLE(100)},
};

static const TickType_t blink_time[3] = {
		[RGB_BLINK_NONE] = portMAX_DELAY,
		[RGB_BLINK_SLOW] = pdMS_TO_TICKS(750),
		[RGB_BLINK_FAST] = pdMS_TO_TICKS(250)
};

static uint16_t blinkRate;
static uint16_t cur_color;
static uint8_t blinkToggle=0;

static QueueHandle_t rgbQueue;

static void setPWMColor(uint8_t color_index)
{
	ESP_LOGD(RGBLED_LOG,"Setting LED to Color index %d",color_index);

	ledc_set_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_0, rgbled_colors[color_index].red_duty);
	ledc_update_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_0);

	ledc_set_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_1, rgbled_colors[color_index].green_duty);
	ledc_update_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_1);

	ledc_set_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_2, rgbled_colors[color_index].blue_duty);
	ledc_update_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_2);

}

static void rgbledTask(void *param)
{
	uint32_t rgb_color;

	blinkRate=0;
	while(1)
	{
		if(xQueueReceive(rgbQueue,&rgb_color,blink_time[blinkRate]) !=pdPASS)
		{
			ESP_LOGD(RGBLED_LOG,"Notify Timeout");
			if(blinkToggle)
			{
				setPWMColor(RGB_NONE);
				blinkToggle=0;
			}
			else
			{
				setPWMColor(cur_color);
				blinkToggle=1;
			}
		} else {
			cur_color=rgb_color & 0xff;
			setPWMColor(cur_color);

			blinkRate=(rgb_color & 0xff00) >> 8;
			ESP_LOGD(RGBLED_LOG,"rgb_color %x blinkRate %x, cur_color %x",rgb_color,blinkRate,cur_color);

		}
	}
}

void initRGBLed(void)
{
    ledc_timer_config_t ledc_timer = {
        .bit_num = LEDC_TIMER_10_BIT, //set timer counter bit number
        .freq_hz = 5000,              //set frequency of pwm
        .speed_mode = LEDC_HIGH_SPEED_MODE,   //timer mode,
        .timer_num = LEDC_TIMER_0    //timer index
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = RED_LED,
        .intr_type = LEDC_INTR_DISABLE,
        //set LEDC mode, from ledc_mode_t
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        //set LEDC timer source, if different channel use one timer,
        //the frequency and bit_num of these channels should be the same
        .timer_sel = LEDC_TIMER_0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.channel=LEDC_CHANNEL_1;
    ledc_channel.gpio_num = GREEN_LED;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));


    ledc_channel.channel=LEDC_CHANNEL_2;
    ledc_channel.gpio_num = BLUE_LED;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    rgbQueue=xQueueCreate(1,sizeof(uint16_t));
    BaseType_t err = xTaskCreate(rgbledTask,"RGBLED",1650,NULL,RGBLED_TASK_PRIORITY,NULL);
    if(err == pdFAIL)
    {
    	ESP_LOGE(RGBLED_LOG,"RGBLED Task Create Failed");
    	ESP_ERROR_CHECK(-1);
    }


    //setRGBColor(RGB_BLINK_NONE,RGB_NONE);

    //testRGBLed();
}

void setRGBColor(uint8_t blink,uint8_t color)
{
	uint16_t colorMsg=blink<<8 | color;
	xQueueOverwrite(rgbQueue,&colorMsg);
}

void testRGBLed()
{
    while(1)
    {
    	for(int n=0;n<RGB_MAX_COLOR;n++)
    	{

    		setRGBColor(RGB_BLINK_FAST,n);
    		vTaskDelay(pdMS_TO_TICKS(4000));
    	}
    }

}

