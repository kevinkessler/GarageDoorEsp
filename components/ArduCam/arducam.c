/*
 * arducam.c
 *
 *  Created on: Jul 6, 2017
 *      Author: kevin
 */

#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "arducam.h"
#include "ov5642_regs.h"
#include "esp_heap_alloc_caps.h"

// Pin assignments
#define ARDUCAM_SDA 21
#define ARDUCAM_SCL 22
#define ARDUCAM_MOSI 18
#define ARDUCAM_MISO 13
#define ARDUCAM_SCK 14
#define ARDUCAM_CS 15
//#define ARDUCAM_SPI_CLOCK_HZ 8000000
#define ARDUCAM_SPI_CLOCK_HZ 1000000

#define ARDUCAM_ADDR 0x78
#define ARDUCAM_I2C_FREQ_HZ 400000
#define ARDUCAM_I2C_PORT 0
#define ARDUCAM_TAG "ArduCam"

// Make Eclipse stop complaining
#define true 1
#define false 0


static spi_device_handle_t spi;

/*
static uint8_t wrSensorReg8_8(uint8_t regID, uint8_t regDat)
{
	uint8_t cmd[2];
	esp_err_t err;

	cmd[0]= (ARDUCAM_ADDR) | I2C_MASTER_WRITE;
	cmd[1] = regID;

	i2c_cmd_handle_t i2c_handle=i2c_cmd_link_create();
    i2c_master_start(i2c_handle);
    i2c_master_write(i2c_handle,cmd,2,true);
    i2c_master_stop(i2c_handle);

    err=i2c_master_cmd_begin(ARDUCAM_I2C_PORT,i2c_handle, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_handle);

    if(err == ESP_OK)
    	return 1;

    return 0;

}
*/

static uint8_t wrSensorReg16_8(uint16_t regID, uint8_t regDat)
{
	uint8_t cmd[3];
	esp_err_t err;

	cmd[0]= (ARDUCAM_ADDR) | I2C_MASTER_WRITE;
	cmd[1] = (regID >> 8) & 0xff;
	cmd[2] = regID & 0xff;

	i2c_cmd_handle_t i2c_handle=i2c_cmd_link_create();
    i2c_master_start(i2c_handle);
    i2c_master_write(i2c_handle,cmd,3,true);
    i2c_master_write_byte(i2c_handle,regDat,true);
    i2c_master_stop(i2c_handle);

    err=i2c_master_cmd_begin(ARDUCAM_I2C_PORT,i2c_handle, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_handle);

    if(err == ESP_OK)
    		return 1;

    ESP_LOGW("ARDUCAM_TAG","wrSensorReg16_8 returned %x",err);
    return 0;

}

static uint8_t wrSensorRegs16_8(const struct sensor_reg reglist[])
{

	const struct sensor_reg *next=reglist;
	uint16_t reg_addr=next->reg;
	uint8_t reg_val=next->val;

	//i2c_cmd_handle_t i2c_handle=i2c_cmd_link_create();

	while((reg_addr != 0xffff) | (reg_val != 0xff))
	{
		wrSensorReg16_8(reg_addr,reg_val);
		next++;
		reg_addr=next->reg;
		reg_val=next->val;


/*		*cmd= (ARDUCAM_ADDR) | I2C_MASTER_WRITE;
		*(cmd+1) = (reg_addr >> 8) & 0xff;
		*(cmd+2) = reg_addr & 0xff;

		ESP_LOGI(ARDUCAM_TAG,"CMD %x %x %x", *cmd,*(cmd+1),*(cmd+2));
	    i2c_master_start(i2c_handle);
	    i2c_master_write(i2c_handle,cmd,3,true);
	    i2c_master_write_byte(i2c_handle,reg_val,true);*/



	}

/*    i2c_master_stop(i2c_handle);

    err=i2c_master_cmd_begin(ARDUCAM_I2C_PORT,i2c_handle, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_handle);
    free(cmd_mem);

    if(err == ESP_OK)
    	return 1;

    ESP_LOGW("ARDUCAM_TAG","wrSensorRegs16_8 returned %x",err);
    	return 0;
*/
	return 1;

}

static uint8_t rdSensorReg16_8(uint16_t regID, uint8_t *regDat)
{
	uint8_t cmd[3];
	esp_err_t err;

	cmd[0]= (ARDUCAM_ADDR) | I2C_MASTER_WRITE;
	cmd[1] = (regID >> 8) & 0xff;
	cmd[2] = regID & 0xff;

	i2c_cmd_handle_t i2c_handle=i2c_cmd_link_create();
    i2c_master_start(i2c_handle);
    i2c_master_write(i2c_handle,cmd,3,true);

    i2c_master_start(i2c_handle);
    i2c_master_write_byte(i2c_handle,(ARDUCAM_ADDR) | I2C_MASTER_READ,true);
    i2c_master_read_byte(i2c_handle,regDat,true);

    i2c_master_stop(i2c_handle);

    err=i2c_master_cmd_begin(ARDUCAM_I2C_PORT,i2c_handle, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_handle);

    if(err == ESP_OK)
    		return 1;

    ESP_LOGW("ARDUCAM_TAG","rdSensorReg16_8 returned %x",err);
    return 0;
}

static void ArduCam_write_reg(uint8_t addr, uint8_t data)
{
	esp_err_t err;

	spi_transaction_t t;
	memset(&t,0,sizeof(t));

	uint8_t spi_data[2];
	spi_data[0] = addr | 0x80;
	spi_data[1] = data;

	t.length=16;
	t.tx_buffer=spi_data;
	err=spi_device_transmit(spi,&t);
	if(err != ESP_OK)
		ESP_LOGW(ARDUCAM_TAG,"spi_device_transmit error %x",err);

}

static uint8_t ArduCam_read_reg(uint8_t addr)
{
	esp_err_t err;
	spi_transaction_t t;
	uint8_t spi_data;

	memset(&t,0,sizeof(t));

	t.length=8;
	spi_data = addr & 0x7f;
	t.tx_buffer=&spi_data;
	err=spi_device_transmit(spi,&t);
	if(err != ESP_OK)
	{
		ESP_LOGI(ARDUCAM_TAG,"read_reg spi_device_transmit error %x",err);
		return 0xff;
	}


	memset(&t,0,sizeof(t));
	t.length=8;
	t.rx_buffer=&spi_data;
	err=spi_device_transmit(spi,&t);
	if (err != ESP_OK)
	{
		ESP_LOGI(ARDUCAM_TAG,"read_reg spi_device_transmit error %x",err);
	}

	return spi_data;
}

void ArduCam_set_RAW_size(uint8_t size)
{
	switch (size)
	{
		case OV5642_640x480:
			wrSensorRegs16_8(OV5642_1280x960_RAW);
			wrSensorRegs16_8(OV5642_640x480_RAW);
			break;
		case OV5642_1280x960:
			wrSensorRegs16_8(OV5642_1280x960_RAW);
			break;
		case OV5642_1920x1080:
			wrSensorRegs16_8(ov5642_RAW);
			wrSensorRegs16_8(OV5642_1920x1080_RAW);
			break;
		case OV5642_2592x1944:
			wrSensorRegs16_8(ov5642_RAW);
			break;
	 }
}

void ArduCam_set_JPEG_size(uint8_t size)
{

  switch (size)
  {
    case OV5642_320x240:
      wrSensorRegs16_8(ov5642_320x240);
      break;
    case OV5642_640x480:
      wrSensorRegs16_8(ov5642_640x480);
      break;
    case OV5642_1024x768:
      wrSensorRegs16_8(ov5642_1024x768);
      break;
    case OV5642_1280x960:
      wrSensorRegs16_8(ov5642_1280x960);
      break;
    case OV5642_1600x1200:
      wrSensorRegs16_8(ov5642_1600x1200);
      break;
    case OV5642_2048x1536:
      wrSensorRegs16_8(ov5642_2048x1536);
      break;
    case OV5642_2592x1944:
      wrSensorRegs16_8(ov5642_2592x1944);
      break;
    default:
      wrSensorRegs16_8(ov5642_320x240);
      break;
  }
}


void ArduCam_set_Light_Mode(uint8_t Light_Mode)
{
	switch(Light_Mode)
	{
		case Advanced_AWB:
			wrSensorReg16_8(0x3406 ,0x0 );
			wrSensorReg16_8(0x5192 ,0x04);
			wrSensorReg16_8(0x5191 ,0xf8);
			wrSensorReg16_8(0x518d ,0x26);
			wrSensorReg16_8(0x518f ,0x42);
			wrSensorReg16_8(0x518e ,0x2b);
			wrSensorReg16_8(0x5190 ,0x42);
			wrSensorReg16_8(0x518b ,0xd0);
			wrSensorReg16_8(0x518c ,0xbd);
			wrSensorReg16_8(0x5187 ,0x18);
			wrSensorReg16_8(0x5188 ,0x18);
			wrSensorReg16_8(0x5189 ,0x56);
			wrSensorReg16_8(0x518a ,0x5c);
			wrSensorReg16_8(0x5186 ,0x1c);
			wrSensorReg16_8(0x5181 ,0x50);
			wrSensorReg16_8(0x5184 ,0x20);
			wrSensorReg16_8(0x5182 ,0x11);
			wrSensorReg16_8(0x5183 ,0x0 );
			break;
		case Simple_AWB:
			wrSensorReg16_8(0x3406 ,0x00);
			wrSensorReg16_8(0x5183 ,0x80);
			wrSensorReg16_8(0x5191 ,0xff);
			wrSensorReg16_8(0x5192 ,0x00);
			break;
		case Manual_day:
			wrSensorReg16_8(0x3406 ,0x1 );
			wrSensorReg16_8(0x3400 ,0x7 );
			wrSensorReg16_8(0x3401 ,0x32);
			wrSensorReg16_8(0x3402 ,0x4 );
			wrSensorReg16_8(0x3403 ,0x0 );
			wrSensorReg16_8(0x3404 ,0x5 );
			wrSensorReg16_8(0x3405 ,0x36);
			break;
		case Manual_A:
			wrSensorReg16_8(0x3406 ,0x1 );
			wrSensorReg16_8(0x3400 ,0x4 );
			wrSensorReg16_8(0x3401 ,0x88);
			wrSensorReg16_8(0x3402 ,0x4 );
			wrSensorReg16_8(0x3403 ,0x0 );
			wrSensorReg16_8(0x3404 ,0x8 );
			wrSensorReg16_8(0x3405 ,0xb6);
			break;
		case Manual_cwf:
			wrSensorReg16_8(0x3406 ,0x1 );
			wrSensorReg16_8(0x3400 ,0x6 );
			wrSensorReg16_8(0x3401 ,0x13);
			wrSensorReg16_8(0x3402 ,0x4 );
			wrSensorReg16_8(0x3403 ,0x0 );
			wrSensorReg16_8(0x3404 ,0x7 );
			wrSensorReg16_8(0x3405 ,0xe2);
			break;
		case Manual_cloudy:
			wrSensorReg16_8(0x3406 ,0x1 );
			wrSensorReg16_8(0x3400 ,0x7 );
			wrSensorReg16_8(0x3401 ,0x88);
			wrSensorReg16_8(0x3402 ,0x4 );
			wrSensorReg16_8(0x3403 ,0x0 );
			wrSensorReg16_8(0x3404 ,0x5 );
			wrSensorReg16_8(0x3405 ,0x0);
			break;
		default :
			break;
	 }
}

void ArduCam_set_Color_Saturation(uint8_t Color_Saturation)
{
	switch(Color_Saturation)
	{
		case Saturation4:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5583 ,0x80);
			wrSensorReg16_8(0x5584 ,0x80);
			wrSensorReg16_8(0x5580 ,0x02);
		break;
		case Saturation3:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5583 ,0x70);
			wrSensorReg16_8(0x5584 ,0x70);
			wrSensorReg16_8(0x5580 ,0x02);
		break;
		case Saturation2:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5583 ,0x60);
			wrSensorReg16_8(0x5584 ,0x60);
			wrSensorReg16_8(0x5580 ,0x02);
		break;
		case Saturation1:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5583 ,0x50);
			wrSensorReg16_8(0x5584 ,0x50);
			wrSensorReg16_8(0x5580 ,0x02);
		break;
		case Saturation0:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5583 ,0x40);
			wrSensorReg16_8(0x5584 ,0x40);
			wrSensorReg16_8(0x5580 ,0x02);
		break;
		case Saturation_1:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5583 ,0x30);
			wrSensorReg16_8(0x5584 ,0x30);
			wrSensorReg16_8(0x5580 ,0x02);
		break;
			case Saturation_2:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5583 ,0x20);
			wrSensorReg16_8(0x5584 ,0x20);
			wrSensorReg16_8(0x5580 ,0x02);
		break;
			case Saturation_3:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5583 ,0x10);
			wrSensorReg16_8(0x5584 ,0x10);
			wrSensorReg16_8(0x5580 ,0x02);
		break;
			case Saturation_4:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5583 ,0x00);
			wrSensorReg16_8(0x5584 ,0x00);
			wrSensorReg16_8(0x5580 ,0x02);
		break;
	}
}

void ArduCam_set_Brightness(uint8_t Brightness)
{

	switch(Brightness)
	{
		case Brightness4:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5589 ,0x40);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x558a ,0x00);
		break;
		case Brightness3:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5589 ,0x30);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x558a ,0x00);
		break;
		case Brightness2:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5589 ,0x20);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x558a ,0x00);
		break;
		case Brightness1:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5589 ,0x10);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x558a ,0x00);
		break;
		case Brightness0:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5589 ,0x00);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x558a ,0x00);
		break;
		case Brightness_1:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5589 ,0x10);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x558a ,0x08);
		break;
		case Brightness_2:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5589 ,0x20);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x558a ,0x08);
		break;
		case Brightness_3:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5589 ,0x30);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x558a ,0x08);
		break;
		case Brightness_4:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5589 ,0x40);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x558a ,0x08);
		break;
	}

}


void ArduCam_set_Contrast(uint8_t Contrast)
{
	switch(Contrast)
	{
		case Contrast4:
	  	wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x30);
			wrSensorReg16_8(0x5588 ,0x30);
			wrSensorReg16_8(0x558a ,0x00);
		break;
		case Contrast3:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x2c);
			wrSensorReg16_8(0x5588 ,0x2c);
			wrSensorReg16_8(0x558a ,0x00);
		break;
		case Contrast2:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x28);
			wrSensorReg16_8(0x5588 ,0x28);
			wrSensorReg16_8(0x558a ,0x00);
		break;
		case Contrast1:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x24);
			wrSensorReg16_8(0x5588 ,0x24);
			wrSensorReg16_8(0x558a ,0x00);
		break;
		case Contrast0:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x20);
			wrSensorReg16_8(0x5588 ,0x20);
			wrSensorReg16_8(0x558a ,0x00);
		break;
		case Contrast_1:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x1C);
			wrSensorReg16_8(0x5588 ,0x1C);
			wrSensorReg16_8(0x558a ,0x1C);
		break;
		case Contrast_2:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x18);
			wrSensorReg16_8(0x5588 ,0x18);
			wrSensorReg16_8(0x558a ,0x00);
		break;
		case Contrast_3:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x14);
			wrSensorReg16_8(0x5588 ,0x14);
			wrSensorReg16_8(0x558a ,0x00);
		break;
		case Contrast_4:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x10);
			wrSensorReg16_8(0x5588 ,0x10);
			wrSensorReg16_8(0x558a ,0x00);
		break;
	}
}


void ArduCam_set_hue(uint8_t degree)
{
	switch(degree)
	{
		case degree_180:
	  	wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x01);
			wrSensorReg16_8(0x5581 ,0x80);
			wrSensorReg16_8(0x5582 ,0x00);
			wrSensorReg16_8(0x558a ,0x32);
		break;
		case degree_150:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x01);
			wrSensorReg16_8(0x5581 ,0x6f);
			wrSensorReg16_8(0x5582 ,0x40);
			wrSensorReg16_8(0x558a ,0x32);
		break;
		case degree_120:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x01);
			wrSensorReg16_8(0x5581 ,0x40);
			wrSensorReg16_8(0x5582 ,0x6f);
			wrSensorReg16_8(0x558a ,0x32);
		break;
		case degree_90:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x01);
			wrSensorReg16_8(0x5581 ,0x00);
			wrSensorReg16_8(0x5582 ,0x80);
			wrSensorReg16_8(0x558a ,0x02);
		break;
		case degree_60:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x01);
			wrSensorReg16_8(0x5581 ,0x40);
			wrSensorReg16_8(0x5582 ,0x6f);
			wrSensorReg16_8(0x558a ,0x02);
		break;
		case degree_30:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x01);
			wrSensorReg16_8(0x5581 ,0x6f);
			wrSensorReg16_8(0x5582 ,0x40);
			wrSensorReg16_8(0x558a ,0x02);
		break;
		case degree_0:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x01);
			wrSensorReg16_8(0x5581 ,0x80);
			wrSensorReg16_8(0x5582 ,0x00);
			wrSensorReg16_8(0x558a ,0x01);
		break;
		case degree30:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x01);
			wrSensorReg16_8(0x5581 ,0x6f);
			wrSensorReg16_8(0x5582 ,0x40);
			wrSensorReg16_8(0x558a ,0x01);
		break;
		case degree60:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x01);
			wrSensorReg16_8(0x5581 ,0x40);
			wrSensorReg16_8(0x5582 ,0x6f);
			wrSensorReg16_8(0x558a ,0x01);
		break;
		case degree90:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x01);
			wrSensorReg16_8(0x5581 ,0x00);
			wrSensorReg16_8(0x5582 ,0x80);
			wrSensorReg16_8(0x558a ,0x31);
		break;
		case degree120:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x01);
			wrSensorReg16_8(0x5581 ,0x40);
			wrSensorReg16_8(0x5582 ,0x6f);
			wrSensorReg16_8(0x558a ,0x31);
		break;
		case degree150:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x01);
			wrSensorReg16_8(0x5581 ,0x6f);
			wrSensorReg16_8(0x5582 ,0x40);
			wrSensorReg16_8(0x558a ,0x31);
		break;
	}

}


void ArduCam_set_Special_effects(uint8_t Special_effect)
{
	switch(Special_effect)
	{
		case Bluish:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x18);
			wrSensorReg16_8(0x5585 ,0xa0);
			wrSensorReg16_8(0x5586 ,0x40);
		break;
		case Greenish:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x18);
			wrSensorReg16_8(0x5585 ,0x60);
			wrSensorReg16_8(0x5586 ,0x60);
		break;
		case Reddish:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x18);
			wrSensorReg16_8(0x5585 ,0x80);
			wrSensorReg16_8(0x5586 ,0xc0);
		break;
		case BW:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x18);
			wrSensorReg16_8(0x5585 ,0x80);
			wrSensorReg16_8(0x5586 ,0x80);
		break;
		case Negative:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x40);
		break;

			case Sepia:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x18);
			wrSensorReg16_8(0x5585 ,0x40);
			wrSensorReg16_8(0x5586 ,0xa0);
		break;
		case Normal:
			wrSensorReg16_8(0x5001 ,0x7f);
			wrSensorReg16_8(0x5580 ,0x00);
		break;
	}
}

void ArduCam_set_Exposure_level(uint8_t level)
{
	switch(level)
	{
		case Exposure_17_EV:
	  	wrSensorReg16_8(0x3a0f ,0x10);
			wrSensorReg16_8(0x3a10 ,0x08);
			wrSensorReg16_8(0x3a1b ,0x10);
			wrSensorReg16_8(0x3a1e ,0x08);
			wrSensorReg16_8(0x3a11 ,0x20);
			wrSensorReg16_8(0x3a1f ,0x10);
		break;
		case Exposure_13_EV:
			wrSensorReg16_8(0x3a0f ,0x18);
			wrSensorReg16_8(0x3a10 ,0x10);
			wrSensorReg16_8(0x3a1b ,0x18);
			wrSensorReg16_8(0x3a1e ,0x10);
			wrSensorReg16_8(0x3a11 ,0x30);
			wrSensorReg16_8(0x3a1f ,0x10);
		break;
		case Exposure_10_EV:
			wrSensorReg16_8(0x3a0f ,0x20);
			wrSensorReg16_8(0x3a10 ,0x18);
			wrSensorReg16_8(0x3a11 ,0x41);
			wrSensorReg16_8(0x3a1b ,0x20);
			wrSensorReg16_8(0x3a1e ,0x18);
			wrSensorReg16_8(0x3a1f ,0x10);
		break;
		case Exposure_07_EV:
			wrSensorReg16_8(0x3a0f ,0x28);
			wrSensorReg16_8(0x3a10 ,0x20);
			wrSensorReg16_8(0x3a11 ,0x51);
			wrSensorReg16_8(0x3a1b ,0x28);
			wrSensorReg16_8(0x3a1e ,0x20);
			wrSensorReg16_8(0x3a1f ,0x10);
		break;
		case Exposure_03_EV:
			wrSensorReg16_8(0x3a0f ,0x30);
			wrSensorReg16_8(0x3a10 ,0x28);
			wrSensorReg16_8(0x3a11 ,0x61);
			wrSensorReg16_8(0x3a1b ,0x30);
			wrSensorReg16_8(0x3a1e ,0x28);
			wrSensorReg16_8(0x3a1f ,0x10);
		break;
		case Exposure_default:
			wrSensorReg16_8(0x3a0f ,0x38);
			wrSensorReg16_8(0x3a10 ,0x30);
			wrSensorReg16_8(0x3a11 ,0x61);
			wrSensorReg16_8(0x3a1b ,0x38);
			wrSensorReg16_8(0x3a1e ,0x30);
			wrSensorReg16_8(0x3a1f ,0x10);
			wrSensorReg16_8(0x3a0f ,0x40);
			wrSensorReg16_8(0x3a10 ,0x38);
			wrSensorReg16_8(0x3a11 ,0x71);
			wrSensorReg16_8(0x3a1b ,0x40);
			wrSensorReg16_8(0x3a1e ,0x38);
			wrSensorReg16_8(0x3a1f ,0x10);
		break;
		case Exposure07_EV:
			wrSensorReg16_8(0x3a0f ,0x48);
			wrSensorReg16_8(0x3a10 ,0x40);
			wrSensorReg16_8(0x3a11 ,0x80);
			wrSensorReg16_8(0x3a1b ,0x48);
			wrSensorReg16_8(0x3a1e ,0x40);
			wrSensorReg16_8(0x3a1f ,0x20);
		break;
		case Exposure10_EV:
			wrSensorReg16_8(0x3a0f ,0x50);
			wrSensorReg16_8(0x3a10 ,0x48);
			wrSensorReg16_8(0x3a11 ,0x90);
			wrSensorReg16_8(0x3a1b ,0x50);
			wrSensorReg16_8(0x3a1e ,0x48);
			wrSensorReg16_8(0x3a1f ,0x20);
		break;
		case Exposure13_EV:
			wrSensorReg16_8(0x3a0f ,0x58);
			wrSensorReg16_8(0x3a10 ,0x50);
			wrSensorReg16_8(0x3a11 ,0x91);
			wrSensorReg16_8(0x3a1b ,0x58);
			wrSensorReg16_8(0x3a1e ,0x50);
			wrSensorReg16_8(0x3a1f ,0x20);
		break;
		case Exposure17_EV:
			wrSensorReg16_8(0x3a0f ,0x60);
			wrSensorReg16_8(0x3a10 ,0x58);
			wrSensorReg16_8(0x3a11 ,0xa0);
			wrSensorReg16_8(0x3a1b ,0x60);
			wrSensorReg16_8(0x3a1e ,0x58);
			wrSensorReg16_8(0x3a1f ,0x20);
		break;
	}
}
void ArduCam_set_Sharpness(uint8_t Sharpness)
{
	switch(Sharpness)
	{
		case Auto_Sharpness_default:
	  	wrSensorReg16_8(0x530A ,0x00);
			wrSensorReg16_8(0x530c ,0x0 );
			wrSensorReg16_8(0x530d ,0xc );
			wrSensorReg16_8(0x5312 ,0x40);
		break;
		case Auto_Sharpness1:
			wrSensorReg16_8(0x530A ,0x00);
			wrSensorReg16_8(0x530c ,0x4 );
			wrSensorReg16_8(0x530d ,0x18);
			wrSensorReg16_8(0x5312 ,0x20);
		break;
		case Auto_Sharpness2:
			wrSensorReg16_8(0x530A ,0x00);
			wrSensorReg16_8(0x530c ,0x8 );
			wrSensorReg16_8(0x530d ,0x30);
			wrSensorReg16_8(0x5312 ,0x10);
		break;
		case Manual_Sharpnessoff:
			wrSensorReg16_8(0x530A ,0x08);
			wrSensorReg16_8(0x531e ,0x00);
			wrSensorReg16_8(0x531f ,0x00);
		break;
		case Manual_Sharpness1:
			wrSensorReg16_8(0x530A ,0x08);
			wrSensorReg16_8(0x531e ,0x04);
			wrSensorReg16_8(0x531f ,0x04);
		break;
		case Manual_Sharpness2:
			wrSensorReg16_8(0x530A ,0x08);
			wrSensorReg16_8(0x531e ,0x08);
			wrSensorReg16_8(0x531f ,0x08);
		break;
		case Manual_Sharpness3:
			wrSensorReg16_8(0x530A ,0x08);
			wrSensorReg16_8(0x531e ,0x0c);
			wrSensorReg16_8(0x531f ,0x0c);
		break;
		case Manual_Sharpness4:
			wrSensorReg16_8(0x530A ,0x08);
			wrSensorReg16_8(0x531e ,0x0f);
			wrSensorReg16_8(0x531f ,0x0f);
		break;
		case Manual_Sharpness5:
			wrSensorReg16_8(0x530A ,0x08);
			wrSensorReg16_8(0x531e ,0x1f);
			wrSensorReg16_8(0x531f ,0x1f);
		break;
	}

}

void ArduCam_set_Mirror_Flip(uint8_t Mirror_Flip)
{
	uint8_t reg_val;
	 switch(Mirror_Flip)
	{
		case MIRROR:
			rdSensorReg16_8(0x3818,&reg_val);
			reg_val = reg_val|0x00;
			reg_val = reg_val&0x9F;
			wrSensorReg16_8(0x3818 ,reg_val);
			rdSensorReg16_8(0x3621,&reg_val);
			reg_val = reg_val|0x20;
			wrSensorReg16_8(0x3621, reg_val );

		break;
		case FLIP:
			rdSensorReg16_8(0x3818,&reg_val);
			reg_val = reg_val|0x20;
			reg_val = reg_val&0xbF;
			wrSensorReg16_8(0x3818 ,reg_val);
			rdSensorReg16_8(0x3621,&reg_val);
			reg_val = reg_val|0x20;
			wrSensorReg16_8(0x3621, reg_val );
		break;
		case MIRROR_FLIP:
			rdSensorReg16_8(0x3818,&reg_val);
			reg_val = reg_val|0x60;
			reg_val = reg_val&0xFF;
			wrSensorReg16_8(0x3818 ,reg_val);
			rdSensorReg16_8(0x3621,&reg_val);
			reg_val = reg_val&0xdf;
			wrSensorReg16_8(0x3621, reg_val );
		break;
		case Normal:
			rdSensorReg16_8(0x3818,&reg_val);
			reg_val = reg_val|0x40;
			reg_val = reg_val&0xdF;
			wrSensorReg16_8(0x3818 ,reg_val);
			rdSensorReg16_8(0x3621,&reg_val);
			reg_val = reg_val&0xdf;
			wrSensorReg16_8(0x3621, reg_val );
			break;
	}
}


void ArduCam_set_Compress_quality(uint8_t quality)
{
switch(quality)
	{
		case high_quality:
			wrSensorReg16_8(0x4407, 0x02);
			break;
		case default_quality:
			wrSensorReg16_8(0x4407, 0x04);
			break;
		case low_quality:
			wrSensorReg16_8(0x4407, 0x08);
			break;
	}
}

void ArduCam_Test_Pattern(uint8_t Pattern)
{
  switch(Pattern)
	{
		case Color_bar:
			wrSensorReg16_8(0x503d , 0x80);
			wrSensorReg16_8(0x503e, 0x00);
			break;
		case Color_square:
			wrSensorReg16_8(0x503d , 0x85);
			wrSensorReg16_8(0x503e, 0x12);
			break;
		case BW_square:
			wrSensorReg16_8(0x503d , 0x85);
			wrSensorReg16_8(0x503e, 0x1a);
			break;
		case DLI:
			wrSensorReg16_8(0x4741 , 0x4);
			break;
	}

}
void ArduCam_clear_fifo_flag()
{
	ArduCam_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void ArduCam_set_bit(uint8_t addr, uint8_t bit)
{
	uint8_t temp;

	temp = ArduCam_read_reg(addr);
	ArduCam_write_reg(addr, temp | bit);
}

void ArduCam_clear_bit(uint8_t addr, uint8_t bit)
{
	uint8_t temp;

	temp = ArduCam_read_reg(addr);
	ArduCam_write_reg(addr, temp & (~bit));
}

uint8_t ArduCam_get_bit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = ArduCam_read_reg(addr);
  temp = temp & bit;
  return temp;
}

void ArduCam_start_capture()
{
	ArduCam_write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

uint32_t ArduCam_read_fifo_length(void)
{
	uint32_t len1,len2,len3,length=0;
	len1 = ArduCam_read_reg(FIFO_SIZE1);
	len2 = ArduCam_read_reg(FIFO_SIZE2);
	len3 = ArduCam_read_reg(FIFO_SIZE3) & 0x7f;
	length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;

	return length;
}

static void initI2C()
{
	i2c_config_t conf;
    esp_err_t err;

    conf.mode=I2C_MODE_MASTER;
    conf.sda_io_num=ARDUCAM_SDA;
    conf.sda_pullup_en=GPIO_PULLUP_DISABLE;
    conf.scl_io_num=ARDUCAM_SCL;
    conf.scl_pullup_en=GPIO_PULLUP_DISABLE;
    conf.master.clk_speed=ARDUCAM_I2C_FREQ_HZ;

    err=i2c_param_config(ARDUCAM_I2C_PORT, &conf);
    if(err != ESP_OK)
    	ESP_LOGW(ARDUCAM_TAG,"i2c_param_config returned %x",err);

    i2c_driver_install(ARDUCAM_I2C_PORT, conf.mode, 0, 0, 0);
    if(err != ESP_OK)
    	ESP_LOGW(ARDUCAM_TAG,"i2c_driver_install returned %x",err);
}

static uint8_t initSPI()
{
	esp_err_t err;

	spi_bus_config_t buscfg = {
			.miso_io_num=ARDUCAM_MISO,
			.mosi_io_num=ARDUCAM_MOSI,
			.sclk_io_num=ARDUCAM_SCK,
	        .quadwp_io_num=-1,
	        .quadhd_io_num=-1
	};

	spi_device_interface_config_t devcfg = {
			.clock_speed_hz=ARDUCAM_SPI_CLOCK_HZ,
			.mode=0,
			.spics_io_num=ARDUCAM_CS,
			.queue_size=1
	};

	err = spi_bus_initialize(HSPI_HOST, &buscfg,1);
	if(err != ESP_OK)
	{
		ESP_LOGW(ARDUCAM_TAG,"spi_bus_init error %x",err);
		return 0;
	}

	err=spi_bus_add_device(HSPI_HOST,&devcfg,&spi);
	if(err != ESP_OK)
	{
		ESP_LOGW(ARDUCAM_TAG,"spi_bus_add_device error %x",err);
		return 0;
	}

	ArduCam_write_reg(ARDUCHIP_TEST1, 0x55);
	uint8_t result=ArduCam_read_reg(ARDUCHIP_TEST1);
	if(result != 0x55)
	{
		ESP_LOGW(ARDUCAM_TAG,"SPI Test Failed with result = %x",result);
		return 0;
	}

	return 1;

}

void print_buffer(uint8_t *buffer, uint16_t len)
{
	uint16_t fullLines=len / 8;
	uint8_t partLine = len % 8;

	for (uint8_t n=0; n<fullLines + 1; n++)
	{
		ESP_LOGI(ARDUCAM_TAG,"%x %x %x %x %x %x %x %x",buffer[n*8],buffer[n*8+1],buffer[n*8+2],buffer[n*8+3],buffer[n*8+4],buffer[n*8+5],buffer[n*8+6],buffer[n*8+7]);
	}

}

void transfer_picture()
{
	spi_transaction_t t;
	esp_err_t err;

	uint32_t size=ArduCam_read_fifo_length();
	ESP_LOGI(ARDUCAM_TAG,"FIFO Length %d",size);

	memset(&t,0,sizeof(t));

	uint8_t spi_data = BURST_FIFO_READ;

	t.length=8;
	t.tx_buffer=&spi_data;
	err=spi_device_transmit(spi,&t);
	if(err != ESP_OK)
		ESP_LOGW(ARDUCAM_TAG,"spi_device_transmit error %x",err);

	uint8_t *rxBuffer=pvPortMallocCaps(1280, MALLOC_CAP_DMA);

	uint16_t fullBuffers=size/1280;
	uint16_t partBuffer= size % 1280;

	for (int n=0;n<fullBuffers;n++)
	{
		memset(&t,0,sizeof(t));

		t.rx_buffer = rxBuffer;
		t.length = 1280;
		err=spi_device_transmit(spi,&t);
		if(err != ESP_OK)
			ESP_LOGW(ARDUCAM_TAG,"spi_device_transmit error %x",err);

		print_buffer(rxBuffer,1280);
	}

	memset(&t,0,sizeof(t));
	t.rx_buffer=rxBuffer;
	t.length = partBuffer;
	err=spi_device_transmit(spi,&t);
	if(err != ESP_OK)
		ESP_LOGW(ARDUCAM_TAG,"spi_device_transmit error %x",err);

	print_buffer(rxBuffer,partBuffer);

	ArduCam_clear_fifo_flag();
}

void take_picture()
{
	ArduCam_clear_bit(ARDUCHIP_GPIO,GPIO_PWDN_MASK);
	ArduCam_clear_fifo_flag();
	ArduCam_write_reg(ARDUCHIP_FRAMES,0x00);

	ArduCam_start_capture();

	while(!ArduCam_get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));

	uint32_t size=ArduCam_read_fifo_length();
	ESP_LOGI(ARDUCAM_TAG,"FIFO Length %d",size);

	transfer_picture();
}

void ArduCam_init()
{

	initI2C();
	initSPI();

    // Initialize OV5642 for JPG as OV5642_MINI_5MP_PLUS
    wrSensorReg16_8(0x3008, 0x80);
    wrSensorRegs16_8(OV5642_QVGA_Preview);
    wrSensorRegs16_8(ov5642_320x240);
    wrSensorReg16_8(0x3818, 0xa8);
    wrSensorReg16_8(0x3621, 0x10);
    wrSensorReg16_8(0x3801, 0xb0);
    wrSensorReg16_8(0x4407, 0x04);

    uint8_t vid;
    uint8_t pid;
    rdSensorReg16_8(OV5642_CHIPID_HIGH,&vid);
    rdSensorReg16_8(OV5642_CHIPID_LOW,&pid);
    if((vid != 0x56) | (pid != 0x42))
    		ESP_LOGW(ARDUCAM_TAG,"Invalid Chip ID, VID = %x, PID = %x",vid,pid);

    ArduCam_write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    ArduCam_set_bit(ARDUCHIP_GPIO,GPIO_PWDN_MASK);
    vTaskDelay(800/ portTICK_RATE_MS);

    take_picture();
}
