

#ifndef ___ARDUCAM_COMMUN__H__
#define ___ARDUCAM_COMMUN__H__


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_system.h"

#include "driver/i2c.h"
#include "driver/spi.h"
#include "driver/gpio.h"
#include "esp8266_peri.h"


#define ARDUCAM_CS_PIN 	GPIO_NUM_16
#define CAM_CS_BEGIN 	gpio_set_level(ARDUCAM_CS_PIN, 0)
#define CAM_CS_END 		gpio_set_level(ARDUCAM_CS_PIN, 1)


#define ARDUCHIP_FIFO      		0x04  //FIFO and I2C control
#define FIFO_CLEAR_MASK    		0x01
#define FIFO_START_MASK    		0x02
#define FIFO_RDPTR_RST_MASK     0x10
#define FIFO_WRPTR_RST_MASK     0x20

#define CAP_DONE_MASK      		0x08
#define ARDUCHIP_TRIG      		0x41  //Trigger source

#define FIFO_SIZE1				0x42  //Camera write FIFO size[7:0] for burst to read
#define FIFO_SIZE2				0x43  //Camera write FIFO size[15:8]
#define FIFO_SIZE3				0x44  //Camera write FIFO size[18:16]

#define ARDUCHIP_FRAMES			  0x01

#define MAX_FIFO_SIZE		0x5FFFF			//384KByte

#define BURST_FIFO_READ			0x3C  //Burst FIFO read operation

esp_err_t i2c_write(uint8_t reg, uint8_t *data, size_t len);
esp_err_t i2c_read(uint8_t reg, uint8_t *data, size_t len);

void spi_write_reg(uint8_t addr, uint8_t data);
uint8_t spi_read_reg(uint8_t addr);
void setBitOrder(uint8_t bitOrder);
uint8_t spi_transfer(uint8_t data);

esp_err_t ardu_cam_init();

#endif