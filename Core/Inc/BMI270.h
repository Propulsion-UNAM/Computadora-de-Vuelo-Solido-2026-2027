/*
 * BMI270.h
 *
 *  Created on: 29 Nis 2026
 *      Author: senha
 */

#ifndef INC_BMI270_H_
#define INC_BMI270_H_

// Value
#define BMI270_CHIP_VALUE 				0x24
#define CONFIG_SIZE 					8192
#define CHUNK_SIZE  					128

// Initialization Registers
#define BMI270_PWR_CONF					0x7C
#define INIT_CTRL 						0x59
#define INIT_ADDR_LSB					0X5E
#define INIT_ADDR_MSB					0x5F

// Status Registers
#define INTERNAL_STATUS 				0x21

// Command Register
#define BMI270_CMD 						0x7E

#include "main.h"
#include "string.h"
#include "stdbool.h"

typedef struct
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;

	uint8_t spi_rx_buffer[14];
	uint8_t spi_tx_buffer[14];

	int16_t raw_x;
	int16_t raw_y;
	int16_t raw_z;

	float offset_x;
	float offset_y;
	float offset_z;

	float gyro_x;
	float gyro_y;
	float gyro_z;

	float prev_gyro_x;
	float prev_gyro_y;
	float prev_gyro_z;

	float raw_acc_x;
	float raw_acc_y;
	float raw_acc_z;

	float acc_x;
    float acc_y;
    float acc_z;
}BMI270_t;



void struct_init(BMI270_t *bmi270,SPI_HandleTypeDef *hspi,GPIO_TypeDef *cs_port,uint16_t cs_pin);
bool SPI_Transmit(uint8_t *pdata,uint8_t size,uint32_t timeout,BMI270_t *bmi270);
bool SPI_Recive(uint8_t *txdata,uint8_t*rxdata,uint8_t size,uint32_t timeout,BMI270_t *bmi270);
uint8_t BMI270_init(BMI270_t *bmi270);
void SPI_init(BMI270_t *bmi270);
uint8_t read_CHIP_ID(BMI270_t *bmi270);
void BMI270_gyro_calibrate(BMI270_t *bmi270);

#endif /* INC_BMI270_H_ */
