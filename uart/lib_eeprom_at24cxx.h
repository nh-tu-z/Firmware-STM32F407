/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIB_EEPROM_AT24CXX_H
#define __LIB_EEPROM_AT24CXX_H

#ifdef __cplusplus
 extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
#include "main.h"	 

/* Exported constants/macros -------------------------------------------------*/
#define EEPROM_AT24CXX_SCL_Port_Clk 				RCC_APB2Periph_GPIOB
#define EEPROM_AT24CXX_SCL_Port							GPIOB
#define EEPROM_AT24CXX_SCL_Pin							GPIO_Pin_10
#define EEPROM_AT24CXX_SCL_PinMode					GPIO_Mode_AF_OD
#define EEPROM_AT24CXX_SCL_PinSpeed					GPIO_Speed_50MHz

#define EEPROM_AT24CXX_SDA_Port_Clk 				RCC_APB2Periph_GPIOB
#define EEPROM_AT24CXX_SDA_Port							GPIOB
#define EEPROM_AT24CXX_SDA_Pin							GPIO_Pin_11
#define EEPROM_AT24CXX_SDA_PinMode					GPIO_Mode_AF_OD
#define EEPROM_AT24CXX_SDA_PinSpeed					GPIO_Speed_50MHz

#define EEPROM_AT24CXX_I2C									I2C2
#define EEPROM_AT24CXX_I2C_Clk							RCC_APB1Periph_I2C2 // all I2C periphs are in APB1
#define EEPROM_AT24CXX_Address							0x50


// eeprom address
#define EEPROM_AT24CXX_FLASH_CNT_ADDR				0x0000
#define EEPROM_AT24CXX_THERMOCOUPLE_TYPE_ADDR 0x0004

#define EEPROM_AT24CXX_SETPOINT_BASE_ADDR		0x1000
#define EEPROM_AT24CXX_SETPOINT_SIZE				0x0100 // 256 bytes
#define EEPROM_AT24CXX_MAX_SETPOINT_TABLE		16

#define EEPROM_AT24CXX_SCHEDULE_BASE_ADDR		0x0100
#define EEPROM_AT24CXX_SCHEDULE_SIZE				0x0100 // 256 bytes
#define EEPROM_AT24CXX_MAX_SCHEDULE_TABLE		8

/* Exported types ------------------------------------------------------------*/
	 	 
/* Exported function prototypes ----------------------------------------------*/
	
void eeprom_at24cxx_init(void);
uint8_t eeprom_at24cxx_read_multi(uint8_t* memaddress,
	uint16_t romaddress,uint16_t num_of_byte);
uint8_t eeprom_at24cxx_write_multi(uint8_t* memaddress,
	uint16_t romaddress,uint16_t num_of_byte);

void eeprom_at24cxx_inc_flash_cnt(void);

#ifdef __cplusplus
}
#endif

#endif 
