/* Includes ------------------------------------------------------------------*/
#include "lib_eeprom_at24cxx.h"
/* External functions define -------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
extern control_setpoint_struct_typedef control_setpoint_struct;
extern control_pid_param_struct_typedef control_pid_param_struct;
extern control_schedule_struct_typedef control_schedule_struct;

extern led_display_value_struct_typedef led_display_value_struct;

uint8_t eeprom_thermocouple_type = MAX31856_CR1_THERMOCOUPLE_TYPE_K; // neu k tim duoc eeprom - mac dinh la type K
uint8_t eeprom_current_setpoint_table = 1;
uint8_t eeprom_current_schedule_table = 1;
uint8_t eeprom_found = 0;

/* Private types -------------------------------------------------------------*/

/* Private const/macros ------------------------------------------------------*/
static uint8_t eeprom_at24cxx_write_page(uint8_t* memaddress,
	uint16_t pageaddress,uint16_t num_of_byte);

uint8_t I2C_CheckAddress(I2C_TypeDef* I2Cx, uint8_t address)
{
	/* ------------------------------------------------------ 
	Check whether there is an I2C device with a specified 
	7-bit address or not.
	return 0 -> no device found
	return 1 -> device found
	------------------------------------------------------ */
	uint8_t timeout;
	
	// start
	I2C_GenerateSTART(I2Cx,ENABLE);
	timeout = 10;
	while ((!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT))&&(timeout))
	{
		delay_ms(1);
		timeout--;
	}
	if (timeout == 0)
	{
		I2C_GenerateSTOP(I2Cx,ENABLE);
		return 0;
	}
	
	// send address
	I2C_Send7bitAddress(I2Cx,address<<1,I2C_Direction_Transmitter);
	timeout = 30;
	while ((!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))&&(timeout))
	{
		delay_ms(1);
		timeout--;
	}
	if (timeout == 0)
	{
		I2C_GenerateSTOP(I2Cx,ENABLE);
		return 0;
	}
	
	// stop
	I2C_GenerateSTOP(I2Cx,ENABLE);
	return 1;	
}

/* Private functions body ----------------------------------------------------*/

void eeprom_at24cxx_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	uint32_t flash_cnt;
	uint8_t i;
	
	// gpio init
	RCC_APB2PeriphClockCmd(EEPROM_AT24CXX_SCL_Port_Clk,ENABLE);
	GPIO_InitStructure.GPIO_Pin = EEPROM_AT24CXX_SCL_Pin;
	GPIO_InitStructure.GPIO_Mode = EEPROM_AT24CXX_SCL_PinMode;
	GPIO_InitStructure.GPIO_Speed = EEPROM_AT24CXX_SCL_PinSpeed;
	GPIO_Init(EEPROM_AT24CXX_SCL_Port,&GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(EEPROM_AT24CXX_SDA_Port_Clk,ENABLE);
	GPIO_InitStructure.GPIO_Pin = EEPROM_AT24CXX_SDA_Pin;
	GPIO_InitStructure.GPIO_Mode = EEPROM_AT24CXX_SDA_PinMode;
	GPIO_InitStructure.GPIO_Speed = EEPROM_AT24CXX_SDA_PinSpeed;
	GPIO_Init(EEPROM_AT24CXX_SDA_Port,&GPIO_InitStructure);
	
	// i2c init
	RCC_APB1PeriphClockCmd(EEPROM_AT24CXX_I2C_Clk,ENABLE);
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(EEPROM_AT24CXX_I2C,&I2C_InitStructure);
	I2C_Cmd(EEPROM_AT24CXX_I2C,ENABLE);
	
	// reset eeprom
	//flash_cnt = 0;
	//eeprom_at24cxx_write_multi((uint8_t*)&flash_cnt,EEPROM_AT24CXX_FLASH_CNT_ADDR,4);
	
	// eeprom init
	eeprom_found = I2C_CheckAddress(EEPROM_AT24CXX_I2C,EEPROM_AT24CXX_Address);
	if (eeprom_found)
	{
		// eeprom found, read flash cnt
		eeprom_at24cxx_read_multi((uint8_t*)&flash_cnt,EEPROM_AT24CXX_FLASH_CNT_ADDR,4);
				
		if ((flash_cnt == 0)||(flash_cnt == 0xFFFFFFFF))
		{
			//blank eeprom
			led_set_text(&led_display_value_struct,"BLK");
			
			eeprom_thermocouple_type = MAX31856_CR1_THERMOCOUPLE_TYPE_K;
			eeprom_at24cxx_write_multi((uint8_t*)&eeprom_thermocouple_type,EEPROM_AT24CXX_THERMOCOUPLE_TYPE_ADDR,1);
			
			control_setpoint_struct.num_of_point = 0;
			control_setpoint_struct.current_segment = 0;
			for(i = 0; i < MAX_NUM_OF_SETPOINT; i++)
			{
				control_setpoint_struct.time[i] = 0.0f;
				control_setpoint_struct.value[i] = 0.0f;	
			}
			
			for(i = 0; i < EEPROM_AT24CXX_MAX_SETPOINT_TABLE; i++)
			{
				eeprom_at24cxx_write_multi((uint8_t*)&control_setpoint_struct,
					EEPROM_AT24CXX_SETPOINT_BASE_ADDR + (i-1)*EEPROM_AT24CXX_SETPOINT_SIZE,
					sizeof(control_setpoint_struct_typedef));
			}
			
			control_schedule_struct.num_of_temp_point = 0;
			for(i = 0; i < MAX_NUM_OF_TEMP_POINT; i++)
			{
				control_schedule_struct.temp[i] = 0.0f;
				control_schedule_struct.Kp[i] = 0.0f;
				control_schedule_struct.Ki[i] = 0.0f;
				control_schedule_struct.Kd[i] = 0.0f;
			}
			
			for(i = 0; i < EEPROM_AT24CXX_MAX_SCHEDULE_TABLE; i++)
			{
				eeprom_at24cxx_write_multi((uint8_t*)&control_schedule_struct,
					EEPROM_AT24CXX_SCHEDULE_BASE_ADDR + (i-1)*EEPROM_AT24CXX_SCHEDULE_SIZE,
					sizeof(control_schedule_struct_typedef));
			}
			
			flash_cnt = 1;
			eeprom_at24cxx_write_multi((uint8_t*)&flash_cnt,EEPROM_AT24CXX_FLASH_CNT_ADDR,4);
			
			led_set_text(&led_display_value_struct,"----");
		}
		else
		{
			// read thermocouple type
			eeprom_at24cxx_read_multi((uint8_t*)&eeprom_thermocouple_type,EEPROM_AT24CXX_THERMOCOUPLE_TYPE_ADDR,1);
			
			// read setpoint table 1
			eeprom_at24cxx_read_multi((uint8_t*)&control_setpoint_struct,
					EEPROM_AT24CXX_SETPOINT_BASE_ADDR,
					sizeof(control_setpoint_struct_typedef));
			// read schedule table 1
			eeprom_at24cxx_read_multi((uint8_t*)&control_schedule_struct,
					EEPROM_AT24CXX_SCHEDULE_BASE_ADDR,
					sizeof(control_schedule_struct_typedef));
		}
	}
	else
	{
		// no eeprom found
		led_set_text(&led_display_value_struct,"Flt3");
	}
}

/* Read and write data between eeprom and mcu
memaddress: pointer to the base address of mcu
romaddress: start address to be written or read
num_of_byte: number of byte to be written or read
return value:
0: error
1: success
*/
uint8_t eeprom_at24cxx_read_multi(uint8_t* memaddress,
	uint16_t romaddress,uint16_t num_of_byte)
{
	uint8_t timeout;
	uint8_t addhigh = romaddress>>8;
	uint8_t addlow = romaddress&0xff;
		
	// start
	I2C_GenerateSTART(EEPROM_AT24CXX_I2C,ENABLE);
	timeout = 10;
	while ((!I2C_CheckEvent(EEPROM_AT24CXX_I2C,I2C_EVENT_MASTER_MODE_SELECT))&&(timeout))
	{
		delay_ms(1);
		timeout--;
	}
	if (timeout == 0)
	{
		I2C_GenerateSTOP(EEPROM_AT24CXX_I2C,ENABLE);
		return 0;
	}
	
	// send address + W
	I2C_Send7bitAddress(EEPROM_AT24CXX_I2C,EEPROM_AT24CXX_Address<<1,I2C_Direction_Transmitter);
	timeout = 30;
	while ((!I2C_CheckEvent(EEPROM_AT24CXX_I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))&&(timeout))
	{
		delay_ms(1);
		timeout--;
	}
	if (timeout == 0)
	{
		I2C_GenerateSTOP(EEPROM_AT24CXX_I2C,ENABLE);
		return 0;
	}
	
	// send reg address (dummy write)
	I2C_SendData(EEPROM_AT24CXX_I2C,addhigh);
	while (!I2C_CheckEvent(EEPROM_AT24CXX_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
	}
	I2C_SendData(EEPROM_AT24CXX_I2C,addlow);
	while (!I2C_CheckEvent(EEPROM_AT24CXX_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
	}
	
	// start repeated
	I2C_GenerateSTART(EEPROM_AT24CXX_I2C,ENABLE);
	while (!I2C_CheckEvent(EEPROM_AT24CXX_I2C,I2C_EVENT_MASTER_MODE_SELECT))
	{
	}
	
	// send address + R
	I2C_Send7bitAddress(EEPROM_AT24CXX_I2C,EEPROM_AT24CXX_Address<<1,I2C_Direction_Receiver);
	while (!I2C_CheckEvent(EEPROM_AT24CXX_I2C,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
	}
	
	// read multi byte
	I2C_AcknowledgeConfig(EEPROM_AT24CXX_I2C,ENABLE);
	while(num_of_byte)
	{
		if (num_of_byte == 1)
		{
			I2C_AcknowledgeConfig(EEPROM_AT24CXX_I2C,DISABLE);
		}
		while (!I2C_CheckEvent(EEPROM_AT24CXX_I2C,I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
		}
		*memaddress = I2C_ReceiveData(EEPROM_AT24CXX_I2C);
		memaddress++;
		num_of_byte--;
	}	
	
	// stop
	I2C_GenerateSTOP(EEPROM_AT24CXX_I2C,ENABLE);
	delay_ms(5);
	return 1;	
}

static uint8_t eeprom_at24cxx_write_page(uint8_t* memaddress,
	uint16_t pageaddress,uint16_t num_of_byte)
{
	uint8_t timeout;
	uint8_t addhigh = pageaddress>>8;
	uint8_t addlow = pageaddress&0xff;
		
	// start
	I2C_GenerateSTART(EEPROM_AT24CXX_I2C,ENABLE);
	timeout = 10;
	while ((!I2C_CheckEvent(EEPROM_AT24CXX_I2C,I2C_EVENT_MASTER_MODE_SELECT))&&(timeout))
	{
		delay_ms(1);
		timeout--;
	}
	if (timeout == 0)
	{
		I2C_GenerateSTOP(EEPROM_AT24CXX_I2C,ENABLE);
		return 0;
	}
	
	// send address + W
	I2C_Send7bitAddress(EEPROM_AT24CXX_I2C,EEPROM_AT24CXX_Address<<1,I2C_Direction_Transmitter);
	timeout = 30;
	while ((!I2C_CheckEvent(EEPROM_AT24CXX_I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))&&(timeout))
	{
		delay_ms(1);
		timeout--;
	}
	if (timeout == 0)
	{
		I2C_GenerateSTOP(EEPROM_AT24CXX_I2C,ENABLE);
		return 0;
	}
	
	// send reg address 
	I2C_SendData(EEPROM_AT24CXX_I2C,addhigh);
	while (!I2C_CheckEvent(EEPROM_AT24CXX_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
	}
	I2C_SendData(EEPROM_AT24CXX_I2C,addlow);
	while (!I2C_CheckEvent(EEPROM_AT24CXX_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
	}

	// write multi byte
	while(num_of_byte)
	{
		I2C_SendData(EEPROM_AT24CXX_I2C,*memaddress);
		while (!I2C_CheckEvent(EEPROM_AT24CXX_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
		}
		memaddress++;
		num_of_byte--;
	}	
	
	// stop
	I2C_GenerateSTOP(EEPROM_AT24CXX_I2C,ENABLE);
	return 1;
}

uint8_t eeprom_at24cxx_write_multi(uint8_t* memaddress,
	uint16_t romaddress,uint16_t num_of_byte)
{
	uint16_t num_of_byte_write;
	uint8_t write_result;
	uint8_t first_page_remain = 32 - romaddress&0x001F;
	uint8_t first_page = 1;
	
	while(num_of_byte)
	{
		if (first_page)
		{
			if (num_of_byte > first_page_remain)
			{
				num_of_byte_write = first_page_remain;
			}
			else
			{
				num_of_byte_write = num_of_byte;
			}
			first_page = 0;
		}
		else
		{
			if (num_of_byte > 32)
			{
				num_of_byte_write = 32;
			}
			else
			{
				num_of_byte_write = num_of_byte;
			}
		}
		
		num_of_byte -= num_of_byte_write;
		
		write_result = eeprom_at24cxx_write_page(memaddress,romaddress,num_of_byte_write);
		memaddress += num_of_byte_write;
		romaddress += num_of_byte_write;
		delay_ms(5);
		if (write_result == 0)
		{
			return 0;
		}
	}
	
	delay_ms(5);
	return 1;
}

void eeprom_at24cxx_inc_flash_cnt(void)
{
	uint32_t flash_cnt;
	eeprom_at24cxx_read_multi((uint8_t*)&flash_cnt,EEPROM_AT24CXX_FLASH_CNT_ADDR,4);
	flash_cnt++;
	eeprom_at24cxx_write_multi((uint8_t*)&flash_cnt,EEPROM_AT24CXX_FLASH_CNT_ADDR,4);
}
