#include "control_functions.h"
#include "global_variables.h"

// extern PCD_HandleTypeDef hpcd_USB_FS;
// extern ADC_HandleTypeDef hadc;
// extern I2C_HandleTypeDef hi2c1;

uint8_t AD5933_init (I2C_HandleTypeDef* handle,	uint16_t address)
	{
		uint8_t data[2];
		data[0] = 0x81;
		data[1] = 0x08;
		uint8_t receive[2];
		HAL_I2C_Master_Transmit(handle, address, data, 2, 1000);
		HAL_Delay(1);
		data[0] = 0x8f;
		if (HAL_I2C_Master_Receive(handle, address, receive, 1, 1000) != HAL_OK) receive[0] = 0xff;
		return receive[0];
	}
	

uint8_t DS1085_init (I2C_HandleTypeDef* handle,	uint16_t address)
	{
		return 0;
	}


uint8_t AD5933_set_gain (I2C_HandleTypeDef* handle,	uint16_t address, uint16_t gain)
	{
		return 0;
	}
	

uint8_t AD5933_set_start (I2C_HandleTypeDef* handle,	uint16_t address, uint16_t frequency)
	{
		return 0;
	}
	

uint8_t AD5933_set_steps (I2C_HandleTypeDef* handle,	uint16_t address, uint16_t steps)
	{
		return 0;
	}
	

uint8_t AD5933_set_increment (I2C_HandleTypeDef* handle,	uint16_t address, int increment)
	{
		uint32_t frequency_code = (increment / (AD5933_clock/4)) * 2^27;
		uint8_t D0, D1, D2;
		D0 = (uint8_t) ((frequency_code >> 24) & 0x000F);
		D1 = (uint8_t) ((frequency_code >> 16) & 0x000F);
		D2 = (uint8_t) (frequency_code >> 24);
		uint8_t to_send[2];
		int status = 0;
		to_send[0] = 0x87;
		to_send[1] = D0;
		status += HAL_I2C_Master_Transmit(handle, AD5933_address, to_send, sizeof(to_send)/8, 100);
		to_send[0] = 0x86;
		to_send[1] = D1;
		status += HAL_I2C_Master_Transmit(handle, AD5933_address, to_send, sizeof(to_send)/8, 100);
		to_send[0] = 0x85;
		to_send[1] = D2;
		status += HAL_I2C_Master_Transmit(handle, AD5933_address, to_send, sizeof(to_send)/8, 100);
		return status;
	}
	

uint8_t AD5933_set_external_clock (I2C_HandleTypeDef* handle,	uint16_t address)
	{
		return 0;
	}

uint8_t AD5933_set_internal_clock (I2C_HandleTypeDef* handle,	uint16_t address)
	{
		return 0;
	}


uint8_t DS1085_set_oscillator (I2C_HandleTypeDef* handle,	uint16_t address)
	{
		return 0;
	}
	

uint8_t DS1085_set_divider (I2C_HandleTypeDef* handle,	uint16_t address)
	{
		return 0;
	}
	

uint8_t DS1085_set_offset (I2C_HandleTypeDef* handle,	uint16_t address)
	{
		return 0;
	}
	

uint8_t DS1085_set_mux (I2C_HandleTypeDef* handle,	uint16_t address)
	{
		return 0;
	}
	

uint8_t DS1085_enable_out  (I2C_HandleTypeDef* handle,	uint16_t address)
	{
		return 0;
	}
	

uint8_t DS1085_disable_out  (I2C_HandleTypeDef* handle,	uint16_t address)
	{
		return 0;
	}
	

uint8_t AD5933_enter_standby (I2C_HandleTypeDef* handle,	uint16_t address)
	{
		return 0;
	}
	

uint8_t AD5933_enter_initialize (I2C_HandleTypeDef* handle,	uint16_t address)
	{
		return 0;
	}
	

uint8_t AD5933_enter_sweep (I2C_HandleTypeDef* handle,	uint16_t address)
	{
		return 0;
	}
	

uint8_t AD5933_get_temperature (I2C_HandleTypeDef* handle,	uint16_t address)
	{
		uint8_t data[2];
		data[0] = 0x80;
		data[1] = 0x90;
		HAL_I2C_Master_Transmit(handle, address, data, 2, 1000);
		HAL_Delay(1);
		
		data[0] = 0x92;
		HAL_I2C_Master_Transmit(handle, address, data, 1, 1000);
		HAL_I2C_Master_Receive(handle, address, data, 1, 1000);
		return data[0];
	}
	

uint8_t AD5933_get_real (I2C_HandleTypeDef* handle,	uint16_t address, int *real)
	{
		return 0;
	}
	

uint8_t AD5933_get_imaginary (I2C_HandleTypeDef* handle,	uint16_t address, int *imaginary)
	{
		return 0;
	}
	

uint8_t AD5933_is_DFT_Complete (I2C_HandleTypeDef* handle,	uint16_t address)
	{
		return 1;
	}
	

uint8_t AD5933_is_sweep (I2C_HandleTypeDef* handle,	uint16_t address)
	{
		return 1;
	}
	
uint8_t MCU_get_temperature(ADC_HandleTypeDef* hadc1)
	{
		HAL_ADC_Start(hadc1);
		HAL_Delay(5);
		double temperature = 30 + (((0.00080566406*HAL_ADC_GetValue(hadc1)) - 1.43)/0.043);
		return (int)temperature;
	}
	
