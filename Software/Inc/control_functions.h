#pragma once

#ifndef COMMAND_FUNCTIONS_INCLUDED
#define COMMAND_FUNCTIONS_INCLUDED

#include "stdint.h"
#include "stm32f1xx_hal.h"



uint8_t AD5933_init (I2C_HandleTypeDef* handle,	uint16_t address);
uint8_t DS1085_init (I2C_HandleTypeDef* handle,	uint16_t address);

uint8_t AD5933_set_gain (I2C_HandleTypeDef* handle,	uint16_t address, uint16_t gain);
uint8_t AD5933_set_start (I2C_HandleTypeDef* handle,	uint16_t address, uint16_t frequency);
uint8_t AD5933_set_steps (I2C_HandleTypeDef* handle,	uint16_t address, uint16_t steps);
uint8_t AD5933_set_increment (I2C_HandleTypeDef* handle,	uint16_t address, int increment);

uint8_t AD5933_set_external_clock (I2C_HandleTypeDef* handle,	uint16_t address);
uint8_t AD5933_set_internal_clock (I2C_HandleTypeDef* handle,	uint16_t address);

uint8_t DS1085_set_oscillator (I2C_HandleTypeDef* handle,	uint16_t address);
uint8_t DS1085_set_divider (I2C_HandleTypeDef* handle,	uint16_t address);
uint8_t DS1085_set_offset (I2C_HandleTypeDef* handle,	uint16_t address);
uint8_t DS1085_set_mux (I2C_HandleTypeDef* handle,	uint16_t address);


uint8_t DS1085_enable_out  (I2C_HandleTypeDef* handle,	uint16_t address);
uint8_t DS1085_disable_out  (I2C_HandleTypeDef* handle,	uint16_t address);

uint8_t AD5933_enter_standby (I2C_HandleTypeDef* handle,	uint16_t address);
uint8_t AD5933_enter_initialize (I2C_HandleTypeDef* handle,	uint16_t address);
uint8_t AD5933_enter_sweep (I2C_HandleTypeDef* handle,	uint16_t address);

uint8_t AD5933_get_temperature (I2C_HandleTypeDef* handle,	uint16_t address);
uint8_t AD5933_get_real (I2C_HandleTypeDef* handle,	uint16_t address, int *real);
uint8_t AD5933_get_imaginary (I2C_HandleTypeDef* handle,	uint16_t address, int *imaginary);

uint8_t AD5933_is_DFT_Complete (I2C_HandleTypeDef* handle,	uint16_t address);
uint8_t AD5933_is_sweep (I2C_HandleTypeDef* handle,	uint16_t address);

uint8_t MCU_get_temperature(ADC_HandleTypeDef* hadc1);

#endif
