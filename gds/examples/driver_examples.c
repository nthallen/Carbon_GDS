/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"

void PM_I2C_example(void)
{
	struct io_descriptor *PM_I2C_io;

	i2c_m_sync_get_io_descriptor(&PM_I2C, &PM_I2C_io);
	i2c_m_sync_enable(&PM_I2C);
	i2c_m_sync_set_slaveaddr(&PM_I2C, 0x12, I2C_M_SEVEN);
	io_write(PM_I2C_io, (uint8_t *)"Hello World!", 12);
}

void ADC_I2C_example(void)
{
	struct io_descriptor *ADC_I2C_io;

	i2c_m_sync_get_io_descriptor(&ADC_I2C, &ADC_I2C_io);
	i2c_m_sync_enable(&ADC_I2C);
	i2c_m_sync_set_slaveaddr(&ADC_I2C, 0x12, I2C_M_SEVEN);
	io_write(ADC_I2C_io, (uint8_t *)"Hello World!", 12);
}

void MS_I2C_example(void)
{
	struct io_descriptor *MS_I2C_io;

	i2c_m_sync_get_io_descriptor(&MS_I2C, &MS_I2C_io);
	i2c_m_sync_enable(&MS_I2C);
	i2c_m_sync_set_slaveaddr(&MS_I2C, 0x12, I2C_M_SEVEN);
	io_write(MS_I2C_io, (uint8_t *)"Hello World!", 12);
}
