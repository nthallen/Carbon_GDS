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

static uint8_t PM_I2C_example_str[12] = "Hello World!";

void PM_I2C_tx_complete(struct i2c_m_async_desc *const i2c)
{
}

void PM_I2C_example(void)
{
	struct io_descriptor *PM_I2C_io;

	i2c_m_async_get_io_descriptor(&PM_I2C, &PM_I2C_io);
	i2c_m_async_enable(&PM_I2C);
	i2c_m_async_register_callback(&PM_I2C, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)PM_I2C_tx_complete);
	i2c_m_async_set_slaveaddr(&PM_I2C, 0x12, I2C_M_SEVEN);

	io_write(PM_I2C_io, PM_I2C_example_str, 12);
}

static uint8_t ADC_I2C_example_str[12] = "Hello World!";

void ADC_I2C_tx_complete(struct i2c_m_async_desc *const i2c)
{
}

void ADC_I2C_example(void)
{
	struct io_descriptor *ADC_I2C_io;

	i2c_m_async_get_io_descriptor(&ADC_I2C, &ADC_I2C_io);
	i2c_m_async_enable(&ADC_I2C);
	i2c_m_async_register_callback(&ADC_I2C, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)ADC_I2C_tx_complete);
	i2c_m_async_set_slaveaddr(&ADC_I2C, 0x12, I2C_M_SEVEN);

	io_write(ADC_I2C_io, ADC_I2C_example_str, 12);
}

static uint8_t MS_I2C_example_str[12] = "Hello World!";

void MS_I2C_tx_complete(struct i2c_m_async_desc *const i2c)
{
}

void MS_I2C_example(void)
{
	struct io_descriptor *MS_I2C_io;

	i2c_m_async_get_io_descriptor(&MS_I2C, &MS_I2C_io);
	i2c_m_async_enable(&MS_I2C);
	i2c_m_async_register_callback(&MS_I2C, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)MS_I2C_tx_complete);
	i2c_m_async_set_slaveaddr(&MS_I2C, 0x12, I2C_M_SEVEN);

	io_write(MS_I2C_io, MS_I2C_example_str, 12);
}
