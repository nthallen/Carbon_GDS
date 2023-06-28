/** @file i2c.h Carbon_GDS */
#ifndef I2C_H_INCLUDED
#define I2C_H_INCLUDED

#include "gds_pins.h"

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>

#include "subbus.h"

#include <hal_i2c_m_async.h>

extern struct i2c_m_async_desc PM_I2C;

extern struct i2c_m_async_desc ADC_I2C;

extern struct i2c_m_async_desc MS_I2C;

void PM_I2C_PORT_init(void);
void PM_I2C_CLOCK_init(void);
void PM_I2C_init(void);

void ADC_I2C_PORT_init(void);
void ADC_I2C_CLOCK_init(void);
void ADC_I2C_init(void);

void MS_I2C_PORT_init(void);
void MS_I2C_CLOCK_init(void);
void MS_I2C_init(void);


#define I2C_ADC_ENABLE_DEFAULT true	// All On board ADC's
#define I2C_PM_ENABLE_DEFAULT true	// Circuit 3 Power Monitor
#define I2C_MS_ENABLE_DEFAULT true	// On board MS8607 PTRH

#define I2C_ADC_BASE_ADDR 0x20
#define I2C_ADC_STATUS_OFFSET 0x00
#define I2C_ADC_STATUS_NREGS 1
#define I2C_ADC_ADS_OFFSET (I2C_ADC_STATUS_OFFSET+I2C_ADC_STATUS_NREGS)
#define I2C_ADC_ADS_NREGS 16
#define I2C_ADC_NREADS_NREGS 1
#define I2C_ADC_NREGS (I2C_ADC_ADS_OFFSET+I2C_ADC_ADS_NREGS+I2C_ADC_NREADS_NREGS)
#define I2C_ADC_HIGH_ADDR (I2C_ADC_BASE_ADDR+I2C_ADC_NREGS-1)


extern subbus_driver_t sb_i2c_adc;
extern subbus_driver_t sb_i2c_pm;
extern subbus_driver_t sb_i2c_ms;
void i2c_adc_enable(bool value);
void i2c_pm_enable(bool value);
void i2c_ms_enable(bool value);

#endif
