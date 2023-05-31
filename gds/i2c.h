/** @file i2c.h Carbon_GDS */
#ifndef I2C_H_INCLUDED
#define I2C_H_INCLUDED
#include "subbus.h"

#include <hal_i2c_m_async.h>

#include <hal_i2c_m_async.h>

#include <hal_i2c_m_async.h>

extern struct i2c_m_async_desc PM_I2C;

extern struct i2c_m_async_desc ADC_I2C;

extern struct i2c_m_async_desc MS_I2C;

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

#define I2C_MS_BASE_ADDR 0x20
#define I2C_MS_STATUS_OFFSET 0x00
#define I2C_MS_STATUS_NREGS 1
#define I2C_MS_ADS_OFFSET (I2C_MS_STATUS_OFFSET+I2C_MS_STATUS_NREGS)
#define I2C_MS_ADS_NREGS 16
#define I2C_MS_NREADS_NREGS 1
#define I2C_MS_NREGS (I2C_MS_ADS_OFFSET+I2C_MS_ADS_NREGS+I2C_MS_NREADS_NREGS)
#define I2C_MS_HIGH_ADDR (I2C_MS_BASE_ADDR+I2C_MS_NREGS-1)


extern subbus_driver_t sb_i2c_adc;
void i2c_adc_enable(bool value);
extern struct i2c_m_async_desc ADC_I2C;
void ADC_I2C_PORT_init(void);
void ADC_I2C_CLOCK_init(void);
void ADC_I2C_init(void);

extern subbus_driver_t sb_i2c_pm;
void i2c_pm_enable(bool value);
extern struct i2c_m_async_desc PM_I2C;
void PM_I2C_PORT_init(void);
void PM_I2C_CLOCK_init(void);
void PM_I2C_init(void);

extern subbus_driver_t sb_i2c_ms;
void i2c_ms_enable(bool value);
extern struct i2c_m_async_desc MS_I2C;
void MS_I2C_PORT_init(void);
void MS_I2C_CLOCK_init(void);
void MS_I2C_init(void);

#endif
