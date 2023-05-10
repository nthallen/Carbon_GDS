/** @file i2c.h Carbon_GDS */
#ifndef I2C_H_INCLUDED
#define I2C_H_INCLUDED
#include "subbus.h"

#define I2C_ADC_ENABLE_DEFAULT true	// All On board ADC's
#define I2C_C3P_ENABLE_DEFAULT true	// Circuit 3 Power Monitor
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
extern subbus_driver_t sb_i2c_c3p;
extern subbus_driver_t sb_i2c_ms;
void i2c_adc_enable(bool value);
void i2c_c3p_enable(bool value);
void i2c_ms_enable(bool value);

#endif
