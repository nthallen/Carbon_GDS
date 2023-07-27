/************************************************************************/
/* July 4, 2023 7:12 AM	file i2c_pm.c  
	
	gds I2C interface to Power Monitor board for 28V Circuit 3
  LTC4151 addr 1 . I2C ADDR: 0x6Ah 
	
	NOTE: Needs RTC timer module for delays

 ************************************************************************/
#include <hpl_pm_base.h>
#include <peripheral_clk_config.h>
#include <hpl_gclk_base.h>
#include "gds_pins.h"
#include "i2c.h"
#include "subbus.h"
#include "rtc_timer.h"
 
#define pow2(X) (float)(1<<X)

struct i2c_m_async_desc PM_I2C;

static bool i2c_pm_enabled = I2C_PM_ENABLE_DEFAULT;

static struct io_descriptor *PM_I2C_io;
static volatile bool I2C_txfr_complete = true;
static volatile bool I2C_error_seen = false;
/** i2c error codes are defined in hal/include/hpl_i2c_m_sync.h
 *  named I2C_ERR_* and I2C_OK
 */
static volatile int32_t I2C_error = I2C_OK;
static volatile uint8_t pm_ov_status = 0;

static void i2c_write(int16_t i2c_addr, const uint8_t *obuf, int16_t nbytes);
static void i2c_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes);

static uint8_t pm_ibuf[I2C_PM_MAX_READ_LENGTH];

static uint8_t ltc4151_regadr[1] = { LTC4151_CMD };


/* These addresses belong to the I2C_PM module
 * 0x80 R:  ST: 16b I2C Status
 * 0x81 R:   I: 16b Current SENSE (A7:0, B7:4)
 * 0x82 R:   V: 16b Voltage Vin (C7:0, D7:4)
 * 0x83 R:  VA: 16b ADIN Aux Voltage (E7:0, F7:4)
 * 0x84 R:  CR: 16b CONTROL Register (G7:0)
 */
static subbus_cache_word_t i2c_pm_cache[I2C_PM_HIGH_ADDR-I2C_PM_BASE_ADDR+1] = {
  { 0, 0, true,  false, false,  false, false }, // Offset 0x00: R: 16b I2C Status
  { 0, 0, true,  false, false,  false, false }, // Offset 0x01: R: I Current SENSE
  { 0, 0, true,  false, false,  false, false }, // Offset 0x02: R: V Voltage Vin
  { 0, 0, true,  false, false,  false, false }, // Offset 0x03: R: VA ADIN Aux Voltage
  { 4, 0, true,  false,  true,  false, false }, // Offset 0x04: RW: CONTROL Register
// .cache	.wvalue	.readable	.was_read	.writable	.written	.dynamic
};

/* LTC4151 I Readings: 
    Vsense|Max = 81.92mV
    Current-Sense Resistance : Max Current
    0.001ohm : 81.92A Max
    0.0025ohms : 32.78A Max
 */
/****************************************************
 *	ltc4151 Driver State Machine
 * (ltc4151_scan - Scan for all Power Monitors in chain. Not implemented)
 * (ltc4151_init - Possible Write CONTROL register for known mode. Not implemented)
 *	ltc4151_readpg - Send Read Command
 *	ltc4151_readpg_tx - Receive all Register contents (6Bytes)
 *	ltc4151_cache - Store readings in cache 
 */
enum ltc4151_state_t {
//        ltc4151_init, 
        ltc4151_readpg, ltc4151_readpg_tx, ltc4151_cache,
        };

// Future: Should be a 9x3 array holding I, V and VA readings for all possible PMons
typedef struct {
  bool enabled;
  enum ltc4151_state_t state;
  uint32_t I;	// I 12-Bit Data of Current Sense Voltage with 20μV LSB and 81.92mV FS
  uint32_t V;	// Vin 12-Bit Data of VIN Voltage with 25mV LSB and 102.4V Full-Scale
  uint32_t VA;	// Vaux 12-Bit Data of Current Sense Voltage with 500μV LSB and 2.048V FS
  uint32_t endtime;
//  uint32_t delay;
} ltc4151_poll_def;

static ltc4151_poll_def ltc4151 = {
    I2C_PM_ENABLE_DEFAULT
    , ltc4151_readpg //    , ltc4151_init
//	, 0, 0, 0
//	, 0, 0
};


/**
 * poll_ltc4151() is only called when I2C_txfr_complete = true
 *    and I2C bus is free
 * return true if we are relinquishing the I2C bus
 */
static bool poll_ltc4151() {
  /// uint32_t delay = 0x00000000;
  if (!i2c_pm_enabled || !I2C_txfr_complete ) return true;
  switch (ltc4151.state) {
    
///  Will need to scan for all Power Monitors, an maybe init all...
    
    case ltc4151_readpg:
      // send generic request ...
			i2c_write(PM_I2C_ADDR, ltc4151_regadr, 1);	
			ltc4151.state = ltc4151_readpg_tx;
      return false;

    case ltc4151_readpg_tx:
      // .. read 6 byte page to get I, V, VA
      i2c_read(PM_I2C_ADDR, pm_ibuf, 6);
      ltc4151.endtime = rtc_current_count + ( 1 * RTC_COUNTS_PER_MSEC ); // 1 msec
			ltc4151.state = ltc4151_cache;
      return false;
      
    case ltc4151_cache:
      if ( rtc_current_count <= ltc4151.endtime ) return false;
      ltc4151.I = (   // update ltc4151 struct
         (((uint16_t)pm_ibuf[0])<<4)
        | ((uint16_t)pm_ibuf[1])>>4);
      ltc4151.V = (   // update ltc4151 struct
         (((uint16_t)pm_ibuf[2])<<4)
        | ((uint16_t)pm_ibuf[3])>>4);
      ltc4151.VA = (   // update ltc4151 struct
         (((uint16_t)pm_ibuf[4])<<4)
        | ((uint16_t)pm_ibuf[5])>>4);
      // update cache
      i2c_pm_cache[0x01].cache = ltc4151.I;
      i2c_pm_cache[0x02].cache = ltc4151.V;
      i2c_pm_cache[0x03].cache = ltc4151.VA;
      ltc4151.state = ltc4151_readpg;
      return true;

    default:
      assert(false, __FILE__, __LINE__);
   }
   return true;
}


// i2c functions

static void i2c_write(int16_t i2c_addr, const uint8_t *obuf, int16_t nbytes) {
  assert(I2C_txfr_complete, __FILE__, __LINE__);
  I2C_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&PM_I2C, i2c_addr, I2C_M_SEVEN);
  io_write(PM_I2C_io, obuf, nbytes);
}

static void i2c_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes) {
  assert(I2C_txfr_complete, __FILE__, __LINE__);
  I2C_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&PM_I2C, i2c_addr, I2C_M_SEVEN);
  io_read(PM_I2C_io, ibuf, nbytes);
}

void i2c_pm_enable(bool value) {
  i2c_pm_enabled = value;
}

#define I2C_INTFLAG_ERROR (1<<7)

static void I2C_async_error(struct i2c_m_async_desc *const i2c, int32_t error) {
  I2C_txfr_complete = true;
  I2C_error_seen = true;
  I2C_error = error;
  if (sb_cache_was_read(i2c_pm_cache, I2C_PM_STATUS_OFFSET)) {
    sb_cache_update(i2c_pm_cache, I2C_PM_STATUS_OFFSET, 0);
  }
  if (I2C_error >= -7 && I2C_error <= -2) {
    uint16_t val = i2c_pm_cache[I2C_PM_STATUS_OFFSET].cache;
    val |= (1 << (7+I2C_error));
    sb_cache_update(i2c_pm_cache, I2C_PM_STATUS_OFFSET, val);
  }
  if (error == I2C_ERR_BUS) {
    hri_sercomi2cm_write_STATUS_reg(PM_I2C.device.hw, SERCOM_I2CM_STATUS_BUSERR);
    hri_sercomi2cm_clear_INTFLAG_reg(PM_I2C.device.hw, I2C_INTFLAG_ERROR);
  }
}

static void I2C_txfr_completed(struct i2c_m_async_desc *const i2c) {
  I2C_txfr_complete = true;
}

static void i2c_pm_reset() {
  if (!sb_i2c_pm.initialized) {
    PM_I2C_init();
    i2c_m_async_get_io_descriptor(&PM_I2C, &PM_I2C_io);
    i2c_m_async_enable(&PM_I2C);
    i2c_m_async_register_callback(&PM_I2C, I2C_M_ASYNC_ERROR, (FUNC_PTR)I2C_async_error);
    i2c_m_async_register_callback(&PM_I2C, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)I2C_txfr_completed);
    i2c_m_async_register_callback(&PM_I2C, I2C_M_ASYNC_RX_COMPLETE, (FUNC_PTR)I2C_txfr_completed);

    sb_i2c_pm.initialized = true;
  }
}
//  End of I2C functions

//	PM_I2C Driver
void PM_I2C_PORT_init(void)
{
	gpio_set_pin_pull_mode(C3P_SDA, GPIO_PULL_OFF);
	gpio_set_pin_function(C3P_SDA, PINMUX_PA16C_SERCOM1_PAD0);

	gpio_set_pin_pull_mode(C3P_SCL, GPIO_PULL_OFF);
	gpio_set_pin_function(C3P_SCL, PINMUX_PA17C_SERCOM1_PAD1);
}

void PM_I2C_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_SLOW, CONF_GCLK_SERCOM1_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBAMASK_SERCOM1_bit(MCLK);
}

void PM_I2C_init(void)
{
	PM_I2C_CLOCK_init();
	i2c_m_async_init(&PM_I2C, SERCOM1);
	PM_I2C_PORT_init();
}
//	End of PM_I2C Driver

// Main poll loop

enum i2c_state_t {i2c_ltc4151, i2c_jic };
static enum i2c_state_t i2c_state = i2c_ltc4151;

void i2c_pm_poll(void) {
	// cycle between P&T and RH
  if (!i2c_pm_enabled) return;
	  if (true) {	//  do we need condition here? 
    switch (i2c_state) {
      case i2c_ltc4151:
        if (poll_ltc4151()) {
          i2c_state = i2c_ltc4151;  // Never change state or machine
        }
        break;
      case i2c_jic:
        // jic extra machine here. Not used
          i2c_state = i2c_ltc4151;
        break;
      default:
        assert(false, __FILE__, __LINE__);
    }
  }
}

subbus_driver_t sb_i2c_pm = {
  I2C_PM_BASE_ADDR, I2C_PM_HIGH_ADDR, // address range
  i2c_pm_cache,
  i2c_pm_reset,
  i2c_pm_poll,
  0, // Dynamic function
  false // initialized
};
