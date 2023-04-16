/*
 * This file was based on code originally generated from Atmel START in main.c, though
 * it has been heavily modified. Nevertheless, whenever the Atmel START project is
 * updated, any changes to main.c (which is no longer compiled into this project)
 * should be reviewed to determine whether they should be propagated into the
 * project.
 */

#include "gds_driver_init.h"
#include "subbus.h"
#include "ser_control.h"
#include "gds_usb.h"

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	system_init();
// usb_init();

  if (subbus_add_driver(&sb_base)
   || subbus_add_driver(&sb_fail_sw)
   || subbus_add_driver(&sb_board_desc)
   || subbus_add_driver(&sb_control)
   || subbus_add_driver(&sb_usb)
// || subbus_add_driver(&sb_spi)
// || subbus_add_driver(&sb_rtc)
// || subbus_add_driver(&sb_ps_spi)
// || subbus_add_driver(&sb_cmd)
  ) {
    while (true) ; // some driver is misconfigured.
  }
  subbus_reset(); // Resets all drivers
  while (1) {
    subbus_poll();
  }
}
