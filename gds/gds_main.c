#include "gds_driver_init.h"
#include "subbus.h"
//#include "usb_start.h"

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	system_init();
// usb_init();

  if (subbus_add_driver(&sb_base)
   || subbus_add_driver(&sb_fail_sw)
   || subbus_add_driver(&sb_board_desc)
// || subbus_add_driver(&sb_ser_control)
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
