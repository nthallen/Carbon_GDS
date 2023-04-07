#include "gds_driver_init.h"
#include "usb_start.h"

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	system_init();
  usb_init();

	/* Replace with your application code */
	while (1) {
	}
}
