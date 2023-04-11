/*
 * This file is based on code originally generated from Atmel START as usb_start.h
 * Whenever the Atmel START project is updated, changes to usb_start.h must be
 * reviewed and copied here as appropriate.
 */
#ifndef GDS_USB_H
#define GDS_USB_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "cdcdf_acm.h"
#include "cdcdf_acm_desc.h"

void cdcd_acm_example(void);
void cdc_device_acm_init(void);

/**
 * \brief Initialize USB
 */
void usb_init(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // GDS_USB_H
