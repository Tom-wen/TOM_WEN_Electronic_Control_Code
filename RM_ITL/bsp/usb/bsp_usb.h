#ifndef __BSP_USB_H__
#define __BSP_USB_H__

#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "main.h"

void usb_send(uint8_t* buf, uint16_t len);

#endif
