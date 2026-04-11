#include "bsp_usb.h"

void usb_send(uint8_t* buf, uint16_t len)
{
    CDC_Transmit_HS((uint8_t *)buf, len);
}
