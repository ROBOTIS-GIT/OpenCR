/*
 *  usb.c
 *
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram
 */

#include "usb.h"
#include "core/usb_cdc/usbd_core.h"
#include "core/usb_cdc/usbd_desc.h"
#include "core/usb_cdc/usbd_cdc.h"
#include "core/usb_cdc/usbd_cdc_interface.h"


USBD_HandleTypeDef USBD_Device;






void usbInit(void)
{
  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);

  /* Add CDC Interface Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);

  /* Start Device Process */
  USBD_Start(&USBD_Device);
}
