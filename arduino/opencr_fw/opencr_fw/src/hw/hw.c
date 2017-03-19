/*
 *  hw.c
 *
 *  Created on: 2016. 7. 13.
 *      Author: Baram
 */

#include "hw.h"

#include "core/usb_cdc/usbd_core.h"
#include "core/usb_cdc/usbd_desc.h"
#include "core/usb_cdc/usbd_cdc.h"
#include "core/usb_cdc/usbd_cdc_interface.h"


USBD_HandleTypeDef USBD_Device;



void hwUsbInit(void);


void hwInit(void)
{
  bspInit();

	microsInit();
	hwUsbInit();

	vcpInit();
}

void hwUsbInit(void)
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
