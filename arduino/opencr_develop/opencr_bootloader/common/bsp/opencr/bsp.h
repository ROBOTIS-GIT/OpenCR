#ifndef BSP_H
#define BSP_H

#include <stdint.h>

#include "def.h"
#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"
#include "system_clock.h"

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"

#include "led.h"
#include "button.h"
#include "wdg.h"


#define USE_USB_FS





void bsp_init();
void bsp_deinit();

#endif

