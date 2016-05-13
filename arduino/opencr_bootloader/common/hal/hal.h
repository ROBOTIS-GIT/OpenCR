#ifndef HAL_H
#define HAL_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"

#include "util.h"
#include "led.h"
#include "delay.h"
#include "flash.h"
#include "button.h"




void hal_init();



#ifdef __cplusplus
}
#endif


#endif

