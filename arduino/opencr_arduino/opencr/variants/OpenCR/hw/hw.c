/*
 *  drv.c
 *
 *  Created on: 2016. 7. 13.
 *      Author: Baram, PBHP
 */

#include "hw.h"
#include "variant.h"





void hw_init(void)
{
  drv_adc_init();
  drv_spi_init();
  drv_micros_init();
  drv_uart_init();
  drv_pwm_init();
  drv_timer_init();
  drv_i2c_init();
  drv_exti_init();
  drv_dxl_init();
  drv_eeprom_init();
  drv_rtc_init();
  
  drvCanInit();

  flash_init();
  vcp_init();
  wdg_init();
}
