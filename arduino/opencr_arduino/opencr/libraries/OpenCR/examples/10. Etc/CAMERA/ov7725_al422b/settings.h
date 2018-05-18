#ifndef __SETTINGS_H
#define __SETTINGS_H

#define BOARD_LED_PIN           BDPIN_LED_STATUS    //Status LED
#define LED_RATE                500000              // in microseconds; should toggle every 0.5sec
#define _USE_FULLSCREEN         false

#if _USE_FULLSCREEN == true
  extern uint16_t image_buf[];
  //masked_image_buf is not available due to RAM overflow
#else
  extern uint16_t image_buf[];
  extern uint16_t masked_image_buf[];
#endif


#endif