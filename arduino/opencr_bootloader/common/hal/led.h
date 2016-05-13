#ifndef LED_H
#define LED_H

#include <stdint.h>

void led_init(void);
void led_on(uint8_t ch);
void led_off(uint8_t ch);
void led_toggle(uint8_t ch);

#endif

