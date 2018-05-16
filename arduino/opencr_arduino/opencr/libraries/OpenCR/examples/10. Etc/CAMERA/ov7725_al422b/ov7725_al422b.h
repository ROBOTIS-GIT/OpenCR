#ifndef _OV7725_AL422B_H_
#define _OV7725_AL422B_H_

#include <Arduino.h>
#include "XPT2046.h"
#include "./src/OV7725/OV7725.h"
#include "./src/sccb/sccb.h"
#include "./src/tftLcd.h"

enum shape_list {CIRCLE, SQUARE};

void lcdInit(void);
void drawShape(shape_list shape, uint16_t x_pos, uint16_t y_pos, uint16_t radius, uint16_t color);
void swapXY();
void drawText(uint16_t x_pos, uint16_t y_pos, const uint8_t *string, uint8_t ch_size, uint16_t color);
void setGRamArea(uint8_t size, bool swap);


#endif