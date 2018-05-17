#ifndef _OV7725_AL422B_H_
#define _OV7725_AL422B_H_

#include <Arduino.h>
#include "XPT2046.h"
#include "./src/OV7725/OV7725.h"
#include "./src/sccb/sccb.h"
#include "./src/tftLcd.h"

enum shape_list {CIRCLE, SQUARE};

typedef struct {
  float maxRed;
  float maxGreen;
  float maxBlue;
  float minRed;
  float minGreen;
  float minBlue;
} color_range_t;

const color_range_t selected_color[] = {
  {1.0, 0.5, 0.5, 0.5, 0.0, 0.0},	//red object
  {0.5, 1.0, 0.5, 0.0, 0.5, 0.0},	//green object
  {0.6, 0.6, 1.0, 0.0, 0.0, 0.4},	//blue object
};

void lcdInit(void);
void drawShape(shape_list shape, uint16_t x_pos, uint16_t y_pos, uint16_t radius, uint16_t color);
void clearScreen(uint16_t color);
void lcdRotation(uint8_t rotation);
void drawText(uint16_t x_pos, uint16_t y_pos, const uint8_t *string, uint8_t ch_size, uint16_t color);
void setGRamArea(uint8_t size);

void colorFilter(uint16_t *image, uint8_t size);
bool colorFinder(const uint16_t *image, uint32_t pixelLocation);


#endif