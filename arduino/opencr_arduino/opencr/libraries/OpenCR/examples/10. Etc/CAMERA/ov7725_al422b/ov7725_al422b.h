#ifndef _OV7725_AL422B_H_
#define _OV7725_AL422B_H_

#include <Arduino.h>
#include <DynamixelSDK.h>
#include "XPT2046.h"
#include "./src/OV7725/OV7725.h"
#include "./src/sccb/sccb.h"
#include "./src/tftLcd.h"

extern  uint32_t panGoalPosition;
extern  uint32_t tiltGoalPosition;
extern  uint32_t panPresentPosition;
extern  uint32_t tiltPresentPosition;
extern  uint8_t  dxlError;
extern  int      dxlCommResult;

typedef struct {
  uint16_t coordX;
  uint16_t coordY;
  uint16_t object_width;
  uint16_t object_height;
  float object_weight;
  bool track;
} object_info_t;

typedef struct {
  float maxRed;
  float maxGreen;
  float maxBlue;
  float minRed;
  float minGreen;
  float minBlue;
} color_range_t;

const color_range_t selected_color[] = {
  {1.0, 0.5, 0.5, 0.4, 0.0, 0.0},	//red
  {0.6, 0.6, 1.0, 0.0, 0.0, 0.4},	//blue
};

void lcdInit(void);
void getSingleTouchPoint(uint16_t* touch_list);
void drawScreen();
void clearScreen(uint16_t color);
void lcdRotation(uint8_t rotation);
void drawText(uint16_t x_pos, uint16_t y_pos, const uint8_t *string, uint8_t ch_size, uint16_t color);
void setGRamArea(uint8_t size);

void colorFilter(uint16_t *image, uint8_t size);
bool colorFinder(const uint16_t *image, uint32_t pixelLocation);
void objectFinder(const uint16_t *masked_image);
void cellWeight(void);
void findCenterCell(void);
void displayInfo(uint8_t fps);

void initDynamixel(dynamixel::PortHandler *hPort, dynamixel::PacketHandler *hPacket);
void trackObject(dynamixel::PortHandler *hPort, dynamixel::PacketHandler *hPacket);
void movePlatform(dynamixel::PortHandler *hPort, dynamixel::PacketHandler *hPacket);


#endif