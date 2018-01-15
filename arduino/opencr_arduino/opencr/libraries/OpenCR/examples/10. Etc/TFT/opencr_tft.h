//Before, running this project. you have to know that the coordinate of LCD and Touch is different. 
//The relationship between LCD and touch is,
//LCD   x : 0 ~ 240,    y : 0 ~ 320   (origin : when you set the OpenCR direction with the usb port facing downside, left-upper point is (0,0))
//Touch x : 1950 ~ 127, y : 1950 ~ 127
//So, I'am gonna support you by using convCoordinateTouch2LCD function

#include <SPI.h>
#include "Adafruit_GFX.h"
#include "LCD.h"
#include "XPT2046.h"
#include "Touch.h"
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>

#define XPOS 0
#define YPOS 1

#define TOUCH_MIN 127
#define TOUCH_MAX 1950
#define TOUCH_RANGE (TOUCH_MAX-TOUCH_MIN)

#define LCD_MAX_X 240
#define LCD_MAX_Y 320

enum shape_list {CIRCLE, SQUARE};

void getSingleTouchPoint(uint16_t* touch_list);
void tftInit(void);
    