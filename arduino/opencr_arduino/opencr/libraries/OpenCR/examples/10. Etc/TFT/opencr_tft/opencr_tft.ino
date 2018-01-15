//Before, running this project. you have to know that the coordinate of LCD and Touch is different. 
//The relationship between LCD and touch is,
//LCD   x : 0 ~ 240,    y : 0 ~ 320   (origin : when you set the OpenCR direction with the usb port facing downside, left-upper point is (0,0))
//Touch x : 2016 ~ 127, y : 2016 ~ 127
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

void getSingleTouchPoint(uint16_t* touch_list){

  SPI.setClockDivider(SPI_CLOCK_DIV32);
  Tp.tp_show_single_info(touch_list);
  touch_list[XPOS] = constrain(touch_list[XPOS], TOUCH_MIN, TOUCH_MAX);
  touch_list[YPOS] = constrain(touch_list[YPOS], TOUCH_MIN, TOUCH_MAX);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
}


uint16_t lcd_list[2];
uint16_t* convCoordinateTouch2LCD(uint16_t* touch_list){
   
   lcd_list[XPOS] = LCD_MAX_X*(TOUCH_MAX-touch_list[XPOS])/TOUCH_RANGE;
   lcd_list[YPOS] = LCD_MAX_Y*(TOUCH_MAX-touch_list[YPOS])/TOUCH_RANGE;
   return lcd_list;
}


void drawShape(shape_list shape, uint16_t x_pos, uint16_t y_pos, uint16_t radius, uint16_t color){

  switch(shape)
  {
    case CIRCLE:
     Tft.lcd_draw_circle(x_pos, y_pos, radius, color);
     break;
     
    case SQUARE:
     Tft.lcd_fill_rect(x_pos, y_pos, radius, radius, color);
     break;
  }
 
}

void drawText(uint16_t x_pos, uint16_t y_pos, const uint8_t *string, uint8_t ch_size, uint16_t color){
  Tft.lcd_display_string(x_pos, y_pos, string, ch_size, color) ;
}

void drawTouch(uint16_t x_pos, uint16_t y_pos, uint16_t color){
  Tft.lcd_draw_point(x_pos,y_pos,color);
}

////////////////////////////////////
uint16_t touch_pos[2];


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  __SD_CS_DISABLE();
  SPI.beginFast();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);

  Tft.lcd_init();
  Tp.tp_init();
    
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t pre_time;
    
  uint16_t *lcd_pos;
 
  getSingleTouchPoint(touch_pos);
  lcd_pos = convCoordinateTouch2LCD(touch_pos);

  if(millis() - pre_time >= 50)
  {
    pre_time = millis();
    Serial.print("touch : ");
    Serial.print(touch_pos[XPOS]);
    Serial.print(" , ");
    Serial.println(touch_pos[YPOS]);
    Serial.print("lcd : ");
    Serial.print(lcd_pos[XPOS]);
    Serial.print(" , ");
    Serial.println(lcd_pos[YPOS]);
    Serial.println();
  
    drawShape(SQUARE, 120, 160, 50, RED);
    drawText(40,40, (const uint8_t *)"Hello, World!!!",16, RED);
    drawTouch(lcd_pos[XPOS], lcd_pos[YPOS], RED);
    Tft.drawFrame();
  }
}
