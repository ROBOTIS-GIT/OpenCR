#include "ov7725_al422b.h"

void lcdInit()
{
    __SD_CS_DISABLE();
    SPI.beginFast();
    SPI.setDataMode(SPI_MODE3);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV4);

    TFTLCD.lcd_init();
}

void drawShape(shape_list shape, uint16_t x_pos, uint16_t y_pos, uint16_t radius, uint16_t color)
{
  switch(shape)
  {
    case CIRCLE:
     TFTLCD.lcd_draw_circle(x_pos, y_pos, radius, color);
     break;
     
    case SQUARE:
     TFTLCD.lcd_fill_rect(x_pos, y_pos, radius, radius, color);
     break;
  }
}

void swapXY()
{
  TFTLCD.MVRotation(2);
}

void drawText(uint16_t x_pos, uint16_t y_pos, const uint8_t *string, uint8_t ch_size, uint16_t color)
{
  TFTLCD.lcd_display_string(x_pos, y_pos, string, ch_size, color);
}

void setGRamArea(uint8_t size, bool swap)
{
  TFTLCD.setLcdMemoryArea(size, swap);
}