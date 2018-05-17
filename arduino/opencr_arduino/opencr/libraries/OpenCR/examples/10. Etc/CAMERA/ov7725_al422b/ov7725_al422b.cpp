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

void clearScreen(uint16_t color)
{
  TFTLCD.lcd_clear_screen(color);
}

void lcdRotation(uint8_t rotation)
{
  TFTLCD.LCDRotation(rotation);
}

void drawText(uint16_t x_pos, uint16_t y_pos, const uint8_t *string, uint8_t ch_size, uint16_t color)
{
  TFTLCD.lcd_display_string(x_pos, y_pos, string, ch_size, color);
}

void setGRamArea(uint8_t size)
{
  TFTLCD.setLcdMemoryArea(size);
}

/********************
 * Image Processing
 * Pixel : GGGBBBBB RRRRRGGG
 ********************/
void colorFilter(uint16_t *image, uint8_t size)
{
  uint32_t pixelCount = 0;
  uint16_t img_width = LCD_WIDTH;
  uint16_t img_height = LCD_HEIGHT;

  if((size == QUARTERVIEW) || (size == QQVGA))
  {
    img_width = LCD_WIDTH / 2;
    img_height = LCD_HEIGHT / 2;
  }

  // maxRed = MIN(((TARGET_COLOR & 0xF800) + (COLOR_RANGE << 11 )), 0xF800);
  // maxGreen = MIN(((TARGET_COLOR & 0x07E0) + (COLOR_RANGE << 5)), 0x07E0);
  // maxBlue = MIN(((TARGET_COLOR & 0x001F) + COLOR_RANGE), 0x001F);
  // minRed = MAX(((TARGET_COLOR & 0xF800) - (COLOR_RANGE << 11)), 0x0000);
  // minGreen = MAX(((TARGET_COLOR & 0x07E0) - (COLOR_RANGE << 5)), 0x0000);
  // minBlue = MAX(((TARGET_COLOR & 0x001F) - COLOR_RANGE), 0x0000);

  for(; pixelCount < img_width*img_height; pixelCount++)
  {
    if(colorFinder(image, pixelCount))
    {
      //change the detected color to CYAN
      image[pixelCount] = BLUE;
    }
    else
    {
      image[pixelCount] = BLACK;
    }
  }
}

bool colorFinder(const uint16_t *image, uint32_t pixelLocation)
{
  const color_range_t* target_color = &selected_color[2];	//select color to detect
  uint16_t pixelColor;
  float propRed;
  float propGreen;
  float propBlue;
  uint8_t rgbSUM;

  pixelColor = image[pixelLocation];
  rgbSUM = ((pixelColor >> 11) & 0x1F) + ((pixelColor >> 5) & 0x3F) + (pixelColor & 0x1F);
  propRed = (float)((pixelColor >> 11) & 0x1F) / rgbSUM;
  propGreen = (float)((pixelColor >> 5) & 0x3F) / rgbSUM;
  propBlue = (float)(pixelColor & 0x001F) / rgbSUM;

  //compare Red range
  if ((propRed > target_color->minRed) && (propRed < target_color->maxRed))
  {
	//compare Green range
	if ((propGreen > target_color->minGreen) && (propGreen < target_color->maxGreen))
	{
	  //compare Blue ranges
	  if ((propBlue > target_color->minBlue) && (propBlue < target_color->maxBlue))
	  {
	    //if RGB is within TARGET_COLOR range, leave the color or else erase pixel to BLACK.
		  return true;
	  }
	  return false;
	}
	return false;
  }
  return false;
}