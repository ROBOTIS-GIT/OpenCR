

#include <LCD.h>
#include <SPI.h>

#define USE_FRAME_BUFFER     1


#if USE_FRAME_BUFFER == 1
uint16_t frame_buf[320*240];
#endif

void TFT::lcd_init()
{
	__LCD_DC_OUT();
	__LCD_DC_SET();

	__LCD_CS_OUT();
    __LCD_CS_SET();

	__LCD_BKL_OUT();
    __LCD_BKL_OFF();

	//Driving ability Setting
	lcd_write_register(0xEA,0x00); //PTBA[15:8]
	lcd_write_register(0xEB,0x20); //PTBA[7:0]
	lcd_write_register(0xEC,0x0C); //STBA[15:8]
	lcd_write_register(0xED,0xC4); //STBA[7:0]
	lcd_write_register(0xE8,0x38); //OPON[7:0]
	lcd_write_register(0xE9,0x10); //OPON1[7:0]
	lcd_write_register(0xF1,0x01); //OTPS1B
	lcd_write_register(0xF2,0x10); //GEN
	//Gamma 2.2 Setting
	lcd_write_register(0x40,0x01); //
	lcd_write_register(0x41,0x00); //
	lcd_write_register(0x42,0x00); //
	lcd_write_register(0x43,0x10); //
	lcd_write_register(0x44,0x0E); //
	lcd_write_register(0x45,0x24); //
	lcd_write_register(0x46,0x04); //
	lcd_write_register(0x47,0x50); //
	lcd_write_register(0x48,0x02); //
	lcd_write_register(0x49,0x13); //
	lcd_write_register(0x4A,0x19); //
	lcd_write_register(0x4B,0x19); //
	lcd_write_register(0x4C,0x16); //
	lcd_write_register(0x50,0x1B); //
	lcd_write_register(0x51,0x31); //
	lcd_write_register(0x52,0x2F); //
	lcd_write_register(0x53,0x3F); //
	lcd_write_register(0x54,0x3F); //
	lcd_write_register(0x55,0x3E); //
	lcd_write_register(0x56,0x2F); //
	lcd_write_register(0x57,0x7B); //
	lcd_write_register(0x58,0x09); //
	lcd_write_register(0x59,0x06); //
	lcd_write_register(0x5A,0x06); //
	lcd_write_register(0x5B,0x0C); //
	lcd_write_register(0x5C,0x1D); //
	lcd_write_register(0x5D,0xCC); //
	//Power Voltage Setting
	lcd_write_register(0x1B,0x1B); //VRH=4.65V
	lcd_write_register(0x1A,0x01); //BT (VGH~15V,VGL~-10V,DDVDH~5V)
	lcd_write_register(0x24,0x2F); //VMH(VCOM High voltage ~3.2V)
	lcd_write_register(0x25,0x57); //VML(VCOM Low voltage -1.2V)
	//****VCOM offset**///
	lcd_write_register(0x23,0x88); //for Flicker adjust //can reload from OTP
	//Power on Setting
	lcd_write_register(0x18,0x34); //I/P_RADJ,N/P_RADJ, Normal mode 60Hz
	lcd_write_register(0x19,0x01); //OSC_EN='1', start Osc
	lcd_write_register(0x01,0x00); //DP_STB='0', out deep sleep
	lcd_write_register(0x1F,0x88);// GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0, STB=0
	delay(5);
	lcd_write_register(0x1F,0x80);// GAS=1, VOMG=00, PON=0, DK=0, XDK=0, DVDH_TRI=0, STB=0
	delay(5);
	lcd_write_register(0x1F,0x90);// GAS=1, VOMG=00, PON=1, DK=0, XDK=0, DVDH_TRI=0, STB=0
	delay(5);
	lcd_write_register(0x1F,0xD0);// GAS=1, VOMG=10, PON=1, DK=0, XDK=0, DDVDH_TRI=0, STB=0
	delay(5);
	//262k/65k color selection
	lcd_write_register(0x17,0x05); //default 0x06 262k color // 0x05 65k color
	//SET PANEL
	lcd_write_register(0x36,0x00); //SS_P, GS_P,REV_P,BGR_P
	//Display ON Setting
	lcd_write_register(0x28,0x38); //GON=1, DTE=1, D=1000
	delay(40);
	lcd_write_register(0x28,0x3F); //GON=1, DTE=1, D=1100

	lcd_write_register(0x16,0x18);
	//Set GRAM Area
	lcd_write_register(0x02,0x00);
	lcd_write_register(0x03,0x00); //Column Start
	lcd_write_register(0x04,0x00);
	lcd_write_register(0x05,0xEF); //Column End
	lcd_write_register(0x06,0x00);
	lcd_write_register(0x07,0x00); //Row Start
	lcd_write_register(0x08,0x01);
	lcd_write_register(0x09,0x3F); //Row End

    lcd_clear_screen(WHITE);
	__LCD_BKL_ON();
}

//draw a point on the lcd with the specified color.
//hwXpos specify x position.
//hwYpos specify y position.
//hwColor color of the point.
#if 1
void TFT::lcd_draw_point(uint16_t hwXpos, uint16_t hwYpos, uint16_t hwColor)
{
	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

  frame_buf[hwYpos*LCD_WIDTH+hwXpos] = hwColor>>8 | hwColor<<8;
}

#else
void TFT::lcd_draw_point(uint16_t hwXpos, uint16_t hwYpos, uint16_t hwColor)
{
	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

	lcd_set_cursor(hwXpos, hwYpos);
	lcd_write_byte(0x22, LCD_CMD);
  lcd_write_word(hwColor);
}
#endif



void TFT::drawFrame(void)
{

#if USE_FRAME_BUFFER == 1
  lcd_set_cursor(0, 0);
	lcd_write_byte(0x22, LCD_CMD);

  __LCD_DC_SET();
  __LCD_CS_CLR();

/*
  for (int i=0; i<LCD_WIDTH*LCD_HEIGHT; i++)
  {
    __LCD_WRITE_BYTE(frame_buf[i] >> 8);
    __LCD_WRITE_BYTE(frame_buf[i] & 0xFF);
  }
  */
  SPI.writeFast(frame_buf, LCD_WIDTH*LCD_HEIGHT*2);

  __LCD_CS_SET();

#endif
}

//display a char at the specified position on lcd.
void TFT::lcd_display_char(uint16_t hwXpos, //specify x position.
                         uint16_t hwYpos, //specify y position.
                         uint8_t chChr,   //a char is display.
                         uint8_t chSize,  //specify the size of the char
                         uint16_t hwColor) //specify the color of the char
{
	uint8_t i, j, chTemp;
	uint16_t hwYpos0 = hwYpos, hwColorVal = 0;

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}


    for (i = 0; i < chSize; i ++) {
		if (FONT_1206 == chSize) {
			chTemp = pgm_read_byte(&c_chFont1206[chChr - 0x20][i]);
		} else if (FONT_1608 == chSize) {
			chTemp = pgm_read_byte(&c_chFont1608[chChr - 0x20][i]);
		}

        for (j = 0; j < 8; j ++) {
    		if (chTemp & 0x80) {
				hwColorVal = hwColor;
				lcd_draw_point(hwXpos, hwYpos, hwColorVal);
    		}
			chTemp <<= 1;
			hwYpos ++;
			if ((hwYpos - hwYpos0) == chSize) {
				hwYpos = hwYpos0;
				hwXpos ++;
				break;
			}
		}
    }
}


//_pow
static uint32_t _pow(uint8_t m, uint8_t n)
{
	uint32_t result = 1;

	while(n --) result *= m;
	return result;
}

//display a number at the specified position on lcd.
void TFT::lcd_display_num(uint16_t hwXpos,  //specify x position.
                          uint16_t hwYpos, //specify y position.
                          uint32_t chNum,  //a number is display.
                          uint8_t chLen,   //length ot the number
                          uint8_t chSize,  //specify the size of the number
                          uint16_t hwColor) //specify the color of the number
{
	uint8_t i;
	uint8_t chTemp, chShow = 0;

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

	for(i = 0; i < chLen; i ++) {
		chTemp = (chNum / _pow(10, chLen - i - 1)) % 10;
		if(chShow == 0 && i < (chLen - 1)) {
			if(chTemp == 0) {
				lcd_display_char(hwXpos + (chSize / 2) * i, hwYpos, ' ', chSize, hwColor);
				continue;
			} else {
				chShow = 1;
			}
		}
	 	lcd_display_char(hwXpos + (chSize / 2) * i, hwYpos, chTemp + '0', chSize, hwColor);
	}
}

//display a string at the specified position on lcd.
void TFT::lcd_display_string(uint16_t hwXpos, //specify x position.
                         uint16_t hwYpos,   //specify y position.
                         const uint8_t *pchString,  //a pointer to string
                         uint8_t chSize,    // the size of the string
                         uint16_t hwColor)  // specify the color of the string
{

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

    while (*pchString != '\0') {
        if (hwXpos > (LCD_WIDTH - chSize / 2)) {
			hwXpos = 0;
			hwYpos += chSize;
			if (hwYpos > (LCD_HEIGHT - chSize)) {
				hwYpos = hwXpos = 0;
				lcd_clear_screen(0x00);
			}
		}

        lcd_display_char(hwXpos, hwYpos, (uint8_t)*pchString, chSize, hwColor);
        hwXpos += chSize / 2;
        pchString ++;
    }
}

//draw a line at the specified position on lcd.
void TFT::lcd_draw_line(uint16_t hwXpos0, //specify x0 position.
                      uint16_t hwYpos0, //specify y0 position.
                      uint16_t hwXpos1, //specify x1 position.
                      uint16_t hwYpos1, //specify y1 position.
                      uint16_t hwColor) //specify the color of the line
{
	int x = hwXpos1 - hwXpos0;
    int y = hwYpos1 - hwYpos0;
    int dx = abs(x), sx = hwXpos0 < hwXpos1 ? 1 : -1;
    int dy = -abs(y), sy = hwYpos0 < hwYpos1 ? 1 : -1;
    int err = dx + dy, e2;

	if (hwXpos0 >= LCD_WIDTH || hwYpos0 >= LCD_HEIGHT || hwXpos1 >= LCD_WIDTH || hwYpos1 >= LCD_HEIGHT) {
		return;
	}

    for (;;){
        lcd_draw_point(hwXpos0, hwYpos0 , hwColor);
        e2 = 2 * err;
        if (e2 >= dy) {
            if (hwXpos0 == hwXpos1) break;
            err += dy; hwXpos0 += sx;
        }
        if (e2 <= dx) {
            if (hwYpos0 == hwYpos1) break;
            err += dx; hwYpos0 += sy;
        }
    }
}

//draw a circle at the specified position on lcd.
void TFT::lcd_draw_circle(uint16_t hwXpos,  //specify x position.
                        uint16_t hwYpos,  //specify y position.
                        uint16_t hwRadius, //specify the radius of the circle.
                        uint16_t hwColor)  //specify the color of the circle.
{
	int x = -hwRadius, y = 0, err = 2 - 2 * hwRadius, e2;

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

    do {
        lcd_draw_point(hwXpos - x, hwYpos + y, hwColor);
        lcd_draw_point(hwXpos + x, hwYpos + y, hwColor);
        lcd_draw_point(hwXpos + x, hwYpos - y, hwColor);
        lcd_draw_point(hwXpos - x, hwYpos - y, hwColor);
        e2 = err;
        if (e2 <= y) {
            err += ++ y * 2 + 1;
            if(-x == y && e2 <= x) e2 = 0;
        }
        if(e2 > x) err += ++ x * 2 + 1;
    } while(x <= 0);
}

//fill a rectangle out at the specified position on lcd.
void TFT::lcd_fill_rect(uint16_t hwXpos,  //specify x position.
                   uint16_t hwYpos,  //specify y position.
                   uint16_t hwWidth, //specify the width of the rectangle.
                   uint16_t hwHeight, //specify the height of the rectangle.
                   uint16_t hwColor)  //specify the color of rectangle.
{
	uint16_t i, j;

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

	for(i = 0; i < hwHeight; i ++){
		for(j = 0; j < hwWidth; j ++){
			lcd_draw_point(hwXpos + j, hwYpos + i, hwColor);
		}
	}
}

//draw a vertical line at the specified position on lcd.
void TFT::lcd_draw_v_line(uint16_t hwXpos, //specify x position.
                        uint16_t hwYpos, //specify y position.
                        uint16_t hwHeight, //specify the height of the vertical line.
                        uint16_t hwColor)  //specify the color of the vertical line.
{
	uint16_t i, y1 = min(hwYpos + hwHeight, LCD_HEIGHT - 1);

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

    for (i = hwYpos; i < y1; i ++) {
        lcd_draw_point(hwXpos, i, hwColor);
    }
}

//draw a horizonal line at the specified position on lcd.
void TFT::lcd_draw_h_line(uint16_t hwXpos, //specify x position.
                        uint16_t hwYpos,  //specify y position.
                        uint16_t hwWidth, //specify the width of the horizonal line.
                        uint16_t hwColor) //specify the color of the horizonal line.
{
	uint16_t i, x1 = min(hwXpos + hwWidth, LCD_WIDTH - 1);

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

    for (i = hwXpos; i < x1; i ++) {
        lcd_draw_point(i, hwYpos, hwColor);
    }
}

void TFT::lcd_draw_rect(uint16_t hwXpos,  //specify x position.
                      uint16_t hwYpos,  //specify y position.
                      uint16_t hwWidth, //specify the width of the rectangle.
                      uint16_t hwHeight, //specify the height of the rectangle.
                      uint16_t hwColor)  //specify the color of rectangle.
{
	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

	lcd_draw_h_line(hwXpos, hwYpos, hwWidth, hwColor);
	lcd_draw_h_line(hwXpos, hwYpos + hwHeight, hwWidth, hwColor);
	lcd_draw_v_line(hwXpos, hwYpos, hwHeight, hwColor);
	lcd_draw_v_line(hwXpos + hwWidth, hwYpos, hwHeight + 1, hwColor);
}


TFT Tft = TFT();
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
