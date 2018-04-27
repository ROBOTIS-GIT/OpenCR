/*******************************************************************************
* Copyright (c) 2018, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Baram */

#include <LcdTouchPanel.h>
#include "lds.h"



#define DISTANCE_MAX    1000      // 100.0 cm 



lds_scan_t lds_scan;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115760);
  Serial2.begin(230400);

  ldsInit(&lds_scan);
  tftInit();

  delay(100);
  Serial2.print("b");

  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);  
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t pre_time;


  if (Serial.available())
  {
    Serial2.write(Serial.read());
  }

  while(Serial2.available() > 0)
  {
    if (ldsUpdate(&lds_scan, Serial2.read()) == true)
    {
      drawPoint();
    }
  }
  
  if (millis()-pre_time >= 50)
  {
    pre_time = millis();

    /*
    Serial.print(lds_scan.data[1].range);
    Serial.print(" ");
    Serial.print(lds_scan.data[1].intensity);
    Serial.print(" ");
    Serial.print(lds_scan.scan_time);
    Serial.println(" ");    
    */
  }

}

void drawPoint(void)
{
  int16_t cx = 240/2;
  int16_t cy = 320/2;
  int16_t x;
  int16_t y;
  int32_t d;

  // double fx;
  // double fy;
  double r;
  double offset_r = 90. * M_PI / 180.;
  static uint8_t mode = 0;
  float  h;
  uint16_t color;
  uint8_t rgb[3];

  //uint32_t pre_time = millis();

  Tft.lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, BLACK);


  if (mode == 0)
  {
    for (int i=0; i<360; i++)
    {
      // range normalize
      d = constrain(lds_scan.data[i].range, 0, 3500);
      d = map(d, 0, DISTANCE_MAX, 0, 120);

      r = i * 2.0 * M_PI / 360.;
      x = cx + cos(r+offset_r) * d;
      y = cy - sin(r+offset_r) * d;

      x = constrain(x, 0, 237);
      y = constrain(y, 0, 319-11);

      // intensity normalize
      h = map(constrain(lds_scan.data[i].intensity, 0, 5000), 0, 5000, 0, 1000) / 1000.;
      hslToRgb(h, 0.5, 0.5, rgb);
      color = color565(rgb[0], rgb[1], rgb[2]);

      Tft.lcd_fill_rect(x, y, 3, 3, color);

      if (i == 0)
      {
        x = cx + cos(r+offset_r) * 118;
        y = cy - sin(r+offset_r) * 118;      
        Tft.lcd_draw_line(cx, cy, x, y, WHITE);
      }
    }

    drawPalletBar();
  }

  if (mode == 1)
  {
    for (int i=0; i<320; i+=2)
    {
      d = constrain(lds_scan.data[i].range, 0, 2000);
      d = map(d, 0, 2000, 0, 200);
      Tft.lcd_draw_line(0, i, d, i, GREEN);
    }
    for (int i=1; i<320; i+=2)
    {
      d = constrain(lds_scan.data[i-1].intensity, 0, 5000);
      d = map(d, 0, 5000, 0, 200);
      Tft.lcd_draw_line(0, i, d, i, WHITE);
    }    
  }
  
  Tft.drawFrame();


  if (digitalRead(BDPIN_PUSH_SW_1) == HIGH)
  {
    mode = 0;
  }
  if (digitalRead(BDPIN_PUSH_SW_2) == HIGH)
  {
    mode = 1;
  }
}

void drawPallet(void)
{
  uint8_t rgb[3];
  float h;

  for (int y=0; y<320; y++)
  {
    h = (float)y / 320.;
    hslToRgb(h, 0.5, 0.5, rgb);
    Tft.lcd_draw_line(0, 319-y, 240-1, 319-y, color565(rgb[0], rgb[1], rgb[2]));    
  }  
  Tft.drawFrame();  
}

void drawPalletBar(void)
{
  uint8_t rgb[3];
  float h;

  for (int x=0; x<240; x++)
  {
    h = (float)x / 240.;
    hslToRgb(h, 0.5, 0.5, rgb);
    Tft.lcd_draw_line(x, 319-10, x, 319, color565(rgb[0], rgb[1], rgb[2]));    
  }  
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) 
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
/**
* Adapted from http://stackoverflow.com/questions/2353211/hsl-to-rgb-color-conversion
* Converts an HSL color value to RGB. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSL_color_space.
* Assumes h, s, and l are contained in the set [0, 1] and
* returns r, g, and b in the set [0, 255].
*
* @param   Number  h       The hue
* @param   Number  s       The saturation
* @param   Number  l       The lightness
* @return  Array           The RGB representation
*/
void hslToRgb(float h, float s, float l, byte * rgbIn) {
  float r, g, b;

  if (s == 0) {
    r = g = b = l; // achromatic
  } else {
    float q = l < 0.5 ? l * (1 + s) : l + s - l * s;
    float p = 2 * l - q;
    r = hue2rgb(p, q, h + 1.0 / 3);
    g = hue2rgb(p, q, h);
    b = hue2rgb(p, q, h - 1.0 / 3);
  }
  rgbIn[0] = min(r * 255, 255);
  rgbIn[1] = min(g * 255, 255);
  rgbIn[2] = min(b * 255, 255);
}

float hue2rgb (float p, float q, float t) {
  if (t < 0) t += 1;
  if (t > 1) t -= 1;
  if (t < 1.0 / 6) return p + (q - p) * 6 * t;
  if (t < 1.0 / 2) return q;
  if (t < 2.0 / 3) return p + (q - p) * (2.0 / 3 - t) * 6;
  return p;
}
