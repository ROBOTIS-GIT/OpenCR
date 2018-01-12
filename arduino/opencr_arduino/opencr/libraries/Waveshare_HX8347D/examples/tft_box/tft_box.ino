/***************************************************
  This is our library for the Adafruit HX8357D Breakout
  ----> http://www.adafruit.com/products/2050

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Waveshare_HX8347D.h"




Waveshare_HX8347D tft = Waveshare_HX8347D();




void setup() {
  uint32_t t_time;
  
  Serial.begin(9600);

  tft.begin();


  

  tft.setRotation(1);  
  
  tft.fillScreen(HX8347D_BLACK);
}


void loop(void) {
  static int x[8], y[8];
  static int x_dir[8] = {0,};
  static int x_width[8];
  static int x_step[8];
  static int x_color[8];

  uint32_t t_time;
  int i;

  
  x_width[0] = 30;  y[0] =       0;  x_step[0] = 1; x_color[0] = HX8347D_RED;
  x_width[1] = 30;  y[1] =      32;  x_step[1] = 2; x_color[1] = HX8347D_BLUE;
  x_width[2] = 30;  y[2] = y[1]+32;  x_step[2] = 3; x_color[2] = HX8347D_GREEN;
  x_width[3] = 30;  y[3] = y[2]+32;  x_step[3] = 4; x_color[3] = HX8347D_CYAN;
  x_width[4] = 30;  y[4] = y[3]+32;  x_step[4] = 5; x_color[4] = HX8347D_MAGENTA;
  x_width[5] = 30;  y[5] = y[4]+32;  x_step[5] = 6; x_color[5] = HX8347D_YELLOW;
  x_width[6] = 30;  y[6] = y[5]+32;  x_step[6] = 7; x_color[6] = HX8347D_WHITE;


  t_time = micros();


  for(i=0; i<7; i++)
  {
    tft.fillRect(x[i], y[i], x_width[i], x_width[i], x_color[i]);    
  }
  tft.drawFrame();
  
  for(i=0; i<7; i++)
  {  
    tft.fillRect(x[i], y[i], x_width[i], x_width[i], HX8347D_BLACK);      
    
    if( x_dir[i] == 0 )
      x[i] += x_step[i];
    else
      x[i] -= x_step[i];
  
    if(x[i]>240-x_width[i])
    {
      x_dir[i] ^= 1;
      x[i] = 240-x_width[i];
    }
  
    if(x[i]<0)
    {
      x_dir[i] ^= 1;
      x[i] = 0;
    }
  }
  
  uint32_t process_time;

  process_time = micros() - t_time;


  tft.setCursor(0, 260);
  tft.setTextColor(HX8347D_GREEN, HX8347D_BLACK);
  tft.setTextSize(3);
  tft.println((process_time/1000) + String(" ms "));    
  
  tft.setCursor(0, 300);
  tft.println(1000/(process_time/1000) + String(" FPS  "));
}


