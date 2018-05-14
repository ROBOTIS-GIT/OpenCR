
#include "Waveshare_HX8347D.h"
#include <avr/pgmspace.h>
#include <limits.h>
#include <SPI.h>


#define USE_FRAME_BUFFER     1



#define spiWrite(x)     SPI.transfer(x)
#define spiWrite16(x)   SPI.transfer16(x)


#if USE_FRAME_BUFFER == 1
uint16_t frame_buf[320*240];
#endif


Waveshare_HX8347D::Waveshare_HX8347D(int8_t cs, int8_t dc, int8_t led, int8_t rst) : Adafruit_GFX(HX8347D_TFTWIDTH, HX8347D_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _led  = led;
  _rst  = rst;
}



void Waveshare_HX8347D::writeCommand(uint8_t c) {
  digitalWrite(_dc, LOW);
  digitalWrite(_cs, LOW);

  spiWrite(c);

  digitalWrite(_cs, HIGH);
}


void Waveshare_HX8347D::writeData(uint8_t c) {
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);

  spiWrite(c);

  digitalWrite(_cs, HIGH);
}


void Waveshare_HX8347D::writeData16(uint16_t c) {
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);

  spiWrite16(c);

  digitalWrite(_cs, HIGH);
}


void Waveshare_HX8347D::writeReg(uint8_t cmd, uint8_t param) {
  digitalWrite(_dc, LOW);

  digitalWrite(_cs, LOW);
  spiWrite(cmd);

  digitalWrite(_dc, HIGH);

  spiWrite(param);
  digitalWrite(_cs, HIGH);
}


void Waveshare_HX8347D::writeReg16(uint8_t cmd, uint16_t param) {
  digitalWrite(_dc, LOW);

  digitalWrite(_cs, LOW);
  spiWrite(cmd);


  digitalWrite(_dc, HIGH);

  spiWrite16(param);
  digitalWrite(_cs, HIGH);
}


void Waveshare_HX8347D::begin(uint8_t type) {

  pinMode(_led, OUTPUT);
  pinMode(_dc,  OUTPUT);
  pinMode(_cs,  OUTPUT);

  digitalWrite(_cs, HIGH);

  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(5, HIGH);
  digitalWrite(4, HIGH);

  //SPI.begin();
  SPI.beginFast();

  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);

  initRegs();

  setLedPower(100);
}

const uint8_t initdataQT2[] PROGMEM =
{
  //driving ability
  0x40| 2, 0xEA, 0x00,
  0x40| 2, 0xEB, 0x20,
  0x40| 2, 0xEC, 0x0C,
  0x40| 2, 0xED, 0xC4,
  0x40| 2, 0xE8, 0x40,
  0x40| 2, 0xE9, 0x38,
  0x40| 2, 0xF1, 0x01,
  0x40| 2, 0xF2, 0x10,
  0x40| 2, 0x27, 0xA3,
  //power voltage
  0x40| 2, 0x1B, 0x1B,
  0x40| 2, 0x1A, 0x01,
  0x40| 2, 0x24, 0x2F,
  0x40| 2, 0x25, 0x57,
  //VCOM offset
  0x40| 2, 0x23, 0x8D,
  //power on
  0x40| 2, 0x18, 0x36,
  0x40| 2, 0x19, 0x01, //start osc
  0x40| 2, 0x01, 0x00, //wakeup
  0x40| 2, 0x1F, 0x88,
  0xC0| 5, //5ms
  0x40| 2, 0x1F, 0x80,
  0xC0| 5, //5ms
  0x40| 2, 0x1F, 0x90,
  0xC0| 5, //5ms
  0x40| 2, 0x1F, 0xD0,
  0xC0| 5, //5ms
  //color selection
  0x40| 2, 0x17, 0x05, //0x05=65k, 0x06=262k
  //panel characteristic
  0x40| 2, 0x36, 0x00,
  //display options
  0x40| 2, 0x16, 0xA8,
  0x40| 2, 0x03, 0x00, //x0
  0x40| 2, 0x02, 0x00, //x0
  0x40| 2, 0x05, ((HX8347D_TFTWIDTH-1)>>0)&0xFF,
  0x40| 2, 0x04, ((HX8347D_TFTWIDTH-1)>>8)&0xFF,
  0x40| 2, 0x07, 0x00, //y0
  0x40| 2, 0x06, 0x00, //y0
  0x40| 2, 0x09, ((HX8347D_TFTHEIGHT-1)>>0)&0xFF,
  0x40| 2, 0x08, ((HX8347D_TFTHEIGHT-1)>>8)&0xFF,
  //display on
  0x40| 2, 0x28, 0x38,
  0xC0|50, //50ms
  0x40| 2, 0x28, 0x3C,
  0xC0| 5, //5ms
  0xFF   , 0xFF
};


void Waveshare_HX8347D::initRegs(void)
{
  uint_least8_t c, d, i;
  const PROGMEM uint8_t *ptr;

  //reset
  digitalWrite(_cs, HIGH);

  //send init commands and data
  ptr = &initdataQT2[0];
  while(1)
  {
    c = *(ptr);
    ptr++;
    if(c == 0xFF) //end of data
    {
      break;
    }
    switch(c&0xC0)
    {
      case 0x40: //command + data
        for(i=c&0x3F; i!=0; i-=2)
        {
          c = *(ptr);
          ptr++;
          d = *(ptr);
          ptr++;
          writeReg(c, d);
        }
        break;
      case 0xC0: //delay
        c = c&0x3F;
        delay(c);
        break;
    }
  }

  //clear display buffer
  //fillScreen(0);

  return;
}

void Waveshare_HX8347D::setLedPower(uint_least8_t power)
{
  if(power == 0) //off
  {
    analogWrite(_led, 0);
    digitalWrite(_led, LOW);
  }
  else if(power >= 100) //100%
  {
    analogWrite(_led, 255);
    digitalWrite(_led, HIGH);
  }
  else //1...99%
  {
    analogWrite(_led, (uint16_t)power*255/100);
  }

  return;
}


void Waveshare_HX8347D::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
 uint16_t y1) {

  writeReg(0x03, (x0>>0)); //set x0
  writeReg(0x02, (x0>>8)); //set x0
  writeReg(0x05, (x1>>0)); //set x1
  writeReg(0x04, (x1>>8)); //set x1
  writeReg(0x07, (y0>>0)); //set y0
  writeReg(0x06, (y0>>8)); //set y0
  writeReg(0x09, (y1>>0)); //set y1
  writeReg(0x08, (y1>>8)); //set y1

  writeCommand(0x22);
}


void Waveshare_HX8347D::pushColor(uint16_t color) {

  writeData16(color);
}


#if USE_FRAME_BUFFER == 1

void Waveshare_HX8347D::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  frame_buf[y*_width+x] = color>>8 | color<<8;
}


void Waveshare_HX8347D::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;

  if((y+h-1) >= _height)
    h = _height-y;

  color = color>>8 | color<<8;

  for(int i=0; i<h; i++)
  {
    frame_buf[(y+i)*_width+x] = color;
  }
}


void Waveshare_HX8347D::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;


  color = color>>8 | color<<8;

  for(int i=0; i<w; i++)
  {
    frame_buf[y*_width+x+i] = color;
  }


}

void Waveshare_HX8347D::fillScreen(uint16_t color) {
  fillRect(0, 0, _width, _height, color);
}

// fill a rectangle
void Waveshare_HX8347D::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  int32_t x_o = x;
  int32_t y_o = y;

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  uint8_t hi = color >> 8, lo = color;

  color = lo<<8 | hi<<0;

  for(y=0; y<h; y++) {
    for(x=0; x<w; x++) {
      frame_buf[(y_o+y)*_width+(x_o+x)] = color;
    }
  }
}
#else












void Waveshare_HX8347D::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x,y,x,y);
  writeData16(color);
}


void Waveshare_HX8347D::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;

  if((y+h-1) >= _height)
    h = _height-y;

  setAddrWindow(x, y, x, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);

  while (h--) {
    writeData16(color);
  }

  digitalWrite(_cs, HIGH);
}


void Waveshare_HX8347D::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);

  while (w--) {
    writeData(color);
  }

  digitalWrite(_cs, HIGH);
}

void Waveshare_HX8347D::fillScreen(uint16_t color) {
  fillRect(0, 0, _width, _height, color);
}


// fill a rectangle
void Waveshare_HX8347D::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      pushColor(color);
    }
  }
}

#endif


















// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Waveshare_HX8347D::color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void Waveshare_HX8347D::setRotation(uint8_t m) {

  uint_least8_t p;


  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     //writedata(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
     _width  = HX8347D_TFTWIDTH;
     _height = HX8347D_TFTHEIGHT;
     p = 0xA8; //MY=1 MX=0 MV=1 ML=0 BGR=1
     break;
   case 1:
     //writedata(MADCTL_MV | MADCTL_MY | MADCTL_RGB);
     _width  = HX8347D_TFTHEIGHT;
     _height = HX8347D_TFTWIDTH;
     p = 0x08; //MY=0 MX=0 MV=0 ML=0 BGR=1
     break;
  case 2:
    //writedata( MADCTL_RGB);
     _width  = HX8347D_TFTWIDTH;
     _height = HX8347D_TFTHEIGHT;
     p = 0x68; //MY=0 MX=1 MV=1 ML=0 BGR=1
    break;
   case 3:
     //writedata(MADCTL_MX | MADCTL_MV | MADCTL_RGB);
     _width  = HX8347D_TFTHEIGHT;
     _height = HX8347D_TFTWIDTH;
     p = 0xC8; //MY=1 MX=0 MV=1 ML=0 BGR=1
     break;
  }

  writeReg(0x16, p);
  setAddrWindow(0, 0, _width-1, _height-1);
}


void Waveshare_HX8347D::invertDisplay(boolean i) {
  //writecommand(i ? HX8357_INVON : HX8357_INVOFF);
}


void Waveshare_HX8347D::drawFrame(void)
{

#if USE_FRAME_BUFFER == 1
  setAddrWindow(0, 0, _width-1, _height-1);

  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);

  SPI.writeFast(frame_buf, _width*_height*2);


  digitalWrite(_cs, HIGH);
#endif
}


////////// stuff not actively being used, but kept for posterity


uint8_t Waveshare_HX8347D::spiRead(void) {
  uint8_t r = 0;

  r = SPI.transfer(0x00);

  return r;
}

 uint8_t Waveshare_HX8347D::readData(void) {
   digitalWrite(_dc, HIGH);
   digitalWrite(_cs, LOW);
   uint8_t r = spiRead();
   digitalWrite(_cs, HIGH);

   return r;
}


uint8_t Waveshare_HX8347D::readCommand(uint8_t c, uint8_t index) {
   digitalWrite(_dc, LOW);
   digitalWrite(_cs, LOW);

   spiwrite(c);

   digitalWrite(_dc, HIGH);
   uint8_t r = spiRead();
   digitalWrite(_cs, HIGH);
   return r;
}
