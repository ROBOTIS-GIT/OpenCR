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

#ifndef _WAVESHARE_HX8347D_H
#define _WAVESHARE_HX8347D_H

#include <Arduino.h>
#include <Print.h>
#include <Adafruit_GFX.h>
#include <avr/pgmspace.h>



#define HX8347D_TFTWIDTH  320
#define HX8347D_TFTHEIGHT 240


#define	HX8347D_BLACK   0x0000
#define	HX8347D_BLUE    0x001F
#define	HX8347D_RED     0xF800
#define	HX8347D_GREEN   0x07E0
#define HX8347D_CYAN    0x07FF
#define HX8347D_MAGENTA 0xF81F
#define HX8347D_YELLOW  0xFFE0
#define HX8347D_WHITE   0xFFFF


class Waveshare_HX8347D : public Adafruit_GFX {

 public:

  Waveshare_HX8347D(int8_t _cs = 10, int8_t _dc = 7, int8_t _led = 9, int8_t _rst = -1);

  void      begin(uint8_t type = 0);
  void      setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
  void      pushColor(uint16_t color);
  void      fillScreen(uint16_t color);
  void      drawPixel(int16_t x, int16_t y, uint16_t color);
  void      drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  void      drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  void      fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void      setRotation(uint8_t r);
  void      invertDisplay(boolean i);
  uint16_t  color565(uint8_t r, uint8_t g, uint8_t b);


  void      spiwrite(uint8_t);
  uint8_t   spiRead(void);


  uint8_t   readData(void);
  uint8_t   readCommand(uint8_t reg, uint8_t index = 0);

  void      writeCommand(uint8_t c);
  void      writeData(uint8_t d);
  void      writeData16(uint16_t c);

  void      writeReg(uint8_t cmd, uint8_t param);
  void      writeReg16(uint8_t cmd, uint16_t param);

  void      setLedPower(uint_least8_t power);

  void      drawFrame(void);

 private:
  uint8_t tabcolor;
  int8_t  _cs, _dc, _rst, _led;


  void initRegs(void);
};

#endif
