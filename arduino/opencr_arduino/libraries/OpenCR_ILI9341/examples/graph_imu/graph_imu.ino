
#include <Adafruit_GFX.h>    // Core graphics library
#include "OpenCR_ILI9341.h" // Hardware-specific library
#include <SPI.h>
#include <SD.h>
#include <IMU.h>


cIMU    IMU;


// TFT display and SD card will share the hardware SPI interface.
// Hardware SPI pins are specific to the Arduino board type and
// cannot be remapped to alternate pins.  For Arduino Uno,
// Duemilanove, etc., pin 11 = MOSI, pin 12 = MISO, pin 13 = SCK.

#define TFT_DC    6
#define TFT_CS    7
#define TFT_MOSI  3
#define TFT_MISO  2
#define TFT_CLK   4
#define TFT_RST   5

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// work in line numbers.  Font height in ht
int16_t ht = 16, top = 0, line, lines = 19, scroll;

void setup()
{
    
    uint16_t id = tft.readID();
    tft.begin();
    tft.setRotation(0);     //Portrait
    tft.fillScreen(BLACK);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(2);     // System font is 8 pixels.  ht = 8*2=16
    tft.setCursor(100, 0);
    tft.print("ID = 0x");
    tft.println(id, HEX);

    lines = tft.height();
    
    tft.setCursor(0, 0);
    tft.fillScreen(BLACK);

    IMU.begin();

    scroll = lines-1;
}

uint8_t line_x_buf[320];
uint8_t line_y_buf[320];
uint8_t line_z_buf[320];

void loop()
{
    static int x = 0;
    static int y = 0;
    static int z = 0;
    
    tft.setCursor(0, (scroll + top) * 1);
    //tft.setCursor(0,lines );
    if (++scroll >= lines) scroll = 0;
    //tft.vertScroll(top*ht, lines * ht, (scroll) * ht);
    //tft.drawPixel( x, (scroll + top) * 1, YELLOW);    
    //tft.vertScroll(top, lines, scroll*1);

    tft.drawPixel( line_x_buf[scroll], scroll, BLACK);    
    tft.drawPixel( line_y_buf[scroll], scroll, BLACK);    
    tft.drawPixel( line_z_buf[scroll], scroll, BLACK);    
    
    tft.drawPixel( x, scroll, YELLOW);    
    tft.drawPixel( y, scroll, BLUE);    
    tft.drawPixel( z, scroll, RED);    
    
    line_x_buf[scroll] = x;
    line_y_buf[scroll] = y;
    line_z_buf[scroll] = z;
    
    tft.vertScroll(top, lines, scroll);
    
    
    delay(1);
    line++;
    

    IMU.update();

    x = IMU.angle[0]/10 + tft.width()/2;
    y = IMU.angle[1]/10 + tft.width()/2;
    z = IMU.angle[2]    + tft.width()/2;
}

