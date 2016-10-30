
#include <Adafruit_GFX.h>    // Core graphics library
#include "OpenCR_ILI9341.h" // Hardware-specific library
#include <SPI.h>
#include <SD.h>

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
int16_t ht = 16, top = 0, line, lines = 20, scroll;

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
    if (id == 0x9320 || id == 0x9325 || id == 0xB509) {
        top = 0;                      // these controllers scroll full screen
        lines = tft.height() / ht;    // we are in portrait mode
    }
    if (id == 0x7783) {
        tft.println("can NOT scroll");
        while (1);                    // die.
    }
    tft.setCursor(0, 0);
    for (line = 1; line < 21; line++) tft.println(String(line) + ": ");
}

void loop()
{
    tft.setCursor(0, (scroll + top) * ht);
    if (++scroll >= lines) scroll = 0;
    tft.vertScroll(top * ht, lines * ht, (scroll) * ht);
    tft.println(String(line) + ": [" + String(scroll) + "]  ");
    delay(1);
    line++;
}

