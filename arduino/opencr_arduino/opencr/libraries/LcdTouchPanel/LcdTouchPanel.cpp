#include <LcdTouchPanel.h>

void tftInit()
{

    __SD_CS_DISABLE();
    SPI.beginFast();
    SPI.setDataMode(SPI_MODE3);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV4);

    Tft.lcd_init();
    Tp.tp_init();
}

void getSingleTouchPoint(uint16_t* touch_list)
{

    //Make sure, that before reading the touch info, you must decrease the SPI speed under 2MHz. After finish this process, return to SPI_CLOCK_DIV4.
    SPI.setClockDivider(SPI_CLOCK_DIV32);
    Tp.tp_show_single_info(touch_list);
    touch_list[XPOS] = constrain(touch_list[XPOS], TOUCH_MIN, TOUCH_MAX);
    touch_list[YPOS] = constrain(touch_list[YPOS], TOUCH_MIN, TOUCH_MAX);
    SPI.setClockDivider(SPI_CLOCK_DIV4);
}

