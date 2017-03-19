/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Bryan Newbold.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/


#ifndef _HARDWARETIMER_H_
#define _HARDWARETIMER_H_

#include <inttypes.h>





class HardwareTimer {
private:
    uint8_t tim_num;
public:
    HardwareTimer(uint8_t timerNum);

    void pause(void);
    void resume(void);
    void stop(void);
    void start(void);
    uint16_t setPeriod(uint32_t microseconds);
    void attachInterrupt(voidFuncPtr handler);
    void detachInterrupt(void);
    void refresh(void);
};

#endif
