/**
The MIT License (MIT)
Copyright (c) 2019 by Daniel Eichhorn, ThingPulse
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Please note: We are spending a lot of time to write and maintain open source codes
Please support us by buying our products from https://thingpulse.com/shop/

See more at https://thingpulse.com

Many thanks go to various contributors such as Adafruit, Waveshare.
*/

#ifndef _MINIGRAFX_ILI9163H_
#define _MINIGRAFX_ILI9163H_

#include "Arduino.h"
#include "Print.h"
#include "DisplayDriver.h"
#if defined (__AVR__)
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif


#define ILI9163_TFTWIDTH  128
#define ILI9163_TFTHEIGHT 160

#define ILI9163_NOP     0x00
#define ILI9163_SWRESET 0x01

#define ILI9163_IDXRD   0x00 //0xDD // ILI9341 only, indexed control register read

#define ILI9163_INVOFF  0x20
#define ILI9163_INVON   0x21
#define ILI9163_GAMMASET 0x26
#define ILI9163_DISPOFF 0x28
#define ILI9163_DISPON  0x29

#define ILI9163_CASET   0x2A
#define ILI9163_PASET   0x2B
#define ILI9163_RAMWR   0x2C
#define ILI9163_RAMRD   0x2E

#define ILI9163_PTLAR   0x30
#define ILI9163_MADCTL  0x36
#define ILI9163_PIXFMT  0x3A

#define ILI9163_FRMCTR1 0xB1
#define ILI9163_FRMCTR2 0xB2
#define ILI9163_FRMCTR3 0xB3
#define ILI9163_INVCTR  0xB4
#define ILI9163_DFUNCTR 0xB6

#define ILI9163_PWCTR1  0xC0
#define ILI9163_PWCTR2  0xC1
#define ILI9163_PWCTR3  0xC2
#define ILI9163_PWCTR4  0xC3
#define ILI9163_PWCTR5  0xC4
#define ILI9163_VMCTR1  0xC5
#define ILI9163_VMCTR2  0xC7

#define ILI9163_RDID1   0xDA
#define ILI9163_RDID2   0xDB
#define ILI9163_RDID3   0xDC
#define ILI9163_RDID4   0xDD

#define ILI9163_GMCTRP1 0xE0
#define ILI9163_GMCTRN1 0xE1

#define ILI9163_PWCTR6  0xFC


// Color definitions
#define ILI9163_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9163_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9163_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9163_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9163_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9163_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9163_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9163_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9163_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9163_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9163_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9163_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9163_RED         0xF800      /* 255,   0,   0 */
#define ILI9163_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9163_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9163_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9163_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9163_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9163_PINK        0xF81F


class ILI9163_SPI : public DisplayDriver {

 public:

  ILI9163_SPI(int8_t _CS, int8_t _DC, int8_t _MOSI, int8_t _SCLK,
       int8_t _RST, int8_t _MISO);
  ILI9163_SPI(int8_t _CS, int8_t _DC, int8_t _RST = -1);

  void init(void);
  void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
  void setRotation(uint8_t r);

  void writeBuffer(BufferInfo *bufferInfo);

  void pushColor(uint16_t color);
  void setFastRefresh(boolean isFastRefreshEnabled);

  void spiwrite(uint8_t);
  void writecommand(uint8_t c);
  void writedata(uint8_t d);



 private:
  boolean  hwSPI;
  int32_t  _cs, _dc, _rst, _mosi, _miso, _sclk;

};

#endif
