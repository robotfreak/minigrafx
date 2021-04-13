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
#include "ILI9163_SPI.h"
#ifdef __AVR__
  #include <avr/pgmspace.h>
#elif defined(ESP8266) || defined(ESP32)
  #include <pgmspace.h>
#endif

#ifndef ARDUINO_STM32_FEATHER
  #include "pins_arduino.h"
  #include "wiring_private.h"
#endif

#include <limits.h>
#include <SPI.h>

#define FILLARRAY(a,n) a[0]=n, memcpy( ((char*)a)+sizeof(a[0]), a, sizeof(a)-sizeof(a[0]) );

// If the SPI library has transaction support, these functions
// establish settings and protect from interference from other
// libraries.  Otherwise, they simply do nothing.
#ifdef SPI_HAS_TRANSACTION
static inline void spi_begin(void) __attribute__((always_inline));
static inline void spi_begin(void) {
#if defined (ARDUINO_ARCH_ARC32)
  // max speed!
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
#else
    // max speed!
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
#endif
}
static inline void spi_end(void) __attribute__((always_inline));
static inline void spi_end(void) {
  SPI.endTransaction();
}
#else
#define spi_begin()
#define spi_end()
#endif


// Constructor when using software SPI.  All output pins are configurable.
ILI9163_SPI::ILI9163_SPI(int8_t cs, int8_t dc, int8_t mosi,
           int8_t sclk, int8_t rst, int8_t miso) : DisplayDriver(ILI9163_TFTWIDTH, ILI9163_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _mosi  = mosi;
  _miso = miso;
  _sclk = sclk;
  _rst  = rst;
  hwSPI = true;
}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
ILI9163_SPI::ILI9163_SPI(int8_t cs, int8_t dc, int8_t rst) : DisplayDriver(ILI9163_TFTWIDTH, ILI9163_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _rst  = rst;
  hwSPI = true;
  _mosi  = _sclk = 0;
}

void ILI9163_SPI::spiwrite(uint8_t c) {

  //Serial.print("0x"); Serial.print(c, HEX); Serial.print(", ");

  if (hwSPI) {
#if defined (__AVR__)
  #ifndef SPI_HAS_TRANSACTION
    uint8_t backupSPCR = SPCR;
    SPCR = mySPCR;
  #endif
    SPDR = c;
    while(!(SPSR & _BV(SPIF)));
  #ifndef SPI_HAS_TRANSACTION
    SPCR = backupSPCR;
  #endif
#else
    SPI.transfer(c);
#endif
  } else {
#if defined(ESP8266) || defined(ESP32) || defined (ARDUINO_ARCH_ARC32)
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) {
  digitalWrite(_mosi, HIGH);
      } else {
  digitalWrite(_mosi, LOW);
      }
      digitalWrite(_sclk, HIGH);
      digitalWrite(_sclk, LOW);
    }
#else
    // Fast SPI bitbang swiped from LPD8806 library
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) {
  //digitalWrite(_mosi, HIGH);
  *mosiport |=  mosipinmask;
      } else {
  //digitalWrite(_mosi, LOW);
  *mosiport &= ~mosipinmask;
      }
      //digitalWrite(_sclk, HIGH);
      *clkport |=  clkpinmask;
      //digitalWrite(_sclk, LOW);
      *clkport &= ~clkpinmask;
    }
#endif
  }
}

void ILI9163_SPI::setFastRefresh(boolean isFastRefreshEnabled) {
  // Not enabled at the moment
}


void ILI9163_SPI::writecommand(uint8_t c) {
#if defined (USE_FAST_PINIO)
  *dcport &= ~dcpinmask;
  *csport &= ~cspinmask;
#else
  digitalWrite(_dc, LOW);
  digitalWrite(_sclk, LOW);
  digitalWrite(_cs, LOW);
#endif

  spiwrite(c);

#if defined (USE_FAST_PINIO)
  *csport |= cspinmask;
#else
  digitalWrite(_cs, HIGH);
#endif
}


void ILI9163_SPI::writedata(uint8_t c) {
#if defined (USE_FAST_PINIO)
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
#else
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
#endif

  spiwrite(c);

#if defined (USE_FAST_PINIO)
  *csport |= cspinmask;
#else
  digitalWrite(_cs, HIGH);
#endif
}


// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80




void ILI9163_SPI::init(void) {
  if (_rst > 0) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, LOW);
  }

  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);

#if defined (USE_FAST_PINIO)
  csport    = portOutputRegister(digitalPinToPort(_cs));
  cspinmask = digitalPinToBitMask(_cs);
  dcport    = portOutputRegister(digitalPinToPort(_dc));
  dcpinmask = digitalPinToBitMask(_dc);
#endif

  if(hwSPI) { // Using hardware SPI
    SPI.begin(_sclk, _miso, _mosi, -1);

    //SPI.begin();

#ifndef SPI_HAS_TRANSACTION
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
  #if defined (_AVR__)
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
    mySPCR = SPCR;
  #elif defined(TEENSYDUINO)
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
  #elif defined (__arm__)
    SPI.setClockDivider(11); // 8-ish MHz (full! speed!)
  #endif
#endif
  } else {
    pinMode(_sclk, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);

#if defined (USE_FAST_PINIO)
    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    clkpinmask  = digitalPinToBitMask(_sclk);
    mosiport    = portOutputRegister(digitalPinToPort(_mosi));
    mosipinmask = digitalPinToBitMask(_mosi);
    *clkport   &= ~clkpinmask;
    *mosiport  &= ~mosipinmask;
#endif
  }

  // toggle RST low to reset
  if (_rst > 0) {
    digitalWrite(_rst, HIGH);
    delay(5);
    digitalWrite(_rst, LOW);
    delay(20);
    digitalWrite(_rst, HIGH);
    delay(150);
  }


  if (hwSPI) spi_begin();

  writecommand(ILI9163_SWRESET); 
  delay(120);
  writecommand(0x11); 
  delay(5);

  writecommand(ILI9163_PIXFMT);
  writedata(0x05);

  writecommand(ILI9163_GAMMASET);    //Gamma curve selected
  writedata(0x04);

  writecommand(0xF2);    //Gamma adjust enable
  writedata(0x01);

  writecommand(ILI9163_GMCTRP1);    //Set Gamma
  writedata(0x3F);
  writedata(0x25);
  writedata(0x1C);
  writedata(0x1E);
  writedata(0x20);
  writedata(0x12);
  writedata(0x2A);
  writedata(0x90);
  writedata(0x24);
  writedata(0x11);
  writedata(0x00);
  writedata(0x00);
  writedata(0x00);
  writedata(0x00);
  writedata(0x00);

  writecommand(ILI9163_GMCTRN1);    //Set Gamma
  writedata(0x20);
  writedata(0x20);
  writedata(0x20);
  writedata(0x20);
  writedata(0x05);
  writedata(0x00);
  writedata(0x15);
  writedata(0xA7);
  writedata(0x3D);
  writedata(0x18);
  writedata(0x25);
  writedata(0x2A);
  writedata(0x2B);
  writedata(0x2B);
  writedata(0x3A);

  writecommand(ILI9163_FRMCTR1);   // Frame control
  writedata(0x08);
  writedata(0x08);

  writecommand(ILI9163_INVCTR);    // Display inversion
  writedata(0x07);

  writecommand(ILI9163_PWCTR1);    //Power control 1
  writedata(0x0A);   
  writedata(0x02);   

  writecommand(ILI9163_PWCTR2);    //Power control 2
  writedata(0x02);   

  writecommand(ILI9163_VMCTR1);    //VCM control
  writedata(0x50); 
  writedata(0x5B); 

  writecommand(ILI9163_VMCTR2);    //VCM offset
  writedata(0x40); 

  writecommand(ILI9163_CASET);    //VCM offset
  writedata(0x00); 
  writedata(0x00); 
  writedata(0x00); 
  writedata(0x7F); 
 
  writecommand(ILI9163_PASET);    //VCM offset
  writedata(0x00); 
  writedata(0x00); 
  writedata(0x00); 
  writedata(0x9F); 
  delay(250);

  writecommand(ILI9163_MADCTL);
  writedata(0xC8);

  writecommand(ILI9163_DISPON);    //Display on

  if (hwSPI) spi_end();

}


void ILI9163_SPI::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
 uint16_t y1) {

  writecommand(ILI9163_CASET); // Column addr set
  writedata(x0 >> 8);
  writedata(x0 & 0xFF);     // XSTART
  writedata(x1 >> 8);
  writedata(x1 & 0xFF);     // XEND

  writecommand(ILI9163_PASET); // Row addr set
  writedata(y0>>8);
  writedata(y0);     // YSTART
  writedata(y1>>8);
  writedata(y1);     // YEND

  writecommand(ILI9163_RAMWR); // write to RAM
}



#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void ILI9163_SPI::setRotation(uint8_t m) {

  if (hwSPI) spi_begin();
  writecommand(ILI9163_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writedata(MADCTL_MX | MADCTL_MY | MADCTL_BGR);
     _width  = ILI9163_TFTWIDTH;
     _height = ILI9163_TFTHEIGHT;
     break;
   case 1:
     writedata(MADCTL_MV | MADCTL_MY | MADCTL_BGR);
     _width  = ILI9163_TFTHEIGHT;
     _height = ILI9163_TFTWIDTH;
     break;
  case 2:
    writedata(MADCTL_BGR);
     _width  = ILI9163_TFTWIDTH;
     _height = ILI9163_TFTHEIGHT;
    break;
   case 3:
     writedata(MADCTL_MX | MADCTL_MV | MADCTL_BGR);
     _width  = ILI9163_TFTHEIGHT;
     _height = ILI9163_TFTWIDTH;
     break;
  }
  if (hwSPI) spi_end();
}

void ILI9163_SPI::pushColor(uint16_t color) {
  if (hwSPI) spi_begin();

  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);


  spiwrite(color >> 8);
  spiwrite(color);


  digitalWrite(_cs, HIGH);

  if (hwSPI) spi_end();

}

void ILI9163_SPI::writeBuffer(BufferInfo *bufferInfo) {
    if (hwSPI) spi_begin();
    uint8_t *buffer = bufferInfo->buffer;
    uint16_t bufferWidth = bufferInfo->bufferWidth;
    uint16_t bufferHeight = bufferInfo->bufferHeight;
    uint16_t xPos = bufferInfo->targetX;
    uint16_t yPos = bufferInfo->targetY;
    uint16_t *palette = bufferInfo->palette;
    uint8_t bitsPerPixel = bufferInfo->bitsPerPixel;

    setAddrWindow(xPos, yPos, xPos + bufferWidth - 1, yPos + bufferHeight -1 );

    digitalWrite(_dc, HIGH);
    digitalWrite(_cs, LOW);
    if (bitsPerPixel == 16) {
      SPI.writeBytes(buffer, bufferWidth * bufferHeight * 2);
      /*for (uint16_t y = 0; y < bufferHeight; y++) {
        for (uint16_t x = 0; x < bufferWidth; x++) {

        }
      }*/
    } else {
      // line buffer is in 16bit target format
      uint8_t lineBuffer[_width * 2];
      uint16_t pos;
      uint8_t bufferByte;
      uint8_t paletteEntry;
      uint16_t color;
      uint8_t shift;
      uint8_t mask = (1 << bitsPerPixel) - 1;
      uint8_t packagesPerBytes = 8 / bitsPerPixel;
      uint16_t bytePos = 0;
      uint16_t pixelCounter = 0;
      uint16_t bufferSize = bufferWidth * bufferHeight / packagesPerBytes;
      uint8_t bytesPerLine = bufferWidth / packagesPerBytes;
      uint16_t x = 0;
      for (uint16_t y = 0; y < bufferHeight; y++) {

        for (uint16_t b = 0; b < bytesPerLine; b++) {

          for (uint8_t p = 0; p < packagesPerBytes; p++) {
            x = b * packagesPerBytes + p;
            bufferByte = buffer[bytePos];
            shift = p * bitsPerPixel;
            paletteEntry = (bufferByte >> shift) & mask;
            color = palette[paletteEntry];
            lineBuffer[x * 2] = color >> 8;
            lineBuffer[x * 2 + 1] = color;
          }
          bytePos++;
        }
        SPI.writeBytes(lineBuffer, bufferWidth * 2);
      }
    }
    digitalWrite(_cs, HIGH);

    if (hwSPI) spi_end();
}
