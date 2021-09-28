/**************************************************************************
  This is a library for several Adafruit displays based on ST77* drivers.

  Works with the Adafruit 1.8" TFT Breakout w/SD card
    ----> http://www.adafruit.com/products/358
  The 1.8" TFT shield
    ----> https://www.adafruit.com/product/802
  The 1.44" TFT breakout
    ----> https://www.adafruit.com/product/2088
  as well as Adafruit raw 1.8" TFT display
    ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams.
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional).

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 **************************************************************************/

#ifndef _ADAFRUIT_GC9106H_
#define _ADAFRUIT_GC9106H_

#include "Arduino.h"
#include "Print.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>

#define GC9106_TFTWIDTH 80
#define GC9106_TFTHEIGHT 160

#define GC_CMD_DELAY 0x80

#define GC9106_RST_DELAY 120    ///< delay ms wait for reset finish
#define GC9106_SLPIN_DELAY 120  ///< delay ms wait for sleep in finish
#define GC9106_SLPOUT_DELAY 120 ///< delay ms wait for sleep out finish

#define GC9106_NOP 0x00
#define GC9106_SWRESET 0x01
#define GC9106_RDDID 0x04
#define GC9106_RDDST 0x09

#define GC9106_SLPIN 0x10
#define GC9106_SLPOUT 0x11
#define GC9106_PTLON 0x12
#define GC9106_NORON 0x13

#define GC9106_INVOFF 0x20
#define GC9106_INVON 0x21
#define GC9106_DISPOFF 0x28
#define GC9106_DISPON 0x29

#define GC9106_CASET 0x2A
#define GC9106_RASET 0x2B
#define GC9106_RAMWR 0x2C
#define GC9106_RAMRD 0x2E

#define GC9106_PTLAR 0x30
#define GC9106_TEOFF 0x34
#define GC9106_TEON 0x35
#define GC9106_MADCTL 0x36
#define GC9106_COLMOD 0x3A
#define GC9106_SCANLSET 0x44

#define GC9106_FRMCTR1 0xB1
#define GC9106_FRMCTR2 0xB2
#define GC9106_FRMCTR3 0xB3
#define GC9106_INVCTR 0xB4
#define GC9106_DISSET5 0XB6

#define GC9106_VREG1CTL 0xE6
#define GC9106_VREG2CTL 0xE7
#define GC9106_GAMMA1 0xF0
#define GC9106_GAMMA2 0xF1
#define GC9106_INTERRE1 0xFE
#define GC9106_INTERRE2 0xEF

#define GC9106_MADCTL_MY 0x80
#define GC9106_MADCTL_MX 0x40
#define GC9106_MADCTL_MV 0x20
#define GC9106_MADCTL_ML 0x10
#define GC9106_MADCTL_BGR 0x08
#define GC9106_MADCTL_MH 0x04
#define GC9106_MADCTL_RGB 0x00

// Some ready-made 16-bit ('565') color settings:
#define GC9106_BLACK 0x0000
#define GC9106_WHITE 0xFFFF
#define GC9106_RED 0xF800
#define GC9106_GREEN 0x07E0
#define GC9106_BLUE 0x001F
#define GC9106_CYAN 0x07FF
#define GC9106_MAGENTA 0xF81F
#define GC9106_YELLOW 0xFFE0
#define GC9106_ORANGE 0xFC00

/// Subclass of SPITFT for GC9106 displays (lots in common!)
class Adafruit_GC9106 : public Adafruit_SPITFT {
public:

/**************************************************************************/
/*!
    @brief  Instantiate GC9106 driver with software SPI
    @param  cs    Chip select pin #
    @param  dc    Data/Command pin #
    @param  mosi  SPI MOSI pin #
    @param  sclk  SPI Clock pin #
    @param  rst   Reset pin # (optional, pass -1 if unused)
    @param  miso  SPI MISO pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
  Adafruit_GC9106(int8_t cs, int8_t dc, int8_t mosi, int8_t sclk, int8_t rst)
    : Adafruit_SPITFT(GC9106_TFTWIDTH, GC9106_TFTHEIGHT, cs, dc, mosi, sclk, rst, -1) {}

/**************************************************************************/
/*!
    @brief  Instantiate GC9106 driver with hardware SPI
    @param  cs    Chip select pin #
    @param  dc    Data/Command pin #
    @param  rst   Reset pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
  Adafruit_GC9106(int8_t cs, int8_t dc, int8_t rst)
    : Adafruit_SPITFT(GC9106_TFTWIDTH, GC9106_TFTHEIGHT, cs, dc, rst) {}

#if !defined(ESP8266)
/**************************************************************************/
/*!
    @brief  Instantiate GC9106 driver with selectable hardware SPI
    @param  spiClass A pointer to an SPI device to use (e.g. &SPI1)
    @param  cs    Chip select pin #
    @param  dc    Data/Command pin #
    @param  rst   Reset pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
  Adafruit_GC9106(SPIClass *spiClass, int8_t cs, int8_t dc, int8_t rst)
    : Adafruit_SPITFT(GC9106_TFTWIDTH, GC9106_TFTHEIGHT, spiClass, cs, dc, rst) {}
#endif // end !ESP8266

  void init(void);
  void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
  void setRotation(uint8_t r);
  void enableDisplay(boolean enable);
  void enableTearing(boolean enable);
  void enableSleep(boolean enable);

protected:
  uint8_t _colstart = 0,   ///< Some displays need this changed to offset
      _rowstart = 0,       ///< Some displays need this changed to offset
      spiMode = SPI_MODE0; ///< Certain display needs MODE3 instead

  void begin(uint32_t freq = 0);
  void commonInit(const uint8_t *cmdList);
  void displayInit(const uint8_t *addr);
  void setColRowStart(int8_t col, int8_t row);
};

#endif // _ADAFRUIT_GC9106H_
