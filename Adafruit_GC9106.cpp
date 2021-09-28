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

#include "Adafruit_GC9106.h"
#include <limits.h>
#ifndef ARDUINO_STM32_FEATHER
#include "pins_arduino.h"
#include "wiring_private.h"
#endif
#include <SPI.h>

#define SPI_DEFAULT_FREQ 32000000 ///< Default SPI data clock frequency

/**************************************************************************/
/*!
    @brief  Companion code to the initiliazation tables. Reads and issues
            a series of LCD commands stored in PROGMEM byte array.
    @param  addr  Flash memory array with commands and data to send
*/
/**************************************************************************/
void Adafruit_GC9106::displayInit(const uint8_t *addr) {

  uint8_t numCommands, cmd, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++); // Number of commands to follow
  while (numCommands--) {              // For each command...
    cmd = pgm_read_byte(addr++);       // Read command
    numArgs = pgm_read_byte(addr++);   // Number of args to follow
    ms = numArgs & GC_CMD_DELAY;       // If hibit set, delay follows args
    numArgs &= ~GC_CMD_DELAY;          // Mask out delay bit
    sendCommand(cmd, addr, numArgs);
    addr += numArgs;

    if (ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if (ms == 255)
        ms = 500; // If 255, delay for 500 ms
      delay(ms);
    }
  }
}

/**************************************************************************/
/*!
    @brief  Initialize GC9106 chip. Connects to the GC9106 over SPI and
            sends initialization procedure commands
    @param  freq  Desired SPI clock frequency
*/
/**************************************************************************/
void Adafruit_GC9106::begin(uint32_t freq) {
  if (!freq) {
    freq = SPI_DEFAULT_FREQ;
  }
  _freq = freq;

  invertOnCommand = GC9106_INVON;
  invertOffCommand = GC9106_INVOFF;

  initSPI(freq, spiMode);
}

// SCREEN INITIALIZATION ***************************************************

// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.

// clang-format off
static const uint8_t PROGMEM
  initCommands[] = {                // Init commands for GC9106 screens
    24,                             // 24 commands in list:
    GC9106_SWRESET,   GC_CMD_DELAY, //  1: Software reset, no args, w/delay
      50,                           //     50 ms delay
    GC9106_INTERRE1,  0,            //  2: Internal Register Enable 1
    GC9106_INTERRE1,  0,            //  3: Internal Register Enable 1
    GC9106_INTERRE1,  0,            //  4: Internal Register Enable 1
    GC9106_INTERRE1,  0,            //  5: Internal Register Enable 1
    GC9106_INTERRE2,  0,            //  6: Internal Register Enable 2
    GC9106_FRMCTR3,   1,            //  7: 
      0x03,
    GC9106_MADCTL,    1,            //  8: Memory Access Control
      0xD8,                         //      MY, MV, ML, BGR
    GC9106_COLMOD,    1,            //  9: Pixel Format Set
      0x05,                         //      16 bits/pixel
    GC9106_DISSET5,   1,            // 10:
      0x11,
    0xAC,             1,            // 11:
      0x0B,
    GC9106_INVCTR,    1,            // 12: Display Inversion Control
      0x21,                         //      Fix high value, 1 dot inversion
    GC9106_FRMCTR1,   1,            // 13:
      0xC0,
    GC9106_VREG1CTL,  2,            // 14: Vreg 1 Control
      0x50, 0x43,                   //      6V, 1.5V
    GC9106_VREG2CTL,  2,            // 15: Vreg 2 Control
      0x56, 0x43,                   //      -3.375V, 1.5V
    GC9106_GAMMA1,    14,           // 16: Set Gamma 1
      0x1F, 0x41, 0x1B, 0x55, 0x36, 0x3D, 0x3E,
      0x00, 0x16, 0x08, 0x09, 0x15, 0x14, 0x0F,
    GC9106_GAMMA2,    14,           // 17: Set Gamma 2
      0x1F, 0x41, 0x1B, 0x55, 0x36, 0x3D, 0x3E,
      0x00, 0x16, 0x08, 0x09, 0x15, 0x14, 0x0F,
    GC9106_INTERRE1,  0,            // 18: Internal Register Enable 1
    0xFF,             0,            // 19: ???
    GC9106_TEON,      1,            // 20: Tearing Effect ON
      0x00,                         //      V-Blank only
    GC9106_SCANLSET,  1,            // 21: Scan Line Set
      0x00,                         //      Tearing effent on at line 0
    GC9106_SLPOUT ,   GC_CMD_DELAY, // 22: Out of sleep mode, no args, w/delay
      10,                           //     10 ms delay
    GC9106_NORON,     GC_CMD_DELAY, // 23: Normal display on, no args, w/delay
      10,                           //     10 ms delay
    GC9106_DISPON,    GC_CMD_DELAY, // 24: Main screen turn on, no args, delay
      255 };                        //     255 = max (500 ms) delay

// clang-format on

/**************************************************************************/
/*!
    @brief  Initialization code common to all ST7735B displays
*/
/**************************************************************************/
void Adafruit_GC9106::init(void) {
  begin();
  displayInit(initCommands);
  _colstart = 24;
  _rowstart = 0;
  setRotation(0);
}

/**************************************************************************/
/*!
  @brief  SPI displays set an address window rectangle for blitting pixels
  @param  x  Top left corner x coordinate
  @param  y  Top left corner x coordinate
  @param  w  Width of window
  @param  h  Height of window
*/
/**************************************************************************/
void Adafruit_GC9106::setAddrWindow(uint16_t x, uint16_t y, uint16_t w,
                                    uint16_t h) {
  x += _xstart;
  y += _ystart;
  uint32_t xa = ((uint32_t)x << 16) | (x + w - 1);
  uint32_t ya = ((uint32_t)y << 16) | (y + h - 1);

  writeCommand(GC9106_CASET); // Column addr set
  SPI_WRITE32(xa);

  writeCommand(GC9106_RASET); // Row addr set
  SPI_WRITE32(ya);

  writeCommand(GC9106_RAMWR); // write to RAM
}

/**************************************************************************/
/*!
    @brief  Set origin of (0,0) and orientation of TFT display
    @param  m  The index for rotation, from 0-3 inclusive
*/
/**************************************************************************/
void Adafruit_GC9106::setRotation(uint8_t m) {
  uint8_t madctl = 0;

  rotation = m & 3; // can't be higher than 3

  switch (rotation) {
  case 0: // Portrait flex at top
    madctl = GC9106_MADCTL_BGR;
    _height = GC9106_TFTHEIGHT;
    _width = GC9106_TFTWIDTH;
    _xstart = _colstart;
    _ystart = _rowstart;
    break;
  case 1: // Landscape flex at right
    madctl = GC9106_MADCTL_MY | GC9106_MADCTL_MV | GC9106_MADCTL_BGR;
    _width = GC9106_TFTHEIGHT;
    _height = GC9106_TFTWIDTH;
    _ystart = _colstart;
    _xstart = _rowstart;
    break;
  case 2: // Portrait flex at bottom
    madctl = GC9106_MADCTL_MX | GC9106_MADCTL_MY | GC9106_MADCTL_BGR;
    _height = GC9106_TFTHEIGHT;
    _width = GC9106_TFTWIDTH;
    _xstart = _colstart;
    _ystart = _rowstart;
    break;
  case 3: // Landscape flex at left
    madctl = GC9106_MADCTL_MX | GC9106_MADCTL_MV | GC9106_MADCTL_BGR;
    _width = GC9106_TFTHEIGHT;
    _height = GC9106_TFTWIDTH;
    _ystart = _colstart;
    _xstart = _rowstart;
    break;
  }

  sendCommand(GC9106_MADCTL, &madctl, 1);
}

/**************************************************************************/
/*!
    @brief  Set origin of (0,0) of display with offsets
    @param  col  The offset from 0 for the column address
    @param  row  The offset from 0 for the row address
*/
/**************************************************************************/
void Adafruit_GC9106::setColRowStart(int8_t col, int8_t row) {
  _colstart = col;
  _rowstart = row;
}

/**************************************************************************/
/*!
 @brief  Change whether display is on or off
 @param  enable True if you want the display ON, false OFF
 */
/**************************************************************************/
void Adafruit_GC9106::enableDisplay(boolean enable) {
  sendCommand(enable ? GC9106_DISPON : GC9106_DISPOFF);
}

/**************************************************************************/
/*!
 @brief  Change whether TE pin output is on or off
 @param  enable True if you want the TE pin ON, false OFF
 */
/**************************************************************************/
void Adafruit_GC9106::enableTearing(boolean enable) {
  sendCommand(enable ? GC9106_TEON : GC9106_TEOFF);
}

/**************************************************************************/
/*!
 @brief  Change whether sleep mode is on or off
 @param  enable True if you want sleep mode ON, false OFF
 */
/**************************************************************************/
void Adafruit_GC9106::enableSleep(boolean enable) {
  sendCommand(enable ? GC9106_SLPIN : GC9106_SLPOUT);
}
