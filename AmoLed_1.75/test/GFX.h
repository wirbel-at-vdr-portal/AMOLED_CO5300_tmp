/*
 * start rewrite from:
 * https://github.com/adafruit/Adafruit-GFX-Library.git
 */
#ifndef _ARDUINO_GFX_H_
#define _ARDUINO_GFX_H_

#include "G.h"
#include "DataBus.h"
#include "ESP32QSPI.h"
#include <Print.h>
#include "colors.h"


#include "_gfxfont.h"





// Many (but maybe not all) non-AVR board installs define macros
// for compatibility with existing PROGMEM-reading AVR code.
// Do our own checks and defines here for good measure...

#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif
#ifndef pgm_read_sbyte
#define pgm_read_sbyte(addr) (*(const signed char *)(addr))
#endif
#ifndef pgm_read_word
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#endif
#ifndef pgm_read_dword
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#endif


// Pointers are a peculiar case...typically 16-bit on AVR boards,
// 32 bits elsewhere.  Try to accommodate both...

#if !defined(__INT_MAX__) || (__INT_MAX__ > 0xFFFF)
#define pgm_read_pointer(addr) ((void *)pgm_read_dword(addr))
#else
#define pgm_read_pointer(addr) ((void *)pgm_read_word(addr))
#endif

#ifndef _swap_uint8_t
#define _swap_uint8_t(a, b) \
  {                         \
    uint8_t t = a;          \
    a = b;                  \
    b = t;                  \
  }
#endif

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) \
  {                         \
    int16_t t = a;          \
    a = b;                  \
    b = t;                  \
  }
#endif

#ifndef _diff
#define _diff(a, b) ((a > b) ? (a - b) : (b - a))
#endif

#ifndef _ordered_in_range
#define _ordered_in_range(v, a, b) ((a <= v) && (v <= b))
#endif

#ifndef _in_range
#define _in_range(v, a, b) ((a > b) ? _ordered_in_range(v, b, a) : _ordered_in_range(v, a, b))
#endif

__attribute__((always_inline)) inline static GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c)
{
  // expression in __AVR__ section may generate "dereferencing type-punned pointer will break strict-aliasing rules" warning
  // In fact, on other platforms (such as STM32) there is no need to do this pointer magic as program memory may be read in a usual way
  // So expression may be simplified
  return gfxFont->glyph + c;
}

__attribute__((always_inline)) inline static uint8_t *pgm_read_bitmap_ptr(const GFXfont *gfxFont)
{
  // expression in __AVR__ section generates "dereferencing type-punned pointer will break strict-aliasing rules" warning
  // In fact, on other platforms (such as STM32) there is no need to do this pointer magic as program memory may be read in a usual way
  // So expression may be simplified
  return gfxFont->bitmap;
}

/// A generic graphics superclass that can handle all sorts of drawing. At a minimum you can subclass and provide drawPixel(). At a maximum you can do a ton of overriding to optimize. Used for any/all Adafruit displays!
class Arduino_GFX : public Print, public Arduino_G
{
public:
  Arduino_GFX(int16_t w, int16_t h); // Constructor

  // This MUST be defined by the subclass:
  virtual bool begin(int32_t speed = -1) = 0;
  virtual void writePixelPreclipped(int16_t x, int16_t y, uint16_t color) = 0;

  // TRANSACTION API / CORE DRAW API
  // These MAY be overridden by the subclass to provide device-specific
  // optimized code.  Otherwise 'generic' versions are used.
  virtual void startWrite();
  virtual void writeFillRectPreclipped(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  virtual void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  virtual void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  virtual void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
  virtual void endWrite(void);

  // CONTROL API
  // These MAY be overridden by the subclass to provide device-specific
  // optimized code.  Otherwise 'generic' versions are used.
  virtual void setRotation(uint8_t r);
  virtual void invertDisplay(bool i);
  virtual void displayOn();
  virtual void displayOff();
  bool enableRoundMode();

  // BASIC DRAW API
  // These MAY be overridden by the subclass to provide device-specific
  // optimized code.  Otherwise 'generic' versions are used.
  // It's good to implement those, even if using transaction API
  void writePixel(int16_t x, int16_t y, uint16_t color);
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void fillScreen(uint16_t color);
  void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
  void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
  void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
  void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
  void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
  void drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
  void fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
  void drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);
  void drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
  void drawXBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);
  void drawGrayscaleBitmap(int16_t x, int16_t y, const uint8_t bitmap[], const uint8_t mask[], int16_t w, int16_t h);
  void drawGrayscaleBitmap(int16_t x, int16_t y, uint8_t *bitmap, uint8_t *mask, int16_t w, int16_t h);
  void draw16bitRGBBitmapWithMask(int16_t x, int16_t y, const uint16_t bitmap[], const uint8_t mask[], int16_t w, int16_t h);
  void draw24bitRGBBitmap(int16_t x, int16_t y, const uint8_t bitmap[], const uint8_t mask[], int16_t w, int16_t h);
  void draw24bitRGBBitmap(int16_t x, int16_t y, uint8_t *bitmap, uint8_t *mask, int16_t w, int16_t h);
  void getTextBounds(const char *string, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h);
  void getTextBounds(const __FlashStringHelper *s, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h);
  void getTextBounds(const String &str, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h);
  void setTextSize(uint8_t s);
  void setTextSize(uint8_t sx, uint8_t sy);
  void setTextSize(uint8_t sx, uint8_t sy, uint8_t pixel_margin);


  void setFont(const GFXfont *f = NULL);

  virtual void flush(bool force_flush = false);


  // adopt from LovyanGFX
  void drawEllipse(int16_t x, int16_t y, int16_t rx, int16_t ry, uint16_t color);
  void writeEllipseHelper(int32_t x, int32_t y, int32_t rx, int32_t ry, uint8_t cornername, uint16_t color);
  void fillEllipse(int16_t x, int16_t y, int16_t rx, int16_t ry, uint16_t color);
  void writeFillEllipseHelper(int32_t x, int32_t y, int32_t rx, int32_t ry, uint8_t cornername, int16_t delta, uint16_t color);
  void drawArc(int16_t x, int16_t y, int16_t r1, int16_t r2, float start, float end, uint16_t color);
  void fillArc(int16_t x, int16_t y, int16_t r1, int16_t r2, float start, float end, uint16_t color);
  void writeFillArcHelper(int16_t cx, int16_t cy, int16_t oradius, int16_t iradius, float start, float end, uint16_t color);

  virtual void writeSlashLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
  virtual void drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color, uint16_t bg);
  virtual void drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bg);
  virtual void drawGrayscaleBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h);
  virtual void drawGrayscaleBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h);
  virtual void drawIndexedBitmap(int16_t x, int16_t y, uint8_t *bitmap, uint16_t *color_index, int16_t w, int16_t h, int16_t x_skip = 0);
  virtual void drawIndexedBitmap(int16_t x, int16_t y, uint8_t *bitmap, uint16_t *color_index, uint8_t chroma_key, int16_t w, int16_t h, int16_t x_skip = 0);
  virtual void draw3bitRGBBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h);
  virtual void draw16bitRGBBitmapWithMask(int16_t x, int16_t y, uint16_t *bitmap, uint8_t *mask, int16_t w, int16_t h);
  virtual void draw16bitRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[], int16_t w, int16_t h);
  virtual void draw16bitRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap, int16_t w, int16_t h);
  virtual void draw16bitRGBBitmapWithTranColor(int16_t x, int16_t y, uint16_t *bitmap, uint16_t transparent_color, int16_t w, int16_t h);
  virtual void draw16bitBeRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap, int16_t w, int16_t h);
  virtual void draw24bitRGBBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h);
  virtual void draw24bitRGBBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h);
  virtual void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg);

  virtual void draw16bitBeRGBBitmapR1(int16_t x, int16_t y, uint16_t *bitmap, int16_t w, int16_t h);

  /**********************************************************************/
  /*!
    @brief  Set text cursor location
    @param  x    X coordinate in pixels
    @param  y    Y coordinate in pixels
  */
  /**********************************************************************/
  void setCursor(int16_t x, int16_t y)
  {
    cursor_x = x;
    cursor_y = y;
  }

  /**********************************************************************/
  /*!
    @brief  Set text bound for printing
    @param  x    X coordinate in pixels
    @param  y    Y coordinate in pixels
  */
  void setTextBound(int16_t x, int16_t y, int16_t w, int16_t h)
  {
    _min_text_x = x;
    _min_text_y = y;
    _max_text_x = x + w - 1;
    _max_text_y = y + h - 1;
  }

  /**********************************************************************/
  /*!
    @brief   Set text font color with transparant background
    @param   c   16-bit 5-6-5 Color to draw text with
    @note    For 'transparent' background, background and foreground
             are set to same color rather than using a separate flag.
  */
  /**********************************************************************/
  void setTextColor(uint16_t c) { textcolor = textbgcolor = c; }

  /**********************************************************************/
  /*!
    @brief   Set text font color with custom background color
    @param   c   16-bit 5-6-5 Color to draw text with
    @param   bg  16-bit 5-6-5 Color to draw background/fill with
  */
  /**********************************************************************/
  void setTextColor(uint16_t c, uint16_t bg)
  {
    textcolor = c;
    textbgcolor = bg;
  }

  /**********************************************************************/
  /*!
  @brief  Set whether text that is too long for the screen width should
          automatically wrap around to the next line (else clip right).
  @param  w  true for wrapping, false for clipping
  */
  /**********************************************************************/
  void setTextWrap(bool w) { wrap = w; }

  virtual size_t write(uint8_t);

  /************************************************************************/
  /*!
    @brief      Get width of the display, accounting for current rotation
    @returns    Width in pixels
  */
  /************************************************************************/
  int16_t width(void) const { return _width; };

  /************************************************************************/
  /*!
    @brief      Get height of the display, accounting for current rotation
    @returns    Height in pixels
  */
  /************************************************************************/
  int16_t height(void) const { return _height; }

  /************************************************************************/
  /*!
    @brief      Get rotation setting for display
    @returns    0 thru 3 corresponding to 4 cardinal rotations
  */
  /************************************************************************/
  uint8_t getRotation(void) const { return _rotation; }

  // get current cursor position (get rotation safe maximum values,
  // using: width() for x, height() for y)
  /************************************************************************/
  /*!
    @brief  Get text cursor X location
    @returns    X coordinate in pixels
  */
  /************************************************************************/
  int16_t getCursorX(void) const { return cursor_x; }

  /************************************************************************/
  /*!
    @brief      Get text cursor Y location
    @returns    Y coordinate in pixels
  */
  /************************************************************************/
  int16_t getCursorY(void) const { return cursor_y; };

  /*!
    @brief   Given 8-bit red, green and blue values, return a 'packed'
             16-bit color value in '565' RGB format (5 bits red, 6 bits
             green, 5 bits blue). This is just a mathematical operation,
             no hardware is touched.
    @param   red    8-bit red brightnesss (0 = off, 255 = max).
    @param   green  8-bit green brightnesss (0 = off, 255 = max).
    @param   blue   8-bit blue brightnesss (0 = off, 255 = max).
    @return  'Packed' 16-bit color value (565 format).
  */
  uint16_t color565(uint8_t red, uint8_t green, uint8_t blue)
  {
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
  }

protected:
  void charBounds(char c, int16_t *x, int16_t *y, int16_t *minx, int16_t *miny, int16_t *maxx, int16_t *maxy);
  int16_t
      _width,  // Display width as modified by current rotation
      _height, // Display height as modified by current rotation
      _max_x,  // x zero base bound (_width - 1)
      _max_y,  // y zero base bound (_height - 1)
      _min_text_x,
      _min_text_y,
      _max_text_x,
      _max_text_y,
      cursor_x, // x location to start print()ing text
      cursor_y; // y location to start print()ing text
  uint16_t
      textcolor,   // 16-bit background color for print()
      textbgcolor; // 16-bit text color for print()
  uint8_t
      textsize_x,        // Desired magnification in X-axis of text to print()
      textsize_y,        // Desired magnification in Y-axis of text to print()
      text_pixel_margin, // Margin for each text pixel
      _rotation;         // Display rotation (0 thru 3)
  bool
      wrap; // If set, 'wrap' text at right edge of display
  GFXfont *gfxFont; // Pointer to special font

  bool _isRoundMode = false;
  int16_t *_roundMinX;
  int16_t *_roundMaxX;
};

#endif // _ARDUINO_GFX_H_
