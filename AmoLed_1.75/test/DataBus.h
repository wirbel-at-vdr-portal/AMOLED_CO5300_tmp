#pragma once
#include <Arduino.h>
#include "debug.h"


#define MSB_16(val) (((val) & 0xFF00) >> 8) | (((val) & 0xFF) << 8)
#define MSB_16_SET(var, val) \
  {                          \
    (var) = MSB_16(val);     \
  }
#define MSB_32_16_16_SET(var, v1, v2)                                                                                   \
  {                                                                                                                     \
    (var) = (((uint32_t)v2 & 0xff00) << 8) | (((uint32_t)v2 & 0xff) << 24) | ((v1 & 0xff00) >> 8) | ((v1 & 0xff) << 8); \
  }




typedef enum
{
  BEGIN_WRITE,
  WRITE_COMMAND_8,
  WRITE_COMMAND_16,
  WRITE_COMMAND_BYTES,
  WRITE_DATA_8,
  WRITE_DATA_16,
  WRITE_BYTES,
  WRITE_C8_D8,
  WRITE_C8_D16,
  WRITE_C8_BYTES,
  WRITE_C16_D16,
  END_WRITE,
  DELAY,
} spi_operation_type_t;

union
{
  uint16_t value;
  struct
  {
    uint8_t lsb;
    uint8_t msb;
  };
} _data16;

class DataBus
{
public:
  DataBus();

  void unused() { (void) _data16; } // avoid compiler warning

  virtual bool begin(int32_t speed = 40000000, int8_t dataMode = -1) = 0;
  virtual void beginWrite() = 0;
  virtual void endWrite() = 0;
  virtual void writeCommand(uint8_t c) = 0;
  virtual void writeCommand16(uint16_t c) = 0;
  virtual void writeCommandBytes(uint8_t *data, uint32_t len) = 0;
  virtual void write(uint8_t) = 0;
  virtual void write16(uint16_t) = 0;
  virtual void writeC8D8(uint8_t c, uint8_t d);
  virtual void writeC16D16(uint16_t c, uint16_t d);
  virtual void writeC8D16(uint8_t c, uint16_t d);
  virtual void writeC8D16D16(uint8_t c, uint16_t d1, uint16_t d2);
  virtual void writeC8D16D16Split(uint8_t c, uint16_t d1, uint16_t d2);
  virtual void writeRepeat(uint16_t p, uint32_t len) = 0;
  virtual void writeBytes(uint8_t *data, uint32_t len) = 0;
  virtual void writePixels(uint16_t *data, uint32_t len) = 0;

  void sendCommand(uint8_t c);
  void sendCommand16(uint16_t c);
  void sendData(uint8_t d);
  void sendData16(uint16_t d);


  virtual void write16bitBeRGBBitmapR1(uint16_t *bitmap, int16_t w, int16_t h);
  virtual void batchOperation(const uint8_t *operations, size_t len);
  virtual void writePattern(uint8_t *data, uint8_t len, uint32_t repeat);
  virtual void writeIndexedPixels(uint8_t *data, uint16_t *idx, uint32_t len);
  virtual void writeIndexedPixelsDouble(uint8_t *data, uint16_t *idx, uint32_t len);
  virtual void writeYCbCrPixels(uint8_t *yData, uint8_t *cbData, uint8_t *crData, uint16_t w, uint16_t h);


protected:
  int32_t _speed;
  int8_t _dataMode;
};
