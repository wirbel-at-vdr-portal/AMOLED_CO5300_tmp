/*
 * start rewrite from:
 * https://github.com/adafruit/Adafruit-GFX-Library.git
 */
#ifndef _ARDUINO_TFT_H_
#define _ARDUINO_TFT_H_

#include "DataBus.h"
#include "GFX.h"

class ESP32QSPI {
public:
  #define ESP32QSPI_MAX_PIXELS_AT_ONCE    1024
  #define ESP32QSPI_FREQUENCY             40000000
  #define ESP32QSPI_SPI_MODE              SPI_MODE0
  #define ESP32QSPI_SPI_HOST              SPI2_HOST
  #define ESP32QSPI_DMA_CHANNEL           SPI_DMA_CH_AUTO

  #define MSB_16(val)                    (((val) & 0xFF00) >> 8) | (((val) & 0xFF) << 8)
  #define MSB_16_SET(var, val)           { (var) = MSB_16(val); }
  #define MSB_32_16_16_SET(var, v1, v2)  { (var) = (((uint32_t)v2 & 0xff00) << 8) | (((uint32_t)v2 & 0xff) << 24) | ((v1 & 0xff00) >> 8) | ((v1 & 0xff) << 8); }

  typedef enum {
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

private:
  int8_t _cs, _sck, _mosi, _miso, _quadwp, _quadhd;
  bool _is_shared_interface;
  int32_t _speed;
  int8_t _dataMode;
  volatile uint32_t * _csPortSet;
  volatile uint32_t * _csPortClr;
  uint32_t _csPinMask;
  spi_device_handle_t _handle;
  spi_transaction_ext_t _spi_tran_ext;
  spi_transaction_t *_spi_tran;

  union {
      uint8_t *_buffer;
      uint16_t *_buffer16;
      uint32_t *_buffer32;
      };
  union {
      uint8_t *_2nd_buffer;
      uint16_t *_2nd_buffer16;
      uint32_t *_2nd_buffer32;
      };
  union {
     uint16_t value;
     struct {
        uint8_t lsb;
        uint8_t msb;
        };
     } _data16;

  __attribute__((always_inline)) inline void CS_HIGH(void) {
     dbg_func;
     *_csPortSet = _csPinMask;
     }
  __attribute__((always_inline)) inline void CS_LOW(void) {
     dbg_func;
     *_csPortClr = _csPinMask;
     }
  __attribute__((always_inline)) inline void POLL_START() {
     dbg_func;
     spi_device_polling_start(_handle, _spi_tran, portMAX_DELAY);
     }
  __attribute__((always_inline)) inline void POLL_END() {
     dbg_func;
     spi_device_polling_end(_handle, portMAX_DELAY);
     }
public:
  ESP32QSPI(int8_t cs, int8_t sck,
            int8_t mosi, int8_t miso, int8_t quadwp, int8_t quadhd,
            bool is_shared_interface = false) :
      _cs(cs), _sck(sck), _mosi(mosi), _miso(miso), _quadwp(quadwp), _quadhd(quadhd),
      _is_shared_interface(is_shared_interface) {}

  bool begin(int32_t speed = 40000000, int8_t dataMode = -1) {
     dbg_func;
     dbg_arg(speed);
     dbg_arg(dataMode);
     _speed = (speed <= -1) ? ESP32QSPI_FREQUENCY : speed;
     _dataMode = (dataMode == -1) ? ESP32QSPI_SPI_MODE : dataMode;
     pinMode(_cs, OUTPUT);
     digitalWrite(_cs, HIGH);
     if (_cs >= 32) {
        _csPinMask = digitalPinToBitMask(_cs);
        _csPortSet = (volatile uint32_t *)GPIO_OUT1_W1TS_REG;
        _csPortClr = (volatile uint32_t *)GPIO_OUT1_W1TC_REG;
        }
     else if (_cs != -1) {
        _csPinMask = digitalPinToBitMask(_cs);
        _csPortSet = (volatile uint32_t *)GPIO_OUT_W1TS_REG;
        _csPortClr = (volatile uint32_t *)GPIO_OUT_W1TC_REG;
        }

     if (speed != -4) {
        spi_bus_config_t buscfg = {
            .mosi_io_num = _mosi,
            .miso_io_num = _miso,
            .sclk_io_num = _sck,
            .quadwp_io_num = _quadwp,
            .quadhd_io_num = _quadhd,
            .data4_io_num = -1,
            .data5_io_num = -1,
            .data6_io_num = -1,
            .data7_io_num = -1,
            .max_transfer_sz = (ESP32QSPI_MAX_PIXELS_AT_ONCE * 16) + 8,
            .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
            .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
            .intr_flags = 0};
        esp_err_t ret = spi_bus_initialize(ESP32QSPI_SPI_HOST, &buscfg, ESP32QSPI_DMA_CHANNEL);
        if (ret != ESP_OK) {
           ESP_ERROR_CHECK(ret);
           return false;
           }
        }

     spi_device_interface_config_t devcfg = {
         .command_bits = 8,
         .address_bits = 24,
         .dummy_bits = 0,
         .mode = (uint8_t)_dataMode,
         .clock_source = SPI_CLK_SRC_DEFAULT,
         .duty_cycle_pos = 0,
         .cs_ena_pretrans = 0,
         .cs_ena_posttrans = 0,
         .clock_speed_hz = _speed,
         .input_delay_ns = 0,
         .spics_io_num = -1, // avoid use system CS control
         .flags = SPI_DEVICE_HALFDUPLEX,
         .queue_size = 1,
         .pre_cb = nullptr,
         .post_cb = nullptr
         };
     esp_err_t ret = spi_bus_add_device(ESP32QSPI_SPI_HOST, &devcfg, &_handle);
     if (ret != ESP_OK) {
        ESP_ERROR_CHECK(ret);
        return false;
        }

     if (!_is_shared_interface)
        spi_device_acquire_bus(_handle, portMAX_DELAY);

     memset(&_spi_tran_ext, 0, sizeof(_spi_tran_ext));
     _spi_tran = (spi_transaction_t*) &_spi_tran_ext;

     _buffer = (uint8_t *)heap_caps_aligned_alloc(16, ESP32QSPI_MAX_PIXELS_AT_ONCE * 2, MALLOC_CAP_DMA);
     if (!_buffer)
        return false;

     _2nd_buffer = (uint8_t *)heap_caps_aligned_alloc(16, ESP32QSPI_MAX_PIXELS_AT_ONCE * 2, MALLOC_CAP_DMA);
     if (!_2nd_buffer)
        return false;

     return true;
     }
  void beginWrite() {
     dbg_func;
     if (_is_shared_interface)
        spi_device_acquire_bus(_handle, portMAX_DELAY);
     }
  void endWrite() {
     dbg_func;
     if (_is_shared_interface)
        spi_device_release_bus(_handle);
     }
  void writeCommand(uint8_t c) {
     dbg_func;
     dbg_arg(c);
     CS_LOW();
     _spi_tran_ext.base.flags = SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
     _spi_tran_ext.base.cmd = 0x02;
     _spi_tran_ext.base.addr = ((uint32_t)c) << 8;
     _spi_tran_ext.base.tx_buffer = NULL;
     _spi_tran_ext.base.length = 0;
     POLL_START();
     POLL_END();
     CS_HIGH();
     }
  void writeCommand16(uint16_t c) {
     CS_LOW();
     _spi_tran_ext.base.flags = SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
     _spi_tran_ext.base.cmd = 0x02;
     _spi_tran_ext.base.addr = c;
     _spi_tran_ext.base.tx_buffer = NULL;
     _spi_tran_ext.base.length = 0;
     POLL_START();
     POLL_END();
     CS_HIGH();
     }
  void writeCommandBytes(uint8_t *data, uint32_t len) {
     dbg_func;
     dbg_arg(len);
     CS_LOW();
     uint32_t l;
     while(len) {
        l = (len >= (ESP32QSPI_MAX_PIXELS_AT_ONCE << 1)) ? (ESP32QSPI_MAX_PIXELS_AT_ONCE << 1) : len;
        _spi_tran_ext.base.flags = SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
        _spi_tran_ext.base.tx_buffer = data;
        _spi_tran_ext.base.length = l << 3;
        POLL_START();
        POLL_END();
        len -= l;
        data += l;
        }
     CS_HIGH();
     }
  void write(uint8_t d) {
     dbg_func;
     dbg_arg(d);
     CS_LOW();
     _spi_tran_ext.base.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_MODE_QIO;
     _spi_tran_ext.base.cmd = 0x32;
     _spi_tran_ext.base.addr = 0x003C00;
     _spi_tran_ext.base.tx_data[0] = d;
     _spi_tran_ext.base.length = 8;
     POLL_START();
     POLL_END();
     CS_HIGH();
     }
  void write16(uint16_t d) {
     dbg_func;
     dbg_arg(d);
     CS_LOW();
     _spi_tran_ext.base.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_MODE_QIO;
     _spi_tran_ext.base.cmd = 0x32;
     _spi_tran_ext.base.addr = 0x003C00;
     _spi_tran_ext.base.tx_data[0] = d >> 8;
     _spi_tran_ext.base.tx_data[1] = d;
     _spi_tran_ext.base.length = 16;
     POLL_START();
     POLL_END();
     CS_HIGH();
     }
  void writeC8D8(uint8_t c, uint8_t d) {
     dbg_func;
     dbg_arg(c);
     dbg_arg(d);
     CS_LOW();
     _spi_tran_ext.base.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
     _spi_tran_ext.base.cmd = 0x02;
     _spi_tran_ext.base.addr = ((uint32_t)c) << 8;
     _spi_tran_ext.base.tx_data[0] = d;
     _spi_tran_ext.base.length = 8;
     POLL_START();
     POLL_END();
     CS_HIGH();
     }
  void writeC16D16(uint16_t c, uint16_t d) {
     dbg_func;
     dbg_arg(c);
     dbg_arg(d);
     writeCommand16(c);
     write16(d);
     }
  void writeC8D16(uint8_t c, uint16_t d) {
     dbg_func;
     dbg_arg(c);
     dbg_arg(d);
     CS_LOW();
     _spi_tran_ext.base.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
     _spi_tran_ext.base.cmd = 0x02;
     _spi_tran_ext.base.addr = ((uint32_t)c) << 8;
     _spi_tran_ext.base.tx_data[0] = d >> 8;
     _spi_tran_ext.base.tx_data[1] = d;
     _spi_tran_ext.base.length = 16;
     POLL_START();
     POLL_END();
     CS_HIGH();
     }
  void writeC8D16D16(uint8_t c, uint16_t d1, uint16_t d2) {
     dbg_func;
     dbg_arg(c);
     dbg_arg(d1);
     dbg_arg(d2);
     CS_LOW();
     _spi_tran_ext.base.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
     _spi_tran_ext.base.cmd = 0x02;
     _spi_tran_ext.base.addr = ((uint32_t)c) << 8;
     _spi_tran_ext.base.tx_data[0] = d1 >> 8;
     _spi_tran_ext.base.tx_data[1] = d1;
     _spi_tran_ext.base.tx_data[2] = d2 >> 8;
     _spi_tran_ext.base.tx_data[3] = d2;
     _spi_tran_ext.base.length = 32;
     POLL_START();
     POLL_END();
     CS_HIGH();
     }
  void writeC8D16D16Split(uint8_t c, uint16_t d1, uint16_t d2) {
     dbg_func;
     dbg_arg(c);
     dbg_arg(d1);
     dbg_arg(d2);
     CS_LOW();
     _spi_tran_ext.base.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
     _spi_tran_ext.base.cmd = 0x02;
     _spi_tran_ext.base.addr = ((uint32_t)c) << 8;
     _spi_tran_ext.base.tx_data[0] = d1 >> 8;
     _spi_tran_ext.base.tx_data[1] = d1;
     _spi_tran_ext.base.tx_data[2] = d2 >> 8;
     _spi_tran_ext.base.tx_data[3] = d2;
     _spi_tran_ext.base.length = 32;
     POLL_START();
     POLL_END();
     CS_HIGH();
     }
  void writeRepeat(uint16_t p, uint32_t len) {
     dbg_func;
     dbg_arg(p);
     dbg_arg(len);
     bool first_send = true;
     uint16_t bufLen = (len >= ESP32QSPI_MAX_PIXELS_AT_ONCE) ? ESP32QSPI_MAX_PIXELS_AT_ONCE : len;
     int16_t xferLen, l;
     uint32_t c32;
     MSB_32_16_16_SET(c32, p, p);
     l = (bufLen + 1) / 2;
     for (uint32_t i = 0; i < l; i++) {
         _buffer32[i] = c32;
         }
     CS_LOW();
     while(len) {
        xferLen = (bufLen <= len) ? bufLen : len; // How many this pass?
        if (first_send) {
           _spi_tran_ext.base.flags = SPI_TRANS_MODE_QIO;
           _spi_tran_ext.base.cmd = 0x32;
           _spi_tran_ext.base.addr = 0x003C00;
           first_send = false;
           }
        else {
           _spi_tran_ext.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD |
                                      SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
           }
        _spi_tran_ext.base.tx_buffer = _buffer16;
        _spi_tran_ext.base.length = xferLen << 4;
        POLL_START();
        POLL_END();
        len -= xferLen;
        }
     CS_HIGH();
     }
  void writePixels(uint16_t *data, uint32_t len) {
     dbg_func;
     dbg_arg(len);
     CS_LOW();
     uint32_t l, l2;
     uint16_t p1, p2;
     bool first_send = true;
     while(len) {
        l = (len > ESP32QSPI_MAX_PIXELS_AT_ONCE) ? ESP32QSPI_MAX_PIXELS_AT_ONCE : len;
        if (first_send) {
           _spi_tran_ext.base.flags = SPI_TRANS_MODE_QIO;
           _spi_tran_ext.base.cmd = 0x32;
           _spi_tran_ext.base.addr = 0x003C00;
           first_send = false;
           }
        else {
           _spi_tran_ext.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD |
                                      SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
           }
        l2 = l >> 1;
        for(uint32_t i = 0; i < l2; ++i) {
           p1 = *data++;
           p2 = *data++;
           MSB_32_16_16_SET(_buffer32[i], p1, p2);
           }
        if (l & 1) {
           p1 = *data++;
           MSB_16_SET(_buffer16[l - 1], p1);
           }
        _spi_tran_ext.base.tx_buffer = _buffer32;
        _spi_tran_ext.base.length = l << 4;
        POLL_START();
        POLL_END();
        len -= l;
        }
     CS_HIGH();
     }
  void writeBytes(uint8_t *data, uint32_t len) {
     dbg_func;
     dbg_arg(len);
     CS_LOW();
     uint32_t l;
     bool first_send = true;
     while(len) {
        l = (len >= (ESP32QSPI_MAX_PIXELS_AT_ONCE << 1)) ? (ESP32QSPI_MAX_PIXELS_AT_ONCE << 1) : len;
        if (first_send) {
           _spi_tran_ext.base.flags = SPI_TRANS_MODE_QIO;
           _spi_tran_ext.base.cmd = 0x32;
           _spi_tran_ext.base.addr = 0x003C00;
           first_send = false;
           }
        else {
           _spi_tran_ext.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD |
                                      SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
           }
        _spi_tran_ext.base.tx_buffer = data;
        _spi_tran_ext.base.length = l << 3;
        POLL_START();
        POLL_END();
        len -= l;
        data += l;
        }
     CS_HIGH();
     }
  void sendCommand(uint8_t c) {
     dbg_func;
     dbg_arg(c);
     beginWrite();
     writeCommand(c);
     endWrite();
     }
  void sendCommand16(uint16_t c) {
     dbg_func;
     dbg_arg(c);
     beginWrite();
     writeCommand16(c);
     endWrite();
     }
  void sendData(uint8_t d) {
     dbg_func;
     dbg_arg(d);
     beginWrite();
     write(d);
     endWrite();
     }
  void sendData16(uint16_t d) {
     dbg_func;
     dbg_arg(d);
     beginWrite();
     write16(d);
     endWrite();
     }
  void write16bitBeRGBBitmapR1(uint16_t *bitmap, int16_t w, int16_t h) {
     if (h > ESP32QSPI_MAX_PIXELS_AT_ONCE) {
        log_e("h > ESP32QSPI_MAX_PIXELS_AT_ONCE, h: %d", h);
        }
     else {
        CS_LOW();
        uint32_t l = h << 4;
        bool first_send = true;
        uint16_t *p;
        uint16_t *origin_offset = bitmap + ((h - 1) * w);
        for(int16_t i = 0; i < w; i++) {
           if (first_send) {
              _spi_tran_ext.base.flags = SPI_TRANS_MODE_QIO;
              _spi_tran_ext.base.cmd = 0x32;
              _spi_tran_ext.base.addr = 0x003C00;
              first_send = false;
              }
           else {
              POLL_END();
              _spi_tran_ext.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD |
                                         SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
              }
           p = origin_offset + i;
           for(int16_t j = 0; j < h; j++) {
              _buffer16[j] = *p;
              p -= w;
              }
           _spi_tran_ext.base.tx_buffer = _buffer16;
           _spi_tran_ext.base.length = l;
           POLL_START();
           }
        POLL_END();
        CS_HIGH();
        }
     }
  void batchOperation(const uint8_t *operations, size_t len) {
     dbg_func;
     dbg_arg(len);
     for (size_t i = 0; i < len; ++i) {
        uint8_t l = 0;
        switch(operations[i]) {
           case BEGIN_WRITE:
              beginWrite();
              break;
           case WRITE_COMMAND_8:
              writeCommand(operations[++i]);
              break;
           case WRITE_COMMAND_16:
              _data16.msb = operations[++i];
              _data16.lsb = operations[++i];
              writeCommand16(_data16.value);
              break;
           case WRITE_DATA_8:
              write(operations[++i]);
              break;
           case WRITE_DATA_16:
              _data16.msb = operations[++i];
              _data16.lsb = operations[++i];
              write16(_data16.value);
              break;
           case WRITE_BYTES:
              l = operations[++i];
              memcpy(_buffer, operations + i + 1, l);
              i += l;
              writeBytes(_buffer, l);
              break;
           case WRITE_C8_D8:
              l = operations[++i];
              writeC8D8(l, operations[++i]);
              break;
           case WRITE_C8_D16:
              l = operations[++i];
              _data16.msb = operations[++i];
              _data16.lsb = operations[++i];
              writeC8D16(l, _data16.value);
              break;
           case WRITE_C8_BYTES: {
              uint8_t c = operations[++i];
              l = operations[++i];
              memcpy(_buffer, operations + i + 1, l);
              i += l;
              writeC8Bytes(c, _buffer, l);
              }
              break;
           case WRITE_C16_D16:
              break;
           case END_WRITE:
              endWrite();
              break;
           case DELAY:
              delay(operations[++i]);
              break;
           default:
              printf("Unknown operation id at %d: %d\n", i, operations[i]);
              break;
           }
        }
     }
  void writePattern(uint8_t *data, uint8_t len, uint32_t repeat) {
     dbg_func;
     dbg_arg(len);
     dbg_arg(repeat);
     while(repeat--) {
        writeBytes(data, len);
        }
     }
  void writeIndexedPixels(uint8_t *data, uint16_t *idx, uint32_t len) {
     dbg_func;
     dbg_func;
     dbg_arg(*idx);
     dbg_arg(len);
     CS_LOW();
     uint32_t l, l2;
     uint16_t p1, p2;
     bool first_send = true;
     while(len) {
        l = (len > ESP32QSPI_MAX_PIXELS_AT_ONCE) ? ESP32QSPI_MAX_PIXELS_AT_ONCE : len;
        if (first_send) {
           _spi_tran_ext.base.flags = SPI_TRANS_MODE_QIO;
           _spi_tran_ext.base.cmd = 0x32;
           _spi_tran_ext.base.addr = 0x003C00;
           first_send = false;
           }
        else {
           _spi_tran_ext.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD |
                                      SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
           }
        l2 = l >> 1;
        for(uint32_t i = 0; i < l2; ++i) {
           p1 = idx[*data++];
           p2 = idx[*data++];
           MSB_32_16_16_SET(_buffer32[i], p1, p2);
           }
        if (l & 1) {
           p1 = idx[*data++];
           MSB_16_SET(_buffer16[l - 1], p1);
           }
        _spi_tran_ext.base.tx_buffer = _buffer32;
        _spi_tran_ext.base.length = l << 4;
        POLL_START();
        POLL_END();
        len -= l;
        }
     CS_HIGH();
     }
  void writeIndexedPixelsDouble(uint8_t *data, uint16_t *idx, uint32_t len) {
     dbg_func;
     dbg_arg(*idx);
     dbg_arg(len);
     CS_LOW();
     uint32_t l;
     uint16_t p;
     bool first_send = true;
     while(len) {
        l = (len > (ESP32QSPI_MAX_PIXELS_AT_ONCE >> 1)) ? (ESP32QSPI_MAX_PIXELS_AT_ONCE >> 1) : len;
        if (first_send) {
           _spi_tran_ext.base.flags = SPI_TRANS_MODE_QIO;
           _spi_tran_ext.base.cmd = 0x32;
           _spi_tran_ext.base.addr = 0x003C00;
           first_send = false;
           }
        else {
           _spi_tran_ext.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD |
                                      SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
           }
        for(uint32_t i = 0; i < l; ++i) {
           p = idx[*data++];
           MSB_32_16_16_SET(_buffer32[i], p, p);
           }
        _spi_tran_ext.base.tx_buffer = _buffer32;
        _spi_tran_ext.base.length = l << 5;
        POLL_START();
        POLL_END();
        len -= l;
        }
     CS_HIGH();
     }
  void writeYCbCrPixels(uint8_t *yData, uint8_t *cbData, uint8_t *crData, uint16_t w, uint16_t h) {
     static const int16_t CR2R16[] = {
        19, 20, 22, 23, 25, 27, 28, 30, 31, 33, 35, 36, 38, 39, 41, 43, 44, 46, 47, 49, 51, 52, 54, 55, 57, 59, 60, 62, 63, 65, 67, 68,
        70, 71, 73, 75, 76, 78, 79, 81, 83, 84, 86, 87, 89, 91, 92, 94, 95, 97, 99, 100, 102, 103, 105, 106, 108, 110, 111, 113, 114,
        116, 118, 119, 121, 122, 124, 126, 127, 129, 130, 132, 134, 135, 137, 138, 140, 142, 143, 145, 146, 148, 150, 151, 153, 154,
        156, 158, 159, 161, 162, 164, 166, 167, 169, 170, 172, 174, 175, 177, 178, 180, 182, 183, 185, 186, 188, 189, 191, 193, 194,
        196, 197, 199, 201, 202, 204, 205, 207, 209, 210, 212, 213, 215, 217, 218, 220, 221, 223, 225, 226, 228, 229, 231, 233, 234,
        236, 237, 239, 241, 242, 244, 245, 247, 249, 250, 252, 253, 255, 257, 258, 260, 261, 263, 264, 266, 268, 269, 271, 272, 274,
        276, 277, 279, 280, 282, 284, 285, 287, 288, 290, 292, 293, 295, 296, 298, 300, 301, 303, 304, 306, 308, 309, 311, 312, 314,
        316, 317, 319, 320, 322, 324, 325, 327, 328, 330, 332, 333, 335, 336, 338, 340, 341, 343, 344, 346, 347, 349, 351, 352, 354,
        355, 357, 359, 360, 362, 363, 365, 367, 368, 370, 371, 373, 375, 376, 378, 379, 381, 383, 384, 386, 387, 389, 391, 392, 394,
        395, 397, 399, 400, 402, 403, 405, 407, 408, 410, 411, 413, 415, 416, 418, 419, 421, 423, 424, 426
        };
     static const int16_t CB2G16[] = {
        -50, -50, -49, -49, -49, -48, -48, -47, -47, -47, -46, -46, -45, -45, -45, -44, -44, -43, -43, -43, -42, -42, -42, -41, -41,
        -40, -40, -40, -39, -39, -38, -38, -38, -37, -37, -36, -36, -36, -35, -35, -34, -34, -34, -33, -33, -33, -32, -32, -31, -31,
        -31, -30, -30, -29, -29, -29, -28, -28, -27, -27, -27, -26, -26, -25, -25, -25, -24, -24, -24, -23, -23, -22, -22, -22, -21,
        -21, -20, -20, -20, -19, -19, -18, -18, -18, -17, -17, -16, -16, -16, -15, -15, -14, -14, -14, -13, -13, -13, -12, -12, -11,
        -11, -11, -10, -10, -9, -9, -9, -8, -8, -7, -7, -7, -6, -6, -5, -5, -5, -4, -4, -4, -3, -3, -2, -2, -2, -1, -1, 0, 0, 0, 1,
          1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 15,
         15, 16, 16, 16, 17, 17, 18, 18, 18, 19, 19, 20, 20, 20, 21, 21, 22, 22, 22, 23, 23, 24, 24, 24, 25, 25, 25, 26, 26, 27, 27,
         27, 28, 28, 29, 29, 29, 30, 30, 31, 31, 31, 32, 32, 33, 33, 33, 34, 34, 34, 35, 35, 36, 36, 36, 37, 37, 38, 38, 38, 39, 39,
         40, 40, 40, 41, 41, 42, 42, 42, 43, 43, 43, 44, 44, 45, 45, 45, 46, 46, 47, 47, 47, 48, 48, 49, 49, 49, 50
         };
     static const int16_t CR2G16[] = {
        -276, -275, -274, -274, -273, -272, -271, -270, -270, -269, -268, -267, -266, -265, -265, -264, -263, -262, -261, -261, -260,
        -259, -258, -257, -257, -256, -255, -254, -253, -252, -252, -251, -250, -249, -248, -248, -247, -246, -245, -244, -244, -243,
        -242, -241, -240, -239, -239, -238, -237, -236, -235, -235, -234, -233, -232, -231, -231, -230, -229, -228, -227, -226, -226,
        -225, -224, -223, -222, -222, -221, -220, -219, -218, -218, -217, -216, -215, -214, -213, -213, -212, -211, -210, -209, -209,
        -208, -207, -206, -205, -205, -204, -203, -202, -201, -200, -200, -199, -198, -197, -196, -196, -195, -194, -193, -192, -192,
        -191, -190, -189, -188, -187, -187, -186, -185, -184, -183, -183, -182, -181, -180, -179, -179, -178, -177, -176, -175, -174,
        -174, -173, -172, -171, -170, -170, -169, -168, -167, -166, -165, -165, -164, -163, -162, -161, -161, -160, -159, -158, -157,
        -157, -156, -155, -154, -153, -152, -152, -151, -150, -149, -148, -148, -147, -146, -145, -144, -144, -143, -142, -141, -140,
        -139, -139, -138, -137, -136, -135, -135, -134, -133, -132, -131, -131, -130, -129, -128, -127, -126, -126, -125, -124, -123,
        -122, -122, -121, -120, -119, -118, -118, -117, -116, -115, -114, -113, -113, -112, -111, -110, -109, -109, -108, -107, -106,
        -105, -105, -104, -103, -102, -101, -100, -100, -99, -98, -97, -96, -96, -95, -94, -93, -92, -92, -91, -90, -89, -88, -87, -87,
         -86, -85, -84, -83, -83, -82, -81, -80, -79, -79, -78, -77, -76, -75, -74, -74, -73, -72, -71, -70, -70, -69
        };
     static const int16_t CB2B16[] = {
        19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71, 73, 75, 77, 79, 81,
        83, 85, 87, 89, 91, 93, 95, 97, 99, 102, 104, 106, 108, 110, 112, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 136,
        138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 180, 182, 184, 186,
        188, 190, 192, 194, 196, 198, 200, 202, 204, 206, 208, 210, 212, 214, 216, 219, 221, 223, 225, 227, 229, 231, 233, 235, 237,
        239, 241, 243, 245, 247, 249, 251, 253, 255, 257, 259, 261, 263, 265, 267, 269, 271, 273, 275, 277, 279, 281, 283, 285, 287,
        289, 291, 293, 295, 297, 299, 301, 303, 305, 307, 309, 311, 313, 315, 317, 319, 321, 323, 325, 327, 329, 331, 333, 335, 338,
        340, 342, 344, 346, 348, 350, 352, 354, 356, 358, 360, 362, 364, 366, 368, 370, 372, 374, 376, 378, 380, 382, 384, 386, 388,
        390, 392, 394, 396, 398, 400, 402, 404, 406, 408, 410, 412, 414, 416, 418, 420, 422, 424, 426, 428, 430, 432, 434, 436, 438,
        440, 442, 444, 446, 448, 450, 452, 455, 457, 459, 461, 463, 465, 467, 469, 471, 473, 475, 477, 479, 481, 483, 485, 487, 489,
        491, 493, 495, 497, 499, 501, 503, 505, 507, 509, 511, 513, 515, 517, 519, 521, 523, 525, 527, 529, 531, 533
        };
     static const int16_t Y2I16[] = {
        -19, -17, -16, -15, -14, -13, -12, -10, -9, -8, -7, -6, -5, -3, -2, -1, 0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 12, 13, 14, 15, 16, 17,
        19, 20, 21, 22, 23, 24, 26, 27, 28, 29, 30, 31, 33, 34, 35, 36, 37, 38, 40, 41, 42, 43, 44, 45, 47, 48, 49, 50, 51, 52, 54, 55,
        56, 57, 58, 59, 61, 62, 63, 64, 65, 66, 68, 69, 70, 71, 72, 73, 75, 76, 77, 78, 79, 80, 82, 83, 84, 85, 86, 87, 88, 90, 91, 92,
        93, 94, 95, 97, 98, 99, 100, 101, 102, 104, 105, 106, 107, 108, 109, 111, 112, 113, 114, 115, 116, 118, 119, 120, 121, 122, 123,
        125, 126, 127, 128, 129, 130, 132, 133, 134, 135, 136, 137, 139, 140, 141, 142, 143, 144, 146, 147, 148, 149, 150, 151, 153,
        154, 155, 156, 157, 158, 160, 161, 162, 163, 164, 165, 167, 168, 169, 170, 171, 172, 173, 175, 176, 177, 178, 179, 180, 182,
        183, 184, 185, 186, 187, 189, 190, 191, 192, 193, 194, 196, 197, 198, 199, 200, 201, 203, 204, 205, 206, 207, 208, 210, 211,
        212, 213, 214, 215, 217, 218, 219, 220, 221, 222, 224, 225, 226, 227, 228, 229, 231, 232, 233, 234, 235, 236, 238, 239, 240,
        241, 242, 243, 245, 246, 247, 248, 249, 250, 252, 253, 254, 255, 256, 257, 258, 260, 261, 262, 263, 264, 265, 267, 268, 269,
        270, 271, 272, 274, 275, 276, 277, 278
        };
     static const uint16_t CLIPRBE[] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 8, 8, 8, 8, 8, 8, 8, 16, 16, 16, 16, 16, 16, 16, 16, 24, 24,
        24, 24, 24, 24, 24, 24, 32, 32, 32, 32, 32, 32, 32, 32, 40, 40, 40, 40, 40, 40, 40, 40, 48, 48, 48, 48, 48, 48, 48, 48, 56,
        56, 56, 56, 56, 56, 56, 56, 64, 64, 64, 64, 64, 64, 64, 64, 72, 72, 72, 72, 72, 72, 72, 72, 80, 80, 80, 80, 80, 80, 80, 80,
        88, 88, 88, 88, 88, 88, 88, 88, 96, 96, 96, 96, 96, 96, 96, 96, 104, 104, 104, 104, 104, 104, 104, 104, 112, 112, 112, 112,
        112, 112, 112, 112, 120, 120, 120, 120, 120, 120, 120, 120, 128, 128, 128, 128, 128, 128, 128, 128, 136, 136, 136, 136, 136,
        136, 136, 136, 144, 144, 144, 144, 144, 144, 144, 144, 152, 152, 152, 152, 152, 152, 152, 152, 160, 160, 160, 160, 160, 160,
        160, 160, 168, 168, 168, 168, 168, 168, 168, 168, 176, 176, 176, 176, 176, 176, 176, 176, 184, 184, 184, 184, 184, 184, 184,
        184, 192, 192, 192, 192, 192, 192, 192, 192, 200, 200, 200, 200, 200, 200, 200, 200, 208, 208, 208, 208, 208, 208, 208, 208,
        216, 216, 216, 216, 216, 216, 216, 216, 224, 224, 224, 224, 224, 224, 224, 224, 232, 232, 232, 232, 232, 232, 232, 232, 240,
        240, 240, 240, 240, 240, 240, 240, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248,
        248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248,
        248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248,
        248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248,
        248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248,
        248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248,
        248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248,
        248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248,
        248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248,
        248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248
        };
     static const uint16_t CLIPGBE[] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 8192, 8192, 8192, 8192, 16384, 16384, 16384, 16384, 24576, 24576, 24576, 24576, 32768, 32768, 32768,
        32768, 40960, 40960, 40960, 40960, 49152, 49152, 49152, 49152, 57344, 57344, 57344, 57344, 1, 1, 1, 1, 8193, 8193, 8193, 8193,
        16385, 16385, 16385, 16385, 24577, 24577, 24577, 24577, 32769, 32769, 32769, 32769, 40961, 40961, 40961, 40961, 49153, 49153,
        49153, 49153, 57345, 57345, 57345, 57345, 2, 2, 2, 2, 8194, 8194, 8194, 8194, 16386, 16386, 16386, 16386, 24578, 24578, 24578,
        24578, 32770, 32770, 32770, 32770, 40962, 40962, 40962, 40962, 49154, 49154, 49154, 49154, 57346, 57346, 57346, 57346, 3, 3,
        3, 3, 8195, 8195, 8195, 8195, 16387, 16387, 16387, 16387, 24579, 24579, 24579, 24579, 32771, 32771, 32771, 32771, 40963, 40963,
        40963, 40963, 49155, 49155, 49155, 49155, 57347, 57347, 57347, 57347, 4, 4, 4, 4, 8196, 8196, 8196, 8196, 16388, 16388, 16388,
        16388, 24580, 24580, 24580, 24580, 32772, 32772, 32772, 32772, 40964, 40964, 40964, 40964, 49156, 49156, 49156, 49156, 57348,
        57348, 57348, 57348, 5, 5, 5, 5, 8197, 8197, 8197, 8197, 16389, 16389, 16389, 16389, 24581, 24581, 24581, 24581, 32773, 32773,
        32773, 32773, 40965, 40965, 40965, 40965, 49157, 49157, 49157, 49157, 57349, 57349, 57349, 57349, 6, 6, 6, 6, 8198, 8198, 8198,
        8198, 16390, 16390, 16390, 16390, 24582, 24582, 24582, 24582, 32774, 32774, 32774, 32774, 40966, 40966, 40966, 40966, 49158,
        49158, 49158, 49158, 57350, 57350, 57350, 57350, 7, 7, 7, 7, 8199, 8199, 8199, 8199, 16391, 16391, 16391, 16391, 24583, 24583,
        24583, 24583, 32775, 32775, 32775, 32775, 40967, 40967, 40967, 40967, 49159, 49159, 49159, 49159, 57351, 57351, 57351, 57351,
        57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351,
        57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351,
        57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351,
        57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351,
        57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351,
        57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351,
        57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351,
        57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351,
        57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351,
        57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351, 57351
        };
     static const uint16_t CLIPBBE[] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 256, 256, 256, 256, 256,
        256, 256, 256, 512, 512, 512, 512, 512, 512, 512, 512, 768, 768, 768, 768, 768, 768, 768, 768, 1024, 1024, 1024, 1024, 1024,
        1024, 1024, 1024, 1280, 1280, 1280, 1280, 1280, 1280, 1280, 1280, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1792, 1792,
        1792, 1792, 1792, 1792, 1792, 1792, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2304, 2304, 2304, 2304, 2304, 2304, 2304,
        2304, 2560, 2560, 2560, 2560, 2560, 2560, 2560, 2560, 2816, 2816, 2816, 2816, 2816, 2816, 2816, 2816, 3072, 3072, 3072, 3072,
        3072, 3072, 3072, 3072, 3328, 3328, 3328, 3328, 3328, 3328, 3328, 3328, 3584, 3584, 3584, 3584, 3584, 3584, 3584, 3584, 3840,
        3840, 3840, 3840, 3840, 3840, 3840, 3840, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4352, 4352, 4352, 4352, 4352, 4352,
        4352, 4352, 4608, 4608, 4608, 4608, 4608, 4608, 4608, 4608, 4864, 4864, 4864, 4864, 4864, 4864, 4864, 4864, 5120, 5120, 5120,
        5120, 5120, 5120, 5120, 5120, 5376, 5376, 5376, 5376, 5376, 5376, 5376, 5376, 5632, 5632, 5632, 5632, 5632, 5632, 5632, 5632,
        5888, 5888, 5888, 5888, 5888, 5888, 5888, 5888, 6144, 6144, 6144, 6144, 6144, 6144, 6144, 6144, 6400, 6400, 6400, 6400, 6400,
        6400, 6400, 6400, 6656, 6656, 6656, 6656, 6656, 6656, 6656, 6656, 6912, 6912, 6912, 6912, 6912, 6912, 6912, 6912, 7168, 7168,
        7168, 7168, 7168, 7168, 7168, 7168, 7424, 7424, 7424, 7424, 7424, 7424, 7424, 7424, 7680, 7680, 7680, 7680, 7680, 7680, 7680,
        7680, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936,
        7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936,
        7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936,
        7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936,
        7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936,
        7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936,
        7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936,
        7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936,
        7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936,
        7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936,
        7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936,
        7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936,
        7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936,
        7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936, 7936
        };


     if (w > (ESP32QSPI_MAX_PIXELS_AT_ONCE / 2)) {
        //DataBus::writeYCbCrPixels(yData, cbData, crData, w, h);
        int cols = w >> 1;
        uint8_t pxCb, pxCr;
        int16_t pxR, pxG, pxB, pxY;
        for(int row = 0; row < h;) {
           for(int col = 0; col < cols; ++col) {
              pxCb = *cbData++;
              pxCr = *crData++;
              pxR = CR2R16[pxCr];
              pxG = -CB2G16[pxCb] - CR2G16[pxCr];
              pxB = CB2B16[pxCb];
              pxY = Y2I16[*yData++];
              _data16.value = CLIPRBE[pxY + pxR] | CLIPGBE[pxY + pxG] | CLIPBBE[pxY + pxB];
              write(_data16.lsb);
              write(_data16.msb);
              pxY = Y2I16[*yData++];
              _data16.value = CLIPRBE[pxY + pxR] | CLIPGBE[pxY + pxG] | CLIPBBE[pxY + pxB];
              write(_data16.lsb);
              write(_data16.msb);
              }
           if (++row & 1) {
              // rollback CbCr data
              cbData -= cols;
              crData -= cols;
              }
           }
        }
     else {
        bool first_send = true;
        int cols = w >> 1;
        int rows = h >> 1;
        uint8_t *yData2 = yData + w;
        uint16_t *dest = _buffer16;
        uint16_t *dest2 = dest + w;
        uint16_t out_bits = w << 5;
        uint8_t pxCb, pxCr;
        int16_t pxR, pxG, pxB, pxY;
        CS_LOW();
        for(int row = 0; row < rows; ++row) {
           for(int col = 0; col < cols; ++col) {
              pxCb = *cbData++;
              pxCr = *crData++;
              pxR = CR2R16[pxCr];
              pxG = -CB2G16[pxCb] - CR2G16[pxCr];
              pxB = CB2B16[pxCb];
              pxY = Y2I16[*yData++];
              *dest++ = CLIPRBE[pxY + pxR] | CLIPGBE[pxY + pxG] | CLIPBBE[pxY + pxB];
              pxY = Y2I16[*yData++];
              *dest++ = CLIPRBE[pxY + pxR] | CLIPGBE[pxY + pxG] | CLIPBBE[pxY + pxB];
              pxY = Y2I16[*yData2++];
              *dest2++ = CLIPRBE[pxY + pxR] | CLIPGBE[pxY + pxG] | CLIPBBE[pxY + pxB];
              pxY = Y2I16[*yData2++];
              *dest2++ = CLIPRBE[pxY + pxR] | CLIPGBE[pxY + pxG] | CLIPBBE[pxY + pxB];
              }
           yData += w;
           yData2 += w;
           if (first_send) {
              _spi_tran_ext.base.flags = SPI_TRANS_MODE_QIO;
              _spi_tran_ext.base.cmd = 0x32;
              _spi_tran_ext.base.addr = 0x003C00;
              first_send = false;
              }
           else {
              POLL_END();
              _spi_tran_ext.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD |
                                         SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
              }
           if (row & 1) {
              _spi_tran_ext.base.tx_buffer = _2nd_buffer32;
              dest = _buffer16;
              }
           else {
              _spi_tran_ext.base.tx_buffer = _buffer32;
              dest = _2nd_buffer16;
              }
           _spi_tran_ext.base.length = out_bits;
           POLL_START();
           dest2 = dest + w;
           }
        POLL_END();
        CS_HIGH();
        }
     }
  void writeC8Bytes(uint8_t c, uint8_t *data, uint32_t len) {
     dbg_func;
     dbg_arg(c);
     dbg_arg(len);
     CS_LOW();
     _spi_tran_ext.base.flags = SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
     _spi_tran_ext.base.cmd = 0x02;
     _spi_tran_ext.base.addr = ((uint32_t)c) << 8;
     _spi_tran_ext.base.tx_buffer = data;
     _spi_tran_ext.base.length = len << 3;
     POLL_START();
     POLL_END();
     CS_HIGH();
     }
};

class Arduino_TFT : public Arduino_GFX
{
public:
  Arduino_TFT(void *bus, int8_t rst, uint8_t r, bool ips, int16_t w, int16_t h, uint8_t col_offset1, uint8_t row_offset1, uint8_t col_offset2, uint8_t row_offset2);

  // This SHOULD be defined by the subclass:
  void setRotation(uint8_t r) override;

  // This MUST be defined by the subclass:
  // and also protected function: tftInit()
  virtual void writeAddrWindow(int16_t x, int16_t y, uint16_t w, uint16_t h) = 0;

  bool begin(int32_t speed = -1);
  void startWrite(void) override;
  void endWrite(void) override;
  void writePixelPreclipped(int16_t x, int16_t y, uint16_t color) override;
  void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) override;
  void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) override;
  void writeFillRectPreclipped(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override;

  virtual void writeRepeat(uint16_t color, uint32_t len);

  void setAddrWindow(int16_t x, int16_t y, uint16_t w, uint16_t h);
  virtual void writeColor(uint16_t color);

  virtual void writePixels(uint16_t *data, uint32_t size);
  virtual void writeIndexedPixels(uint8_t *bitmap, uint16_t *color_index, uint32_t len);
  virtual void writeIndexedPixelsDouble(uint8_t *bitmap, uint16_t *color_index, uint32_t len);

  virtual void drawYCbCrBitmap(int16_t x, int16_t y, uint8_t *yData, uint8_t *cbData, uint8_t *crData, int16_t w, int16_t h);

  void writeBytes(uint8_t *data, uint32_t size);
  void pushColor(uint16_t color);

  void writeSlashLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) override;
  void drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color, uint16_t bg) override;
  void drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bg) override;
  void drawGrayscaleBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h) override;
  void drawGrayscaleBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h) override;
  void drawIndexedBitmap(int16_t x, int16_t y, uint8_t *bitmap, uint16_t *color_index, int16_t w, int16_t h, int16_t x_skip = 0) override;
  void draw16bitRGBBitmapWithMask(int16_t x, int16_t y, uint16_t *bitmap, uint8_t *mask, int16_t w, int16_t h) override;
  void draw16bitRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[], int16_t w, int16_t h) override;
  void draw16bitRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap, int16_t w, int16_t h) override;
  void draw16bitBeRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap, int16_t w, int16_t h) override;
  void draw16bitBeRGBBitmapR1(int16_t x, int16_t y, uint16_t *bitmap, int16_t w, int16_t h) override;
  void draw24bitRGBBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h) override;
  void draw24bitRGBBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h) override;
  void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg) override;

protected:
  virtual void tftInit() = 0;

  ESP32QSPI *_bus;
  int8_t _rst;
  bool _ips;
  uint8_t COL_OFFSET1, ROW_OFFSET1;
  uint8_t COL_OFFSET2, ROW_OFFSET2;
  uint8_t _xStart, _yStart;
  int16_t _currentX, _currentY;
  uint16_t _currentW, _currentH;
  int8_t _override_datamode = -1;

private:
};

#endif
