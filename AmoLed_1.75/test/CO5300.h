#pragma once

#include "GFX.h"
#include "TFT.h"


#include "CO5300_defines.h"






class Arduino_CO5300 : public Arduino_TFT
{
private:

protected:
  void tftInit() override;

public:
  Arduino_CO5300(
      DataBus *bus, int8_t rst = -1, uint8_t r = 0,
      int16_t w = CO5300_MAXWIDTH, int16_t h = CO5300_MAXHEIGHT,
      uint8_t col_offset1 = 0, uint8_t row_offset1 = 0, uint8_t col_offset2 = 0, uint8_t row_offset2 = 0);

  bool begin(int32_t speed = -1) override;

  void writeAddrWindow(int16_t x, int16_t y, uint16_t w, uint16_t h) override;
  void setRotation(uint8_t r) override;
  void invertDisplay(bool) override;
  void displayOn() override;
  void displayOff() override;

  void setBrightness(uint8_t brightness);
  void setContrast(uint8_t contrast);




};
