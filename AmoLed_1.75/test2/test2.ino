#include <Wire.h>
#include "myDisplay.h"


#define SDA 15
#define SCL 14
#define LCD_SDIO0 4
#define LCD_SDIO1 5
#define LCD_SDIO2 6
#define LCD_SDIO3 7
#define LCD_SCLK 38
#define LCD_CS 12
#define LCD_RESET 39
#define LCD_WIDTH 466
#define LCD_HEIGHT 466







AMOLED_CO5300 *tft = new AMOLED_CO5300(466,466);



void setup(void) {
  Serial.begin(115200);
  delay(1000);
  //tft = new AMOLED_CO5300(466,466);

  Wire.begin(SDA, SCL);

  Serial.println("Arduino_GFX AsciiTable example");

  int numCols = LCD_WIDTH / 8;
  int numRows = LCD_HEIGHT / 10;





  // Init Display
  if (!tft->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  tft->fillScreen(MAGENTA);
  // ---------------- working fine until here. --------------------

tft->Debug(true);
  Serial.println("before tft->setBrightness(255);");


  tft->setBrightness(255); delay(200);

  Serial.println("white line, 0 deg");
  for(int i=0; i<100; i++) tft->drawPixel(100+i, 100, WHITE);

  Serial.println("red line, 90 deg");
  tft->setRotation(1);
  for(int i=0; i<100; i++) tft->drawPixel(100+i, 100, RED);

  Serial.println("blue line, 180 deg");
  tft->setRotation(2);
  for(int i=0; i<100; i++) tft->drawPixel(100+i, 100, BLUE);

  Serial.println("yellow line, 270 deg");
  tft->setRotation(2);
  for(int i=0; i<100; i++) tft->drawPixel(100+i, 100, YELLOW);

tft->Debug(false);


  tft->setTextColor(GREEN);
  for (int x = 0; x < numRows; x++) {
    tft->setCursor(10 + x * 8, 2);
    tft->print(x, 16);
  }
  tft->setTextColor(BLUE);
  for (int y = 0; y < numCols; y++) {
    tft->setCursor(2, 12 + y * 10);
    tft->print(y, 16);
  }

  char c = 0;
  for (int y = 0; y < numRows; y++) {
    for (int x = 0; x < numCols; x++) {
      tft->drawChar(10 + x * 8, 12 + y * 10, c++, WHITE, BLACK, 1);
    }
  }


}
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF

void loop() {
  // put your main code here, to run repeatedly:

  delay(1000);
}



void testdrawtext(char *text, uint16_t color) {
  tft->setCursor(100, 100);
  tft->setTextColor(color);
  tft->setTextWrap(true);
  tft->print(text);

  tft->drawLine(0, 0, 460, 460, YELLOW);
}
