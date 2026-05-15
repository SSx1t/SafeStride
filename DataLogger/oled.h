#ifndef OLED_H
#define OLED_H

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class oLed : public Adafruit_SSD1306 {
public:
  oLed(int width, int height)
    : Adafruit_SSD1306(width, height, &Wire, -1) {}

  bool begin() {
    return Adafruit_SSD1306::begin(SSD1306_SWITCHCAPVCC, 0x3C);
  }
};

#endif