#ifndef BAROMETRIC_PRESSURE_H
#define BAROMETRIC_PRESSURE_H

#include <Wire.h>
#include <Adafruit_BMP085.h>

#define ULTRA_LOW_POWER 0

class BarometricPressure {
private:
  Adafruit_BMP085 bmp;

public:
  BarometricPressure(uint8_t mode = ULTRA_LOW_POWER) {
    (void)mode;
  }

  bool begin() {
    return bmp.begin();
  }

  float getPressure() {
    return bmp.readPressure(); // Pascals
  }

  float getAltitude(float baselinePressure = 101325.0f, bool seaLevel = false) {
    (void)seaLevel;
    return bmp.readAltitude(baselinePressure); // meters
  }
};

#endif