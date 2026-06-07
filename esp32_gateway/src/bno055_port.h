#ifndef BNO055_ESP32_H_
#define BNO055_ESP32_H_

#include <Arduino.h>
#include <Wire.h>
#include "bno055.h"

inline void bno055_delay(int time) {
  delay(time);
}

inline void bno055_writeData(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(BNO055_I2C_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

inline void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len) {
  Wire.beginTransmission(BNO055_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // Repeated start
  
  Wire.requestFrom((uint8_t)BNO055_I2C_ADDR, len);
  for (uint8_t i = 0; i < len; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
    }
  }
}

#endif  // BNO055_ESP32_H_
