#pragma once

#include <helpers/ESP32Board.h>
#include <Arduino.h>

#define PIN_GPS_EN 1

class JacoSensorBoard : public ESP32Board {
public:
  void begin() {
    // GPS Power Control (NPN transistor on GND)
    pinMode(PIN_GPS_EN, OUTPUT);
    digitalWrite(PIN_GPS_EN, HIGH); // Enable GPS by default

    ESP32Board::begin();
  }

  uint16_t getBattMilliVolts() override {
    // Jaco didn't specify a battery read pin for his custom board,
    // but typically it might be on an analog pin.
    // If not specified, we return 0 or use a default if it's based on some other design.
    #ifdef PIN_VBAT_READ
      return ESP32Board::getBattMilliVolts();
    #else
      return 0;
    #endif
  }

  const char* getManufacturerName() const override {
    return "Jaco Custom ESP32-C3";
  }
};
