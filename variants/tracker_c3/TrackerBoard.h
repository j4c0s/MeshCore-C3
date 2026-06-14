#pragma once

#include <helpers/ESP32Board.h>
#include <Arduino.h>

class TrackerBoard : public ESP32Board {
public:
  void begin() {
    ESP32Board::begin();

    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_DEEPSLEEP) {
      long wakeup_source = esp_sleep_get_gpio_wakeup_status();
      if (wakeup_source & (1 << P_LORA_DIO_1)) {
        startup_reason = BD_STARTUP_RX_PACKET;
      }
    }
  }

  const char* getManufacturerName() const override {
    return "Tracker C3";
  }
};
