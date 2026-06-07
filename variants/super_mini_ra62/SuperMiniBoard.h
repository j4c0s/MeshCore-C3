#pragma once

#include <helpers/ESP32Board.h>
#include <Arduino.h>

#include <driver/rtc_io.h>
#include <driver/uart.h>

class SuperMiniBoard : public ESP32Board {
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

  #ifdef PIN_VBAT_READ
    pinMode(PIN_VBAT_READ, INPUT);
  #endif

  #ifdef P_LORA_TX_LED
    pinMode(P_LORA_TX_LED, OUTPUT);
    digitalWrite(P_LORA_TX_LED, LOW);
  #endif
  }

  uint16_t getBattMilliVolts() override {
  #ifdef PIN_VBAT_READ
    analogReadResolution(12);
    uint32_t raw = 0;
    for (int i = 0; i < 8; i++) {
      raw += analogRead(PIN_VBAT_READ);
    }
    raw = raw / 8;

    return ((6.52 * raw) / 4096.0) * 1000;
  #else
    return 0;
  #endif
  }

  const char* getManufacturerName() const override {
    return "SuperMini C3";
  }
};
