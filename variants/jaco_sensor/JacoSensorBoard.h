#pragma once

#include <helpers/ESP32Board.h>
#include <Arduino.h>

#include <driver/rtc_io.h>
#include <driver/uart.h>

#define PIN_GPS_EN 1

class JacoSensorBoard : public ESP32Board {
public:
  void begin() {
    // GPS Power Control (NPN transistor on GND)
    pinMode(PIN_GPS_EN, OUTPUT);
    digitalWrite(PIN_GPS_EN, LOW); // GPS off by default

    ESP32Board::begin();

    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_DEEPSLEEP) {
      uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
      if (wakeup_pin_mask & (1ULL << P_LORA_DIO_1)) {
        startup_reason = BD_STARTUP_RX_PACKET;
      }
    }
  }

  // MeshCore uses sleep() for its internal powersaving (Light Sleep)
  void sleep(uint32_t secs) override {
    // Only enter sleep if no GPS cycle is active
    extern bool is_gps_cycle_active;
    if (!is_gps_cycle_active) {
       ESP32Board::sleep(secs);
    }
  }

  void enterDeepSleep(uint32_t secs, int8_t wake_pin = -1) {
    digitalWrite(PIN_GPS_EN, LOW);

    if (P_LORA_DIO_1 >= 0) {
      gpio_set_direction((gpio_num_t)P_LORA_DIO_1, GPIO_MODE_INPUT);
      esp_deep_sleep_enable_gpio_wakeup(1ULL << P_LORA_DIO_1, ESP_GPIO_WAKEUP_GPIO_HIGH);
    }

    if (wake_pin >= 0) {
      gpio_set_direction((gpio_num_t)wake_pin, GPIO_MODE_INPUT);
      esp_deep_sleep_enable_gpio_wakeup(1ULL << wake_pin, ESP_GPIO_WAKEUP_GPIO_HIGH);
    }

    if (secs > 0) {
      esp_sleep_enable_timer_wakeup((uint64_t)secs * 1000000ULL);
    }

    esp_deep_sleep_start();
  }

  uint16_t getBattMilliVolts() override {
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
