#include <Arduino.h>
#include <SPI.h>
#include "target.h"

JacoSensorBoard board;

// Use the standard SPI instance
RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, SPI);

WRAPPER_CLASS radio_driver(radio, board);

ESP32RTCClock fallback_clock;
AutoDiscoverRTCClock rtc_clock(fallback_clock);

#if ENV_INCLUDE_GPS
  #include <helpers/sensors/MicroNMEALocationProvider.h>
  MicroNMEALocationProvider nmea = MicroNMEALocationProvider(Serial1, &rtc_clock);
  EnvironmentSensorManager sensors = EnvironmentSensorManager(nmea);
#else
  EnvironmentSensorManager sensors;
#endif

bool radio_init() {
  fallback_clock.begin();

  // Initialize I2C with specified pins before RTC and sensors
  #if defined(PIN_BOARD_SDA) && defined(PIN_BOARD_SCL)
    Wire.begin(PIN_BOARD_SDA, PIN_BOARD_SCL);
  #endif

  rtc_clock.begin(Wire);

  // Initialize GPS serial port
  #if ENV_INCLUDE_GPS
    #ifdef GPS_BAUD_RATE
      Serial1.begin(GPS_BAUD_RATE, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    #else
      Serial1.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    #endif
  #endif

  // Manual hardware reset for SX1262
  if (P_LORA_RESET != RADIOLIB_NC) {
    pinMode(P_LORA_RESET, OUTPUT);
    digitalWrite(P_LORA_RESET, LOW);
    delay(10);
    digitalWrite(P_LORA_RESET, HIGH);
    delay(10);
  }

  // Initialize SPI with custom pins
  SPI.begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI);

  // Use std_init with SPI instance
  return radio.std_init(&SPI);
}

uint32_t radio_get_rng_seed() {
  return radio.random(0x7FFFFFFF);
}

void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr) {
  radio.setFrequency(freq);
  radio.setSpreadingFactor(sf);
  radio.setBandwidth(bw);
  radio.setCodingRate(cr);
}

void radio_set_tx_power(int8_t dbm) {
  radio.setOutputPower(dbm);
}

mesh::LocalIdentity radio_new_identity() {
  RadioNoiseListener rng(radio);
  return mesh::LocalIdentity(&rng);  // create new random identity
}
