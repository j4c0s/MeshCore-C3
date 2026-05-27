#include "JacoMesh.h"
#include <SPIFFS.h>
#include "target.h"
#include <RTClib.h>

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

// Power State
bool is_gps_cycle_active = false;
uint32_t gps_start_ms = 0;
uint32_t active_mode_end_ms = 0;
uint32_t last_report_hour = 0xFFFFFFFF;
uint32_t last_report_pvt_hour = 0xFFFFFFFF;

// GPS Persistence
RTC_DATA_ATTR double last_known_lat = 0.0;
RTC_DATA_ATTR double last_known_lon = 0.0;
RTC_DATA_ATTR float last_known_alt = 0.0;
RTC_DATA_ATTR bool has_any_gps_fix_ever = false;

// Global logger
void log_ts(const char* format, ...) {
    char time_buf[16];
    uint32_t now = rtc_clock.getCurrentTime();
    DateTime dt(now);
    snprintf(time_buf, sizeof(time_buf), "[%02d:%02d:%02d] ", dt.hour(), dt.minute(), dt.second());
    Serial.print(time_buf);

    char log_buf[192];
    va_list args;
    va_start(args, format);
    vsnprintf(log_buf, sizeof(log_buf), format, args);
    va_end(args);
    Serial.println(log_buf);
}

StdRNG fast_rng;
SimpleMeshTables tables;
JacoMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void setup() {
  Serial.begin(115200);
  delay(1000);

  log_ts("--- Jaco Sensor Starting (Minimal Main) ---");

  board.begin();
  if (!radio_init()) {
    log_ts("Radio init failed!");
    while(1);
  }

  fast_rng.begin(radio_get_rng_seed());
  SPIFFS.begin(true);
  IdentityStore store(SPIFFS, "/identity");

  if (!store.load("_main", the_mesh.self_id)) {
    the_mesh.self_id = radio_new_identity();
    store.save("_main", the_mesh.self_id);
  }

  sensors.begin();
  the_mesh.begin(&SPIFFS);
  the_mesh.getNodePrefs()->powersaving_enabled = 1;

  CayenneLPP dummy(64);
  sensors.querySensors(0xFF, dummy);

  // Boot GPS cycle
  is_gps_cycle_active = true;
  gps_start_ms = millis();
  digitalWrite(PIN_GPS_ENABLE, HIGH);
  log_ts("[PWR] Initializing sensors and GPS.");
}

void loop() {
  static char command_buf[160];
  int clen = strlen(command_buf);
  while (Serial.available() && clen < sizeof(command_buf) - 1) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (clen > 0) {
        command_buf[clen] = 0;
        char reply[160] = {0};
        log_ts("[CONSOLE] Executing: %s", command_buf);
        the_mesh.handleCommand(0, command_buf, reply);
        if (reply[0]) log_ts(" -> %s", reply);
        command_buf[0] = 0; clen = 0;

        active_mode_end_ms = millis() + ACTIVE_MODE_DURATION_MS;
        if (!is_gps_cycle_active) {
          is_gps_cycle_active = true;
          gps_start_ms = millis();
          digitalWrite(PIN_GPS_ENABLE, HIGH);
        }
      }
    } else {
      command_buf[clen++] = c;
      command_buf[clen] = 0;
    }
  }

  sensors.loop();
  the_mesh.loop();
  rtc_clock.tick();

  uint32_t now_utc = rtc_clock.getCurrentTime();
  uint32_t current_hour = now_utc / 3600;
  uint32_t current_minute = (now_utc / 60) % 60;

  // REPORTING LOGIC
  // 1. Hourly Group Report (:00)
  if (now_utc > 1704067200 && current_minute == 0 && current_hour != last_report_hour) {
    if (!is_gps_cycle_active) {
      log_ts("[PWR] Hourly cycle start.");
      is_gps_cycle_active = true;
      gps_start_ms = millis();
      digitalWrite(PIN_GPS_ENABLE, HIGH);
    }
  }
  // 2. Mid-hour Private Report (:30)
  else if (now_utc > 1704067200 && current_minute == 30 && current_hour != last_report_pvt_hour) {
     if (!is_gps_cycle_active) {
      log_ts("[PWR] Mid-hour cycle start.");
      is_gps_cycle_active = true;
      gps_start_ms = millis();
      digitalWrite(PIN_GPS_ENABLE, HIGH);
    }
  }

  if (is_gps_cycle_active) {
    static uint32_t gps_fix_acquired_ms = 0;
    static bool is_gps_stabilizing = false;

    auto loc = sensors.getLocationProvider();
    if (loc && loc->isValid() && sensors.node_lat != 0.0) {
      if (!is_gps_stabilizing) {
        is_gps_stabilizing = true;
        gps_fix_acquired_ms = millis();
        log_ts("[GPS] Fix acquired. Stabilizing for 10s...");
      }

      if (millis() - gps_fix_acquired_ms > GPS_STABILIZE_MS) {
        double lat = sensors.node_lat;
        double lon = sensors.node_lon;
        float alt = sensors.node_altitude;
        double dist = has_any_gps_fix_ever ? calculateDistance(last_known_lat, last_known_lon, lat, lon) : 0;

        // Determine which report to send based on clock
        if (current_minute == 0) {
            the_mesh.sendGroupReport(lat, lon, alt, dist);
            last_report_hour = current_hour;
        } else if (current_minute == 30) {
            the_mesh.sendPrivateReport(lat, lon, alt, dist);
            last_report_pvt_hour = current_hour;
        } else {
            // Probably on-demand or boot report
            the_mesh.sendGroupReport(lat, lon, alt, dist);
        }

        last_known_lat = lat;
        last_known_lon = lon;
        last_known_alt = alt;
        has_any_gps_fix_ever = true;

        is_gps_cycle_active = false;
        is_gps_stabilizing = false;
        digitalWrite(PIN_GPS_ENABLE, LOW);
      }
    } else if (millis() - gps_start_ms > GPS_TIMEOUT_MS) {
      log_ts("[GPS] Timeout waiting for fix.");
      is_gps_cycle_active = false;
      is_gps_stabilizing = false;
      digitalWrite(PIN_GPS_ENABLE, LOW);
    }
  }
}
