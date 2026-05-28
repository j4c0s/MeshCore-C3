#include "TrackerMesh.h"
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
uint32_t last_report_group_ts = 0;
uint32_t last_report_pvt_ts = 0;

// GPS Persistence
RTC_DATA_ATTR double last_known_lat = 0.0;
RTC_DATA_ATTR double last_known_lon = 0.0;
RTC_DATA_ATTR float last_known_alt = 0.0;
RTC_DATA_ATTR bool has_any_gps_fix_ever = false;

// Global Prefs
TrackerPrefs t_prefs;

void loadTrackerPrefs() {
  File file = SPIFFS.open("/tracker_prefs", "r");
  if (file && file.size() == sizeof(t_prefs)) {
    file.read((uint8_t*)&t_prefs, sizeof(t_prefs));
    file.close();
  } else {
    // Defaults: Disabled reporting
    t_prefs.channel_hash = 0x4C;
    uint8_t def_key[16] = { 0x3e, 0x72, 0x3e, 0x16, 0xeb, 0x4b, 0x25, 0x64, 0xfa, 0x23, 0x5e, 0x9f, 0x81, 0x6d, 0x49, 0x76 };
    memcpy(t_prefs.channel_key, def_key, 16);
    t_prefs.group_interval_mins = 0;
    t_prefs.pvt_interval_mins = 0;
    memset(t_prefs.pvt_scope, 0, sizeof(t_prefs.pvt_scope));
  }
}

void saveTrackerPrefs() {
  File file = SPIFFS.open("/tracker_prefs", "w", true);
  if (file) {
    file.write((uint8_t*)&t_prefs, sizeof(t_prefs));
    file.close();
  }
}

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
TrackerMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void setup() {
  Serial.begin(115200);
  delay(1000);

  if(!SPIFFS.begin(true)){
     Serial.println("SPIFFS Mount Failed");
  }
  loadTrackerPrefs();

  log_ts("--- MeshTracker Starting ---");

  board.begin();
  if (!radio_init()) {
    log_ts("Radio init failed!");
    while(1);
  }

  fast_rng.begin(radio_get_rng_seed());

  IdentityStore store(SPIFFS, "/identity");
  if (!store.load("_main", the_mesh.self_id)) {
    the_mesh.self_id = radio_new_identity();
    store.save("_main", the_mesh.self_id);
  }

  sensors.begin();
  the_mesh.begin(&SPIFFS);

  // Tracker Branded Defaults
  if (strlen(the_mesh.getNodePrefs()->node_name) == 0 || strcmp(the_mesh.getNodePrefs()->node_name, "sensor") == 0) {
     strcpy(the_mesh.getNodePrefs()->node_name, "MeshTracker");
  }
  if (strlen(the_mesh.getNodePrefs()->password) == 0 || strcmp(the_mesh.getNodePrefs()->password, "password") == 0) {
     strcpy(the_mesh.getNodePrefs()->password, "123456");
  }

  the_mesh.getNodePrefs()->powersaving_enabled = 1;

  CayenneLPP dummy(64);
  sensors.querySensors(0xFF, dummy);

  // Initial GPS cycle on boot
  is_gps_cycle_active = true;
  gps_start_ms = millis();
  digitalWrite(PIN_GPS_ENABLE, HIGH);
  auto loc = sensors.getLocationProvider();
  if (loc) loc->syncTime();
  log_ts("[PWR] Initializing GPS for boot cycle.");
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
          auto loc = sensors.getLocationProvider();
          if (loc) loc->syncTime();
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

  if (now_utc > 1704067200) {
      uint32_t now_mins = now_utc / 60;

      if (t_prefs.group_interval_mins > 0 && (now_mins % t_prefs.group_interval_mins) == 0 && now_mins != last_report_group_ts) {
        if (!is_gps_cycle_active) {
          log_ts("[PWR] Scheduled Group Report cycle.");
          is_gps_cycle_active = true;
          gps_start_ms = millis();
          digitalWrite(PIN_GPS_ENABLE, HIGH);
          auto loc = sensors.getLocationProvider();
          if (loc) loc->syncTime();
        }
      }

      if (t_prefs.pvt_interval_mins > 0 && ((now_mins + (t_prefs.group_interval_mins == t_prefs.pvt_interval_mins ? 1 : 0)) % t_prefs.pvt_interval_mins) == 0 && now_mins != last_report_pvt_ts) {
        if (!is_gps_cycle_active) {
          log_ts("[PWR] Scheduled Private Report cycle.");
          is_gps_cycle_active = true;
          gps_start_ms = millis();
          digitalWrite(PIN_GPS_ENABLE, HIGH);
          auto loc = sensors.getLocationProvider();
          if (loc) loc->syncTime();
        }
      }
  }

  if (is_gps_cycle_active) {
    static uint32_t gps_fix_acquired_ms = 0;
    static bool is_gps_stabilizing = false;
    static uint32_t last_gps_debug = 0;

    auto loc = sensors.getLocationProvider();
    bool has_basic_fix = (loc && loc->isValid() && sensors.node_lat != 0.0);

    // Periodic GPS status debug
    if (millis() - last_gps_debug > 5000) {
        last_gps_debug = millis();
        if (loc) {
            float current_hdop = loc->getHDOP();
            log_ts("[GPS-DEBUG] Uptime: %lus | Sats: %d | Acc: ~%.0fm | Fix: %s",
                   (millis() - gps_start_ms) / 1000,
                   (int)loc->satellitesCount(),
                   hdopToMeters(current_hdop),
                   has_basic_fix ? "YES" : "NO");
        }
    }

    if (has_basic_fix) {
      if (!is_gps_stabilizing) {
        is_gps_stabilizing = true;
        gps_fix_acquired_ms = millis();
        log_ts("[GPS] Fix acquired. Waiting for precision (<%.0fm) or %ds timeout...", TARGET_ACCURACY_METERS, GPS_STABILIZE_TIMEOUT_MS/1000);
      }

      float current_hdop = loc->getHDOP();
      float current_acc = hdopToMeters(current_hdop);
      bool is_precise_enough = (current_acc <= TARGET_ACCURACY_METERS);
      bool is_timed_out = (millis() - gps_fix_acquired_ms > GPS_STABILIZE_TIMEOUT_MS);

      if (is_precise_enough || is_timed_out) {
        if (is_precise_enough) log_ts("[GPS) Target precision reached (~%.0fm).", current_acc);
        else log_ts("[GPS] Stabilization timeout. Sending best available fix (~%.0fm).", current_acc);

        double lat = sensors.node_lat;
        double lon = sensors.node_lon;
        float alt = sensors.node_altitude;
        double dist = has_any_gps_fix_ever ? calculateDistance(last_known_lat, last_known_lon, lat, lon) : 0;

        uint32_t now_mins = now_utc / 60;
        bool sent = false;
        if (t_prefs.group_interval_mins > 0 && (now_mins % t_prefs.group_interval_mins) == 0) {
            the_mesh.sendGroupReport(lat, lon, alt, dist, current_hdop);
            last_report_group_ts = now_mins;
            sent = true;
        }

        if (t_prefs.pvt_interval_mins > 0 && ((now_mins + (t_prefs.group_interval_mins == t_prefs.pvt_interval_mins ? 1 : 0)) % t_prefs.pvt_interval_mins) == 0) {
            the_mesh.sendPrivateReport(lat, lon, alt, dist, current_hdop);
            last_report_pvt_ts = now_mins;
            sent = true;
        }

        if (!sent) {
            the_mesh.sendGroupReport(lat, lon, alt, dist, current_hdop);
        }

        log_ts("[GPS] Final Fix: %.6f, %.6f. Distance moved: %.1fm. Report sent.", lat, lon, dist);

        last_known_lat = lat;
        last_known_lon = lon;
        last_known_alt = alt;
        has_any_gps_fix_ever = true;

        is_gps_cycle_active = false;
        is_gps_stabilizing = false;
        digitalWrite(PIN_GPS_ENABLE, LOW);
        log_ts("[PWR] Cycle complete. GPS OFF.");
      }
    } else if (millis() - gps_start_ms > GPS_TIMEOUT_MS) {
      log_ts("[GPS] Timeout waiting for fix. Giving up.");
      is_gps_cycle_active = false;
      is_gps_stabilizing = false;
      digitalWrite(PIN_GPS_ENABLE, LOW);
    }
  }
}
