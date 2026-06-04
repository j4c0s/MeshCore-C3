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
bool pwr_log_done = false;
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
    memset(t_prefs.channel_scope, 0, sizeof(t_prefs.channel_scope));
  }
}

void saveTrackerPrefs() {
  File file = SPIFFS.open("/tracker_prefs", "w", true);
  if (file) {
    file.write((uint8_t*)&t_prefs, sizeof(t_prefs));
    file.close();
  }
}


StdRNG fast_rng;
SimpleMeshTables tables;
TrackerMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void scanI2C() {
  Serial.println("Scanning I2C...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("I2C device at 0x%02X\n", addr);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if(!SPIFFS.begin(true)){
     Serial.println("SPIFFS Mount Failed");
  }

  board.begin();
  scanI2C();
  if (!radio_init()) {
    Serial.println("Radio init failed!");
    while(1);
  }

  loadTrackerPrefs();
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
  Serial.println("Boot: Starting initial GPS fix cycle...");
  is_gps_cycle_active = true;
  gps_start_ms = millis();
  digitalWrite(PIN_GPS_EN, HIGH);
  auto loc = sensors.getLocationProvider();
  if (loc) {
    loc->syncTime();
    // Clear NMEA buffer to ensure we don't use old data
    while (Serial1.available()) Serial1.read();
  }
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
        Serial.printf("> %s\n", command_buf);
        the_mesh.handleCommand(0, command_buf, reply);
        if (reply[0]) Serial.printf("Reply: %s\n", reply);
        command_buf[0] = 0; clen = 0;

        pwr_log_done = false;
        active_mode_end_ms = millis() + ACTIVE_MODE_DURATION_MS;
        if (!is_gps_cycle_active) {
          is_gps_cycle_active = true;
          gps_start_ms = millis();
          digitalWrite(PIN_GPS_EN, HIGH);
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

  static uint32_t last_manual_read = 0;

  uint32_t now_utc = rtc_clock.getCurrentTime();
  static bool time_was_synced = false;

  // REPORTING LOGIC
  // 1704067200 = 1 Jan 2024
  // Wait until we get a real time sync from GPS
  if (now_utc > 1704067200) {
      if (!time_was_synced) {
        DateTime dt = DateTime(now_utc);
        Serial.printf("Time synced! UTC: %02d:%02d:%02d\n", dt.hour(), dt.minute(), dt.second());
        time_was_synced = true;
      }
      uint32_t now_mins = now_utc / 60;

      // If interval is set but we haven't reported yet, OR if it's time for next report
      if (t_prefs.group_interval_mins > 0 && now_mins != last_report_group_ts) {
        if ((now_mins % t_prefs.group_interval_mins) == 0 || last_report_group_ts == 0) {
           if (!is_gps_cycle_active) {
             Serial.printf("Reporting Trigger (Group): Now=%u, Last=%u, Int=%u\n", now_mins, last_report_group_ts, t_prefs.group_interval_mins);
             last_report_group_ts = now_mins;

             // Trigger manual sensor read before cycle
             CayenneLPP dummy(64);
             sensors.querySensors(0xFF, dummy);

             is_gps_cycle_active = true;
             gps_start_ms = millis();
             digitalWrite(PIN_GPS_EN, HIGH);
             auto loc = sensors.getLocationProvider();
             if (loc) {
               loc->syncTime();
               while (Serial1.available()) Serial1.read();
             }
           }
        }
      }

      if (t_prefs.pvt_interval_mins > 0 && now_mins != last_report_pvt_ts) {
        if (((now_mins + (t_prefs.group_interval_mins == t_prefs.pvt_interval_mins ? 1 : 0)) % t_prefs.pvt_interval_mins) == 0 || last_report_pvt_ts == 0) {
           if (!is_gps_cycle_active) {
             Serial.printf("Reporting Trigger (Private): Now=%u, Last=%u, Int=%u\n", now_mins, last_report_pvt_ts, t_prefs.pvt_interval_mins);
             last_report_pvt_ts = now_mins;

             // Trigger manual sensor read before cycle
             CayenneLPP dummy(64);
             sensors.querySensors(0xFF, dummy);

             is_gps_cycle_active = true;
             gps_start_ms = millis();
             digitalWrite(PIN_GPS_EN, HIGH);
             auto loc = sensors.getLocationProvider();
             if (loc) {
               loc->syncTime();
               while (Serial1.available()) Serial1.read();
             }
           }
        }
      }
  }

  if (is_gps_cycle_active) {
    pwr_log_done = false;
    static uint32_t gps_fix_acquired_ms = 0;
    static uint32_t last_status_log = 0;
    static bool is_gps_stabilizing = false;

    auto loc = sensors.getLocationProvider();
    bool has_basic_fix = (loc && loc->isValid() && sensors.node_lat != 0.0);

    if (millis() - last_status_log > 5000) {
      last_status_log = millis();
      if (loc) {
        Serial.printf("GPS Status: %lus, Sats: %d, Fix: %s\n", (millis() - gps_start_ms)/1000, loc->satellitesCount(), has_basic_fix?"YES":"NO");
      } else {
        Serial.println("GPS Error: No Provider");
      }
    }

    if (has_basic_fix) {
      if (!is_gps_stabilizing) {
        is_gps_stabilizing = true;
        gps_fix_acquired_ms = millis();
      }

      float current_hdop = loc->getHDOP();
      float current_acc = hdopToMeters(current_hdop);
      bool is_precise_enough = (current_acc <= TARGET_ACCURACY_METERS);
      bool is_timed_out = (millis() - gps_fix_acquired_ms > GPS_STABILIZE_TIMEOUT_MS);

      static uint32_t last_acc_log = 0;
      if (millis() - last_acc_log > 2000) {
        last_acc_log = millis();
        Serial.printf("Stabilizing: %lum, Target: %lum\n", (uint32_t)current_acc, (uint32_t)TARGET_ACCURACY_METERS);
      }

      if (is_precise_enough || is_timed_out) {
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

        last_known_lat = lat;
        last_known_lon = lon;
        last_known_alt = alt;
        has_any_gps_fix_ever = true;

        is_gps_cycle_active = false;
        is_gps_stabilizing = false;
        digitalWrite(PIN_GPS_EN, LOW);
      }
    } else if (millis() - gps_start_ms > GPS_TIMEOUT_MS) {
      is_gps_cycle_active = false;
      is_gps_stabilizing = false;
      digitalWrite(PIN_GPS_EN, LOW);
    }
  }

  // POWERSAVING - Determine next sleep duration
  if (!is_gps_cycle_active && (active_mode_end_ms == 0 || millis() > active_mode_end_ms)) {
      // Logic for calculating next required wake-up
      uint32_t wait_group = 3600; // default 1h
      uint32_t wait_pvt = 3600;

      if (t_prefs.group_interval_mins > 0) {
          uint32_t now_mins = rtc_clock.getCurrentTime() / 60;
          uint32_t mins_to_next = t_prefs.group_interval_mins - (now_mins % t_prefs.group_interval_mins);
          wait_group = mins_to_next * 60;
      }
      if (t_prefs.pvt_interval_mins > 0) {
          uint32_t now_mins = rtc_clock.getCurrentTime() / 60;
          uint32_t mins_to_next = t_prefs.pvt_interval_mins - (now_mins % t_prefs.pvt_interval_mins);
          wait_pvt = mins_to_next * 60;
      }

      uint32_t sleep_secs = min(wait_group, wait_pvt);
      if (sleep_secs < 10) sleep_secs = 60; // minimum 1 min sleep if very close to next interval

      pwr_log_done = true;

      Serial.flush();
      delay(100);
      radio.getIrqFlags();

      // IMPORTANT: In light sleep (powersaving_enabled=1), MeshCore::loop() handles sleep automatically.
      // But we set the internal 'inhibit_sleep' state if we want to bypass it.
      // For this Universal version, we use the board's class logic.
  }
}
