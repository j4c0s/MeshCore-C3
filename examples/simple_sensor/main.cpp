#include "SensorMesh.h"
#include <math.h>
#include <stdarg.h>
#include <SPIFFS.h>
#include <RTClib.h>
#include "target.h"

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

#define PIN_GPS_ENABLE 1
#define ACTIVE_MODE_DURATION_MS 300000  // 5 minutes
#define ON_DEMAND_GPS_DURATION_MS 120000 // 2 minutes stay-awake for GPS
#define GPS_TIMEOUT_MS 600000          // 10 minutes
#define GPS_STABILIZE_MS 10000         // 10 seconds stabilization after fix

// Persistent state in RTC memory
RTC_DATA_ATTR double last_known_lat = 0.0;
RTC_DATA_ATTR double last_known_lon = 0.0;
RTC_DATA_ATTR float last_known_alt = 0.0;
RTC_DATA_ATTR bool has_any_gps_fix_ever = false;

// Power and Report State
bool is_gps_cycle_active = false;
uint32_t gps_start_ms = 0;
uint32_t active_mode_end_ms = 0;
uint32_t last_report_hour = 0xFFFFFFFF;
uint32_t last_report_pvt_hour = 0xFFFFFFFF;

// Haversine formula to calculate distance in meters
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = (lat1) * M_PI / 180.0;
    lat2 = (lat2) * M_PI / 180.0;
    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return 6371000.0 * c;
}

// Global logger with timestamp
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

class MyMesh : public SensorMesh {
public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : SensorMesh(board, radio, ms, rng, rtc, tables),
       battery_data(12*24, 5*60)
  {
  }

  float findInaVoltage() {
    for (uint8_t ch = 1; ch <= 4; ch++) {
      float v = getVoltage(ch);
      if (v > 0.1f) return v;
    }
    return 0.0f;
  }

  void formatAllInaVoltages(char* buf, size_t len) {
    char* dp = buf;
    int ofs = 0;
    for (uint8_t ch = 1; ch <= 4; ch++) {
      float v = getVoltage(ch);
      if (v > 0.1f) {
        ofs += snprintf(dp + ofs, len - ofs, "CH%d:%.2fV ", ch, v);
      }
    }
  }

  void buildReport(char* buf, size_t len, double lat, double lon, float alt, double dist) {
    char ina_info[64] = {0};
    formatAllInaVoltages(ina_info, sizeof(ina_info));
    float temp = getTemperature(TELEM_CHANNEL_SELF);
    float hum = getRelativeHumidity(TELEM_CHANNEL_SELF);

    snprintf(buf, len, "Pos:%.6f,%.6f Alt:%.1f Dist:%.1fm | %s | ATH:%.1fC/%.1f%%",
             lat, lon, alt, dist, ina_info, temp, hum);
  }

  void sendGroupReport(double lat, double lon, float alt, double dist) {
    mesh::GroupChannel channel;
    channel.hash[0] = 0x4C;
    const uint8_t key[16] = { 0x3e, 0x72, 0x3e, 0x16, 0xeb, 0x4b, 0x25, 0x64, 0xfa, 0x23, 0x5e, 0x9f, 0x81, 0x6d, 0x49, 0x76 };
    memcpy(channel.secret, key, 16);
    memset(channel.secret + 16, 0x00, 16);

    uint8_t data[MAX_PACKET_PAYLOAD];
    uint32_t ts = getRTCClock()->getCurrentTime();
    memcpy(data, &ts, 4);
    data[4] = 0x00;

    char report[160];
    buildReport(report, sizeof(report), lat, lon, alt, dist);
    int len = 5 + snprintf((char*)&data[5], sizeof(data) - 5, "%s: %s", getNodeName(), report);

    auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, channel, data, len);
    if (pkt) {
      sendFlood(pkt, (uint32_t)0, (uint8_t)3);
      log_ts("[MESH] Group report sent.");
    }
  }

  void sendPrivateReport(double lat, double lon, float alt, double dist) {
    auto contact = acl.getClientByIdx(0);
    if (contact && contact->permissions != 0) {
      uint8_t data[MAX_PACKET_PAYLOAD];
      uint32_t ts = getRTCClock()->getCurrentTimeUnique();
      memcpy(data, &ts, 4);
      data[4] = (TXT_TYPE_PLAIN << 2);

      char report[160];
      buildReport(report, sizeof(report), lat, lon, alt, dist);
      int tlen = snprintf((char*)&data[5], sizeof(data) - 5, "[PVT] %s: %s", getNodeName(), report);

      auto pkt = createDatagram(PAYLOAD_TYPE_TXT_MSG, contact->id, contact->shared_secret, data, 5 + tlen);
      if (pkt) {
        if (contact->out_path_len != OUT_PATH_UNKNOWN) {
          sendDirect(pkt, contact->out_path, contact->out_path_len);
        } else {
          sendFlood(pkt, (uint32_t)0, (uint8_t)3);
        }
        log_ts("[MESH] Private report sent to Admin.");
      }
    }
  }

protected:
  Trigger low_batt, critical_batt;
  TimeSeriesData battery_data;

  void onSensorDataRead() override {
    float v = findInaVoltage();
    if (v > 0.1f) {
      battery_data.recordData(getRTCClock(), v);
    }
  }

  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) override {
    battery_data.calcMinMaxAvg(getRTCClock(), start_secs_ago, end_secs_ago, &dest[0], TELEM_CHANNEL_SELF, LPP_VOLTAGE);
    return 1;
  }

  bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) override {
    if (strcmp(command, "status") == 0) {
      char ina_info[64] = {0};
      formatAllInaVoltages(ina_info, sizeof(ina_info));
      float temp = getTemperature(TELEM_CHANNEL_SELF);
      float hum = getRelativeHumidity(TELEM_CHANNEL_SELF);
      sprintf(reply, "INA:%s, ATH:%.1fC/%.1f%%, GPS:%.6f,%.6f", ina_info, temp, hum, last_known_lat, last_known_lon);
      return true;
    }
    return false;
  }

  void startGpsOnDemand() {
    active_mode_end_ms = millis() + ON_DEMAND_GPS_DURATION_MS;
    if (!is_gps_cycle_active) {
      is_gps_cycle_active = true;
      gps_start_ms = millis();
      digitalWrite(PIN_GPS_ENABLE, HIGH);
      auto loc = sensors.getLocationProvider();
      if (loc) loc->syncTime(); // Clear NMEA buffer
      log_ts("[PWR] GPS triggered on demand.");
    }
  }

  void onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx, const uint8_t* secret, uint8_t* data, size_t len) override {
    SensorMesh::onPeerDataRecv(packet, type, sender_idx, secret, data, len);
    if (type == PAYLOAD_TYPE_REQ || type == PAYLOAD_TYPE_TXT_MSG) {
       startGpsOnDemand();
    }
  }

  void onGroupDataRecv(mesh::Packet* packet, uint8_t type, const mesh::GroupChannel& channel, uint8_t* data, size_t len) override {
    SensorMesh::onGroupDataRecv(packet, type, channel, data, len);
    if (channel.hash[0] == 0x4C) {
       startGpsOnDemand();
    }
  }
};

StdRNG fast_rng;
SimpleMeshTables tables;
MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void setup() {
  Serial.begin(115200);
  delay(1000);

  log_ts("--- Jaco Sensor Starting ---");

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

  // Initial GPS cycle
  is_gps_cycle_active = true;
  gps_start_ms = millis();
  digitalWrite(PIN_GPS_ENABLE, HIGH);
  auto loc = sensors.getLocationProvider();
  if (loc) loc->syncTime(); // Clear NMEA buffer
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

  if (now_utc > 1704067200 && current_minute == 0 && current_hour != last_report_hour) {
    if (!is_gps_cycle_active) {
      log_ts("[PWR] Hourly cycle start.");
      is_gps_cycle_active = true;
      gps_start_ms = millis();
      digitalWrite(PIN_GPS_ENABLE, HIGH);
      auto loc = sensors.getLocationProvider();
      if (loc) loc->syncTime(); // Clear NMEA buffer
    }
  } else if (now_utc > 1704067200 && current_minute == 30 && current_hour != last_report_pvt_hour) {
     if (!is_gps_cycle_active) {
      log_ts("[PWR] Mid-hour cycle start.");
      is_gps_cycle_active = true;
      gps_start_ms = millis();
      digitalWrite(PIN_GPS_ENABLE, HIGH);
      auto loc = sensors.getLocationProvider();
      if (loc) loc->syncTime(); // Clear NMEA buffer
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

        if (current_minute == 0) {
            the_mesh.sendGroupReport(lat, lon, alt, dist);
            last_report_hour = current_hour;
        } else if (current_minute == 30) {
            the_mesh.sendPrivateReport(lat, lon, alt, dist);
            last_report_pvt_hour = current_hour;
        } else {
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
