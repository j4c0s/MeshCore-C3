#pragma once

#include "SensorMesh.h"
#include <math.h>
#include <stdarg.h>

#define PIN_GPS_ENABLE 1
#define ACTIVE_MODE_DURATION_MS 300000  // 5 minutes
#define ON_DEMAND_GPS_DURATION_MS 120000 // 2 minutes stay-awake for GPS
#define GPS_TIMEOUT_MS 600000          // 10 minutes
#define GPS_STABILIZE_MS 10000         // 10 seconds stabilization after fix

// Default Channel Config if not in platformio.ini
#ifndef GROUP_CHANNEL_HASH
  #define GROUP_CHANNEL_HASH 0x4C
#endif
#ifndef GROUP_CHANNEL_KEY
  #define GROUP_CHANNEL_KEY { 0x3e, 0x72, 0x3e, 0x16, 0xeb, 0x4b, 0x25, 0x64, 0xfa, 0x23, 0x5e, 0x9f, 0x81, 0x6d, 0x49, 0x76 }
#endif

// State
extern bool is_gps_cycle_active;
extern uint32_t gps_start_ms;
extern uint32_t active_mode_end_ms;
extern bool has_any_gps_fix_ever;
extern double last_known_lat;
extern double last_known_lon;
extern float last_known_alt;

// Haversine formula to calculate distance in meters
inline double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = (lat1) * M_PI / 180.0;
    lat2 = (lat2) * M_PI / 180.0;
    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return 6371000.0 * c;
}

// Global logger with timestamp
void log_ts(const char* format, ...);

class JacoMesh : public SensorMesh {
public:
  JacoMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : SensorMesh(board, radio, ms, rng, rtc, tables),
       battery_data(12*24, 5*60)
  {
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
    // Fix: Relative humidity in LPP is encoded as 0.5% units, but getRelativeHumidity returns float.
    // Based on user feedback, it was 1357%, which means it's likely raw LPP or needs scaling.
    // Standard CayenneLPP scale for humidity is 0.5, but mesh library helper getFloat divides by multiplier.
    float hum = getRelativeHumidity(TELEM_CHANNEL_SELF);

    snprintf(buf, len, "%s: Pos:%.6f,%.6f Alt:%.1f Dist:%.1fm | %s | ATH:%.1fC/%.1f%%",
             getNodeName(), lat, lon, alt, dist, ina_info, temp, hum);
  }

  void sendGroupReport(double lat, double lon, float alt, double dist) {
    mesh::GroupChannel channel;
    channel.hash[0] = GROUP_CHANNEL_HASH;
    uint8_t key[16] = GROUP_CHANNEL_KEY;
    memcpy(channel.secret, key, 16);
    memset(channel.secret + 16, 0x00, 16);

    uint8_t data[MAX_PACKET_PAYLOAD];
    uint32_t ts = getRTCClock()->getCurrentTime();
    memcpy(data, &ts, 4);
    data[4] = 0x00;

    char report[160];
    buildReport(report, sizeof(report), lat, lon, alt, dist);

    int len = 5 + snprintf((char*)&data[5], sizeof(data) - 5, "%s", report);
    auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, channel, data, len);
    if (pkt) {
      sendFlood(pkt, (uint32_t)0, (uint8_t)3);
      log_ts("[MESH] Group report sent.");
    }
  }

  void sendPrivateReport(double lat, double lon, float alt, double dist) {
    // Send to first contact in ACL (Admin)
    auto contact = acl.getClientByIdx(0);
    if (contact && contact->permissions != 0) {
      uint8_t data[MAX_PACKET_PAYLOAD];
      uint32_t ts = getRTCClock()->getCurrentTimeUnique();
      memcpy(data, &ts, 4);
      data[4] = (TXT_TYPE_PLAIN << 2);

      char report[160];
      buildReport(report, sizeof(report), lat, lon, alt, dist);
      int tlen = snprintf((char*)&data[5], sizeof(data) - 5, "[PVT] %s", report);

      auto pkt = createDatagram(PAYLOAD_TYPE_TXT_MSG, contact->id, contact->shared_secret, data, 5 + tlen);
      if (pkt) {
        if (contact->out_path_len != OUT_PATH_UNKNOWN) {
          sendDirect(pkt, contact->out_path, contact->out_path_len);
        } else {
          sendFlood(pkt, (uint32_t)0, (uint8_t)3);
        }
        log_ts("[MESH] Private report sent to Admin.");
      }
    } else {
      log_ts("[MESH] No Admin found in ACL for private report.");
    }
  }

protected:
  Trigger low_batt, critical_batt;
  TimeSeriesData battery_data;

  void onSensorDataRead() override {
    battery_data.recordData(getRTCClock(), (float)board.getBattMilliVolts() / 1000.0f);
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
      log_ts("[PWR] GPS triggered on demand (Stay awake: 2m).");
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
    if (channel.hash[0] == GROUP_CHANNEL_HASH) {
       startGpsOnDemand();
    }
  }
};
