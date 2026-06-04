#pragma once

#include "SensorMesh.h"
#include <math.h>
#include <stdarg.h>

#define ACTIVE_MODE_DURATION_MS 300000  // 5 minutes
#define ON_DEMAND_GPS_DURATION_MS 120000 // 2 minutes stay-awake for GPS
#define GPS_TIMEOUT_MS 600000          // 10 minutes
#define GPS_STABILIZE_TIMEOUT_MS 20000 // Max 20s wait for accuracy
#define TARGET_ACCURACY_METERS 25.0f   // Target accuracy in meters

// State Persistence (Shared with main.cpp)
extern double last_known_lat;
extern double last_known_lon;
extern float last_known_alt;
extern bool has_any_gps_fix_ever;

// Power and Report State (Shared with main.cpp)
extern bool is_gps_cycle_active;
extern uint32_t gps_start_ms;
extern uint32_t active_mode_end_ms;
extern uint32_t last_report_group_ts;
extern uint32_t last_report_pvt_ts;

struct TrackerPrefs {
  uint8_t channel_hash;
  uint8_t channel_key[16];
  uint16_t group_interval_mins;
  uint16_t pvt_interval_mins;
  char channel_scope[32];
} extern t_prefs;

void loadTrackerPrefs();
void saveTrackerPrefs();

// Haversine formula
inline double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = (lat1) * M_PI / 180.0;
    lat2 = (lat2) * M_PI / 180.0;
    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return 6371000.0 * c;
}

// Convert HDOP to approximate meters
inline float hdopToMeters(float hdop) {
    return hdop * 5.0f;
}

class TrackerMesh : public SensorMesh {
public:
  TrackerMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : SensorMesh(board, radio, ms, rng, rtc, tables),
       battery_data(12*24, 5*60)
  {
  }

  float findInaVoltage() {
    for (uint8_t ch = 2; ch <= 4; ch++) {
      float v = getVoltage(ch);
      if (v > 0.1f) return v;
    }
    return 0.0f;
  }

  void formatAllInaData(char* buf, size_t len) {
    char* dp = buf;
    int ofs = 0;
    for (uint8_t ch = 2; ch <= 4; ch++) {
      float v = getVoltage(ch);
      float a = getCurrent(ch);
      if (v > 0.1f) {
        ofs += snprintf(dp + ofs, len - ofs, "CH%d:%.2fV/%.0fmA ", ch-1, v, a * 1000.0f);
      }
    }
  }

  void buildReport(char* buf, size_t len, double lat, double lon, float alt, double dist, float hdop) {
    float temp = getTemperature(TELEM_CHANNEL_SELF);
    float hum = getRelativeHumidity(TELEM_CHANNEL_SELF);
    float acc = hdopToMeters(hdop);

    // Format: $TRK,lat,lon,alt,dist,acc,V1,A1,V2,A2,V3,A3,temp,hum
    // Voltages in V, Currents in mA
    int ofs = snprintf(buf, len, "$TRK,%.6f,%.6f,%.1f,%.1f,%.0f", lat, lon, alt, dist, acc);

    // INA3221 usually populates channels starting from 2 in EnvironmentSensorManager
    for (uint8_t ch = 2; ch <= 4; ch++) {
      float v = getVoltage(ch);
      float a = getCurrent(ch);
      ofs += snprintf(buf + ofs, len - ofs, ",%.2f,%.0f", v, a * 1000.0f);
    }

    snprintf(buf + ofs, len - ofs, ",%.1f,%.1f", temp, hum);
  }

  void sendGroupReport(double lat, double lon, float alt, double dist, float hdop) {
    mesh::GroupChannel channel;
    channel.hash[0] = t_prefs.channel_hash;
    memcpy(channel.secret, t_prefs.channel_key, 16);
    memset(channel.secret + 16, 0x00, 16);

    uint8_t data[MAX_PACKET_PAYLOAD];
    uint32_t ts = getRTCClock()->getCurrentTime();
    memcpy(data, &ts, 4);
    data[4] = 0x00;

    char report[160];
    buildReport(report, sizeof(report), lat, lon, alt, dist, hdop);
    int len = 5 + snprintf((char*)&data[5], sizeof(data) - 5, "%s: %s", getNodeName(), report);

    auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, channel, data, len);
    if (pkt) {
      if (strlen(t_prefs.channel_scope) > 0 && strcmp(t_prefs.channel_scope, "none") != 0) {
        auto region = region_map.findByName(t_prefs.channel_scope);
        if (region) {
          TransportKey keys[2];
          int n = region_map.getTransportKeysFor(*region, keys, 2);
          if (n > 0) {
            uint16_t codes[2];
            codes[0] = keys[0].calcTransportCode(pkt);
            codes[1] = (n > 1) ? keys[1].calcTransportCode(pkt) : 0;
            sendFlood(pkt, codes, (uint32_t)0, (uint8_t)3);
            return;
          }
        }
      }
      sendFlood(pkt, (uint32_t)0, (uint8_t)3);
    }
  }

  void sendPrivateReport(double lat, double lon, float alt, double dist, float hdop) {
    auto contact = acl.getClientByIdx(0);
    if (contact && contact->permissions != 0) {
      uint8_t data[MAX_PACKET_PAYLOAD];
      uint32_t ts = getRTCClock()->getCurrentTimeUnique();
      memcpy(data, &ts, 4);
      data[4] = (TXT_TYPE_PLAIN << 2);

      char report[160];
      buildReport(report, sizeof(report), lat, lon, alt, dist, hdop);
      int tlen = snprintf((char*)&data[5], sizeof(data) - 5, "[PVT] %s: %s", getNodeName(), report);

      auto pkt = createDatagram(PAYLOAD_TYPE_TXT_MSG, contact->id, contact->shared_secret, data, 5 + tlen);
      if (pkt) {
        if (contact->out_path_len != OUT_PATH_UNKNOWN) {
          sendDirect(pkt, contact->out_path, contact->out_path_len);
        } else {
          sendFlood(pkt, (uint32_t)0, (uint8_t)3);
        }
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
    if (StrHelper::startsWith(command, "status")) {
      char ina_info[100] = {0};
      formatAllInaData(ina_info, sizeof(ina_info));
      float temp = getTemperature(TELEM_CHANNEL_SELF);
      float hum = getRelativeHumidity(TELEM_CHANNEL_SELF);
      sprintf(reply, "INA:%s, ATH:%.1fC/%.1f%%, GPS:%.6f,%.6f", ina_info, temp, hum, last_known_lat, last_known_lon);
      return true;
    }

    if (StrHelper::startsWith(command, "tracker help")) {
      strcpy(reply, "set channel.key {hex}, set channel.scope {name|none}, set reporting.group {mins}, set reporting.pvt {mins}, status");
      return true;
    }

    if (StrHelper::startsWith(command, "set channel.key ")) {
      mesh::Utils::fromHex(t_prefs.channel_key, 16, &command[16]);
      uint8_t full_hash[32];
      mesh::Utils::sha256(full_hash, 32, t_prefs.channel_key, 16);
      t_prefs.channel_hash = full_hash[0];
      saveTrackerPrefs();
      sprintf(reply, "OK - Channel Configured (Hash: %02X)", t_prefs.channel_hash);
      return true;
    }
    if (StrHelper::startsWith(command, "set channel.scope ")) {
      const char* name = &command[18];
      if (strcmp(name, "none") == 0) {
          t_prefs.channel_scope[0] = 0;
      } else {
          StrHelper::strncpy(t_prefs.channel_scope, name, sizeof(t_prefs.channel_scope));
          if (region_map.findByName(t_prefs.channel_scope) == NULL) {
              auto r = region_map.putRegion(t_prefs.channel_scope, 0);
              if (r) {
                region_map.save(_fs);
              }
          }
      }
      saveTrackerPrefs();
      sprintf(reply, "OK - Group channel scope: %s", t_prefs.channel_scope[0] ? t_prefs.channel_scope : "none");
      return true;
    }
    if (StrHelper::startsWith(command, "set reporting.group ")) {
      t_prefs.group_interval_mins = atoi(&command[20]);
      saveTrackerPrefs();
      sprintf(reply, "OK - Group reporting every %d mins", t_prefs.group_interval_mins);
      return true;
    }
    if (StrHelper::startsWith(command, "set reporting.pvt ")) {
      t_prefs.pvt_interval_mins = atoi(&command[18]);
      saveTrackerPrefs();
      sprintf(reply, "OK - Private reporting every %d mins", t_prefs.pvt_interval_mins);
      return true;
    }

    return false;
  }

  void startGpsOnDemand() {
    active_mode_end_ms = millis() + ON_DEMAND_GPS_DURATION_MS;
    if (!is_gps_cycle_active) {
      is_gps_cycle_active = true;
      gps_start_ms = millis();
      digitalWrite(PIN_GPS_EN, HIGH);
      auto loc = sensors.getLocationProvider();
      if (loc) {
        loc->syncTime();
      }
    }
  }

  void onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx, const uint8_t* secret, uint8_t* data, size_t len) override {
    SensorMesh::onPeerDataRecv(packet, type, sender_idx, secret, data, len);
    if (type == PAYLOAD_TYPE_REQ || type == PAYLOAD_TYPE_TXT_MSG) {
       startGpsOnDemand();
    }
  }

  void onGroupDataRecv(mesh::Packet* packet, uint8_t type, const mesh::GroupChannel& channel, uint8_t* data, size_t len) override {
    if (channel.hash[0] == t_prefs.channel_hash) {
       startGpsOnDemand();
    }
  }
};
