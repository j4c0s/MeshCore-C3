#include "SensorMesh.h"
#include <math.h>

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

#define PIN_GPS_ENABLE 1
#define ACTIVE_MODE_DURATION_MS 300000  // 5 minutes
#define DEEP_SLEEP_INTERVAL_SEC 3600    // 1 hour
#define GPS_TIMEOUT_MS 600000          // 10 minutes

// Persistent state in RTC memory
RTC_DATA_ATTR double last_known_lat = 0.0;
RTC_DATA_ATTR double last_known_lon = 0.0;
RTC_DATA_ATTR float last_known_alt = 0.0;
RTC_DATA_ATTR bool has_any_gps_fix_ever = false;

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

class MyMesh : public SensorMesh {
public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
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

  void sendChannelAlert(const char* text) {
    mesh::GroupChannel channel;
    channel.hash[0] = 0x4C; // As per snippet

    const uint8_t key[16] = {
      0x3e, 0x72, 0x3e, 0x16, 0xeb, 0x4b, 0x25, 0x64,
      0xfa, 0x23, 0x5e, 0x9f, 0x81, 0x6d, 0x49, 0x76
    };
    memcpy(channel.secret, key, 16);
    memset(channel.secret + 16, 0x00, 16);

    uint8_t data[MAX_PACKET_PAYLOAD];
    uint32_t ts = getRTCClock()->getCurrentTime();
    memcpy(data, &ts, 4);
    data[4] = 0x00;

    char ina_info[64] = {0};
    formatAllInaVoltages(ina_info, sizeof(ina_info));

    int len = 5 + snprintf((char*)&data[5], sizeof(data) - 5, "%s: %s | %s",
                            getNodeName(), text, ina_info);

    auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, channel, data, len);
    if (pkt) {
      sendFlood(pkt);
      Serial.println("[MESH] Alert sent to private channel.");
    }
  }

protected:
  Trigger low_batt, critical_batt;
  TimeSeriesData battery_data;

  void onSensorDataRead() override {
    // Record primary battery (assuming CH1 is primary or uses board.getBattMilliVolts())
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
      sprintf(reply, "INA: %s, GPS: %.6f,%.6f", ina_info, last_known_lat, last_known_lon);
      return true;
    }
    return false;
  }

  // Override to detect authorized logins and stay awake
  void onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx, const uint8_t* secret, uint8_t* data, size_t len) override {
    SensorMesh::onPeerDataRecv(packet, type, sender_idx, secret, data, len);

    // If we received peer data, it means we are talking to someone in our ACL
    // Extend active mode
    extern uint32_t active_mode_end_ms;
    active_mode_end_ms = millis() + ACTIVE_MODE_DURATION_MS;
    Serial.println("[PWR] Authorized peer activity detected. Extending stay-awake mode.");
  }
};

StdRNG fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);
uint32_t active_mode_end_ms = 0;
bool is_gps_cycle_active = false;
uint32_t gps_start_ms = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("--- Jaco Sensor Starting ---");

  board.begin();

  if (!radio_init()) {
    Serial.println("Radio init failed!");
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

  // Force initial sensor read to have telemetry data for first alert
  CayenneLPP dummy(64);
  sensors.querySensors(0xFF, dummy);

  // If we woke up from radio, stay active for a while
  if (board.getStartupReason() == BD_STARTUP_RX_PACKET) {
    active_mode_end_ms = millis() + ACTIVE_MODE_DURATION_MS;
    Serial.println("[PWR] Woke up by Radio. Entering 5-min active mode.");

    // In radio wakeup, we still might want to do a GPS cycle if it's been a while,
    // but the user requested periodic GPS. Let's trigger one if we aren't already.
    is_gps_cycle_active = true;
    gps_start_ms = millis();
    digitalWrite(PIN_GPS_ENABLE, HIGH);
  } else {
    // Normal periodic wake up for GPS cycle
    is_gps_cycle_active = true;
    gps_start_ms = millis();
    digitalWrite(PIN_GPS_ENABLE, HIGH);
    Serial.println("[PWR] Periodic wake up. Starting GPS cycle.");
  }
}

void loop() {
  // Console commands
  static char command_buf[160];
  int clen = strlen(command_buf);
  while (Serial.available() && clen < sizeof(command_buf) - 1) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (clen > 0) {
        command_buf[clen] = 0;
        char reply[160] = {0};
        the_mesh.handleCommand(0, command_buf, reply);
        if (reply[0]) Serial.println(reply);
        command_buf[0] = 0; clen = 0;
        active_mode_end_ms = millis() + ACTIVE_MODE_DURATION_MS; // Stay awake if using serial
      }
    } else {
      command_buf[clen++] = c;
      command_buf[clen] = 0;
    }
  }

  sensors.loop();
  the_mesh.loop();
  rtc_clock.tick();

  if (is_gps_cycle_active) {
    auto loc = sensors.getLocationProvider();
    // Ensure we have a valid fix and non-zero coordinates
    if (loc && loc->isValid() && sensors.node_lat != 0.0) {
      double current_lat = sensors.node_lat;
      double current_lon = sensors.node_lon;
      float current_alt = sensors.node_altitude;

      double dist = 0;
      if (has_any_gps_fix_ever) {
        dist = calculateDistance(last_known_lat, last_known_lon, current_lat, current_lon);
      }

      char msg[64];
      snprintf(msg, sizeof(msg), "Pos:%.6f,%.6f Alt:%.1f Dist:%.1fm",
               current_lat, current_lon, current_alt, dist);

      the_mesh.sendChannelAlert(msg);
      Serial.printf("[GPS] Fix acquired: %.6f,%.6f. Distance: %.1fm.\n", current_lat, current_lon, dist);

      // Update persistent storage
      last_known_lat = current_lat;
      last_known_lon = current_lon;
      last_known_alt = current_alt;
      has_any_gps_fix_ever = true;

      // Finish cycle
      is_gps_cycle_active = false;
      digitalWrite(PIN_GPS_ENABLE, LOW);

      // If we don't have an active mode timer running, we can go back to sleep soon
      if (active_mode_end_ms == 0) {
        active_mode_end_ms = millis() + 15000; // Give time for packet to leave
      }
    } else if (millis() - gps_start_ms > GPS_TIMEOUT_MS) {
      Serial.println("[GPS] Timeout waiting for fix.");
      is_gps_cycle_active = false;
      digitalWrite(PIN_GPS_ENABLE, LOW);

      if (active_mode_end_ms == 0) {
        active_mode_end_ms = millis() + 5000;
      }
    }
  }

  // Handle sleep transition
  if (!is_gps_cycle_active) {
    if (active_mode_end_ms != 0 && millis() > active_mode_end_ms) {
      Serial.println("[PWR] Active period over. Entering Deep Sleep...");
      Serial.flush();
      delay(100);

      // Safety: Clear radio interrupts before sleep
      radio_driver.getRadio().getIrqFlags();

      ((JacoSensorBoard&)board).enterDeepSleep(DEEP_SLEEP_INTERVAL_SEC);
    } else if (active_mode_end_ms == 0) {
        // We finished GPS cycle and had no active mode, sleep now
        Serial.println("[PWR] Cycle finished. Entering Deep Sleep...");
        Serial.flush();
        delay(100);

        radio_driver.getRadio().getIrqFlags();
        ((JacoSensorBoard&)board).enterDeepSleep(DEEP_SLEEP_INTERVAL_SEC);
    }
  }
}
