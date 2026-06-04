# 🛰️ MeshTracker for ESP32-C3

A professional, high-precision LoRa Mesh tracker built on the **MeshCore** framework. Designed for the **ESP32-C3 Super Mini** paired with a **WIO SX1262** and a **NEO-6M GPS**.

## ✨ Key Features

- **Intelligent GPS Cycle**: Powers on the GPS module only when needed for reporting or on-demand requests.
- **Precision Reporting**: Waits for a stable GPS fix and target accuracy (HDOP converted to meters) before transmitting.
- **Dual-Mode Reporting**:
  - **Group Alerts**: Periodic position updates sent to a public or private mesh channel.
  - **Private Tracking**: High-priority updates sent directly to an Admin node.
- **Regional Scoping**: Supports HMAC-based transport codes to restrict position updates to specific mesh regions.
- **Advanced Power Management**: Utilizes MeshCore Light Sleep (powersaving) with serial protection to ensure no data is lost during GPS acquisition.
- **Universal CLI**: Fully configurable at runtime via a serial console.
- **Sensor Telemetry**: Includes multi-channel battery voltage (INA3221), temperature, and humidity (AHT10).
- **RTC Stability**: Uses a DS3231 RTC for precise scheduling, with robust GPS-to-RTC time synchronization and sanity checking.

## 🛠️ Hardware Configuration

### Pinout (ESP32-C3 Super Mini)

| Component | Pin | Function |
| :--- | :--- | :--- |
| **LoRa (SX1262)** | GPIO 2 | SPI SCK |
| | GPIO 3 | SPI MISO |
| | GPIO 4 | SPI MOSI |
| | GPIO 10 | SPI NSS (CS) |
| | GPIO 5 | LORA DIO1 |
| | GPIO 6 | LORA BUSY |
| | GPIO 7 | LORA RESET |
| **GPS (NEO-6M)** | GPIO 20 | TX (to GPS RX) |
| | GPIO 21 | RX (from GPS TX) |
| | GPIO 1 | Power Control (NPN High-side) |
| **I2C (Sensors/RTC)** | GPIO 8 | SDA |
| | GPIO 9 | SCL |

### Sensors Supported
- **INA3221**: Triple-channel voltage/current monitor (Address 0x42).
- **AHT10**: Temperature and Humidity.

## 🚀 Getting Started

### 1. Build & Flash
Use PlatformIO to build and upload the firmware:
```bash
pio run -e mesh-tracker -t upload
```

### 2. Initial Configuration
Connect via Serial (115200 baud) and type `tracker help` to see available commands.

#### Configure Mesh Channel
Set your private 16-byte hex key (this automatically derives the 1-byte channel hash via SHA256):
```text
set channel.key 3e723e16eb4b2564fa235e9f816d4976
```

#### Configure Reporting Intervals
Set the frequency for group and private reports (in minutes):
```text
set reporting.group 60
set reporting.pvt 30
```

#### Set Regional Scope
To limit group alerts to a specific mesh region:
```text
set channel.scope luban
```

## 📡 Messaging & Telemetry

### Report Format
`Pos:51.1173,15.2947 Alt:321.8 Dist:18.4m Acc:~8m | CH3:5.26V | ATH:28.0C/44.5%`

- **Dist**: Haversine distance calculated from the last known stable position.
- **Acc**: Estimated horizontal accuracy in meters (derived from HDOP).
- **ATH**: Ambient Temperature and Humidity from the AHT10 sensor.

### On-Demand Requests
The tracker will automatically wake up and start a GPS acquisition cycle if:
1. It receives an authorized `PAYLOAD_TYPE_REQ`.
2. It receives a text message or data on its configured Group Channel.
3. It receives any Private message from a known Admin.

## 🔋 Power Management
The firmware uses `powersaving_enabled = 1` for Light Sleep.
- **Active Mode**: GPS is powered ON, and the CPU stays awake for stable serial communication.
- **Idle Mode**: GPS is powered OFF (GPIO 1 LOW), and the ESP32-C3 enters Light Sleep between mesh housekeeping tasks.

## ⚖️ License
This project is an extension of **MeshCore** and is released under the **MIT License**.
