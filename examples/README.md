# Smart Home Hub - Arduino Examples

This folder contains Arduino sketch examples for ESP32 devices that communicate with the Smart Home Hub using MQTT and the standardized JSON envelope protocol.

## Overview

Each example demonstrates a different type of smart home device:

- **`speaker_alarm.ino`** - Speaker/alarm controller for audio alerts
- **`smart_bulb.ino`** - RGB LED bulb with dimming and color control
- **`door_lock.ino`** - Smart door lock with servo motor and sensors
- **`alarm_system.ino`** - Complete alarm system with multiple sensors
- **`temperature_sensor.ino`** - Temperature and humidity monitoring
- **`gsm_module.ino`** - GSM/cellular communication for SMS and calls

## Hardware Requirements

### Common Components (All Examples)
- ESP32 DevKit board
- Breadboard and jumper wires
- 5V power supply (for some components)

### Per-Device Components
Refer to the header comments in each `.ino` file for specific hardware requirements.

## Software Requirements

### Arduino IDE Setup
1. Install Arduino IDE 1.8.x or 2.x
2. Install ESP32 board package:
   - Go to File → Preferences
   - Add to Additional Board Manager URLs: `https://dl.espressif.com/dl/package_esp32_index.json`
   - Go to Tools → Board → Boards Manager
   - Search for "ESP32" and install

### Required Libraries
Install these libraries through Arduino IDE Library Manager:

```
WiFi (built-in with ESP32)
PubSubClient by Nick O'Leary
ArduinoJson by Benoit Blanchon
DHT sensor library by Adafruit (for temperature sensor)
ESP32Servo by Kevin Harrington (for door lock)
```

## Configuration

### 1. WiFi Settings
Update in each sketch:
```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
```

### 2. MQTT Broker Settings
Update to match your Smart Home Hub configuration:
```cpp
const char* mqtt_server = "your-broker.local";  // Hub IP or hostname
const int mqtt_port = 8883;                     // MQTT port
const char* mqtt_user = "hub";                  // MQTT username
const char* mqtt_pass = "supersecret";          // MQTT password
```

### 3. Device ID
Each device needs a unique ID:
```cpp
const char* device_id = "bulb-001";  // Change for each device
```

## Protocol Overview

All devices use a standardized JSON envelope format:

### Message Structure
```json
{
  "v": 1,           // Protocol version
  "t": 1234567890,  // Timestamp (milliseconds)
  "c": "blb",       // Capability code (3 chars)
  "e": "on",        // Event/Command name
  "i": 12345,       // Message ID
  "d": {}           // Data payload
}
```

### MQTT Topics
- **Birth**: `iot/{device-id}/birth` - Device registration
- **Events**: `iot/{device-id}/event` - Device status/sensor data
- **Commands**: `iot/{device-id}/cmd` - Commands from hub
- **Acknowledgments**: `iot/{device-id}/ack` - Command responses

### Capability Codes
- `spk` - Speaker/Audio
- `blb` - Bulb/Light
- `lck` - Lock
- `alm` - Alarm System
- `tmp` - Temperature Sensor
- `gsm` - GSM Module

## Device Examples

### 1. Speaker Alarm (`speaker_alarm.ino`)
**Hardware**: Buzzer/speaker on GPIO 25  
**Commands**: `on`, `off`, `beep`, `alarm`, `tone`  
**Features**: Multiple alarm patterns, frequency control

### 2. Smart Bulb (`smart_bulb.ino`)
**Hardware**: RGB LED on GPIOs 16, 17, 18  
**Commands**: `on`, `off`, `dim`, `color`, `blink`  
**Features**: RGB color control, brightness, blinking patterns

### 3. Door Lock (`door_lock.ino`)
**Hardware**: Servo on GPIO 21, reed switch on GPIO 22  
**Commands**: `lock`, `unlock`, `status`, `config`  
**Features**: Auto-lock timer, door position sensing

### 4. Alarm System (`alarm_system.ino`)
**Hardware**: PIR sensor, door sensors, siren  
**Commands**: `arm`, `disarm`, `status`, `config`, `test`  
**Features**: Multiple arm modes, entry/exit delays

### 5. Temperature Sensor (`temperature_sensor.ino`)
**Hardware**: DHT22 sensor on GPIO 23  
**Commands**: `read`, `config`, `calibrate`  
**Features**: Temperature/humidity, heat index, alerts

### 6. GSM Module (`gsm_module.ino`)
**Hardware**: SIM800L module via Serial2  
**Commands**: `sms`, `call`, `status`, `config`, `signal`  
**Features**: SMS sending/receiving, emergency contacts

## Usage Examples

### Testing with Hub API

1. **List devices**:
   ```bash
   curl http://localhost:8080/devices
   ```

2. **Send command**:
   ```bash
   curl -X POST http://localhost:8080/devices/bulb-001/cmd \
     -H "Content-Type: application/json" \
     -d '{"cap": "blb", "cmd": "color", "data": {"r": 255, "g": 0, "b": 0}}'
   ```

### MQTT Testing with mosquitto_pub

```bash
# Turn on bulb
mosquitto_pub -h localhost -p 8883 \
  -t "iot/bulb-001/cmd" \
  -m '{"v":1,"t":1234567890,"c":"blb","e":"on","i":1,"d":{}}'

# Lock door
mosquitto_pub -h localhost -p 8883 \
  -t "iot/lock-001/cmd" \
  -m '{"v":1,"t":1234567890,"c":"lck","e":"lock","i":2,"d":{}}'
```

## Troubleshooting

### Common Issues

1. **WiFi Connection Failed**
   - Check SSID and password
   - Ensure 2.4GHz network (ESP32 doesn't support 5GHz)

2. **MQTT Connection Failed**
   - Verify broker IP/hostname
   - Check port and credentials
   - Ensure TLS certificates are valid

3. **Device Not Appearing in Hub**
   - Check birth message is being sent
   - Verify MQTT topic format
   - Check device ID uniqueness

4. **Commands Not Working**
   - Verify capability codes match
   - Check JSON message format
   - Monitor serial output for errors

### Debug Tips

- Enable Serial Monitor (115200 baud) to see debug messages
- Check MQTT broker logs
- Use MQTT client tools to monitor traffic
- Verify hardware connections with multimeter

## Extending Examples

To create new device types:

1. Copy an existing example
2. Update capability code and device ID
3. Modify hardware pin assignments
4. Implement device-specific commands
5. Update birth message with new capabilities
6. Test with hub API

## Security Notes

These examples use `espClient.setInsecure()` for development. In production:

1. Use proper TLS certificates
2. Implement certificate validation
3. Use strong MQTT passwords
4. Enable MQTT authentication
5. Consider device-specific certificates

## Support

For issues with these examples:
1. Check hardware connections
2. Verify library versions
3. Monitor serial output
4. Test MQTT connectivity separately
5. Refer to Smart Home Hub documentation
