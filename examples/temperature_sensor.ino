/*
 * ESP32 Smart Home Hub - Temperature Sensor
 * 
 * This example demonstrates a temperature and humidity monitoring system
 * using the Smart Home Hub MQTT protocol.
 * 
 * Hardware:
 * - ESP32 DevKit
 * - DHT22 temperature/humidity sensor on GPIO 23
 * - Optional: DS18B20 waterproof temperature sensor on GPIO 24
 * - Optional: Status LED on GPIO 2
 * 
 * Capabilities: "tmp" (Temperature)
 * Commands: "read", "config", "calibrate"
 * Events: "reading", "alert", "status", "error"
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT settings
const char* mqtt_server = "your-broker.local";
const int mqtt_port = 8883;
const char* mqtt_user = "hub";
const char* mqtt_pass = "supersecret";

// Device settings
const char* device_id = "temp-001";
const int DHT_PIN = 23;
const int STATUS_LED_PIN = 2;
const int HEARTBEAT_INTERVAL = 30000; // 30 seconds

#define DHT_TYPE DHT22

// MQTT topics
String birth_topic = "iot/" + String(device_id) + "/birth";
String event_topic = "iot/" + String(device_id) + "/event";
String cmd_topic = "iot/" + String(device_id) + "/cmd";
String ack_topic = "iot/" + String(device_id) + "/ack";

WiFiClientSecure espClient;
PubSubClient client(espClient);
DHT dht(DHT_PIN, DHT_TYPE);

unsigned long lastHeartbeat = 0;
unsigned long lastReading = 0;
unsigned long msgCounter = 0;

// Sensor configuration
int readingInterval = 60000;  // 1 minute default
float tempOffset = 0.0;       // Calibration offset for temperature
float humidityOffset = 0.0;   // Calibration offset for humidity

// Alert thresholds
float tempMinAlert = -10.0;
float tempMaxAlert = 40.0;
float humidityMinAlert = 20.0;
float humidityMaxAlert = 80.0;
bool alertsEnabled = true;

// Current readings
float currentTemp = NAN;
float currentHumidity = NAN;
float heatIndex = NAN;

// Moving average for stability
const int SAMPLE_SIZE = 5;
float tempSamples[SAMPLE_SIZE];
float humiditySamples[SAMPLE_SIZE];
int sampleIndex = 0;
bool samplesReady = false;

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  // Initialize DHT sensor
  dht.begin();
  
  // Initialize sample arrays
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    tempSamples[i] = 0.0;
    humiditySamples[i] = 0.0;
  }
  
  // Connect to WiFi
  setup_wifi();
  
  // Configure MQTT
  espClient.setInsecure(); // For development - use proper certificates in production
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  // Connect to MQTT
  reconnect();
  
  // Send birth message
  sendBirth();
  
  Serial.println("Temperature sensor controller ready!");
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("Received: " + message);
  
  // Parse JSON envelope
  StaticJsonDocument<512> doc;
  deserializeJson(doc, message);
  
  if (doc["v"] == 1 && doc["c"] == "tmp") {
    String command = doc["e"];
    int msgId = doc["i"];
    JsonObject data = doc["d"];
    
    handleCommand(command, data, msgId);
  }
}

void handleCommand(String command, JsonObject data, int msgId) {
  bool success = true;
  String result = "";
  
  if (command == "read") {
    takeSensorReading();
    result = "Reading taken - Temp: " + String(currentTemp, 1) + "°C, Humidity: " + String(currentHumidity, 1) + "%";
    
  } else if (command == "config") {
    if (data.containsKey("interval")) {
      readingInterval = data["interval"] * 1000; // Convert to milliseconds
      result += "Interval: " + String(readingInterval/1000) + "s ";
    }
    if (data.containsKey("temp_min_alert")) {
      tempMinAlert = data["temp_min_alert"];
      result += "Temp min alert: " + String(tempMinAlert) + "°C ";
    }
    if (data.containsKey("temp_max_alert")) {
      tempMaxAlert = data["temp_max_alert"];
      result += "Temp max alert: " + String(tempMaxAlert) + "°C ";
    }
    if (data.containsKey("humidity_min_alert")) {
      humidityMinAlert = data["humidity_min_alert"];
      result += "Humidity min alert: " + String(humidityMinAlert) + "% ";
    }
    if (data.containsKey("humidity_max_alert")) {
      humidityMaxAlert = data["humidity_max_alert"];
      result += "Humidity max alert: " + String(humidityMaxAlert) + "% ";
    }
    if (data.containsKey("alerts_enabled")) {
      alertsEnabled = data["alerts_enabled"];
      result += "Alerts: " + String(alertsEnabled ? "enabled" : "disabled") + " ";
    }
    if (result == "") {
      result = "Configuration retrieved";
    }
    
  } else if (command == "calibrate") {
    if (data.containsKey("temp_offset")) {
      tempOffset = data["temp_offset"];
      result += "Temp offset: " + String(tempOffset) + "°C ";
    }
    if (data.containsKey("humidity_offset")) {
      humidityOffset = data["humidity_offset"];
      result += "Humidity offset: " + String(humidityOffset) + "% ";
    }
    if (result == "") {
      result = "Calibration values retrieved";
    }
    
  } else {
    success = false;
    result = "Unknown command: " + command;
  }
  
  // Send acknowledgment
  sendAck(msgId, success, result);
  
  // Send status event
  sendEvent("status", createStatusData());
}

void takeSensorReading() {
  // Read raw values from DHT sensor
  float rawTemp = dht.readTemperature();
  float rawHumidity = dht.readHumidity();
  
  // Check if readings are valid
  if (isnan(rawTemp) || isnan(rawHumidity)) {
    Serial.println("Failed to read from DHT sensor!");
    
    // Send error event
    StaticJsonDocument<256> doc;
    JsonObject data = doc.to<JsonObject>();
    data["error"] = "sensor_read_failed";
    data["timestamp"] = millis();
    sendEvent("error", data);
    
    return;
  }
  
  // Apply calibration offsets
  float calibratedTemp = rawTemp + tempOffset;
  float calibratedHumidity = rawHumidity + humidityOffset;
  
  // Add to moving average
  tempSamples[sampleIndex] = calibratedTemp;
  humiditySamples[sampleIndex] = calibratedHumidity;
  sampleIndex = (sampleIndex + 1) % SAMPLE_SIZE;
  
  if (sampleIndex == 0) {
    samplesReady = true;
  }
  
  // Calculate moving averages
  if (samplesReady) {
    float tempSum = 0, humiditySum = 0;
    for (int i = 0; i < SAMPLE_SIZE; i++) {
      tempSum += tempSamples[i];
      humiditySum += humiditySamples[i];
    }
    currentTemp = tempSum / SAMPLE_SIZE;
    currentHumidity = humiditySum / SAMPLE_SIZE;
  } else {
    currentTemp = calibratedTemp;
    currentHumidity = calibratedHumidity;
  }
  
  // Calculate heat index (feels like temperature)
  heatIndex = calculateHeatIndex(currentTemp, currentHumidity);
  
  // Send reading event
  StaticJsonDocument<512> doc;
  JsonObject data = doc.to<JsonObject>();
  data["temperature"] = round(currentTemp * 10) / 10.0; // Round to 1 decimal
  data["humidity"] = round(currentHumidity * 10) / 10.0;
  data["heat_index"] = round(heatIndex * 10) / 10.0;
  data["raw_temperature"] = round(rawTemp * 10) / 10.0;
  data["raw_humidity"] = round(rawHumidity * 10) / 10.0;
  data["timestamp"] = millis();
  sendEvent("reading", data);
  
  // Check for alerts
  checkAlerts();
  
  // Update status LED (blink once for successful reading)
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(100);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  Serial.println("Temp: " + String(currentTemp, 1) + "°C, Humidity: " + String(currentHumidity, 1) + "%, Heat Index: " + String(heatIndex, 1) + "°C");
}

float calculateHeatIndex(float temp, float humidity) {
  // Heat index formula (Fahrenheit-based, converted)
  float tempF = temp * 9.0/5.0 + 32.0; // Convert to Fahrenheit
  
  if (tempF < 80.0) {
    return temp; // Heat index only applies at higher temperatures
  }
  
  float hi = -42.379 + 2.04901523 * tempF + 10.14333127 * humidity
           - 0.22475541 * tempF * humidity - 6.83783e-3 * tempF * tempF
           - 5.481717e-2 * humidity * humidity + 1.22874e-3 * tempF * tempF * humidity
           + 8.5282e-4 * tempF * humidity * humidity - 1.99e-6 * tempF * tempF * humidity * humidity;
  
  // Convert back to Celsius
  return (hi - 32.0) * 5.0/9.0;
}

void checkAlerts() {
  if (!alertsEnabled) return;
  
  bool alertTriggered = false;
  String alertType = "";
  String alertMessage = "";
  
  if (currentTemp < tempMinAlert) {
    alertTriggered = true;
    alertType = "temperature_low";
    alertMessage = "Temperature below minimum threshold: " + String(currentTemp, 1) + "°C < " + String(tempMinAlert) + "°C";
  } else if (currentTemp > tempMaxAlert) {
    alertTriggered = true;
    alertType = "temperature_high";
    alertMessage = "Temperature above maximum threshold: " + String(currentTemp, 1) + "°C > " + String(tempMaxAlert) + "°C";
  } else if (currentHumidity < humidityMinAlert) {
    alertTriggered = true;
    alertType = "humidity_low";
    alertMessage = "Humidity below minimum threshold: " + String(currentHumidity, 1) + "% < " + String(humidityMinAlert) + "%";
  } else if (currentHumidity > humidityMaxAlert) {
    alertTriggered = true;
    alertType = "humidity_high";
    alertMessage = "Humidity above maximum threshold: " + String(currentHumidity, 1) + "% > " + String(humidityMaxAlert) + "%";
  }
  
  if (alertTriggered) {
    StaticJsonDocument<512> doc;
    JsonObject data = doc.to<JsonObject>();
    data["alert_type"] = alertType;
    data["message"] = alertMessage;
    data["temperature"] = currentTemp;
    data["humidity"] = currentHumidity;
    data["timestamp"] = millis();
    sendEvent("alert", data);
    
    Serial.println("ALERT: " + alertMessage);
  }
}

JsonObject createStatusData() {
  StaticJsonDocument<512> doc;
  JsonObject data = doc.to<JsonObject>();
  
  data["temperature"] = isnan(currentTemp) ? nullptr : currentTemp;
  data["humidity"] = isnan(currentHumidity) ? nullptr : currentHumidity;
  data["heat_index"] = isnan(heatIndex) ? nullptr : heatIndex;
  data["reading_interval"] = readingInterval / 1000;
  data["temp_offset"] = tempOffset;
  data["humidity_offset"] = humidityOffset;
  data["alerts_enabled"] = alertsEnabled;
  data["temp_min_alert"] = tempMinAlert;
  data["temp_max_alert"] = tempMaxAlert;
  data["humidity_min_alert"] = humidityMinAlert;
  data["humidity_max_alert"] = humidityMaxAlert;
  data["samples_ready"] = samplesReady;
  data["uptime"] = millis();
  data["wifi_rssi"] = WiFi.RSSI();
  
  return data;
}

void sendBirth() {
  StaticJsonDocument<512> doc;
  doc["v"] = 1;
  doc["t"] = millis();
  doc["c"] = "tmp";
  doc["e"] = "birth";
  doc["i"] = msgCounter++;
  
  JsonObject data = doc.createNestedObject("d");
  data["hb"] = 30; // Heartbeat interval in seconds
  data["fw"] = "1.0.0";
  data["sensor_type"] = "DHT22";
  
  JsonArray caps = data.createNestedArray("caps");
  JsonObject cap = caps.createNestedObject();
  cap["c"] = "tmp";
  cap["cmds"] = "read,config,calibrate";
  cap["events"] = "reading,alert,status,error";
  
  String message;
  serializeJson(doc, message);
  client.publish(birth_topic.c_str(), message.c_str());
  
  Serial.println("Birth sent: " + message);
}

void sendEvent(String event, JsonObject eventData) {
  StaticJsonDocument<512> doc;
  doc["v"] = 1;
  doc["t"] = millis();
  doc["c"] = "tmp";
  doc["e"] = event;
  doc["i"] = msgCounter++;
  doc["d"] = eventData;
  
  String message;
  serializeJson(doc, message);
  client.publish(event_topic.c_str(), message.c_str());
  
  Serial.println("Event sent: " + message);
}

void sendAck(int originalMsgId, bool success, String result) {
  StaticJsonDocument<256> doc;
  doc["v"] = 1;
  doc["t"] = millis();
  doc["c"] = "tmp";
  doc["e"] = success ? "ack" : "nack";
  doc["i"] = originalMsgId;
  
  JsonObject data = doc.createNestedObject("d");
  data["result"] = result;
  
  String message;
  serializeJson(doc, message);
  client.publish(ack_topic.c_str(), message.c_str());
  
  Serial.println("Ack sent: " + message);
}

void sendHeartbeat() {
  sendEvent("heartbeat", createStatusData());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect(device_id, mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      client.subscribe(cmd_topic.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  // Take sensor readings at configured interval
  if (millis() - lastReading > readingInterval) {
    takeSensorReading();
    lastReading = millis();
  }
  
  // Send heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeat = millis();
  }
  
  delay(1000);
}
