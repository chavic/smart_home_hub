/*
 * ESP32 Smart Home Hub - Smart Bulb Controller
 * 
 * This example demonstrates controlling an RGB LED bulb
 * using the Smart Home Hub MQTT protocol.
 * 
 * Hardware:
 * - ESP32 DevKit
 * - RGB LED or RGB LED strip connected to:
 *   - Red: GPIO 16
 *   - Green: GPIO 17
 *   - Blue: GPIO 18
 * - Optional: MOSFET drivers for high-power LEDs
 * 
 * Capabilities: "blb" (Bulb)
 * Commands: "on", "off", "dim", "color", "blink"
 * Events: "status", "error"
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT settings
const char* mqtt_server = "your-broker.local";
const int mqtt_port = 8883;
const char* mqtt_user = "hub";
const char* mqtt_pass = "supersecret";

// Device settings
const char* device_id = "bulb-001";
const int RED_PIN = 16;
const int GREEN_PIN = 17;
const int BLUE_PIN = 18;
const int HEARTBEAT_INTERVAL = 30000; // 30 seconds

// MQTT topics
String birth_topic = "iot/" + String(device_id) + "/birth";
String event_topic = "iot/" + String(device_id) + "/event";
String cmd_topic = "iot/" + String(device_id) + "/cmd";
String ack_topic = "iot/" + String(device_id) + "/ack";

WiFiClientSecure espClient;
PubSubClient client(espClient);

unsigned long lastHeartbeat = 0;
unsigned long msgCounter = 0;

// Bulb state
bool isOn = false;
int brightness = 100; // 0-100%
int red = 255;
int green = 255;
int blue = 255;
bool isBlinking = false;
unsigned long blinkInterval = 1000;
unsigned long lastBlink = 0;
bool blinkState = false;

void setup() {
  Serial.begin(115200);
  
  // Initialize LED pins
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  
  // Setup PWM channels
  ledcSetup(0, 5000, 8); // Red channel
  ledcSetup(1, 5000, 8); // Green channel
  ledcSetup(2, 5000, 8); // Blue channel
  
  ledcAttachPin(RED_PIN, 0);
  ledcAttachPin(GREEN_PIN, 1);
  ledcAttachPin(BLUE_PIN, 2);
  
  // Turn off initially
  updateBulb();
  
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
  
  Serial.println("Smart bulb controller ready!");
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
  
  if (doc["v"] == 1 && doc["c"] == "blb") {
    String command = doc["e"];
    int msgId = doc["i"];
    JsonObject data = doc["d"];
    
    handleCommand(command, data, msgId);
  }
}

void handleCommand(String command, JsonObject data, int msgId) {
  bool success = true;
  String result = "";
  
  if (command == "on") {
    isOn = true;
    isBlinking = false;
    result = "Bulb turned on";
    
  } else if (command == "off") {
    isOn = false;
    isBlinking = false;
    result = "Bulb turned off";
    
  } else if (command == "dim") {
    if (data.containsKey("level")) {
      brightness = constrain(data["level"], 0, 100);
      result = "Brightness set to " + String(brightness) + "%";
    } else {
      success = false;
      result = "Missing brightness level";
    }
    
  } else if (command == "color") {
    if (data.containsKey("r") && data.containsKey("g") && data.containsKey("b")) {
      red = constrain(data["r"], 0, 255);
      green = constrain(data["g"], 0, 255);
      blue = constrain(data["b"], 0, 255);
      result = "Color set to RGB(" + String(red) + "," + String(green) + "," + String(blue) + ")";
    } else if (data.containsKey("hex")) {
      String hexColor = data["hex"];
      if (parseHexColor(hexColor)) {
        result = "Color set to " + hexColor;
      } else {
        success = false;
        result = "Invalid hex color format";
      }
    } else {
      success = false;
      result = "Missing color parameters";
    }
    
  } else if (command == "blink") {
    isBlinking = true;
    blinkInterval = data.containsKey("interval") ? data["interval"] : 1000;
    result = "Blinking started with " + String(blinkInterval) + "ms interval";
    
  } else {
    success = false;
    result = "Unknown command: " + command;
  }
  
  // Update bulb state
  updateBulb();
  
  // Send acknowledgment
  sendAck(msgId, success, result);
  
  // Send status event
  sendEvent("status", createStatusData());
}

bool parseHexColor(String hex) {
  if (hex.length() != 7 || hex.charAt(0) != '#') {
    return false;
  }
  
  long number = strtol(hex.substring(1).c_str(), NULL, 16);
  red = (number >> 16) & 0xFF;
  green = (number >> 8) & 0xFF;
  blue = number & 0xFF;
  
  return true;
}

void updateBulb() {
  if (!isOn) {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    return;
  }
  
  // Apply brightness scaling
  int scaledRed = (red * brightness) / 100;
  int scaledGreen = (green * brightness) / 100;
  int scaledBlue = (blue * brightness) / 100;
  
  ledcWrite(0, scaledRed);
  ledcWrite(1, scaledGreen);
  ledcWrite(2, scaledBlue);
}

void handleBlink() {
  if (!isBlinking) return;
  
  if (millis() - lastBlink > blinkInterval) {
    blinkState = !blinkState;
    lastBlink = millis();
    
    if (blinkState) {
      updateBulb();
    } else {
      ledcWrite(0, 0);
      ledcWrite(1, 0);
      ledcWrite(2, 0);
    }
  }
}

JsonObject createStatusData() {
  StaticJsonDocument<256> doc;
  JsonObject data = doc.to<JsonObject>();
  
  data["on"] = isOn;
  data["brightness"] = brightness;
  data["red"] = red;
  data["green"] = green;
  data["blue"] = blue;
  data["blinking"] = isBlinking;
  data["uptime"] = millis();
  data["wifi_rssi"] = WiFi.RSSI();
  
  return data;
}

void sendBirth() {
  StaticJsonDocument<512> doc;
  doc["v"] = 1;
  doc["t"] = millis();
  doc["c"] = "blb";
  doc["e"] = "birth";
  doc["i"] = msgCounter++;
  
  JsonObject data = doc.createNestedObject("d");
  data["hb"] = 30; // Heartbeat interval in seconds
  data["fw"] = "1.0.0";
  
  JsonArray caps = data.createNestedArray("caps");
  JsonObject cap = caps.createNestedObject();
  cap["c"] = "blb";
  cap["cmds"] = "on,off,dim,color,blink";
  cap["events"] = "status,error";
  
  String message;
  serializeJson(doc, message);
  client.publish(birth_topic.c_str(), message.c_str());
  
  Serial.println("Birth sent: " + message);
}

void sendEvent(String event, JsonObject eventData) {
  StaticJsonDocument<512> doc;
  doc["v"] = 1;
  doc["t"] = millis();
  doc["c"] = "blb";
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
  doc["c"] = "blb";
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
  
  // Handle blinking
  handleBlink();
  
  // Send heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeat = millis();
  }
  
  delay(50);
}
